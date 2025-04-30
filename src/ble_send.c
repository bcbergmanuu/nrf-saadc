#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>

#include "adc_control.h"
#include "ble_send.h"
#include "ppi_config.h" 


LOG_MODULE_REGISTER(BLE_MODULE, CONFIG_LOG_DEFAULT_LEVEL);



#define adcResults_size 16

uint8_t notify_ble_resp_on1 = 0;

//general service
static struct bt_uuid_128 respiratory_general_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xa5c47c93, 0x8a02, 0x4ca5, 0x976e, 0xde7b871f11f3));

//notify service 
static struct bt_uuid_128 saadc_notify_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xd67b5503, 0xa25c, 0x4395, 0xbd21, 0xf92823d91442));

//PPI enable
static struct bt_uuid_128 ppi_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xd60b5543, 0xa25c, 0x4395, 0xbd21, 0xf92823d91442));

static uint8_t respiratory_notify_buff[adcResults_size];
static uint8_t ppi_state_buff[1];

const struct bt_data ad[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
	};
		//BT_DATA_BYTES(BT_DATA_UUID128_ALL, motiondata_prop),
// 			BT_DATA(BT_DATA_MANUFACTURER_DATA, uniqueId, sizeof(ui))
// 	};
// }

void notify_changed(struct k_work *work);
void ppi_value_updated(struct k_work *work);

K_WORK_DEFINE(work_notify_changed, notify_changed);
K_WORK_DEFINE(work_ppi_value_updated, ppi_value_updated);
static struct k_work_q work_q_ble;

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_INF("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err 0x%02x)\n", err);
	} else {
		LOG_INF("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)\n", reason);
}

static void le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info) {
	LOG_INF("LE data len updated: TX (len: %d time: %d)"
	       " RX (len: %d time: %d)\n", info->tx_max_len,
	       info->tx_max_time, info->rx_max_len, info->rx_max_time);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_data_len_updated = le_data_length_updated,
};

static void resp_readmotion_data_notify_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	LOG_INF("resp_notify_changed");
	notify_ble_resp_on1 = (value == BT_GATT_CCC_NOTIFY || value == BT_GATT_CCC_INDICATE);
		 
	k_work_submit_to_queue(&work_q_ble, &work_notify_changed);
}

static ssize_t ppi_buffer_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, 
	uint16_t len, uint16_t offset, uint8_t flags) {	
	
	uint8_t *value = attr->user_data;	
	memcpy(value + offset, buf, len);
	
	k_work_submit_to_queue(&work_q_ble, &work_ppi_value_updated);
	return len;
}

static ssize_t ppi_buffer_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	void *buf, uint16_t len, uint16_t offset) {
	
	const uint8_t *value = attr->user_data;	
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(ppi_state_buff));
}

BT_GATT_SERVICE_DEFINE(motion_svc,
	BT_GATT_PRIMARY_SERVICE(&respiratory_general_prop),
	
	BT_GATT_CHARACTERISTIC(&ppi_prop.uuid, 
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		ppi_buffer_read, ppi_buffer_write, ppi_state_buff),				   
	BT_GATT_CHARACTERISTIC(&saadc_notify_prop.uuid, 
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		NULL, NULL, respiratory_notify_buff),	
	BT_GATT_CCC(resp_readmotion_data_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

static void bt_ready(void)
{
	int err;

	LOG_INF("Bluetooth initialized\n");	

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}

	LOG_INF("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = NULL,// auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
	.passkey_confirm = NULL,	
};

void notify_changed(struct k_work *work) {
	(notify_ble_resp_on1 == 1 ? calibrate_and_start : adc_stop)();
	LOG_INF("notify changed to %d", notify_ble_resp_on1);
}

void ppi_value_updated(struct k_work *work) {
	(ppi_state_buff[0] == 1 ? ppi_start : ppi_stop)();
	LOG_INF("ppi state updated: %d", ppi_state_buff[0]);
}

K_THREAD_STACK_DEFINE(threatstack_notifyen, 1024);

int ble_init()
{		
	int err;
	k_work_queue_init(&work_q_ble);

	k_work_queue_start(&work_q_ble, threatstack_notifyen,
		K_THREAD_STACK_SIZEOF(threatstack_notifyen), 95,
		NULL);	

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_ready();

	bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	//clean up bonds
	bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
	
	return err;
}



//void ble_notify_respiratory_proc(int16_t adcitems[]) {		
	// struct bt_gatt_attr *notify_attr = bt_gatt_find_by_uuid(motion_svc.attrs, motion_svc.attr_count, &respiratory_notify_prop.uuid);
	    
    //     if(notify_ble_resp_on1){

	// 		size_t packed_size;        

	// 		encode_data(adcitems, respiratory_notify_buff, &packed_size);
	// 		LOG_INF("notify:");
//			bt_gatt_notify(NULL, notify_attr, respiratory_notify_buff, packed_size);			
//		}    
//}


// K_THREAD_DEFINE(ble_sender, 2048, ble_notify_respiratory_proc,  NULL, NULL, NULL, 7, 0, 0);
SYS_INIT(ble_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
