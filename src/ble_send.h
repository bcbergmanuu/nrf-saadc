#ifndef BLE_SEND__
#define BLE_SEND__
#include <zephyr/types.h>


void SendBufferd(int16_t *data, uint32_t size);




void notify_changed(struct k_work *work);
void ble_notify_changed(struct k_work *work);
void ppi_value_updated(struct k_work *work);



#endif
