#include <zephyr/kernel.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_saadc.h>
#include <zephyr/logging/log.h>
#include "battery.h"
#include "adc_control.h"

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN7, 0);
static const struct device *gpio_battery_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

static int16_t sample;

static void battery_sample_timer_handler(struct k_timer *timer);

K_TIMER_DEFINE(battery_sample_timer, battery_sample_timer_handler, NULL);

static int configure_saadc(void);

void battery_sample_timer_handler(struct k_timer *timer)
{    
    int ret = 0;
	if(k_mutex_lock(&saadc_mutex, K_NO_WAIT) != 0) {
        LOG_INF("mutex locked, skip measure");
        return;
    }
    LOG_INF("configure saadc for measure");
    ret |= configure_saadc();
    if(ret == -1) {
        LOG_INF("mutex busy");
        return;
    }
    ret |= gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, 1);
    nrfx_err_t err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_mode_trigger error: %08x", err);
    
    }
    err = 0;
    nrfx_saadc_uninit();
    k_mutex_unlock(&saadc_mutex);

    ret |= gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, 0);
    
    int adc_mv = ((600*6) * sample) / ((1<<12));    
    uint16_t battery_millivolt = adc_mv * ((R1 + R2) / R2);
    uint8_t percentage = 0;
    battery_get_percentage(&percentage, battery_millivolt);    
    err |= bt_bas_set_battery_level(percentage);
    
    if(err != 0 ) {
        LOG_ERR("err bt set level: %d", err);        
    }

    LOG_INF("SAADC value: %d\n", sample);
    LOG_INF("Battery Voltage: %dmV\n", battery_millivolt);
    LOG_INF("Battery Percentage: %d%%\n", percentage);
    if(err) {
        LOG_ERR("err occured reading battery level %d", err);
    }
}

static BatteryState battery_states[] = {
    {4200, 100},
    {4160, 99},
    {4090, 91},
    {4030, 78},
    {3890, 63},
    {3830, 53},
    {3680, 36},
    {3660, 35},
    {3480, 14},
    {3420, 11},
    {3150, 1}, // 3240
    {0000, 0}  // Below safe level
};

static int configure_saadc(void)
{            
    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) 
    {
            LOG_ERR("nrfx_saadc_mode_trigger error: %08x", err);
            return err;
    }
    
    channel.channel_config.gain = NRF_SAADC_GAIN1_6;

    err = nrfx_saadc_channels_config(&channel, 1);
    if (err != NRFX_SUCCESS) 
    {
        LOG_ERR("nrfx_saadc_channels_config error: %08x", err);
        return err;
    }        
    err = nrfx_saadc_simple_mode_set(BIT(0),
                                        NRF_SAADC_RESOLUTION_12BIT,
                                        NRF_SAADC_OVERSAMPLE_DISABLED,
                                        NULL);
    if (err != NRFX_SUCCESS) {
            LOG_ERR("nrfx_saadc_simple_mode_set error: %08x", err);
            return err;
    }
    
    /* STEP 5.5 - Set buffer where sample will be stored */
    err = nrfx_saadc_buffer_set(&sample, 1);
    if (err != NRFX_SUCCESS) {
            LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);            
    }    
    return err;
}

int battery_get_percentage(uint8_t *battery_percentage, uint16_t battery_millivolt)
{

    // Ensure voltage is within bounds
    if (battery_millivolt > battery_states[0].voltage)
        *battery_percentage = 100;
    if (battery_millivolt < battery_states[ARRAY_SIZE(battery_states)-1].voltage)
        *battery_percentage = 0;

    for (uint8_t i = 0; i < ARRAY_SIZE(battery_states)-1; i++)
    {
        // Find the two points battery_millivolt is between
        if (battery_states[i].voltage >= battery_millivolt && battery_millivolt >= battery_states[i + 1].voltage)
        {
            // Linear interpolation
            *battery_percentage = battery_states[i].percentage +
                                  ((float)(battery_millivolt - battery_states[i].voltage) *
                                   ((float)(battery_states[i + 1].percentage - battery_states[i].percentage) /
                                    (float)(battery_states[i + 1].voltage - battery_states[i].voltage)));

            LOG_DBG("%d %%", *battery_percentage);
            return 0;
        }
    }
    return 0;
}

int battery_init()
{
    int ret = 0;

    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_CHARGE_SPEED, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    ret |= gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_CHARGE_SPEED, 1); // FAST charge 100mA    
    k_timer_start(&battery_sample_timer, K_NO_WAIT, K_SECONDS(2));
    
    if(ret != 0) {
        LOG_ERR("error, %d", ret);
    }    

    return ret;
}

SYS_INIT(battery_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
