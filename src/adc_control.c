/*
 * Copyright (c) 2022 - 2024, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
*/

#include <zephyr/kernel.h>
#include "adc_control.h"
#include <nrfx_saadc.h>
#include <nrfx_gpiote.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>                                                                                                                                                     

#include "ble_send.h"

LOG_MODULE_REGISTER(SAADC_MODULE, CONFIG_LOG_DEFAULT_LEVEL);


const struct device *const gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
struct k_mutex saadc_mutex;

//K_HEAP_DEFINE(BUFFER,512);


/**
 * @defgroup nrfx_saadc_advanced_non_blocking_internal_timer_example Advanced non-blocking sampling with internal timer SAADC example
 * @{
 * @ingroup nrfx_saadc_examples
 *
 * @brief Example showing advanced functionality of nrfx_saadc driver operating in the non-blocking continuous sampling mode with internal timer.
 *
 * @details Application initializes nrfx_saadc driver and starts operating in the non-blocking mode.
 *          In the example, @ref m_single_channel is configured and the SAADC driver is set to the advanced mode
 *          with @p nrfx_saadc_adv_config_t::start_on_end enabled. With that flag @p NRF_SAADC_TASK_START task
 *          is triggered on the @p NRF_SAADC_EVENT_END event automatically by the SAADC driver.
 *
 *          Calibration in a non-blocking manner is triggered by @p nrfx_saadc_offset_calibrate. Then, upon receiving @p NRFX_SAADC_EVT_CALIBRATEDONE
 *          event in @ref saadc_handler() sampling is initiated by calling @p nrfx_saadc_mode_trigger() function.
 *          Consecutive sample tasks are triggered by the internal timer at the sample rate specified in  @ref SAADC_SAMPLE_FREQUENCY symbol.
 *
 *          The example features GPPI channel configured to test the functionality of SAADC. The endpoints are setup up in a way
 *          that @p NRF_SAADC_EVENT_RESULTDONE event is connected with the GPIOTE task that toggles the @ref OUT_GPIO_PIN pin.
 *
 *          Please note that the internal timer can only be used in the non-blocking mode with only a single input channel enabled.
 */

/** @brief Symbol specifying GPIOTE instance to be used. */
#define GPIOTE_INST_IDX 0

/** @brief Symbol specifying GPIO pin used to test the functionality of SAADC. */
//#define OUT_GPIO_PIN LOOPBACK_PIN_1B

/** @brief Acquisition time [us] for source resistance <= 800 kOhm (see SAADC electrical specification). */
#define ACQ_TIME_10K 40UL

/** @brief Conversion time [us] (see SAADC electrical specification). */
#define CONV_TIME 2UL

/** @brief Internal timer frequency [Hz] is derived from PCLK16M (see SAMPLERATE register in SAADC). */
#define INTERNAL_TIMER_FREQ 16000000UL

/** @brief SAADC sample frequency for the continuous sampling. */
#define SAADC_SAMPLE_FREQUENCY 8000UL // 8000UL

/** @brief Internal timer capture and compare value. */
#define INTERNAL_TIMER_CC (INTERNAL_TIMER_FREQ / SAADC_SAMPLE_FREQUENCY)

/**
 * @brief Symbol specifying the number of sample buffers ( @ref m_sample_buffers ).
 * Two buffers are required for performing double-buffered conversions.
 */
#define BUFFER_COUNT 2U

/** @brief Symbol specifying the size of singular sample buffer ( @ref m_sample_buffers ). */
//#define BUFFER_SIZE 5UL //means 40Hz

/** @brief SAADC channel configuration structure for single channel use. */
static const nrfx_saadc_channel_t m_single_channel = SAADC_CHANNEL_SE_ACQ_40US(NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_AIN1, 0);

/** @brief Samples buffer to store values from a single channel ( @ref m_single_channel). */

#define MESSAGE_NUM 5U

static uint16_t m_sample_buffers[BUFFER_COUNT][MESSAGE_NUM];


/** For continuous sampling the sample rate SAADC_SAMPLE_FREQUENCY should fulfill the following criteria (see SAADC Continuous sampling). */
NRFX_STATIC_ASSERT(SAADC_SAMPLE_FREQUENCY <= (1000000UL / (ACQ_TIME_10K + CONV_TIME)));

/** Possible range value of a CC field in the SAMPLERATE register (SAADC) is 80-2047. */
NRFX_STATIC_ASSERT((INTERNAL_TIMER_CC >= 80UL ) && (INTERNAL_TIMER_CC <= 2047UL));

/**
 * @brief Function for handling SAADC driver events.
 *
 * @param[in] p_event Pointer to an SAADC driver event.
 */


int16_t result_items[MESSAGE_NUM] = {0};

static bool continuereading = false;

void adc_stop() {
    continuereading = false;
    
    k_mutex_unlock(&saadc_mutex);  
    nrfx_saadc_uninit();      
}

static void saadc_handler(nrfx_saadc_evt_t const * p_event)
{
    nrfx_err_t status;
    (void)status;

    static uint16_t buffer_index = 1;

    
    uint16_t samples_number;

    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_CALIBRATEDONE:
            LOG_INF("SAADC event: CALIBRATEDONE");

            status = nrfx_saadc_mode_trigger();
            NRFX_ASSERT(status == NRFX_SUCCESS);
            break;

        case NRFX_SAADC_EVT_READY:
            LOG_INF("SAADC event: READY");
            break;

        case NRFX_SAADC_EVT_BUF_REQ:
            LOG_INF("SAADC event: BUF_REQ");

            if (continuereading)
            {
                /* Next available buffer must be set on the NRFX_SAADC_EVT_BUF_REQ event to achieve the continuous conversion. */
                status = nrfx_saadc_buffer_set(m_sample_buffers[buffer_index++], MESSAGE_NUM);
                NRFX_ASSERT(status == NRFX_SUCCESS);
                buffer_index = buffer_index % BUFFER_COUNT;
            }
            break;

        case NRFX_SAADC_EVT_DONE:
            samples_number = p_event->data.done.size; 
            
            for (int16_t i = 0; i < samples_number; i++)
            {                
                result_items[i] = NRFX_SAADC_SAMPLE_GET(RESOLUTION, p_event->data.done.p_buffer, i);
            } 
            SendBufferd(result_items, samples_number); //times 2 bc 16bit
            
            LOG_INF("first item:%d", result_items[0]);
            break;

        case NRFX_SAADC_EVT_FINISHED:
            LOG_INF("SAADC event: FINISHED");
                           
            break;

        default:
            break;
    }
}

#define NRFX_SAADC_BB_CONFIG                                           \
{                                                                               \
    .oversampling      = NRF_SAADC_OVERSAMPLE_256X,                         \
    .burst             = NRF_SAADC_BURST_DISABLED,                              \
    .internal_timer_cc = INTERNAL_TIMER_CC,                                                     \
    .start_on_end      = true,\
}

 
void adc_calibrate_and_start() {
    LOG_INF("adc_calibrate_and_start");
    
    if(k_mutex_lock(&saadc_mutex, K_MSEC(10)) != 0) {
        
        LOG_INF("mutex busy");
        return;
    };    
    //enable_saadc_irq_runtime();

    LOG_INF("mutex aquired");

    continuereading = true;
    nrfx_err_t status;
    status = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    //status = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    status = nrfx_saadc_channel_config(&m_single_channel);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_BB_CONFIG;    
    

    uint32_t channel_mask = nrfx_saadc_channels_configured_get();
    status = nrfx_saadc_advanced_mode_set(channel_mask,
                                          NRF_SAADC_RESOLUTION_12BIT,
                                          &adv_config,
                                          saadc_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    status = nrfx_saadc_buffer_set(m_sample_buffers[0], MESSAGE_NUM);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    
    status = nrfx_saadc_offset_calibrate(saadc_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);    
    //powerup_iopin();
}

/**
 * @brief Function for application main entry.

 */
static int configureSAADC(void)
{          
    
    k_mutex_init(&saadc_mutex);
    // IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_GPIOTE_INST_GET(GPIOTE_INST_IDX)), IRQ_PRIO_LOWEST,
    //             NRFX_GPIOTE_INST_HANDLER_GET(GPIOTE_INST_IDX), 0, 0);
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, 0, 0);
    
    LOG_INF("mutex declared");
    
    // int err = 0;
    // err |= gpio_pin_configure(gpio1_dev, 11, GPIO_OUTPUT); 
	// if(err != 0) {
	// 	LOG_ERR("error configure output pin %d", err);
	// }		

    // nrfx_gpiote_t const gpiote_inst = NRFX_GPIOTE_INSTANCE(GPIOTE_INST_IDX);
    // status = nrfx_gpiote_init(&gpiote_inst, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);`
    // NRFX_ASSERT(status == NRFX_SUCCESS);
    // LOG_INF("GPIOTE status: %s", nrfx_gpiote_init_check(&gpiote_inst) ? "initialized" : "not initialized");

    // pin_on_event_toggle_setup(&gpiote_inst, OUT_GPIO_PIN,
    //                            nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_RESULTDONE));    
    return 0;
}

/** @} */

SYS_INIT(configureSAADC, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
