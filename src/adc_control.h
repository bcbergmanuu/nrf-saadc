#ifndef ADC_CONTROL__
#define ADC_CONTROL__

#include <zephyr/kernel.h>

/**
 * @brief SAADC channel configuration for the single-ended mode with 3 us sample acquisition time.
 *        The 3 us acquisition time will work correctly when the source resistance of @p _pin_p input
 *        analog pin is less than 10 kOhm.
 *
 * This configuration sets up single-ended SAADC channel with the following options:
 * - resistor ladder disabled
 * - gain: 1/6
 * - reference voltage: internal 0.6 V
 * - sample acquisition time: 3 us
 * - burst disabled
 *
 * @param[in] _pin_p Positive input analog pin.
 * @param[in] _index Channel index.
 *
 * @sa nrfx_saadc_channel_t
 */
#define SAADC_CHANNEL_SE_ACQ_40US(_pin_p, _pin_n, _index)        \
{                                                       \
    .channel_config =                                   \
    {                                                   \
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      \
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      \
        .gain       = NRF_SAADC_GAIN1,                \
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,     \
        .acq_time   = NRF_SAADC_ACQTIME_40US,            \
        .mode       = NRF_SAADC_MODE_DIFFERENTIAL,      \
        .burst      = NRF_SAADC_BURST_DISABLED,         \
    },                                                  \
    .pin_p          = (nrf_saadc_input_t)_pin_p,        \
    .pin_n          = (nrf_saadc_input_t)_pin_n,        \
    .channel_index  = _index,                           \
}


extern void calibrate_and_start();
extern void adc_stop();

#endif