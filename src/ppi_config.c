#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

   /* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */
/*
 * See README.md for a description of the application. 
 */ 
LOG_MODULE_REGISTER(PPI_MODULE, CONFIG_LOG_DEFAULT_LEVEL);


void ppi_start() {
    NRF_TIMER1->TASKS_START = 1;
    LOG_INF("ppi started");
}

void ppi_stop() {
    NRF_TIMER1->TASKS_STOP = 1;
    LOG_INF("ppi stopped");
}
/**
 * @brief Function for application main entry.
 */
int ppi_init(void)
{   
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        LOG_INF("waiting HFCLK start");
        k_sleep(K_MSEC(10));        
    }
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    
    // Workaround for PAN item 11, nRF51822-PAN v2.0.pdf. Not needed on second revision chips. 
    //NRF_POWER->TASKS_CONSTLAT = 1;
    
    NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                            GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                            4 << GPIOTE_CONFIG_PSEL_Pos | //pin 0. 4
                            GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;
                            
    NRF_TIMER1->PRESCALER = 0;
    // Adjust the output frequency by adjusting the CC. 
    // Due to PAN item 33, you can't have this at 1 for first revision chips, and will hence be limited to 4 MHz. 
    NRF_TIMER1->CC[0] = 1;
    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
    //NRF_TIMER1->TASKS_START = 1;
    
    NRF_PPI->CH[0].EEP = (uint32_t) &NRF_TIMER1->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[0];
    
    NRF_PPI->CHENSET = PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Pos;

    ppi_stop();
    return 0;
}



SYS_INIT(ppi_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
/** @} */