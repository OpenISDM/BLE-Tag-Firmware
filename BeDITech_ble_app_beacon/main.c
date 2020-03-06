/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

volatile int16_t result = 0;
volatile float precise_result = 0;

// The default value of APP_TIMER_CONFIG_RTC_FREQUENCY is 0 which leads the 
// maximum app timer to be < 512 seconds. The reasons is that Nordic register
// uses 24-bit and each second contains 32768 (=2^15) ticks, so the app timer
// supports only 2^(24-15)=2^9=512 seconds. 
// 
// To support longer app timer, we should adjust the app timer prescaler. It 
// was called APP_TIMER_PRESCALER in previous versions, and it is called 
// APP_TIMER_CONFIG_RTC_FREQUENCY in current version. For details, please 
// refer to the definition of APP_TIMER_TICKS() function defined in app_timer.h
//
#define APP_TIMER_CONFIG_RTC_FREQUENCY 0

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

/*Customized Data Generating*/
APP_TIMER_DEF(btn_state_timer);// Timer for reversing the button press state
APP_TIMER_DEF(battery_measure_timer);

uint8_t BTN_PRS;// Button press indication, idle=0, pressed=1
uint8_t FINAL_DATA[4];// Overall data packet ready to send
uint8_t BATT_LVL;
uint8_t NEW_BATT_LVL;

bool bAdvertiseEnabled = false;

static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);

}

void btn_state_update(int newValue)
{
    BTN_PRS = (uint8_t)newValue;
}

void data_ary_append(void)
{
    uint8_t total[4];
    memset(total, 0, sizeof(total));

    // Use 0x1478 as our identifier
    uint8_t IDENTIFIER_ADV[2];
    memset(IDENTIFIER_ADV, 0, sizeof(IDENTIFIER_ADV));

    // 1478 (0x05C6) 
    IDENTIFIER_ADV[0] = (uint8_t)5;
    IDENTIFIER_ADV[1] = (uint8_t)198;
    
    memcpy(total, IDENTIFIER_ADV, sizeof(IDENTIFIER_ADV));
 
    memcpy(total + 2,   &BTN_PRS,     1 * sizeof(BTN_PRS));
    memcpy(total + 3,   &BATT_LVL,    1 * sizeof(BATT_LVL));

    memcpy(FINAL_DATA, total, sizeof(FINAL_DATA));
    
    // DEBUG CONSOLE
    /*
    NRF_LOG_INFO("BPS_DIA=%d%d%d\r",    BPS_DIA[0], BPS_DIA[1], BPS_DIA[2]);
    NRF_LOG_INFO("BPS_SYS=%d%d%d\r",    BPS_SYS[0], BPS_SYS[1], BPS_SYS[2]);
    NRF_LOG_INFO("PULSE=%d%d%d\r",      PULSE[0], PULSE[1], PULSE[2]);
    NRF_LOG_INFO("BODY_TEMP=%d%d%d\r",  BODY_TEMP[0], BODY_TEMP[1], BODY_TEMP[2]);
    NRF_LOG_INFO("BTN_PRS=%d\r",        BTN_PRS[0]);
    */
}



/*Customized Data Generating*/

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    data_ary_append(); // Build the data array
    manuf_specific_data.data.p_data = FINAL_DATA;
    manuf_specific_data.data.size   = sizeof(FINAL_DATA);

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, NULL, 0);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);

    bAdvertiseEnabled = true;
}


static void renew_advertising(void)
{
    if(bAdvertiseEnabled){
        // STOP
        sd_ble_gap_adv_stop(NULL);
        bAdvertiseEnabled = false;
    }    
    // UPDATE
    advertising_init();
    // RESUME
    advertising_start();  
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        
        case BSP_EVENT_KEY_0:
            //LEDS_INVERT(BSP_LED_1);
            // Change button to "pressed"
            btn_state_update(1);
            
            renew_advertising ();

            // Timer for inverting button state
            err_code = app_timer_start(btn_state_timer, APP_TIMER_TICKS(5000), NULL);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
    err_code = NRF_SUCCESS;
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

   // NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void bsps_init(void)
{
    //ret_code_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    ret_code_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}


// Handler for reversing the button state
static void btn_state_handler(void * p_context)
{   
    UNUSED_PARAMETER(p_context);
    btn_state_update(0);

   // LEDS_INVERT(BSP_LED_1);

    renew_advertising();
}

static void battery_measure_voltage(){

    /* We commented out this block, since it will causes button state to be recovered 
       immediately.

    // Start HFCLK from crystal oscillator, this will give the SAADC higher accuracy 
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    */

    // Configure SAADC singled-ended channel, Internal reference (0.6V) and 1/6 gain.
    NRF_SAADC->CH[0].CONFIG = 
    (SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos) |
    (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos) |
    (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
    (SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos) |
    (SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos) |
    (SAADC_CH_CONFIG_TACQ_3us        << SAADC_CH_CONFIG_TACQ_Pos);

    NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDD << SAADC_CH_PSELP_PSELP_Pos;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

    // Configure the SAADC resolution.
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos;

    // Configure result to be put in RAM at the location of "result" variable.
    NRF_SAADC->RESULT.MAXCNT = 1;
    NRF_SAADC->RESULT.PTR = (uint32_t)&result;

    // No automatic sampling, will trigger with TASKS_SAMPLE
    NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

    // Enable SAADC (would capture analog pins if they were used in CH[0].PSELP)
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;



    // Calibrate the SAADC (only needs to be done once in a while)
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    
    while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0);
    
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    
    while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy <<SAADC_STATUS_STATUS_Pos));

    // Start the SAADC and wait for the started event.
    NRF_SAADC->TASKS_START = 1;
    
    while (NRF_SAADC->EVENTS_STARTED == 0);
    
    NRF_SAADC->EVENTS_STARTED = 0;

    // Do a SAADC sample, will put the result in the configured RAM buffer.
    NRF_SAADC->TASKS_SAMPLE = 1;
    
    while (NRF_SAADC->EVENTS_END == 0);
    
    NRF_SAADC->EVENTS_END = 0;

    precise_result = (float)result / 4551.1f;
    NEW_BATT_LVL = ((uint8_t)(precise_result*10));
    
    // Stop the SAADC, since it is not used anymore.
    NRF_SAADC->TASKS_STOP = 1;
    
    while (NRF_SAADC->EVENTS_STOPPED == 0);

    NRF_SAADC->EVENTS_STOPPED = 0;  
   
    NRF_SAADC->ENABLE = 0;
    NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);

    NVIC_DisableIRQ( SAADC_IRQn );
    NVIC_ClearPendingIRQ(SAADC_IRQn);
      
}

static void battery_measure_handler(){

    battery_measure_voltage();

    if(BATT_LVL != NEW_BATT_LVL){

        BATT_LVL = NEW_BATT_LVL; 
        renew_advertising();

    }

}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create a timer for handling button state, will reverse the btn_state back to normal
    err_code = app_timer_create(&btn_state_timer, APP_TIMER_MODE_SINGLE_SHOT, btn_state_handler);
    APP_ERROR_CHECK(err_code);
 
    err_code = app_timer_create(&battery_measure_timer, APP_TIMER_MODE_REPEATED, battery_measure_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(battery_measure_timer, APP_TIMER_TICKS(180000), NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;

    // Enable DCDC to test for better power management
    //err_code = sd_power_mode_set(NRF_POWER_DCDC_ENABLE);
    NRF_POWER->TASKS_LOWPWR = 1;
    NRF_POWER->DCDCEN = 1;
    //NRF_LOG_INFO("pwr_mngmnt =%s\r\n", nrf_strerror_get(err_code));
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

//
void ble_mac_addr_modify(bool change)
{
    ret_code_t err_code;
    // Log our BLE address (6 bytes).
    ble_gap_addr_t addr;
   
    addr.addr_type     = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    // The address will display as [5]:[4]:[3]:[2]:[1]:[0]
    addr.addr[0]       = 0x55;
    addr.addr[1]       = 0x00;
    addr.addr[2]       = 0x01;
    addr.addr[3]       = 0x00;
    addr.addr[4]       = 0xaf;
    addr.addr[5]       = 0xc1;
    err_code = sd_ble_gap_addr_set(&addr);
    APP_ERROR_CHECK(err_code);

    // Use sd_ble_gap_addr_get() for NRF_SD_BLE_API_VERSION=3
    err_code = sd_ble_gap_addr_get(&addr);
    APP_ERROR_CHECK(err_code);

    /*NRF_LOG_INFO("%02X:%02X:%02X:%02X:%02X:%02X",
                     addr.addr[5], addr.addr[4],
                     addr.addr[3], addr.addr[2],
                     addr.addr[1], addr.addr[0]);
    */
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    lfclk_request();
    // Initialize.
    //log_init();
    timers_init();
    
    battery_measure_voltage();
    BATT_LVL = NEW_BATT_LVL;

    bsps_init();
    power_management_init();
    ble_stack_init();

    ble_mac_addr_modify(true);
    
    renew_advertising();

    // Enter main loop.
    for (;; )
    {
        idle_state_handle();   
    }
}

