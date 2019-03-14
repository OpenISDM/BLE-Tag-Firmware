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


#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(300, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

/*Customized Data Generating*/
APP_TIMER_DEF(restarter_timer);// Timer for restarting advertisement
APP_TIMER_DEF(btn_state_timer);// Timer for reversing the button press state
uint8_t newdata[3];// register

uint8_t BPS_DIA[3];// Blood pressure - diastolic(mmHg), normally less then 80
uint8_t BPS_SYS[3];// Blood pressure - systolic(mmHg), normally less then 120
uint8_t PULSE[3];// Pulse(bpm), average 73
uint8_t BODY_TEMP[3];// Body temperature(*10 degree celsius), normally 365
uint8_t BTN_PRS[1];// Button press indication, idle=0, pressed=1
uint8_t FIN_DATA[13];// Overall data packet ready to send

// Generates random 3-digit number for testing purpose
void generateRandomNumber(void)
{
    // from 999 to 100, set a=999, b=100
    // rand() % (a-b+1) + b
    int dataNum = rand() % 900 + 100;

    uint8_t data[3];
    
    data[0] = dataNum/100;
    data[1] = (dataNum/10)%10;
    data[2] = dataNum%10;

    memcpy(newdata, data, sizeof(newdata));
}

void bps_dia_update(int newValue)
{
    uint8_t data[3];
    data[0] = newValue/100;
    data[1] = (newValue/10)%10;
    data[2] = newValue%10;
    
    memcpy(BPS_DIA, data, sizeof(BPS_DIA));
}

void bps_sys_update(int newValue)
{
    uint8_t data[3];
    data[0] = newValue/100;
    data[1] = (newValue/10)%10;
    data[2] = newValue%10;
    
    memcpy(BPS_SYS, data, sizeof(BPS_SYS));
}

void pulse_update(int newValue)
{
    uint8_t data[3];
    data[0] = newValue/100;
    data[1] = (newValue/10)%10;
    data[2] = newValue%10;
    
    memcpy(PULSE, data, sizeof(PULSE));
}

void body_temp_update(int newValue)
{
    uint8_t data[3];
    data[0] = newValue/100;
    data[1] = (newValue/10)%10;
    data[2] = newValue%10;
    
    memcpy(BODY_TEMP, data, sizeof(BODY_TEMP));
}


void btn_state_update(int newValue)
{
    uint8_t data[1];
    data[0] = newValue;
    
    memcpy(BTN_PRS, data, sizeof(BTN_PRS));
}

void data_ary_append(void)
{
    uint8_t* total = malloc(13 * sizeof(uint8_t));

    memcpy(total,       BPS_DIA,    3 * sizeof(BPS_DIA));
    memcpy(total + 3,   BPS_SYS,    3 * sizeof(BPS_SYS));
    memcpy(total + 6,   PULSE,      3 * sizeof(PULSE));
    memcpy(total + 9,   BODY_TEMP,  3 * sizeof(BODY_TEMP));
    memcpy(total + 12,  BTN_PRS,    1 * sizeof(BTN_PRS));

    memcpy(FIN_DATA, total, sizeof(FIN_DATA));
    free(total);

    // DEBUG CONSOLE
    /*
    NRF_LOG_INFO("BPS_DIA=%d%d%d\r",    BPS_DIA[0], BPS_DIA[1], BPS_DIA[2]);
    NRF_LOG_INFO("BPS_SYS=%d%d%d\r",    BPS_SYS[0], BPS_SYS[1], BPS_SYS[2]);
    NRF_LOG_INFO("PULSE=%d%d%d\r",      PULSE[0], PULSE[1], PULSE[2]);
    NRF_LOG_INFO("BODY_TEMP=%d%d%d\r",  BODY_TEMP[0], BODY_TEMP[1], BODY_TEMP[2]);
    NRF_LOG_INFO("BTN_PRS=%d\r",        BTN_PRS[0]);
    */
}

void ble_tag_test(void)
{
    // Generate random numbers for demo
    int random_bps_dia = rand() % 11 + 70;// 80~70
    int random_bps_sys = rand() % 21 + 100;// 120~100
    int random_pulse = rand() % 4 + 70;// 73~70

    // update bood pressure values
    bps_dia_update(random_bps_dia);
    bps_sys_update(random_bps_sys);
    // update pulse value
    pulse_update(random_pulse);
    // update body temperature value
    body_temp_update(365);
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
    int8_t tx_power_level = -40;// -40, -20, -16, -12, -8, -4, 0, 4 (dbm)
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    data_ary_append(); // Build the data array
    manuf_specific_data.data.p_data = FIN_DATA;
    manuf_specific_data.data.size   = sizeof(FIN_DATA);

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
            LEDS_INVERT(BSP_LED_2_MASK);
            // Change button to "pressed"
            btn_state_update(1);
            sd_ble_gap_adv_stop(NULL);// Stop the advertising first
            advertising_init();// Refresh the advertising data, but not yet broadcasted
            advertising_start();// Resume the advertising action

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

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}

// Handler for advertisement restart
static void restarter_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    // STOP
    sd_ble_gap_adv_stop(NULL);
    // UPDATE
    ble_tag_test();
    advertising_init();
    // RESUME
    advertising_start();
}

// Handler for reversing the button state
static void btn_state_handler(void * p_context)
{   
    UNUSED_PARAMETER(p_context);
    btn_state_update(0);
    LEDS_INVERT(BSP_LED_2_MASK);
}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create a timer for restarting the advertisement
    err_code = app_timer_create(&restarter_timer, APP_TIMER_MODE_REPEATED, restarter_timer_handler);
    APP_ERROR_CHECK(err_code);

    // Create a timer for handling button state, will reverse the btn_state back to normal
    err_code = app_timer_create(&btn_state_timer, APP_TIMER_MODE_SINGLE_SHOT, btn_state_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
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


void ble_mac_addr_modify(bool change)
{
    ret_code_t err_code;
    // Log our BLE address (6 bytes).
    ble_gap_addr_t addr;
   
    addr.addr_type     = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    // The address will display as [5]:[4]:[3]:[2]:[1]:[0]
    addr.addr[0]       = 0xff;
    addr.addr[1]       = 0xff;
    addr.addr[2]       = 0xff;
    addr.addr[3]       = 0xff;
    addr.addr[4]       = 0xcc;
    addr.addr[5]       = 0xdf; // 2MSB must be set 11
    err_code = sd_ble_gap_addr_set(&addr);
    APP_ERROR_CHECK(err_code);

    // Use sd_ble_gap_addr_get() for NRF_SD_BLE_API_VERSION=3
    err_code = sd_ble_gap_addr_get(&addr);
    APP_ERROR_CHECK(err_code);

   // NRF_LOG_INFO("%02X:%02X:%02X:%02X:%02X:%02X",
   //                  addr.addr[5], addr.addr[4],
   //                  addr.addr[3], addr.addr[2],
   //                  addr.addr[1], addr.addr[0]);
}

// Starts timers
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Timer for refreshing the packet data
    err_code = app_timer_start(restarter_timer, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
}


static void set_tx_power()
{
    ret_code_t err_code;

    // Accepted values are -40, -20, -16, -12, -8, -4, 0, 4 (dbm)
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, NULL, -20);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timers_init();
    leds_init();
    power_management_init();
    ble_stack_init();
    ble_mac_addr_modify(true);
    ble_tag_test();
    advertising_init();
    set_tx_power();

    // Start execution.
    NRF_LOG_INFO("Beacon example started.");
    application_timers_start();
    advertising_start();

    // Enter main loop.
    for (;; )
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
