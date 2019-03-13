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
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define DEVICE_NAME                     "HelloWorld"                            /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                0                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */


static void advertising_start();
static void advertising_init_ext(void);
static void restarter_timer_handler(void * p_context);
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

    //NRF_LOG_INFO("Random number i is %d", dataNum);
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

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            //NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            /*NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
            */
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

// Handler for advertisement restart
static void restarter_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    // STOP
    sd_ble_gap_adv_stop(NULL);
    // UPDATE
    ble_tag_test();
    advertising_init_ext();
    // RESUME
    advertising_start(false);
}

// Handler for reversing the button state
static void btn_state_handler(void * p_context)
{   
    UNUSED_PARAMETER(p_context);
    btn_state_update(0);
    LEDS_INVERT(BSP_LED_2_MASK);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create a timer for restarting the advertisement
    err_code = app_timer_create(&restarter_timer, APP_TIMER_MODE_REPEATED, restarter_timer_handler);
    //NRF_LOG_INFO("app_timer_create restart_timer=%s\r\n", nrf_strerror_get(err_code));
    APP_ERROR_CHECK(err_code);

    // Create a timer for handling button state, will reverse the btn_state back to normal
    err_code = app_timer_create(&btn_state_timer, APP_TIMER_MODE_SINGLE_SHOT, btn_state_handler);
    //NRF_LOG_INFO("app_timer_create btn_state_timer=%s\r\n", nrf_strerror_get(err_code));
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);


       /*err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    ble_gap_privacy_params_t prvt_conf;
    memset(&prvt_conf, 0, sizeof(prvt_conf));
    prvt_conf.privacy_mode = BLE_GAP_PRIVACY_MODE_DEVICE_PRIVACY;
    prvt_conf.private_addr_type = BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE ;
    prvt_conf.private_addr_cycle_s = 0;
    err_code = sd_ble_gap_privacy_set(&prvt_conf);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //printf("fast adv=%s\r\n", nrf_strerror_get(err_code));
            APP_ERROR_CHECK(err_code);
            break;

        /*case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
            */

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            //NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

       /* case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;*/
/*
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
*/
        default:
            // No implementation needed.
            break;
    }
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

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    //NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    //printf("erase bonds=%s\r\n", nrf_strerror_get(err_code));
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
            //NRF_LOG_INFO("Button 0 pushed.");
            LEDS_INVERT(BSP_LED_2_MASK);
            // Change button to "pressed"
            btn_state_update(1);
            sd_ble_gap_adv_stop(NULL);// Stop the advertising first
            advertising_init_ext();// Refresh the advertising data, but not yet broadcasted
            advertising_start(false);// Resume the advertising action

            // Timer for inverting button state
            err_code = app_timer_start(btn_state_timer, APP_TIMER_TICKS(5000), NULL);
            //NRF_LOG_INFO("app_timer_start btn_state=%s\r\n", nrf_strerror_get(err_code));
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
    err_code = NRF_SUCCESS;
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
/*static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;  // Struct containing advertising parameters
    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&init, 0, sizeof(init));

    ble_advdata_manuf_data_t                  manuf_data; //Variable to hold manufacturer specific data
    uint8_t data[]                            = "1000760683620"; //Our data to advertise
    manuf_data.company_identifier             =  0x0118; //Nordics company ID
    manuf_data.data.p_data                    = data;
    manuf_data.data.size                      = sizeof(data);
    init.advdata.p_manuf_specific_data = &manuf_data;

  
    init.advdata.name_type = BLE_ADVDATA_NO_NAME; // Use a shortened name
    //init.advdata.short_name_len = 6; // Advertise only first 6 letters of name
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    //init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    //init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}*/


static void advertising_init_ext(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;  // Struct containing advertising parameters
    // Build advertising data struct to pass into @ref ble_advertising_init.
    int8_t tx_power_level = -40;// -40, -20, -16, -12, -8, -4, 0, 4 (dbm)
    memset(&init, 0, sizeof(init));

    
    ble_advdata_manuf_data_t                  manuf_data; //Variable to hold manufacturer specific data
    manuf_data.company_identifier             = 0x0059; //Nordics company ID
    // Build data array
    data_ary_append();
    manuf_data.data.p_data                    = FIN_DATA;
    manuf_data.data.size                      = sizeof(FIN_DATA);
    init.advdata.p_tx_power_level             = &tx_power_level;
    init.advdata.p_manuf_specific_data = &manuf_data;

    init.advdata.name_type = BLE_ADVDATA_NO_NAME; // Use a shortened name
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    //NRF_LOG_INFO("advertising init ext=%s\r\n", nrf_strerror_get(err_code));
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}



/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init()
{
    ret_code_t err_code;
    bsp_event_t startup_event;
    
    LEDS_CONFIGURE(LEDS_MASK);

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    //APP_ERROR_CHECK(err_code);

    //*p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
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


/**@brief Function for starting advertising.
 */
static void advertising_start()
{
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    //printf("advertising start=%s\r\n", nrf_strerror_get(err_code));
    APP_ERROR_CHECK(err_code);
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
    addr.addr[4]       = 0x00;
    addr.addr[5]       = 0xdf; // 2MSB must be set 11
    err_code = sd_ble_gap_addr_set(&addr);
    APP_ERROR_CHECK(err_code);

    // Use sd_ble_gap_addr_get() for NRF_SD_BLE_API_VERSION=3
    err_code = sd_ble_gap_addr_get(&addr);
    //NRF_LOG_INFO("modify mac address=%s\r\n", nrf_strerror_get(err_code));
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
    //printf("app_timer_start restart_timer=%s\r\n", nrf_strerror_get(err_code));
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for application main entry.
 */
int main(void)
{
    //bool erase_bonds;

    // Initialize.
    //log_init();
    timers_init();
    buttons_leds_init();
    power_management_init();
    ble_stack_init();
    //gap_params_init();
    gatt_init();
    ble_mac_addr_modify(true);
    ble_tag_test();
    advertising_init_ext();
    services_init();
    //peer_manager_init();

    // Start execution.
    //NRF_LOG_INFO("Ble tutorial started.");
    //printf("Started~");
    application_timers_start();
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}