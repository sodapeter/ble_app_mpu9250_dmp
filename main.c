/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *  Semiconductor ASA integrated circuit in a product or a software update for
 *  such product, must reproduce the above copyright notice, this list of
 *  conditions and the following disclaimer in the documentation and/or other
 *  materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *  Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *  engineered, decompiled, modified and/or disassembled.
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
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief  UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */
 
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf.h"

#include "app_timer.h"
#include "app_util_platform.h"
#include "nrf_nvic.h"
#include "bsp_btn_ble.h"

#include "ble_mpu_dmp.h"
#include "twi_master.h"
#include "mpu_dmp.h"


#define APP_BLE_CONN_CFG_TAG      1                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED     BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                   "BLUESHELL"                 /**< Name of device. Will be included in the advertising data. */
#define MPU_SERVICE_UUID_TYPE         BLE_UUID_TYPE_VENDOR_BEGIN          /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO         3                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define BLE_DMP_OBSERVER_PRIO         APP_BLE_OBSERVER_PRIO
#define APP_ADV_INTERVAL              64                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS    180                     /**< The advertising timeout (in units of seconds). */

#ifdef USE_DMP
#define MIN_CONN_INTERVAL             MSEC_TO_UNITS(25, UNIT_1_25_MS)       /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL             MSEC_TO_UNITS(50, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#else
#define MIN_CONN_INTERVAL             MSEC_TO_UNITS(25, UNIT_1_25_MS)       /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL             MSEC_TO_UNITS(100, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#endif

#define SLAVE_LATENCY                 0                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT              MSEC_TO_UNITS(4000, UNIT_10_MS)       /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)             /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)            /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT  3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define L2CAP_HDR_LEN                 4                       /**< L2CAP header length. */

#define DEAD_BEEF             0xDEADBEEF                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

BLE_MPU_DEF(m_dmp);                                 /**< BLE MPU service instance. */
NRF_BLE_GATT_DEF(m_gatt);                               /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                         /**< Advertising module instance. */

static uint16_t   m_conn_handle      = BLE_CONN_HANDLE_INVALID;         /**< Handle of the current connection. */
static ble_uuid_t m_adv_uuids[]      =                      /**< Universally unique service identifier. */
{
  {BLE_UUID_MPU_SERVICE, MPU_SERVICE_UUID_TYPE}
};

static volatile bool triggerSensorPolling = false;
APP_TIMER_DEF(m_led_timer);                 // LED Timer


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *      how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num  Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
static void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**
 * @brief Function to be called in timer interrupt.
 *
 * @param[in] p_context     General purpose pointer (unused).
 */
static void led_timer_handler(void *p_context)
{
	triggerSensorPolling = true;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *      the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
  uint32_t        err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
                      (const uint8_t *) DEVICE_NAME,
                      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency   = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *      it to the UART module.
 *
 * @param[in] p_nus  Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void mpu_data_handler(ble_mpu_evt_t * p_evt)
{
  #ifndef USE_DMP
  if (p_evt->type == BLE_MPU_EVT_RX_DATA) {
  
    uint8_t fsr_data = (uint8_t)p_evt->params.rx_data.p_data[0];  
    
    switch (fsr_data){
    case 1:
      {
        mpu_set_accel_fsr(2);
        mpu_set_gyro_fsr(250);
      }
      break;
      
    case 2:
      {
        mpu_set_accel_fsr(4);
        mpu_set_gyro_fsr(500);
      }
      break;
    
    case 3:
      {
        mpu_set_accel_fsr(8);
        mpu_set_gyro_fsr(1000);
      }
      break;
    
    case 4:
      {
        mpu_set_accel_fsr(16);
        mpu_set_gyro_fsr(2000);
      }
      break;
      
    default:
      break;
    }
  }
  #endif
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
  uint32_t     err_code;
  ble_mpu_init_t mpu_init;

  memset(&mpu_init, 0, sizeof(mpu_init));

  mpu_init.data_handler = mpu_data_handler;

  err_code = ble_mpu_init(&m_dmp, &mpu_init);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *      which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *     the disconnect_on_fail config parameter, but instead we use the event handler
 *     mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
  uint32_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
  {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
  uint32_t         err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params          = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle  = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail       = false;
  cp_init.evt_handler          = on_conn_params_evt;
  cp_init.error_handler          = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
  //uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  //APP_ERROR_CHECK(err_code);

  // Prepare wakeup buttons.
  uint32_t err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
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
  uint32_t err_code;

  switch (ble_adv_evt)
  {
    case BLE_ADV_EVT_FAST:
      //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
      //APP_ERROR_CHECK(err_code);
      break;
    case BLE_ADV_EVT_IDLE:
      err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
      APP_ERROR_CHECK(err_code);
      //sleep_mode_enter();
      break;
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
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      {
        //NRF_LOG_INFO("Connected");
        //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        //APP_ERROR_CHECK(err_code);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      
        #ifdef USE_PHY
          ble_gap_phys_t gap_phys_settings;
          gap_phys_settings.tx_phys = BLE_GAP_PHY_1MBPS;
          gap_phys_settings.rx_phys = BLE_GAP_PHY_1MBPS;

          #ifdef S140
            sd_ble_gap_phy_request(m_conn_handle, &gap_phys_settings);
          #else
            sd_ble_gap_phy_update(m_conn_handle, &gap_phys_settings);
          #endif
        #endif
      }
      break;

    case BLE_GAP_EVT_DISCONNECTED:
        //NRF_LOG_INFO("Disconnected");
        // LED indication will be changed when advertising starts.
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
      break;

#ifndef S140
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
      //NRF_LOG_DEBUG("PHY update request.");
      ble_gap_phys_t const phys =
      {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
    } break;
#endif

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      // Pairing not supported
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
      break;
#if !defined (S112)
     case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    {
      ble_gap_data_length_params_t dl_params;

      // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
      memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
      err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
      APP_ERROR_CHECK(err_code);
    } break;
#endif //!defined (S112)
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      // No system attributes have been stored.
      err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_EVT_USER_MEM_REQUEST:
      err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
      ble_gatts_evt_rw_authorize_request_t  req;
      ble_gatts_rw_authorize_reply_params_t auth_reply;

      req = p_ble_evt->evt.gatts_evt.params.authorize_request;

      if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
      {
        if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)   ||
          (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
          (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
        {
          if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
          {
            auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
          }
          else
          {
            auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
          }
          auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
          err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                 &auth_reply);
          APP_ERROR_CHECK(err_code);
        }
      }
    } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

    default:
      // No implementation needed.
      break;
  }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
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


/**@brief Function for handling events from the GATT library. */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
  {
    m_dmp.max_payload_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    //NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_dmp.max_payload_len, m_dmp.max_payload_len);
  }
  //NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
  //        p_gatt->att_mtu_desired_central,
  //        p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
static void gatt_init(void)
{
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
  uint32_t err_code;

  switch (event)
  {
    case BSP_EVENT_SLEEP:
      sleep_mode_enter();
      break;

    case BSP_EVENT_DISCONNECT:
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_ERROR_INVALID_STATE)
      {
        APP_ERROR_CHECK(err_code);
      }
      break;

    case BSP_EVENT_WHITELIST_OFF:
      if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
      {
        err_code = ble_advertising_restart_without_whitelist(&m_advertising);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
          APP_ERROR_CHECK(err_code);
        }
      }
      break;
      
//    case BSP_EVENT_KEY_0:
//      sd_nvic_SystemReset();
//      break;

    default:
      break;
  }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
  uint32_t         err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type      = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = false;
  init.advdata.flags        = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

  init.config.ble_adv_fast_enabled  = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
  bsp_event_t startup_event;

  uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
  uint32_t err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
}

#ifndef USE_DMP
static void gatt_mtu_set(uint16_t att_mtu)
{
  ret_code_t err_code;
    
  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, att_mtu);
  APP_ERROR_CHECK(err_code);
  
  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, att_mtu);
  APP_ERROR_CHECK(err_code);
}

static void data_len_ext_set(bool status)
{
  uint8_t data_length = status ? (247 + L2CAP_HDR_LEN) : (23 + L2CAP_HDR_LEN);
  (void) nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, data_length);
}
#endif

static void updateValue(void)
{

  #ifdef USE_DMP
    float value[4] = {0.0f};
    run_dmp(value);
    ble_mpu_update(&m_dmp, (uint8_t *)value, sizeof(value));
  #else
    float value[10] = {0.0f};
    run_9dof(value);
    ble_mpu_update(&m_dmp, (uint8_t *)value, sizeof(value));
  #endif
}

/**
 * @brief Function that configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_config(void)
{
  if(!nrf_drv_gpiote_is_init()) {
    uint32_t err_code;
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  nrf_gpio_cfg_output(BSP_LED_0);
}

/**
 * @brief Function for starting lfclk needed by APP_TIMER.
 */
static void lfclk_init(void)
{
  uint32_t err_code;
  err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_clock_lfclk_request(NULL);

}

/**@brief Application main function.
 */
int main(void)
{
  uint32_t err_code;
  bool   erase_bonds;

  // Initialize.
  lfclk_init();
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  gpio_config();
  twi_master_init();
  mpu9250_init();

  buttons_leds_init(&erase_bonds);

  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();

  #ifndef USE_DMP
    gatt_mtu_set(247);
    data_len_ext_set(true);
  #endif

  err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);

  // Timer
  err_code = app_timer_create(&m_led_timer, APP_TIMER_MODE_REPEATED, led_timer_handler);
  APP_ERROR_CHECK(err_code);

  // Timer
  err_code = app_timer_start(m_led_timer, APP_TIMER_TICKS(5000), NULL);
  APP_ERROR_CHECK(err_code);

  nrf_gpio_pin_clear(BSP_LED_0);
	triggerSensorPolling = false;
	
  // Enter main loop.
  for (;;)
  {
    if (
        m_conn_handle != BLE_CONN_HANDLE_INVALID
			  &&
        m_dmp.is_notification_enabled == true
       )
    {
      updateValue();
    } //end if
		
    if ( triggerSensorPolling == true )
    {
      nrf_gpio_pin_set(BSP_LED_0);
      nrf_delay_ms(50);
      nrf_gpio_pin_clear(BSP_LED_0);
			triggerSensorPolling = false;
		}
		
    //UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    power_manage();

  } //end for

}

/**
 * @}
 */
