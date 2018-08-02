/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
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
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_MPU)
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_mpu_dmp.h"

#define BLE_UUID_MPU_TX_CHARACTERISTIC 0x0002                      /**< The UUID of the TX Characteristic. */

#define BLE_MPU_MAX_TX_CHAR_LEN          sizeof(float)*4 //sizeof(float)*10

#define MPU_BASE_UUID                   {{0x82, 0xA1, 0xEA, 0xE9, 0xD5, 0x5A, 0x2A, 0x97, 0x97, 0x4C, 0x38, 0x8D, 0x00, 0x00, 0x57, 0xB5}} /**< Used vendor specific UUID. */

#define QUATERNION_CHAR_DESC "Quaternion"


/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_mpu     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_mpu_t * p_mpu, ble_evt_t const * p_ble_evt)
{
    p_mpu->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the SoftDevice.
 *
 * @param[in] p_mpu     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_mpu_t * p_mpu, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_mpu->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_mpu     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_mpu_t * p_mpu, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  
    ble_mpu_evt_t evt;
    evt.p_mpu = p_mpu;
	
    if  (p_evt_write->handle == p_mpu->tx_handles.cccd_handle) {
        if (ble_srv_is_notification_enabled(p_evt_write->data)) {
            p_mpu->is_notification_enabled = true;
            evt.type = BLE_MPU_EVT_COMM_STARTED;
        }else{
            p_mpu->is_notification_enabled = false;
            evt.type = BLE_MPU_EVT_COMM_STOPPED;
        }
				p_mpu->data_handler(&evt);
    }else {
        // Do Nothing. This event is not relevant for this service.
        //
    }
}


/**@brief Function for adding TX characteristic.
 *
 * @param[in] p_mpu       Nordic UART Service structure.
 * @param[in] p_mpu_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_char_add(ble_mpu_t * p_mpu, ble_mpu_init_t const * p_mpu_init)
{
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = (uint8_t *)QUATERNION_CHAR_DESC;
    char_md.char_user_desc_max_size  = sizeof(QUATERNION_CHAR_DESC);
    char_md.char_user_desc_size      = sizeof(QUATERNION_CHAR_DESC);
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_mpu->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPU_TX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = BLE_MPU_MAX_TX_CHAR_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_MPU_MAX_TX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_mpu->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_mpu->tx_handles);
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}


/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_mpu       Nordic UART Service structure.
 * @param[in] p_mpu_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
/*
static uint32_t rx_char_add(ble_mpu_t * p_mpu, const ble_mpu_init_t * p_mpu_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_mpu->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPU_RX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_MPU_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_mpu->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_mpu->rx_handles);
}
*/

void ble_mpu_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_mpu_t * p_mpu = (ble_mpu_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_mpu, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_mpu, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_mpu, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        {
            //notify with empty data that some tx was completed.
            //ble_mpu_evt_t evt = {
            //        .type = BLE_MPU_EVT_TX_RDY,
            //        .p_mpu = p_mpu
            //};
            //p_mpu->data_handler(&evt);
            break;
        }
        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_mpu_init(ble_mpu_t * p_mpu, ble_mpu_init_t const * p_mpu_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t mpu_base_uuid = MPU_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_mpu);
    VERIFY_PARAM_NOT_NULL(p_mpu_init);

    // Initialize the service structure.
    p_mpu->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_mpu->data_handler            = p_mpu_init->data_handler;
    p_mpu->is_notification_enabled = false;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&mpu_base_uuid, &p_mpu->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_mpu->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPU_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_mpu->service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the TX Characteristic.
    err_code = tx_char_add(p_mpu, p_mpu_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


uint32_t ble_mpu_update(ble_mpu_t* p_mpu, uint8_t* quaternion_values, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    VERIFY_PARAM_NOT_NULL(p_mpu);

    if ((p_mpu->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_mpu->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_MPU_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_mpu->tx_handles.value_handle;
    hvx_params.p_data = quaternion_values;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_mpu->conn_handle, &hvx_params);
}

#endif // NRF_MODULE_ENABLED(BLE_MPU)
