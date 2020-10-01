/* This code belongs in ble_acc.c*/
#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_acc.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

#include "stdlib.h"

/**@brief Function for adding the Acceleration Value characteristic.
 *
 * @param[in]   p_acc        Acceleration Service structure.
 * @param[in]   p_acc_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t acc_value_char_add(ble_acc_t * p_acc, const ble_acc_init_t * p_acc_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Acceleration Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;


    // Add metadata for characteristic
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1; // Read-only
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_acc_init->acc_value_char_attr_md.read_perm;
    attr_md.write_perm = p_acc_init->acc_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_acc->uuid_type;
    ble_uuid.uuid = ACCELERATION_VALUE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = ACCELERATION_SIZE;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = ACCELERATION_SIZE;

    err_code = sd_ble_gatts_characteristic_add(p_acc->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_acc->acc_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("err_code: %d", err_code);
        NRF_LOG_INFO("NRF_ERROR_INVALID_PARAM: %d", NRF_ERROR_INVALID_PARAM);
        return err_code;
    }

    return NRF_SUCCESS;
}

/** Initialization function for bluetooth service **/ 
uint32_t ble_acc_init(ble_acc_t * p_acc, const ble_acc_init_t * p_acc_init)
{
    if (p_acc == NULL || p_acc_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_acc->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Acceleration Service UUID
    ble_uuid128_t base_uuid = {ACCELERATION_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_acc->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_acc->uuid_type;
    ble_uuid.uuid = ACCELERATION_SERVICE_UUID;

    // Add the Acceleration Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_acc->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Acceleration Value characteristic
    return acc_value_char_add(p_acc, p_acc_init);
}

/** Function to allow changing characteristic value **/
uint32_t ble_acc_value_update(ble_acc_t * p_acc, uint8_t *acc_value)
{
    NRF_LOG_INFO("In ble_acc_value_update. \r\n"); 
    if (p_acc == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = ACCELERATION_SIZE;
    gatts_value.offset  = 0;
    gatts_value.p_value = acc_value;


    // Update database.
    err_code = sd_ble_gatts_value_set(p_acc->conn_handle,
                                      p_acc->acc_value_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return err_code;
}