#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "step_counter_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "nrf_log.h"

static void on_ble_write(ble_sc_t * service, ble_evt_t * p_ble_evt)
{
    uint32_t data_buffer;
    ble_gatts_value_t rx_data;
    rx_data.len = sizeof(uint32_t);
    rx_data.offset = 0;
    rx_data.p_value = (uint8_t*)&data_buffer;
    
    if(p_ble_evt->evt.gatts_evt.params.write.handle == service->char_handles.value_handle)
    {
        sd_ble_gatts_value_get(service->conn_handle, service->char_handles.value_handle, &rx_data);
    }
    else if(p_ble_evt->evt.gatts_evt.params.write.handle == service->char_handles.cccd_handle)
    {
        sd_ble_gatts_value_get(service->conn_handle, service->char_handles.cccd_handle, &rx_data);
        if(data_buffer == 0x0001)
        {
            NRF_LOG_INFO("Notification enabled\r\n");
        }
        else if(data_buffer == 0x0000)
        {
            NRF_LOG_INFO("Notification disabled\r\n");
        }
    }
}

void ble_sc_on_ble_evt(ble_sc_t * service, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {        
        case BLE_GATTS_EVT_WRITE:
            on_ble_write(service, p_ble_evt);
            break;
        case BLE_GAP_EVT_CONNECTED:
            service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            service->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            break;
    }
}

static uint32_t sc_char_add(ble_sc_t * service)
{
    uint32_t   err_code = 0;
    
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_SC_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_SC_CHARACTERISTC_UUID;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;

    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // Set characteristic length
    attr_char_value.max_len     = 4;
    attr_char_value.init_len    = 4;
    uint8_t value[4]            = {0x12,0x34,0x56,0x78};
    attr_char_value.p_value     = value;

    err_code = sd_ble_gatts_characteristic_add(service->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &service->char_handles);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

void sc_service_init(ble_sc_t * service)
{
    uint32_t   err_code;

    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_SC_BASE_UUID;
    service_uuid.uuid = BLE_UUID_SC_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    
    
    service->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &service->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    // Add characteristic to service
    sc_char_add(service);
}

void sc_update(ble_sc_t * service, int32_t * sc_value)
{
    if (service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t               len = 4;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = service->char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*) sc_value;  

        sd_ble_gatts_hvx(service->conn_handle, &hvx_params);
    }   
}