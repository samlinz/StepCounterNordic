#ifndef OUR_SERVICE_H__
#define OUR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

// FROM_SERVICE_TUTORIAL: Defining 16-bit service and 128-bit base UUIDs

// Service base UUID
#define BLE_UUID_SC_BASE_UUID              {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}}
// 16-bit UUID for service
#define BLE_UUID_SC_SERVICE_UUID                0xF00D
// 16-bit UUID for characteristic
#define BLE_UUID_SC_CHARACTERISTC_UUID          0xBEEF

typedef struct
{
    uint16_t                    conn_handle;
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    char_handles;
} ble_sc_t;

void ble_sc_on_ble_evt(ble_sc_t * service, ble_evt_t * p_ble_evt);

void sc_service_init(ble_sc_t * service);

void sc_update(ble_sc_t * service, int32_t * sc_value);

#endif