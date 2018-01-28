// Askelmittari
// Embedded IoT Programming 2017
// Samuel Lindqvist, Lassi Salomaa

// STD includes
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

// Nordic includes
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
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "bsp.h"

// GPIOTE
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"

// SPI
#include "nrf_drv_spi.h"

// Services
#include "ble_bas.h"
#include "step_counter_service.h"

// BMI
#include "bmi160.h"

// BLE config, use defaults from template
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2
#define DEVICE_NAME                     "Askelmittari" // Name of the device!
#define MANUFACTURER_NAME               "UTU"
#define APP_ADV_INTERVAL                300                             
#define APP_ADV_TIMEOUT_IN_SECONDS      180                             
#define APP_BLE_OBSERVER_PRIO           3                               
#define APP_BLE_CONN_CFG_TAG            1                               
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)
#define SLAVE_LATENCY                   0                               
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS) 
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)           
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)          
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                               
#define SEC_PARAM_BOND                  1                               
#define SEC_PARAM_MITM                  0                               
#define SEC_PARAM_LESC                  0                               
#define SEC_PARAM_KEYPRESS              0                               
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE            
#define SEC_PARAM_OOB                   0                               
#define SEC_PARAM_MIN_KEY_SIZE          7                               
#define SEC_PARAM_MAX_KEY_SIZE          16                              
#define DEAD_BEEF                       0xDEADBEEF                      
NRF_BLE_GATT_DEF(m_gatt);                                               
BLE_ADVERTISING_DEF(m_advertising);                                     

// Connection handle
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

// Battery service instance
BLE_BAS_DEF(m_bas);

// Ignore buttons in BSP module
#define BUTTONS_NUMBER 0

// SPI config
#define SPI_INSTANCE 0
#define SPI_SS_PIN 29
#define SPI_MISO_PIN 28
#define SPI_MOSI_PIN 4
#define SPI_SCK_PIN 3

// SPI
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static volatile bool spi_xfer_done;

// BMI config
#define BMI160_INTERRUPT_PIN 11
static struct bmi160_dev sensor;

// RX buffer
static uint8_t       m_rx_buf[100];

// Step count
static int32_t step_count = 0;

// Step counter service
static ble_sc_t sc_service;

static ble_uuid_t m_adv_uuids[] =
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


static void advertising_start();

// Assert fails in SoftDevice
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

// PM events
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
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
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
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
            advertising_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
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
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

// Init timers, is this needed if we don't declare own timers?
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


// Init device General Access Profile parameters
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    // Set device name
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


// Init GATT
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

// Disconnect
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

// Connection parameters error
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

// Connection parameters module
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

// Go to sleep
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

// Handle advertising events
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;
    NRF_LOG_INFO("Got advertising event");

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


// Handle BLE events
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
    NRF_LOG_INFO("Got BLE event");

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

#ifndef S140
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
#endif

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
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
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

// Init BLE stack
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


// Init peer manager
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

// Init advertising
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}



// Init BLE services
static void services_init(void)
{
  ret_code_t err_code;

  //// Battery service
  ble_bas_init_t bas_init;
  
  memset(&bas_init, 0, sizeof(bas_init));
  
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);
  
  bas_init.evt_handler = NULL;
  bas_init.support_notification = true;
  bas_init.initial_batt_level = 100;
  bas_init.support_notification = NULL;
  
  err_code = ble_bas_init(&m_bas, &bas_init);

  APP_ERROR_CHECK(err_code);

  // TODO: Fix NRF_ERROR_NO_MEM
  // Init custom service
  //sc_service_init(&sc_service);
}

// Init LED
static void leds_init()
{
    ret_code_t err_code;

    err_code = bsp_init(BSP_INIT_LED, NULL);
    APP_ERROR_CHECK(err_code);
}

// Init NRF_LOG
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

// Power manage
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


// Start advertising
static void advertising_start()
{
  ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
}

// Update step count characteristic
void send_step_count() {
  // TODO: Fix step counter BLE service
  //sc_update(&sc_service, &step_count);
  bsp_board_led_invert(1);
  ble_bas_battery_level_update(&m_bas, (uint8_t) step_count);
  return;
}

// Handle interrupt from BMI160
void sensor_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  int8_t rslt = BMI160_OK;

  int8_t err_code = BMI160_OK;
  //err_code = bmi160_read_step_counter(&step_count,  &sensor);

  step_count += 1;
  send_step_count();

  if (err_code != BMI160_OK)
  {
    NRF_LOG_ERROR("Error reading step count");
  }  
}

// Handle SPI event
void spi_evt_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
  spi_xfer_done = true;
}

// Write to BMI160 using SPI
int8_t bmi160_spi_bus_write(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
  spi_xfer_done = false; // set the flag down during transfer

  int32_t error = 0;
  // Allocate array, which lenght is address + number of data bytes to be sent
  uint8_t tx_buff[cnt+1];

  uint16_t stringpos;
  // AND address with 0111 1111; set msb to '0' (write operation)
  tx_buff[0] = reg_addr & 0x7F;

  for (stringpos = 0; stringpos < cnt; stringpos++) {
    tx_buff[stringpos+1] = *(reg_data + stringpos);
  }
  // Do the actual SPI transfer
  nrf_drv_spi_transfer(&spi, tx_buff, cnt+1, NULL, 0);

  while (!spi_xfer_done) {}; // Loop until the transfer is complete

  return (int8_t)error;
}

// Read from BMI through SPI
int8_t bmi160_spi_bus_read(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  spi_xfer_done = false; // set the flag down during transfer
  int32_t error = 0;
  uint8_t tx_buff = reg_addr | 0x80; // OR address with 1000 0000; Read -> set msb to '1';
  uint8_t * rx_buff_pointer;
  uint8_t stringpos;
  rx_buff_pointer = (uint8_t *) (m_rx_buf);

  // Do the actual SPI transfer
  nrf_drv_spi_transfer(&spi, &tx_buff, 1, rx_buff_pointer, len+1);

  while (!spi_xfer_done) {} // Loop until the transfer is complete
  // Copy received bytes to reg_data
  for (stringpos = 0; stringpos < len; stringpos++)
  *(reg_data + stringpos) = m_rx_buf[stringpos + 1];

  return (int8_t)error;
}

// Init SPI
void spi_init()
{
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.ss_pin   = SPI_SS_PIN;
  spi_config.miso_pin = SPI_MISO_PIN;
  spi_config.mosi_pin = SPI_MOSI_PIN;
  spi_config.sck_pin  = SPI_SCK_PIN;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_evt_handler, NULL));
}

// Initialize BMI160
void sensor_init()
{
  // Init SPI 4-wire
  sensor.id = 0;
  sensor.interface = BMI160_SPI_INTF;
  sensor.read = bmi160_spi_bus_read;
  sensor.write = bmi160_spi_bus_write;
  sensor.delay_ms = nrf_delay_ms;

  int8_t err_code = BMI160_OK;
  err_code = bmi160_init(&sensor);

  if (err_code != BMI160_OK)
  {
    NRF_LOG_ERROR("Error initializing BMI");
  }

  int8_t rslt = BMI160_OK;

  /* Select the Output data rate, range of accelerometer sensor */
  sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
  sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
  sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

  /* Select the power mode of accelerometer sensor */
  sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

  /* Select the Output data rate, range of Gyroscope sensor */
  sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
  sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
  sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

  /* Select the power mode of Gyroscope sensor */
  sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

  /* Set the sensor configuration */
  err_code = bmi160_set_sens_conf(&sensor);

  if (err_code != BMI160_OK)
  {
    NRF_LOG_ERROR("Error initializing acc and gyro");
  }

  /* After the above function call, accel_cfg and gyro_cfg parameters in the device 
  structure are set with default values, found in the datasheet of the sensor */
}

// Configure sensor to send interrupt on detected step
void sensor_config()
{
  // Configure interrupt
  struct bmi160_int_settg int_config;

  /* Select the Interrupt channel/pin */
  int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1

  /* Select the Interrupt type */
  int_config.int_type = BMI160_STEP_DETECT_INT;// Choosing Step Detector interrupt
  /* Select the interrupt channel/pin settings */
  int_config.int_pin_settg.output_en = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
  int_config.int_pin_settg.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
  int_config.int_pin_settg.output_type = BMI160_ENABLE;// Choosing active High output
  int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;// Choosing edge triggered output
  int_config.int_pin_settg.input_en = BMI160_DISABLE;// Disabling interrupt pin to act as input
  int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;// non-latched output

  /* Select the Step Detector interrupt parameters, Kindly use the recommended settings for step detector */
  int_config.int_type_cfg.acc_step_detect_int.step_detector_mode = BMI160_STEP_DETECT_NORMAL;
  int_config.int_type_cfg.acc_step_detect_int.step_detector_en = BMI160_ENABLE;// 1-enable, 0-disable the step detector

  /* Set the Step Detector interrupt */
  int8_t err_code = BMI160_OK;
  bmi160_set_int_config(&int_config, &sensor); /* sensor is an instance of the structure bmi160_dev */
  if (err_code != BMI160_OK)
  {
    NRF_LOG_ERROR("Error configuring sensor interrupt");
  }
}

// Start step detector
void start_step_detector()
{
  // Enable the step counter
  uint8_t step_enable = 1;
  int8_t err_code = BMI160_OK;
  err_code = bmi160_set_step_counter(step_enable,  &sensor);
  if (err_code != BMI160_OK)
  {
    NRF_LOG_ERROR("Error starting the step detector");
  }
}

// Configure interrupt from sensor
void gpiote_init()
{
  ret_code_t err_code = NRF_SUCCESS;
  if(!nrf_drv_gpiote_is_init())
  {
    err_code = nrf_drv_gpiote_init();
  }
  APP_ERROR_CHECK(err_code);

  // Low-to-high causes interrupt
  nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  // DISABLE pullup
  config.pull = NRF_GPIO_PIN_NOPULL;
  nrf_drv_gpiote_pin_t INTERRUPT_PIN = BMI160_INTERRUPT_PIN;
  err_code = nrf_drv_gpiote_in_init(INTERRUPT_PIN, &config, sensor_evt_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(INTERRUPT_PIN, true);
  APP_ERROR_CHECK(err_code);
}

// App entry point
int main(void)
{
    log_init();
    NRF_LOG_INFO("Initializing timers");
    timers_init();
    NRF_LOG_INFO("Initializing buttons");
    leds_init();
    NRF_LOG_INFO("Initializing BLE stack");
    ble_stack_init();
    NRF_LOG_INFO("Initializing GAP parameters");
    gap_params_init();
    NRF_LOG_INFO("Initializing GATT");
    gatt_init();
    NRF_LOG_INFO("Initializing advertising");
    advertising_init();
    NRF_LOG_INFO("Initializing BLE services");
    services_init();
    NRF_LOG_INFO("Initializing connection parameters");
    conn_params_init();
    NRF_LOG_INFO("Initializing peer manager");
    peer_manager_init();
    NRF_LOG_INFO("Initializing GPIOTE");
    gpiote_init();
    NRF_LOG_INFO("Initializing SPI");
    spi_init();
    NRF_LOG_INFO("Initializing BMI160");
    sensor_init();
    NRF_LOG_INFO("Configuring step counter");
    sensor_config();

    NRF_LOG_INFO("Initialization succesful");

    // Start advertising
    advertising_start();

    NRF_LOG_INFO("Starting step counter");
    start_step_detector();

    // Main loop
    for (;;)
    {
        bool manage = true;

        if (!spi_xfer_done || NRF_LOG_PROCESS() == false) manage = false;
        if (manage)
        {
            power_manage();
        }

    }
}