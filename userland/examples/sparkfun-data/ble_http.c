#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <ble_advdata.h>
#include <ble_conn_params.h>
#include <ble_hci.h>
#include <ble_stack_handler_types.h>
#include <nordic_common.h>
#include <nrf.h>
#include <nrf_error.h>
#include <nrf_sdm.h>
#include <softdevice_handler.h>

#include <eddystone.h>
#include <simple_adv.h>
#include <simple_ble.h>

#include <nrf51_serialization.h>

#include <console.h>
#include <led.h>
#include <timer.h>
#include <tock.h>

#include "adafruit.io.h"
#include "ble_http.h"


// Intervals for advertising and connections
simple_ble_config_t _ble_config = {
  .device_id         = DEVICE_ID_DEFAULT,
  .adv_name          = "tock-http",
  .adv_interval      = MSEC_TO_UNITS(500, UNIT_0_625_MS),
  .min_conn_interval = MSEC_TO_UNITS(10, UNIT_1_25_MS),
  .max_conn_interval = MSEC_TO_UNITS(1250, UNIT_1_25_MS)
};

// Properties of the BLE HTTP service.
#define BLEHTTP_BASE_UUID {0x30, 0xb3, 0xf6, 0x90, 0x9a, 0x4f, 0x89, 0xb8, 0x1e, 0x46, 0x44, 0xcf, 0x01, 0x00, 0xba, \
                           0x16}
#define BLE_UUID_BLEHTTP_SERVICE     0x0001
#define BLE_UUID_BLEHTTP_HTTPS_CHAR  0x0003
#define BLE_UUID_BLEHTTP_BODY_CHAR   0x0004
#define BLE_UUID_BLEHTTP_WANTED_CHAR 0x0006

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(10, UNIT_1_25_MS)
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(10, UNIT_1_25_MS)
#define SLAVE_LATENCY           0
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)


static const ble_gap_conn_params_t _connection_param = {
  (uint16_t) MIN_CONNECTION_INTERVAL,  // Minimum connection
  (uint16_t) MAX_CONNECTION_INTERVAL,  // Maximum connection
  (uint16_t) SLAVE_LATENCY,            // Slave latency
  (uint16_t) SUPERVISION_TIMEOUT       // Supervision time-out
};

static const ble_gap_scan_params_t _scan_param = {
  .active      = 0,    // Active scanning not set.
  .selective   = 0,    // Selective scanning not set.
  .p_whitelist = NULL, // No whitelist provided.
  .interval    = 0x00A0,
  .window      = 0x0050,
  .timeout     = 0x0000
};


// Override. Don't need for serialization.
__attribute__ ((const))
void ble_address_set (void) {
  // nop
}

char* _http_mesage = NULL;
int _http_len      = 0;

uint16_t _conn_handle       = BLE_CONN_HANDLE_INVALID;
uint16_t _char_handle_https = 0;
uint16_t _char_handle_body  = 0;
uint16_t _char_desc_cccd_handle_body = 0;



typedef enum {
  WRITE_STATE_ENABLE_NOTIFICATIONS,
  WRITE_STATE_PREPARE_WRITE,
  WRITE_STATE_EXECUTE_WRITE,
  WRITE_STATE_DONE,
} write_state_e;

int _write_offset = 0;
write_state_e _write_state = WRITE_STATE_ENABLE_NOTIFICATIONS;

typedef enum {
  BLEHTTP_STATE_IDLE,
  BLEHTTP_STATE_EXECUTING_REQUEST,
} blthttp_state_e;

blthttp_state_e _blehttp_state = BLEHTTP_STATE_IDLE;

static void __write_http_string_loop (void) {
  uint32_t err_code;
  uint8_t buf[2];

  ble_gattc_write_params_t write_params;
  memset(&write_params, 0, sizeof(write_params));

  switch (_write_state) {
    case WRITE_STATE_ENABLE_NOTIFICATIONS: {
      buf[0] = BLE_GATT_HVX_NOTIFICATION;
      buf[1] = 0;

      write_params.write_op = BLE_GATT_OP_WRITE_REQ;
      write_params.handle   = _char_desc_cccd_handle_body;
      write_params.offset   = 0;
      write_params.len      = sizeof(buf);
      write_params.p_value  = buf;

      _write_state = WRITE_STATE_PREPARE_WRITE;
      break;
    }

    case WRITE_STATE_PREPARE_WRITE: {
      // Figure out which portion of the write we are doing on this iteration.
      int len = _http_len - _write_offset;
      if (len > 18) {
        len = 18;
      }

      write_params.handle   = _char_handle_https;
      write_params.write_op = BLE_GATT_OP_PREP_WRITE_REQ;
      write_params.offset   = _write_offset;
      write_params.len      = len;
      write_params.p_value  = (uint8_t*) _http_mesage + _write_offset;

      _write_offset += len;
      if (_write_offset >= _http_len) {
        _write_state = WRITE_STATE_EXECUTE_WRITE;
      }
      break;
    }

    case WRITE_STATE_EXECUTE_WRITE: {
      write_params.handle   = _char_handle_https;
      write_params.write_op = BLE_GATT_OP_EXEC_WRITE_REQ;
      write_params.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
      _write_state = WRITE_STATE_DONE;
      break;
    }

    case WRITE_STATE_DONE: {
      return;
    }
  }

  err_code = sd_ble_gattc_write(_conn_handle, &write_params);
  if (err_code != NRF_SUCCESS) {
    printf("error writing Characteristic 0x%lx\n", err_code);
  }
}

// Write the HTTP request to the gateway.
static void __write_http_string (void) {
  _blehttp_state = BLEHTTP_STATE_EXECUTING_REQUEST;
  _write_offset  = 0;
  _write_state   = WRITE_STATE_ENABLE_NOTIFICATIONS;
  __write_http_string_loop();
}


uint8_t _body[512];
uint8_t _body_len = 0;

// Read the HTTP response body back from the gateway.
static void __start_read (void) {
  uint32_t err_code;
  _body_len = 0;

  // do a read to get the data
  err_code = sd_ble_gattc_read(_conn_handle, _char_handle_body, 0);
  if (err_code != NRF_SUCCESS) {
    printf("error doing read %li\n", err_code);
  }
}

static void __continue_read (const ble_gattc_evt_read_rsp_t* p_read_rsp) {
  uint32_t err_code;

  if (p_read_rsp->offset <= 512 && p_read_rsp->offset + p_read_rsp->len <= 512) {
    // printf("copying into buffer %i %i\n", p_read_rsp->offset, p_read_rsp->len);
    memcpy(_body + p_read_rsp->offset, p_read_rsp->data, p_read_rsp->len);
    _body_len += p_read_rsp->len;
  }

  if (p_read_rsp->len == 22) {
    err_code = sd_ble_gattc_read(_conn_handle, _char_handle_body, p_read_rsp->offset + p_read_rsp->len);
    if (err_code != NRF_SUCCESS) {
      printf("error doing read %li\n", err_code);
    }
  } else {
    printf("%.*s\n", _body_len, _body);
    _blehttp_state = BLEHTTP_STATE_IDLE;

    err_code = sd_ble_gap_disconnect(_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS) {
      printf("error diconnecting??");
    }
  }
}

// Check that the gateway is advertising the correct service UUID.
static bool __is_blehttp_service_present(const ble_gap_evt_adv_report_t *p_adv_report) {
  uint8_t blehttp_service_id[16] = BLEHTTP_BASE_UUID;
  uint32_t index  = 0;
  uint8_t *p_data = (uint8_t *)p_adv_report->data;

  while (index < p_adv_report->dlen) {
    uint8_t field_length = p_data[index];
    uint8_t field_type   = p_data[index + 1];

    if ((field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE ||
         field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE) &&
        (field_length - 1 == 16 &&
         memcmp(blehttp_service_id, &p_data[index + 2], 16) == 0)) {
      return true;
    }
    index += field_length + 1;
  }
  return false;
}


static void __on_ble_evt (ble_evt_t* p_ble_evt) {
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONN_PARAM_UPDATE: {
      // just update them right now
      ble_gap_conn_params_t conn_params;
      memset(&conn_params, 0, sizeof(conn_params));
      conn_params.min_conn_interval = _ble_config.min_conn_interval;
      conn_params.max_conn_interval = _ble_config.max_conn_interval;
      conn_params.slave_latency     = SLAVE_LATENCY;
      conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

      sd_ble_gap_conn_param_update(_conn_handle, &conn_params);
      break;
    }

    case BLE_GAP_EVT_CONNECTED: {
      _conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

      ble_uuid_t httpget_service_uuid = {
        .uuid = BLE_UUID_BLEHTTP_SERVICE,
        .type = BLE_UUID_TYPE_VENDOR_BEGIN,
      };

      // printf("connected handle: %x\n", conn_handle);
      err_code = sd_ble_gattc_primary_services_discover(_conn_handle, BLE_UUID_BLEHTTP_SERVICE, &httpget_service_uuid);
      if (err_code != NRF_SUCCESS) {
        printf("error discovering services 0x%lx\n", err_code);
      }
      break;
    }

    case BLE_EVT_TX_COMPLETE: {
      printf("tx complete\n");
      break;
    }

    case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP: {
      if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS ||
          p_ble_evt->evt.gattc_evt.params.prim_srvc_disc_rsp.count == 0) {
        printf("Service not found\n");
      } else {
        // There should be only one instance of the service at the peer.
        // So only the first element of the array is of interest.
        const ble_gattc_handle_range_t* p_service_handle_range =
          &(p_ble_evt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[0].handle_range);

        // Discover characteristics.
        err_code = sd_ble_gattc_characteristics_discover(_conn_handle, p_service_handle_range);
        if (err_code != NRF_SUCCESS) {
          printf("error discover char 0x%lx\n", err_code);
        }
      }
      break;
    }

    case BLE_GATTC_EVT_CHAR_DISC_RSP: {
      if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS) {
        printf("Characteristic not found\n");
      } else {

        const ble_gattc_evt_char_disc_rsp_t* p_char_disc_rsp;
        p_char_disc_rsp = &(p_ble_evt->evt.gattc_evt.params.char_disc_rsp);

        ble_uuid_t httpget_https_characteristic_uuid = {
          .uuid = BLE_UUID_BLEHTTP_HTTPS_CHAR,
          .type = BLE_UUID_TYPE_VENDOR_BEGIN,
        };
        ble_uuid_t httpget_body_characteristic_uuid = {
          .uuid = BLE_UUID_BLEHTTP_BODY_CHAR,
          .type = BLE_UUID_TYPE_VENDOR_BEGIN,
        };
        // printf("looking for char %i\n", p_char_disc_rsp->count);

        // Iterate through the characteristics and find the correct one.
        uint16_t last_handle = 0;
        for (int i = 0; i < p_char_disc_rsp->count; i++) {
          // printf("char uuid %i\n", p_char_disc_rsp->chars[i].uuid.uuid);
          if (BLE_UUID_EQ(&httpget_https_characteristic_uuid, &(p_char_disc_rsp->chars[i].uuid))) {
            _char_handle_https = p_char_disc_rsp->chars[i].handle_value;
            // printf("found https char handle 0x%x\n", _char_handle_https);

          } else if (BLE_UUID_EQ(&httpget_body_characteristic_uuid, &(p_char_disc_rsp->chars[i].uuid))) {
            _char_handle_body = p_char_disc_rsp->chars[i].handle_value;
            // printf("found body char handle 0x%x\n", _char_handle_body);

            // Descriptor handle is predictable.
            _char_desc_cccd_handle_body = _char_handle_body + 1;
          }

          // Save this so we know where to keep searching from when we continue.
          last_handle = p_char_disc_rsp->chars[i].handle_value;
        }

        if (_char_handle_https == 0 || _char_handle_body == 0) {
          // Are not done discovering, keep looking.
          ble_gattc_handle_range_t p_service_handle_range = {
            .start_handle = last_handle + 1,
            .end_handle   = last_handle + 11,
          };

          err_code = sd_ble_gattc_characteristics_discover(_conn_handle, &p_service_handle_range);
          if (err_code != NRF_SUCCESS) {
            printf("error discover char loop 0x%lx\n", err_code);
          }

        } else {
          // Start write loop to send the HTTP message.
          __write_http_string();
        }
      }
      break;
    }

    case BLE_GATTC_EVT_WRITE_RSP: {
      if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS) {
        printf("write error?? %i\n", p_ble_evt->evt.gattc_evt.gatt_status);
      } else {
        // Continue loop.
        __write_http_string_loop();
      }
      break;
    }

    case BLE_GATTC_EVT_HVX: {
      // Got notification

      if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS) {
        printf("notification bad???\n");
      } else {
        const ble_gattc_evt_hvx_t* p_hvx_evt;
        p_hvx_evt = &(p_ble_evt->evt.gattc_evt.params.hvx);

        // printf("notify handle: 0x%x\n", p_hvx_evt->handle);

        // When we get a notification we know the response is ready to read.
        if (p_hvx_evt->handle == _char_handle_body) {
          // printf("got notification for handle we know\n");
          __start_read();
        }
      }

      break;
    }

    case BLE_GATTC_EVT_READ_RSP: {
      if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS) {
        printf("read bad???\n");
      } else {
        const ble_gattc_evt_read_rsp_t* p_read_rsp;
        p_read_rsp = &(p_ble_evt->evt.gattc_evt.params.read_rsp);
        __continue_read(p_read_rsp);
      }
      break;
    }

    case BLE_GAP_EVT_DISCONNECTED: {
      _conn_handle       = BLE_CONN_HANDLE_INVALID;
      _char_handle_https = 0;
      _char_handle_body  = 0;
      break;
    }

    case BLE_GAP_EVT_ADV_REPORT: {
      const ble_gap_evt_t* p_gap_evt = &p_ble_evt->evt.gap_evt;
      const ble_gap_evt_adv_report_t* p_adv_report = &p_gap_evt->params.adv_report;

      if (__is_blehttp_service_present(p_adv_report)) {
        // Go ahead and connect to the gateway.
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &_scan_param,
                                      &_connection_param);

        if (err_code != NRF_SUCCESS) {
          printf("error connecting to device %li.\n", err_code);
        }
      }
      break;
    }

    default:
      printf("event 0x%x\n", p_ble_evt->header.evt_id);
  }
}

void ble_error (uint32_t error_code) {
  printf("BLE ERROR: Code = 0x%x\n", (int) error_code);
}

// Called by the softdevice (via serialization) when a BLE event occurs.
static void __ble_evt_dispatch (ble_evt_t* p_ble_evt) {
  __on_ble_evt(p_ble_evt);
  ble_conn_params_on_ble_evt(p_ble_evt);
}

// Start advertising to hope a gateway connects.
static void __start_advertising (void) {
  ble_uuid_t adv_uuid = {BLE_UUID_BLEHTTP_WANTED_CHAR, BLE_UUID_TYPE_VENDOR_BEGIN};
  simple_adv_service(&adv_uuid);
}



void blehttp_send_http_message (char* http, int http_len) {
  _http_mesage = http;
  _http_len    = http_len;

  // Scan for advertisements method.
  // err_code = sd_ble_gap_scan_stop();
  // err_code = sd_ble_gap_scan_start(&m_scan_param);
  // // It is okay to ignore this error since we are stopping the scan anyway.
  // if (err_code != NRF_ERROR_INVALID_STATE) {
  //   APP_ERROR_CHECK(err_code);
  // }

  // Advertise and hope a gateway connects.
  __start_advertising();
}

uint32_t blehttp_init (void) {
  uint32_t err_code;

  // Setup simple BLE. This does most of the nordic setup.
  simple_ble_init(&_ble_config);

  // Re-register the callback so that we use our handler and not simple ble's.
  err_code = softdevice_ble_evt_handler_set(__ble_evt_dispatch);
  if (err_code != NRF_SUCCESS) return err_code;

  // Set the UUID in the soft device so it can use it.
  ble_uuid128_t base_uuid = {BLEHTTP_BASE_UUID};
  uint8_t base_uuid_type  = BLE_UUID_TYPE_VENDOR_BEGIN;
  err_code = sd_ble_uuid_vs_add(&base_uuid, &base_uuid_type);
  if (err_code != NRF_SUCCESS) return err_code;

  return NRF_SUCCESS;
}
