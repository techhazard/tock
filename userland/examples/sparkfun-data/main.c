#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <ble_advdata.h>
#include <ble_hci.h>
#include <nordic_common.h>
#include <nrf_error.h>
#include <nrf_sdm.h>
#include <softdevice_handler.h>
#include <ble_conn_params.h>

#include <eddystone.h>
#include <simple_adv.h>
#include <simple_ble.h>

#include <nrf51_serialization.h>

#include <console.h>
#include <led.h>
#include <timer.h>
#include <tock.h>

#include "nrf.h"


void blehttp_init (void);
void init_post (int value);
void write_http_string (void);
void write_http_string_loop (void);

/*******************************************************************************
 * BLE
 ******************************************************************************/

uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;

// Intervals for advertising and connections
simple_ble_config_t ble_config = {
  .platform_id       = 0x00,                // used as 4th octect in device BLE address
  .device_id         = DEVICE_ID_DEFAULT,
  .adv_name          = "tock-http",
  .adv_interval      = MSEC_TO_UNITS(500, UNIT_0_625_MS),
  .min_conn_interval = MSEC_TO_UNITS(10, UNIT_1_25_MS),
  .max_conn_interval = MSEC_TO_UNITS(1250, UNIT_1_25_MS)
};

ble_gap_adv_params_t m_adv_params;

#define CENTRAL_LINK_COUNT 1
#define PERIPHERAL_LINK_COUNT 1


#define BLEHTTP_BASE_UUID {{0x30, 0xb3, 0xf6, 0x90, 0x9a, 0x4f, 0x89, 0xb8, 0x1e, 0x46, 0x44, 0xcf, 0x01, 0x00, 0xba, 0x16}}
#define BLEHTTP_BASE_UUID_B {0x30, 0xb3, 0xf6, 0x90, 0x9a, 0x4f, 0x89, 0xb8, 0x1e, 0x46, 0x44, 0xcf, 0x01, 0x00, 0xba, 0x16}
#define BLE_UUID_BLEHTTP_CHAR 0x0002

// #define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(10, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
// #define MIN_CONNECTION_INTERVAL BLE_GAP_CP_MAX_CONN_INTVL_MIN
// #define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(10, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
// #define MAX_CONNECTION_INTERVAL BLE_GAP_CP_MAX_CONN_INTVL_MIN
#define SLAVE_LATENCY           0                               /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Determines supervision time-out in units of 10 millisecond. */


static const ble_gap_conn_params_t m_connection_param = {
  (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
  (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
  (uint16_t)SLAVE_LATENCY,            // Slave latency
  (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
};

static const ble_gap_scan_params_t m_scan_param = {
  .active = 0,                   // Active scanning not set.
  .selective = 0,                // Selective scanning not set.
  .p_whitelist = NULL,           // No whitelist provided.
  .interval = 0x00A0,
  .window = 0x0050,
  .timeout = 0x0000
};

static uint8_t blehttp_service_id[16] = BLEHTTP_BASE_UUID_B;


static bool _connected_and_ready = false;


__attribute__ ((const))
void ble_address_set (void) {
  // nop
}

uint16_t _char_handle = 0;
// uint16_t _char_decl_handle = 0;
uint16_t _char_desc_cccd_handle = 0;

char get[] = "GET https://j2x.us/\r\nhost: j2x.us\r\n\r\n";


char _post[512];
int _post_len = 0;

char AIO_KEY[] =  "3e83a80b0ed94a338a3b3d26998b0dbe";
char USERNAME[] = "bradjc";
char FEED_NAME[] = "hail-test";

void init_post (int value) {
  char body[256];
  int written;

  written = snprintf(body, 256, "value=%i", value);

  written = snprintf(_post, 512,
                         "POST /api/v2/%s/feeds/%s/data HTTP/1.1\r\n"
                         "Host: io.adafruit.com\r\n"
                         "X-AIO-Key: %s\r\n"
                         "Content-Type: application/x-www-form-urlencoded\r\n"
                         "Content-Length: %i\r\n"
                         "\r\n"
                         "%s",
                         USERNAME, FEED_NAME, AIO_KEY, written, body);
  if (written > 512) {
    printf("umm, couldn't fit post.");
  }
  _post_len = written;
}

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

void write_http_string_loop (void) {
  uint32_t err_code;
  uint8_t buf[2];

  ble_gattc_write_params_t write_params;
  memset(&write_params, 0, sizeof(write_params));

  switch (_write_state) {
    case WRITE_STATE_ENABLE_NOTIFICATIONS: {
      buf[0] = BLE_GATT_HVX_NOTIFICATION;
      buf[1] = 0;

      write_params.write_op = BLE_GATT_OP_WRITE_REQ;
      write_params.handle   = _char_desc_cccd_handle;
      write_params.offset   = 0;
      write_params.len      = sizeof(buf);
      write_params.p_value  = buf;

      _write_state = WRITE_STATE_PREPARE_WRITE;
      break;
    }

    case WRITE_STATE_PREPARE_WRITE: {
      // Figure out which portion of the write we are doing on this iteration.
      int len = _post_len - _write_offset;
      if (len > 18) {
        len = 18;
      }

      write_params.handle     = _char_handle;
      write_params.write_op   = BLE_GATT_OP_PREP_WRITE_REQ;
      write_params.offset     = _write_offset;
      write_params.len        = len;
      write_params.p_value    = _post + _write_offset;

      _write_offset += len;
      if (_write_offset >= _post_len) {
        _write_state = WRITE_STATE_EXECUTE_WRITE;
      }
      break;
    }

    case WRITE_STATE_EXECUTE_WRITE: {
      write_params.handle     = _char_handle;
      write_params.write_op   = BLE_GATT_OP_EXEC_WRITE_REQ;
      write_params.flags      = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
      _write_state = WRITE_STATE_DONE;
      break;
    }

    case WRITE_STATE_DONE: {
      return;
    }
  }

  // printf("write quick get string %i %i %i\n", _write_offset, _write_state, write_params.handle);
  err_code = sd_ble_gattc_write(conn_handle, &write_params);
  if (err_code != NRF_SUCCESS) {
    printf("error writing Characteristic 0x%x\n", err_code);
  }
}

// Write the HTTP request to the gateway.
void write_http_string (void) {
  _blehttp_state = BLEHTTP_STATE_EXECUTING_REQUEST;
  _write_offset = 0;
  _write_state = WRITE_STATE_ENABLE_NOTIFICATIONS;
  write_http_string_loop();
}


uint8_t _body[512];
uint8_t _body_len = 0;

// Read the HTTP response body back from the gateway.
void start_read () {
  uint32_t err_code;
  _body_len = 0;

  // do a read to get the data
  err_code = sd_ble_gattc_read(conn_handle, _char_handle, 0);
  if (err_code != NRF_SUCCESS) {
    printf("error doing read %i\n", err_code);
  }
}

void continue_read (const ble_gattc_evt_read_rsp_t* p_read_rsp) {
  uint32_t err_code;

  if (p_read_rsp->offset <= 512 && p_read_rsp->offset + p_read_rsp->len <= 512) {
    // printf("copying into buffer %i %i\n", p_read_rsp->offset, p_read_rsp->len);
    memcpy(_body+p_read_rsp->offset, p_read_rsp->data, p_read_rsp->len);
    _body_len += p_read_rsp->len;
  }

  if (p_read_rsp->len == 22) {
    err_code = sd_ble_gattc_read(conn_handle, _char_handle, p_read_rsp->offset + p_read_rsp->len);
    if (err_code != NRF_SUCCESS) {
      printf("error doing read %i\n", err_code);
    }
  } else {
    printf("%.*s\n", _body_len, _body);
    _blehttp_state = BLEHTTP_STATE_IDLE;

    err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS) {
      printf("error diconnecting??");
    }
  }
}


static bool is_blehttp_service_present(const ble_gap_evt_adv_report_t *p_adv_report) {
  uint32_t index = 0;
  uint8_t *p_data = (uint8_t *)p_adv_report->data;

  while (index < p_adv_report->dlen) {
    uint8_t field_length = p_data[index];
    uint8_t field_type   = p_data[index+1];

    if ((field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE ||
         field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE) &&
        (field_length - 1 == 16 &&
         memcmp(blehttp_service_id, &p_data[index + 2], 16) == 0)) {

      printf("Connecting to target %02x%02x%02x%02x%02x%02x\n",
                       p_adv_report->peer_addr.addr[0],
                       p_adv_report->peer_addr.addr[1],
                       p_adv_report->peer_addr.addr[2],
                       p_adv_report->peer_addr.addr[3],
                       p_adv_report->peer_addr.addr[4],
                       p_adv_report->peer_addr.addr[5]
                       );

      return true;
    }
    index += field_length + 1;
  }
  return false;
}

static void extract_name(const ble_gap_evt_adv_report_t *p_adv_report, char* buffer) {
  uint32_t index = 0;
  uint8_t *p_data = (uint8_t *)p_adv_report->data;
  ble_uuid_t extracted_uuid;

  while (index < p_adv_report->dlen) {
    uint8_t field_length = p_data[index];
    uint8_t field_type   = p_data[index+1];

    if ((field_type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME)
        || (field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME)) {
      memcpy(buffer, &p_data[index+2], field_length-1);
    }
    index += field_length + 1;
  }
}





// void ble_evt_user_handler (ble_evt_t* p_ble_evt) {
static void on_ble_evt_me (ble_evt_t* p_ble_evt) {
  uint32_t err_code;

  // printf("event %i\n", p_ble_evt->header.evt_id);

  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONN_PARAM_UPDATE: {
      // just update them right now
      ble_gap_conn_params_t conn_params;
      memset(&conn_params, 0, sizeof(conn_params));
      conn_params.min_conn_interval = ble_config.min_conn_interval;
      conn_params.max_conn_interval = ble_config.max_conn_interval;
      conn_params.slave_latency     = SLAVE_LATENCY;
      conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

      sd_ble_gap_conn_param_update(conn_handle, &conn_params);
      break;
    }

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
      const ble_gap_conn_params_t* p_conn_params = &(p_ble_evt->evt.gap_evt.params.conn_param_update_request);
      printf("update req: min: %i, max: %i, slave: %i, timeout: %i\n",
        p_conn_params->min_conn_interval,
        p_conn_params->max_conn_interval,
        p_conn_params->slave_latency,
        p_conn_params->conn_sup_timeout);
      // ble_gap_conn_params_t conn_params;
      // memcpy(&conn_params, p_conn_params, sizeof(ble_gap_conn_params_t));
      // err_code = sd_ble_gap_conn_param_update(conn_handle, &conn_params);
      // if (err_code != NRF_SUCCESS) {
      //   printf("error updating conn params %li\n", err_code);
      // }
      break;
    }

    case BLE_GAP_EVT_CONNECTED: {
      advertising_stop();

      conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

      ble_uuid_t httpget_service_uuid = {
        .uuid = 0x0001,
        .type = BLE_UUID_TYPE_VENDOR_BEGIN,
      };

      printf("connected handle: %x\n", conn_handle);
      uint32_t err_code = sd_ble_gattc_primary_services_discover(conn_handle, 0x0001, &httpget_service_uuid);
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
        const ble_gattc_handle_range_t* p_service_handle_range = &(p_ble_evt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[0].handle_range);

        // Discover characteristics.
        err_code = sd_ble_gattc_characteristics_discover(conn_handle, p_service_handle_range);
        if (err_code != NRF_SUCCESS) {
          printf("error discover char 0x%x\n", err_code);
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

        ble_uuid_t httpget_characteristic_uuid = {
          .uuid = BLE_UUID_BLEHTTP_CHAR,
          .type = BLE_UUID_TYPE_VENDOR_BEGIN,
        };

        // Iterate through the characteristics and find the correct one.
        for (int i = 0; i < p_char_disc_rsp->count; i++) {
          printf("char: uuid: 0x%x type: 0x%x\n", p_char_disc_rsp->chars[i].uuid.uuid, p_char_disc_rsp->chars[i].uuid.type);
          if (BLE_UUID_EQ(&httpget_characteristic_uuid, &(p_char_disc_rsp->chars[i].uuid))) {
            _char_handle = p_char_disc_rsp->chars[i].handle_value;
            // _char_decl_handle = p_char_disc_rsp->chars[i].handle_decl;
            printf("found char handle 0x%x\n", _char_handle);
            // printf("found decl handle 0x%x\n", _char_decl_handle);

            _char_desc_cccd_handle = _char_handle + 1;


            _connected_and_ready = true;

            write_http_string();

            // ble_gattc_handle_range_t descriptor_handle;
            // descriptor_handle.start_handle = _char_handle + 1;
            // descriptor_handle.end_handle = _char_handle + 1;

            // err_code = sd_ble_gattc_descriptors_discover(conn_handle, &descriptor_handle);
            // if (err_code != NRF_SUCCESS) {
            //   printf("error getting descriptors %i\n", err_code);
            // }
            break;
          }
        }
      }
      break;
    }

    // case BLE_GATTC_EVT_DESC_DISC_RSP: {
    //   if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS) {
    //     printf("descriptor not found\n");
    //   } else {
    //     const ble_gattc_evt_desc_disc_rsp_t* p_desc_disc_rsp;
    //     p_desc_disc_rsp = &(p_ble_evt->evt.gattc_evt.params.desc_disc_rsp);

    //     for (int i = 0; i < p_desc_disc_rsp->count; i++) {
    //       printf("desc: uuid: 0x%x type: 0x%x\n", p_desc_disc_rsp->descs[i].uuid.uuid, p_desc_disc_rsp->descs[i].uuid.type);

    //       if (p_desc_disc_rsp->descs[i].uuid.uuid == 0x2902) {
    //         // Found the CCCD descriptor
    //         _char_desc_cccd_handle = p_desc_disc_rsp->descs[i].handle;
    //         printf("desc handle 0x%x\n", _char_desc_cccd_handle);

    //         _connected_and_ready = true;
    //         // write_http_string();
    //         //
    //         //
    //         // do_post();
    //         break;
    //       }
    //     }
    //   }
    //   break;
    // }

    case BLE_GATTC_EVT_WRITE_RSP: {
      if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS) {
        printf("write error?? %i\n", p_ble_evt->evt.gattc_evt.gatt_status);
      } else {
        write_http_string_loop();
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

        printf("notify handle: 0x%x\n", p_hvx_evt->handle);

        if (p_hvx_evt->handle == _char_handle) {
          printf("got notification for handle we know\n");

          start_read();
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
        continue_read(p_read_rsp);
      }
      break;
    }

    case BLE_GAP_EVT_DISCONNECTED: {
      conn_handle = BLE_CONN_HANDLE_INVALID;
      printf("disconnect\n");
      _connected_and_ready = false;
      break;
    }

    case BLE_GAP_EVT_ADV_REPORT: {
      led_toggle(2);


      const ble_gap_evt_t* p_gap_evt = &p_ble_evt->evt.gap_evt;
      const ble_gap_evt_adv_report_t* p_adv_report = &p_gap_evt->params.adv_report;

      if (is_blehttp_service_present(p_adv_report)) {
        char device_name[31] = {'?'};
        extract_name(p_adv_report, device_name);
        printf("found %s\n", device_name);

        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &m_scan_param,
                                      &m_connection_param);

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
  printf("BLE ERROR: Code = 0x%x\n", (int)error_code);
}

static void ble_evt_dispatch_me(ble_evt_t * p_ble_evt) {
  on_ble_evt_me(p_ble_evt);
  ble_conn_params_on_ble_evt(p_ble_evt);
}








void blehttp_init (void) {
  ble_uuid128_t base_uuid = BLEHTTP_BASE_UUID;
  uint8_t base_uuid_type = BLE_UUID_TYPE_VENDOR_BEGIN;

  sd_ble_uuid_vs_add(&base_uuid, &base_uuid_type);
}




void connect_to_gateway (void) {
  uint32_t err_code;

  // err_code = sd_ble_gap_scan_stop();
  // err_code = sd_ble_gap_scan_start(&m_scan_param);
  // // It is okay to ignore this error since we are stopping the scan anyway.
  // if (err_code != NRF_ERROR_INVALID_STATE) {
  //   APP_ERROR_CHECK(err_code);
  // }

  ble_uuid_t adv_uuid = {0x0005, BLE_UUID_TYPE_VENDOR_BEGIN};
  simple_adv_service(&adv_uuid);
}

void sensing_timer_callback (int a, int b, int c, void* ud) {
  printf("cb %i %i\n", _connected_and_ready, _blehttp_state);
  if (_blehttp_state == BLEHTTP_STATE_IDLE) {
    int light = isl29035_read_light_intensity();
    printf("send %i\n", light);
    init_post(light);
    connect_to_gateway();
  }
}

void start_sensing () {
  printf("start sensing\n");
  while(1) {
    delay_ms(10000);
    sensing_timer_callback(0, 0, 0, NULL);
  }
  // timer_in(5000, sensing_timer_callback, NULL);
}


/*******************************************************************************
 * MAIN
 ******************************************************************************/

int main (void) {
  uint32_t err_code;

  printf("[BLE] Find Gateway\n");

  simple_ble_init(&ble_config);

  // Re-register the callback so that we use our handler and not simple ble's.
  err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch_me);
  APP_ERROR_CHECK(err_code);

  // Setup the BLE HTTP library.
  blehttp_init();

  // Start a loop.
  start_sensing();
}
