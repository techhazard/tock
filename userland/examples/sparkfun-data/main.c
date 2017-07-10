#include <stdio.h>

#include <nrf_error.h>

#include <isl29035.h>
#include <timer.h>
#include <tock.h>

#include "adafruit.io.h"
#include "ble_http.h"


void sensing_timer_callback (int a, int b, int c, void* ud);
void start_sensing (void);

char _post[512];
int _post_len = 0;

char AIO_KEY[]   =  "3e83a80b0ed94a338a3b3d26998b0dbe";
char USERNAME[]  = "bradjc";
char FEED_NAME[] = "hail-test";


void sensing_timer_callback (__attribute__ ((unused)) int a,
                             __attribute__ ((unused)) int b,
                             __attribute__ ((unused)) int c,
                             __attribute__ ((unused)) void* ud) {
  char value[200];
  int light;

  light = isl29035_read_light_intensity();
  snprintf(value, 200, "%i", light);

  _post_len = create_post_to_feed(_post, 512, USERNAME, FEED_NAME, AIO_KEY, value);

  blehttp_send_http_message(_post, _post_len);
}

void start_sensing (void) {
  printf("start sensing\n");
  while (1) {
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

  // Setup the BLE HTTP library.
  err_code = blehttp_init();
  if (err_code != NRF_SUCCESS) {
    printf("error setting up BLE HTTP: %lx\n", err_code);
  }

  // Start a loop.
  start_sensing();
}
