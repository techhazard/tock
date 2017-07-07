#include <stdbool.h>
#include <stdio.h>

#include "adafruit.io.h"

int create_post_to_feed (char* buffer,
                         int buffer_len,
                         char* username,
                         char* feedname,
                         char* aio,
                         char* value) {
  char body[256];
  int written;

  // Create a string with the message body so we can get its length.
  written = snprintf(body, 256, "value=%s", value);

  // Create the entire POST message.
  written = snprintf(buffer, buffer_len,
                     "POST /api/v2/%s/feeds/%s/data HTTP/1.1\r\n"
                     "Host: io.adafruit.com\r\n"
                     "X-AIO-Key: %s\r\n"
                     "Content-Type: application/x-www-form-urlencoded\r\n"
                     "Content-Length: %i\r\n"
                     "\r\n"
                     "%s",
                     username, feedname, aio, written, body);
  if (written > 512) {
    return -1;
  }
  return written;
}
