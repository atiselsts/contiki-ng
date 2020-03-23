/*****************************************************************************************************

 Copyright (C) - All Rights Reserved

 SPHERE (an EPSRC IRC), 2013-2018
 University of Bristol
 University of Reading
 University of Southampton

 Filename: fgbridge.c
 Description: SPI Bridge from BLE to 802.15.4
 Primary Contributor(s): Xenofon (Fontas) Fafoutis (xenofon.fafoutis@bristol.ac.uk)

*******************************************************************************************************/

#include "contiki.h"
#include "lib/random.h"
#include "fgbridge.h"

#if SPHERE
#include "sphere-timestamps.h"
#endif

#include <string.h>
#include <stdbool.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/

#define WEARABLE_ADDRESS "\x01\x02\xc0"

#define WEARABLE_RSSI -80

/* Wearable advertisement generation period */
#define GENERATION_INTERVAL CLOCK_SECOND

/*---------------------------------------------------------------------------*/

PROCESS(fgbridge_process, "fgbridge process");

/*---------------------------------------------------------------------------*/

static wearable_adv_t fgbridge_wearable_adv;
static bool is_fgbridge_wearable_adv_present;

static fgbridge_callback_function *callback;
static void *callback_argument;

/*---------------------------------------------------------------------------*/
bool
fgbridge_init(fgbridge_callback_function *cb, void *arg)
{
  callback = cb;
  callback_argument = arg;

  process_start(&fgbridge_process, NULL);

  printf("FGBRIDGE: init OK\n");
  return true;
}

/*---------------------------------------------------------------------------*/

void *
fgbridge_adv_read(uint16_t *length /* out */)
{
  if(is_fgbridge_wearable_adv_present) {
    *length = sizeof(fgbridge_wearable_adv);
    /* Assume that once it's read, the buffer can be reused.
     * This is a fair assumption as processes in Contiki are not preemptive. */
    is_fgbridge_wearable_adv_present = false;
  } else {
    /* return zero length */
    *length = 0;
  }
  return &fgbridge_wearable_adv;
}

/*---------------------------------------------------------------------------*/

static int8_t
get_rssi(void)
{
  static int8_t x = -90;
  static uint8_t down;
  if(down) {
    x--;
    if(x < -95) {
      down = 0;
    }
  } else {
    x++;
    if(x > -30) {
      down = 1;
    }
  }
  return x;
}

static uint8_t
get_x(void)
{
  static int x = -32;
  static uint8_t down;
  if(down) {
    x--;
    if(x < -32) {
      down = 0;
    }
  } else {
    x++;
    if(x > 32) {
      down = 1;
    }
  }
  return (uint8_t)x;
}

static uint8_t
get_y(void)
{
  static int x = 0;
  static uint8_t down;
  if(down) {
    x--;
    if(x < -32) {
      down = 0;
    }
  } else {
    x++;
    if(x > 32) {
      down = 1;
    }
  }
  return (uint8_t)x;
}

static uint8_t
get_z(void)
{
  static int x = 32;
  static uint8_t down;
  if(down) {
    x--;
    if(x < -32) {
      down = 0;
    }
  } else {
    x++;
    if(x > 32) {
      down = 1;
    }
  }
  return (uint8_t)(x + 32);
}

static void
generate_adv(void)
{
  int i;
  uint32_t timestamp;
  uint32_t monitor;
  static uint16_t mc;

#if SPHERE
  timestamp = sphere_timestamp_get();
#else
  timestamp = 0;
#endif
  monitor = 0x0;
  mc++;

  fgbridge_wearable_adv.wid = WEARABLE_ADDRESS[2];
  fgbridge_wearable_adv.mc[0] = mc & 0xff;
  fgbridge_wearable_adv.mc[1] = mc >> 8;
  fgbridge_wearable_adv.rssi = get_rssi();
  for(i = 0; i < FGBRIDGE_SPI_ADV_DATA_LEN / 3; ++i) {
    fgbridge_wearable_adv.data[i * 3] = get_x();
    fgbridge_wearable_adv.data[i * 3 + 1] = get_y();
    fgbridge_wearable_adv.data[i * 3 + 2] = get_z();
  }
  memcpy(fgbridge_wearable_adv.timestamp, &timestamp, sizeof(timestamp));
  memcpy(fgbridge_wearable_adv.monitor, &monitor, sizeof(monitor));

  /* now set the `present` flag */
  is_fgbridge_wearable_adv_present = true;
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(fgbridge_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

  /* wait for some initial timeout */
  etimer_set(&et, CLOCK_SECOND * 30);
  while(1) {
    PROCESS_YIELD();
    if(ev == PROCESS_EVENT_TIMER && data == &et) {
      break;
    }
  }

  etimer_set(&et, GENERATION_INTERVAL);

  while(1) {
    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER && data == &et) {

      if(!is_fgbridge_wearable_adv_present) {
        generate_adv();
      } else {
        printf("FGBRIDGE: buffer busy\n");
      }

      /* call the callback to notify about new data */
      callback(callback_argument);

      /*
       * Use etimer_restart() instead of etimer_reset():
       * this better deals with high-rate notifications,
       * but adds drift to the timer period.
       */
      etimer_restart(&et);
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
