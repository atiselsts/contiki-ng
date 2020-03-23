/*
 * Copyright (c) 2017, University of Bristol - http://www.bristol.ac.uk
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup sphere-spes-2-piezo-sensor
 * @{
 *
 * \file
 * Driver for the SPHERE piezo sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "piezo-sensor.h"
#include "lib/random.h"
#include "sys/ctimer.h"

#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
#define STATE_IDLE        0
#define STATE_MEASURING   1
#define STATE_HAS_READING 2
#define STATE_INT_WAITING 3
/*---------------------------------------------------------------------------*/
#define INT_PIN_COLD    0
#define INT_PIN_HOT     1
#define INT_PIN_NONE    2
/*---------------------------------------------------------------------------*/
#define BOARD_PIEZO_AUX_HOT_ADC   1
#define BOARD_PIEZO_AUX_COLD_ADC  2
/*---------------------------------------------------------------------------*/
#define ADC_NUM_BITS 12
#define ADC_MASK ((1 << ADC_NUM_BITS) - 1)
/*---------------------------------------------------------------------------*/
static uint8_t state;
/*---------------------------------------------------------------------------*/
#define ADC_CONVERSION_HYSTERESIS (CLOCK_SECOND)
#define CAP_DISCHARGE_HYSTERESIS  (CLOCK_SECOND / 100)
#define HEARTBEAT_INTERVAL (CLOCK_SECOND * 60) /* once per minute if there is no data */

static struct ctimer adc_hysteresis_ct;
static struct ctimer cap_hysteresis_ct;
static struct ctimer heartbeat_timer;
/*---------------------------------------------------------------------------*/
static int cold_measurement_start, cold_measurement_end;
static int hot_measurement_start, hot_measurement_end;
/*---------------------------------------------------------------------------*/
static uint8_t int_pin;
/*---------------------------------------------------------------------------*/
/* Perform single-shot ADC reading and store in data */
static void
adc_single_shot_sync(int *data,int pin)
{
  *data = random_rand() & ADC_MASK;
}
/*---------------------------------------------------------------------------*/
static void
cap_ctimer_callback(void *data)
{
  PRINTF("Cap timer fired\n");
}
/*---------------------------------------------------------------------------*/
static void
adc_ctimer_callback(void *data)
{
  PRINTF("ADC timer fired\n");

  /* Perform the 2nd ADC reading for hot and cold. */
  adc_single_shot_sync(&hot_measurement_end, BOARD_PIEZO_AUX_HOT_ADC);
  adc_single_shot_sync(&cold_measurement_end, BOARD_PIEZO_AUX_COLD_ADC);

  /* Calculate flow diff example */
  //hot_flow = hot_measurement_end - hot_measurement_start;
  //cold_flow = cold_measurement_end - cold_measurement_start;
  //printf("Piezo,%01x,%04x,%04x,%04x,%04x\n",int_pin,hot_measurement_start, hot_measurement_end,cold_measurement_start,cold_measurement_end);

  /* Trigger capacitor discharge */
  ctimer_set(&cap_hysteresis_ct, CAP_DISCHARGE_HYSTERESIS,
      cap_ctimer_callback, NULL);
  state = STATE_HAS_READING;

  /* Notify all processes that there is data to collect */
  sensors_changed(&piezo_sensor);
}
/*---------------------------------------------------------------------------*/
static void
start_measurement(void)
{
  state = STATE_MEASURING;

  /* no discharge needed on interrupt */

  /* Take single-shot ADC reading for hot and cold. */
  adc_single_shot_sync(&hot_measurement_start, BOARD_PIEZO_AUX_HOT_ADC);
  adc_single_shot_sync(&cold_measurement_start, BOARD_PIEZO_AUX_COLD_ADC);
  //printf("Piezo hot start value=%d ,cold=%d\n", hot_measurement_start, cold_measurement_start);

  /* Set a callback to fire in ADC_CONVERSION_HYSTERESIS ticks */
  ctimer_set(&adc_hysteresis_ct, ADC_CONVERSION_HYSTERESIS,
      adc_ctimer_callback, NULL);
}
/*---------------------------------------------------------------------------*/
static void
heartbeat_ctimer_callback(void *data)
{
  PRINTF("Heartbeat timer\n");

  /* Not triggered by any interrupt */
  int_pin = INT_PIN_NONE;
  
  /* Start a new measurement even though an interrupt did not happen */
  start_measurement();

  ctimer_set(&heartbeat_timer, HEARTBEAT_INTERVAL, heartbeat_ctimer_callback, NULL);
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  int rv;
  switch(type) {
  case PIEZO_SENSOR_COLD_ADC_START_READING:
    return cold_measurement_start;
  case PIEZO_SENSOR_COLD_ADC_END_READING:
    return cold_measurement_end;
  case PIEZO_SENSOR_COLD_ALL_READINGS:
    /* pack into a single 32-bit value: [ start | end | flag ] */
    rv = (cold_measurement_start & ADC_MASK) << (ADC_NUM_BITS + 1);
    rv |= (cold_measurement_end & ADC_MASK) << 1;
    rv |= (int_pin == INT_PIN_COLD) ? 0x1 : 0;
    return rv;

  case PIEZO_SENSOR_HOT_ADC_START_READING:
    return hot_measurement_start;
  case PIEZO_SENSOR_HOT_ADC_END_READING:
    return hot_measurement_end;
  case PIEZO_SENSOR_HOT_ALL_READINGS:
    /* pack into a single 32-bit value: [ start | end | flag ] */
    rv = (hot_measurement_start & ADC_MASK) << (ADC_NUM_BITS + 1);
    rv |= (hot_measurement_end & ADC_MASK) << 1;
    rv |= (int_pin == INT_PIN_HOT) ? 0x1 : 0;
    return rv;

  case PIEZO_SENSOR_INT_PIN_TRIGGERED:
    return int_pin;

  default:
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
static int
config(int type, int c)
{
  switch(type) {
  case SENSORS_HW_INIT:
    state = STATE_IDLE;

    /* Discharge Capacitor */
    ctimer_set(&cap_hysteresis_ct, CAP_DISCHARGE_HYSTERESIS,
        cap_ctimer_callback, NULL);
    break;

  case SENSORS_ACTIVE:
    state = STATE_IDLE;
    if(c) {
      ctimer_set(&heartbeat_timer, HEARTBEAT_INTERVAL, heartbeat_ctimer_callback, NULL);
    } else {
      ctimer_stop(&heartbeat_timer);
    }
    break;

  default:
    break;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return state;
    break;
  default:
    break;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(piezo_sensor, "Piezo sensor", value, config, status);
/*---------------------------------------------------------------------------*/
/** @} */
