/*
 * Copyright (c) 2016, University of Bristol - http://www.bris.ac.uk/
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
 * \addtogroup sphere-spes-2-pir-sensor
 * @{
 *
 * \file
 * Driver for the SPHERE SPES-2 PIR sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "pir-sensor.h"
#include "lib/random.h"

#include <stdint.h>
/*---------------------------------------------------------------------------*/
uint8_t is_active;
/*---------------------------------------------------------------------------*/
static int
config(int type, int c)
{
  switch(type) {
  case SENSORS_HW_INIT:
    break;
  case SENSORS_ACTIVE:
    is_active = c;
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
    return is_active;
    break;
  default:
    break;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  switch(type) {
  case PIR_SENSOR_VALUE_CURRENT:
    return (random_rand() >> 4) % 2;

  case PIR_SENSOR_VALUE_LAST_SECOND:
    return (random_rand() >> 4) % 2;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(pir_sensor, PIR_SENSOR, value, config, status);
/*---------------------------------------------------------------------------*/
/** @} */
