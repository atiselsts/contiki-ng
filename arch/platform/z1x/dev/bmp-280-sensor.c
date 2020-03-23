/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * \addtogroup sensortag-cc26xx-bmp-sensor
 * @{
 *
 * \file
 *  Driver for the Sensortag-CC26XX BMP280 Altimeter / Pressure Sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "lib/sensors.h"
#include "bmp-280-sensor.h"
#include "sys/ctimer.h"

#include "sensor-values.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
#define BMP280_I2C_ADDRESS                  0x77
/*---------------------------------------------------------------------------*/
/* Registers */
#define ADDR_CALIB                          0x88
#define ADDR_PROD_ID                        0xD0
#define ADDR_RESET                          0xE0
#define ADDR_STATUS                         0xF3
#define ADDR_CTRL_MEAS                      0xF4
#define ADDR_CONFIG                         0xF5
#define ADDR_PRESS_MSB                      0xF7
#define ADDR_PRESS_LSB                      0xF8
#define ADDR_PRESS_XLSB                     0xF9
#define ADDR_TEMP_MSB                       0xFA
#define ADDR_TEMP_LSB                       0xFB
#define ADDR_TEMP_XLSB                      0xFC
/*---------------------------------------------------------------------------*/
/* Reset values */
#define VAL_PROD_ID                         0x58
#define VAL_RESET                           0x00
#define VAL_STATUS                          0x00
#define VAL_CTRL_MEAS                       0x00
#define VAL_CONFIG                          0x00
#define VAL_PRESS_MSB                       0x80
#define VAL_PRESS_LSB                       0x00
#define VAL_TEMP_MSB                        0x80
#define VAL_TEMP_LSB                        0x00
/*---------------------------------------------------------------------------*/
/* Test values */
#define VAL_RESET_EXECUTE                   0xB6
#define VAL_CTRL_MEAS_TEST                  0x55
/*---------------------------------------------------------------------------*/
/* Misc. */
#define MEAS_DATA_SIZE                      6
#define CALIB_DATA_SIZE                     24
/*---------------------------------------------------------------------------*/
#define RES_OFF                             0
#define RES_ULTRA_LOW_POWER                 1
#define RES_LOW_POWER                       2
#define RES_STANDARD                        3
#define RES_HIGH                            5
#define RES_ULTRA_HIGH                      6
/*---------------------------------------------------------------------------*/
/* Bit fields in CTRL_MEAS register */
#define PM_OFF                              0
#define PM_FORCED                           1
#define PM_NORMAL                           3
/*---------------------------------------------------------------------------*/
#define OSRST(v)                            ((v) << 5)
#define OSRSP(v)                            ((v) << 2)
/*---------------------------------------------------------------------------*/
typedef struct bmp_280_calibration {
  uint16_t dig_t1;
  int16_t dig_t2;
  int16_t dig_t3;
  uint16_t dig_p1;
  int16_t dig_p2;
  int16_t dig_p3;
  int16_t dig_p4;
  int16_t dig_p5;
  int16_t dig_p6;
  int16_t dig_p7;
  int16_t dig_p8;
  int16_t dig_p9;
  int32_t t_fine;
} bmp_280_calibration_t;
/*---------------------------------------------------------------------------*/
#define SENSOR_STATUS_DISABLED     0
#define SENSOR_STATUS_INITIALISED  1
#define SENSOR_STATUS_NOT_READY    2
#define SENSOR_STATUS_READY        3

static int enabled = SENSOR_STATUS_DISABLED;
/*---------------------------------------------------------------------------*/
/* A buffer for the raw reading from the sensor */
#define SENSOR_DATA_BUF_SIZE   6

static uint8_t sensor_value[SENSOR_DATA_BUF_SIZE];
/*---------------------------------------------------------------------------*/
/* Wait SENSOR_STARTUP_DELAY clock ticks for the sensor to be ready - ~80ms */
#define SENSOR_STARTUP_DELAY 3

static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
  enabled = SENSOR_STATUS_READY;
  sensors_changed(&bmp_280_sensor);
}
/*---------------------------------------------------------------------------*/
static void
select_on_bus(void)
{
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Initalise the sensor
 */
static void
init(void)
{
  select_on_bus();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Enable/disable measurements
 * \param enable 0: disable, enable otherwise
 *
 * @return      none
 */
static void
enable_sensor(bool enable)
{
  select_on_bus();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Read temperature and pressure data
 * \param data Pointer to a buffer where temperature and pressure will be
 *             written (6 bytes)
 * \return True if valid data could be retrieved
 */
static bool
read_data(uint8_t *data)
{
  bool success;
  uint16_t temp, pressure;

  temp = SENSOR_VALUE_BMP_TEMP;
  pressure = SENSOR_VALUE_BMP_PRES;

  select_on_bus();

  success = true;

  data[0] = 0;
  data[1] = (temp & 0xff00) >> 8;
  data[2] = (temp & 0xff);

  data[3] = 0;
  data[4] = (pressure & 0xff00) >> 8;
  data[5] = (pressure & 0xff);

  return success;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Convert raw data to values in degrees C (temp) and Pascal (pressure)
 * \param data Pointer to a buffer that holds raw sensor data
 * \param temp Pointer to a variable where the converted temperature will be
 *             written
 * \param press Pointer to a variable where the converted pressure will be
 *              written
 */
static void
convert(uint8_t *data, int32_t *temp, uint32_t *press)
{
  /* Pressure */
  *press = (int32_t)((((uint32_t)(data[0])) << 12)
                     | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));

  /* Temperature */
  *temp = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4)
                    | ((uint32_t)data[5] >> 4));
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type BMP_280_SENSOR_TYPE_TEMP or BMP_280_SENSOR_TYPE_PRESS
 * \return Temperature (centi degrees C) or Pressure (Pascal).
 */
static int
value(int type)
{
  int rv;
  int32_t temp = 0;
  uint32_t pres = 0;

  if(enabled != SENSOR_STATUS_READY) {
    PRINTF("Sensor disabled or starting up (%d)\n", enabled);
    return CC26XX_SENSOR_READING_ERROR;
  }

  if((type != BMP_280_SENSOR_TYPE_TEMP) && type != BMP_280_SENSOR_TYPE_PRESS) {
    PRINTF("Invalid type\n");
    return CC26XX_SENSOR_READING_ERROR;
  } else {
    memset(sensor_value, 0, SENSOR_DATA_BUF_SIZE);

    rv = read_data(sensor_value);

    if(rv == 0) {
      return CC26XX_SENSOR_READING_ERROR;
    }

    PRINTF("val: %02x%02x%02x %02x%02x%02x\n",
           sensor_value[0], sensor_value[1], sensor_value[2],
           sensor_value[3], sensor_value[4], sensor_value[5]);

    convert(sensor_value, &temp, &pres);

    if(type == BMP_280_SENSOR_TYPE_TEMP) {
      rv = (int)temp;
    } else if(type == BMP_280_SENSOR_TYPE_PRESS) {
      rv = (int)pres;
    }
  }
  return rv;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the BMP280 sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
static int
configure(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:
    enabled = SENSOR_STATUS_INITIALISED;
    init();
    enable_sensor(0);
    break;
  case SENSORS_ACTIVE:
    /* Must be initialised first */
    if(enabled == SENSOR_STATUS_DISABLED) {
      return SENSOR_STATUS_DISABLED;
    }
    if(enable) {
      enable_sensor(1);
      ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
      enabled = SENSOR_STATUS_NOT_READY;
    } else {
      ctimer_stop(&startup_timer);
      enable_sensor(0);
      enabled = SENSOR_STATUS_INITIALISED;
    }
    break;
  default:
    break;
  }
  return enabled;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 if the sensor is enabled
 */
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return enabled;
    break;
  default:
    break;
  }
  return SENSOR_STATUS_DISABLED;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(bmp_280_sensor, "BMP280", value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
