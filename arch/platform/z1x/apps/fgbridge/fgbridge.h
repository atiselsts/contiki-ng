/*****************************************************************************************************

 Copyright (C) - All Rights Reserved

 SPHERE (an EPSRC IRC), 2013-2018
 University of Bristol
 University of Reading
 University of Southampton

 Filename: fgbridge.c
 Description: Header file for fgbridge.h
 Primary Contributor(s): Xenofon (Fontas) Fafoutis (xenofon.fafoutis@bristol.ac.uk)

*******************************************************************************************************/

#ifndef FGBRIDGE_H_
#define FGBRIDGE_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define FGBRIDGE_IOID_SPI_MISO      IOID_9
#define FGBRIDGE_IOID_SPI_MOSI      IOID_8
#define FGBRIDGE_IOID_SPI_CLK       IOID_10
#define FGBRIDGE_IOID_G2_INT        IOID_12
#define FGBRIDGE_IOID_SPI_CS        IOID_11

#define FGBRIDGE_SPI_CS             (1 << FGBRIDGE_IOID_SPI_CS)
#define FGBRIDGE_G2_INT             (1 << FGBRIDGE_IOID_G2_INT)

#define FGBRIDGE_SPI_FRF            SSI_FRF_MOTO_MODE_3 // polarity 1, phase 1
#define FGBRIDGE_SPI_RATE           4000000

#define FGBRIDGE_CS_DELAY           135

#define FGBRIDGE_SPI_HEADER_LENGTH  10
#define FGBRIDGE_SPI_PDU_LENGTH     40
#define FGBRIDGE_SPI_FRAME_LENGTH   (FGBRIDGE_SPI_HEADER_LENGTH + FGBRIDGE_SPI_PDU_LENGTH)
#define FGBRIDGE_SPI_ADV_DATA_LEN   18

#define FGBRIDGE_BLE_MAX_LENGTH     31
#define FGBRIDGE_BLE_ADDR_LENGTH    6

typedef enum
{
  FGBRIDGE_UL_ADV_MESSAGE = 1,
  FGBRIDGE_UL_SCAN_RESPONSE_MESSAGE,
  FGBRIDGE_UL_CONNECTION_REQUEST_MESSAGE,
  FGBRIDGE_UL_CONNECTION_SUCCESSFUL_MESSAGE,
  FGBRIDGE_UL_CONNECTION_ATTEMPT_FAILED_MESSAGE,
  FGBRIDGE_UL_NOTIFICATION_MESSAGE,
  FGBRIDGE_UL_TAG_DISCONNECTED_MESSAGE,
  FGBRIDGE_UL_UNABLE_TO_DO_WRITE_MESSAGE,
  FGBRIDGE_UL_GIVEN_TAG_NOT_CONNECTED_MESSAGE,
  FGBRIDGE_UL_SUCCESSFUL_WRITE_ACK_MESSAGE,
  FGBRIDGE_UL_UNSUCCESSFUL_WRITE_ACK_MESSAGE,
  FGBRIDGE_UL_RETURN_LIST_OF_CONNECTED_MACS_MESSAGE,
  FGBRIDGE_UL_RETURN_RSSI_OF_CONNECTED_TAG_MESSAGE,
  FGBRIDGE_UL_RSSI_OF_CONNECTED_TAG_STOPPED_MESSAGE,
  FGBRIDGE_UL_RSSI_UNABLE_TO_START_MESSAGE,
  FGBRIDGE_UL_FORCED_DISCONNECT_DUE_TO_TIMEOUT_MESSAGE
} FGBRIDGE_UL_FRAME_TYPE; /* SPI uplink commands (G2 to G1) */

typedef enum
{
  FGBRIDGE_DL_WRITE_TO_TAG_MESSAGE = 80,
  FGBRIDGE_DL_ENQUIRE_LIST_OF_CONNECTED_MACS_MESSAGE,
  FGBRIDGE_DL_START_RSSI_OF_CONNECTED_TAG_MESSAGE,
  FGBRIDGE_DL_STOP_RSSI_OF_CONNECTED_TAG_MESSAGE,
  FGBRIDGE_DL_CONNECTION_AUTHORISED_MESSAGE,
  FGBRIDGE_DL_ENABLE_DATA_CCCD_MESSAGE,
  FGBRIDGE_DL_DISSABLE_DATA_CCCD_MESSAGE,
  FGBRIDGE_DL_ENABLE_BATTERY_CCCD_MESSAGE,
  FGBRIDGE_DL_DISSABLE_BATTERY_CCCD_MESSAGE
} FGBRIDGE_DL_FRAME_TYPE; /* SPI downlink commands (G1 to G2) */


typedef struct {
  uint8_t byte_AA;                                    // Should always be 0xAA
  uint8_t byte_55;                                    // Should always be 0x55
  uint8_t ul_count;                                   // Frame counter
  uint8_t command;                                    // Command
  uint8_t G_ble_addr[FGBRIDGE_BLE_ADDR_LENGTH];       // BLE Address of G (little-endian)
  uint8_t W_ble_addr[FGBRIDGE_BLE_ADDR_LENGTH];       // BLE Address of Wearable (little-endian)
  uint8_t length;                                     // Frame Length
  FGBRIDGE_UL_FRAME_TYPE type;                        // Frame Type
  uint8_t payload[FGBRIDGE_BLE_MAX_LENGTH];           // Frame Payload
  int8_t rssi;                                        // RSSI
} fgbridge_rcv_frame_t;

typedef struct {
  uint8_t header_length;                              // Header Length (should be 2 including flags)
  uint8_t gap_adtype;                                 // BLE GAP Advertisement Data Types
  uint8_t gap_mode;                                   // BLE GAP Flags Discovery Modes
  uint8_t uuid_length;                                // UUID Length (should be 3 including flags)
  uint8_t uuid_flags;                                 // UUID Flags
  uint8_t uuid_lo;                                    // UUID Lower Byte
  uint8_t uuid_hi;                                    // UUID Higher Byte
  uint8_t monitor[4];                                 // Monitoring Data
  uint8_t mc;                                         // ADV Counter (LSB)
  uint8_t data[FGBRIDGE_SPI_ADV_DATA_LEN];            // Acceleration Data
  uint8_t mc_msb;	                                    // ADV Counter (MSB)
} fgbridge_rcv_frame_adv_t;

typedef struct {
  uint8_t wid;                                        // Wearable ID (the last byte of BLE addr)
  uint8_t mc[2];                                      // ADV Counter (LSB)
  int8_t rssi;                                        // RSSI
  uint8_t data[FGBRIDGE_SPI_ADV_DATA_LEN];            // Acceleration Data
  uint8_t timestamp[4];                               // Absolute Sequence Number of TSCH
  uint8_t monitor[4];				                          // Monitoring Data
} wearable_adv_t;

/*---------------------------------------------------------------------------*/

typedef void fgbridge_callback_function(void *resource);

bool fgbridge_init(fgbridge_callback_function *, void *);

void *fgbridge_adv_read(uint16_t *length /* out */);

/*---------------------------------------------------------------------------*/
#endif
