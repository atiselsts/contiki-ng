/**
 * \file
 *         Hardware-specific configuration for EXP5438 board
 */

#ifndef EXP5438_DEF_H_
#define EXP5438_DEF_H_

/*
 * Definitions below are dictated by the hardware and not really changeable.
 *
 * Almost all values are the same as for Tmote Sky (e.g. the radio chip is the same: CC2420)
 */

#define PLATFORM_HAS_LEDS    1
#define PLATFORM_HAS_RADIO   1

#include "cc2420-def.h"

#define CC2420_CONF_SYMBOL_LOOP_COUNT 1302      /* 326us msp430X @ 8MHz */

/* Disable TSCH frame filtering */
#define TSCH_CONF_HW_FRAME_FILTERING  0

/* CPU target speed in Hz */
#define F_CPU 8000000uL /* 8MHz by default */

/* LED ports */
#define LEDS_PxDIR P1DIR
#define LEDS_PxOUT P1OUT
#define LEDS_CONF_RED    0x01
#define LEDS_CONF_GREEN  0x02
#define LEDS_CONF_YELLOW 0x02

#define LEDS_CONF_LEGACY_API 1

/* DCO speed resynchronization for more robust UART, etc. */
/* Not needed from MSP430x5xx since it make use of the FLL */
#define DCOSYNCH_CONF_ENABLED 0
#define DCOSYNCH_CONF_PERIOD 30

#define ROM_ERASE_UNIT_SIZE  512
#define XMEM_ERASE_UNIT_SIZE (64*1024L)

#define CFS_CONF_OFFSET_TYPE    long

/* Use the first 64k of external flash for node configuration */
#define NODE_ID_XMEM_OFFSET     (0 * XMEM_ERASE_UNIT_SIZE)

/* Use the second 64k of external flash for codeprop. */
#define EEPROMFS_ADDR_CODEPROP  (1 * XMEM_ERASE_UNIT_SIZE)

#define CFS_XMEM_CONF_OFFSET    (2 * XMEM_ERASE_UNIT_SIZE)
#define CFS_XMEM_CONF_SIZE      (1 * XMEM_ERASE_UNIT_SIZE)

#define CFS_RAM_CONF_SIZE 4096

/*
 * SPI bus configuration for the TMote Sky.
 */

/* SPI input/output registers. */
#define SPI_TXBUF UCB0TXBUF
#define SPI_RXBUF UCB0RXBUF

                                /* USART0 Tx ready? */
#define SPI_WAITFOREOTx() while ((UCB0STAT & UCBUSY) != 0)
                                /* USART0 Rx ready? */
#define SPI_WAITFOREORx() while ((UCB0IFG & UCRXIFG) == 0)
                                /* USART0 Tx buffer ready? */
#define SPI_WAITFORTxREADY() while ((UCB0IFG & UCTXIFG) == 0)

#define MOSI           1  /* P3.1 - Output: SPI Master out - slave in (MOSI) */
#define MISO           2  /* P3.2 - Input:  SPI Master in - slave out (MISO) */
#define SCK            3  /* P3.3 - Output: SPI Serial Clock (SCLK) */


/* Use the LCD? */
#ifndef WITH_LCD
#define WITH_LCD 0
#endif

/* Platform-specific define for the end of the stack region */
#undef STACK_CONF_ORIGIN
#define STACK_CONF_ORIGIN     ((void *)0x5c00)

#endif /* EXP5438_DEF_H_ */
