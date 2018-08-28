/**
 * \file
 *         Hardware-specific configuration for the CC2420 interface.
 *
 *         Platforms using this ile:
 *         - Tmote Sky
 *         - the  EXP5438 board
 */

#ifndef CC2420_DEF_H_
#define CC2420_DEF_H_

/* Configure radio driver */
#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO   cc2420_driver
#endif /* NETSTACK_CONF_RADIO */

/* Platform-specific (H/W) AES implementation */
#ifndef AES_128_CONF
#define AES_128_CONF cc2420_aes_128_driver
#endif /* AES_128_CONF */

/* Delay between GO signal and SFD: radio fixed delay + 4Bytes preample + 1B SFD -- 1Byte time is 32us
 * ~327us + 129preample = 456 us */
#define RADIO_DELAY_BEFORE_TX ((unsigned)US_TO_RTIMERTICKS(456))
/* Delay between GO signal and start listening
 * ~50us delay + 129preample + ?? = 183 us */
#define RADIO_DELAY_BEFORE_RX ((unsigned)US_TO_RTIMERTICKS(183))
/* Delay between the SFD finishes arriving and it is detected in software */
#define RADIO_DELAY_BEFORE_DETECT 0


#ifndef CC2420_CONF_SFD_TIMESTAMPS
#define CC2420_CONF_SFD_TIMESTAMPS (MAC_CONF_WITH_TSCH) /* TSCH needs SFD timestamping */
#endif /* CC2420_CONF_SFD_TIMESTAMPS */


/*
 * SPI bus - CC2420 pin configuration.
 */

/* P1.2 - Input: FIFOP from CC2420 */
#define CC2420_FIFOP_PORT(type)    P1##type
#define CC2420_FIFOP_PIN           6
/* P1.3 - Input: FIFO from CC2420 */
#define CC2420_FIFO_PORT(type)     P1##type
#define CC2420_FIFO_PIN            5
/* P1.4 - Input: CCA from CC2420 */
#define CC2420_CCA_PORT(type)      P1##type
#define CC2420_CCA_PIN             7
/* P4.1 - Input:  SFD from CC2420 */
#define CC2420_SFD_PORT(type)      P1##type
#define CC2420_SFD_PIN             3
 /* P3.0 - Output: SPI Chip Select (CS_N) */
#define CC2420_CSN_PORT(type)      P3##type
#define CC2420_CSN_PIN             0
/* P4.5 - Output: VREG_EN to CC2420 */
#define CC2420_VREG_PORT(type)     P1##type
#define CC2420_VREG_PIN            4
/* P4.6 - Output: RESET_N to CC2420 */
#define CC2420_RESET_PORT(type)    P1##type
#define CC2420_RESET_PIN           2


#define CC2420_IRQ_VECTOR PORT1_VECTOR

/* Pin status. */
#define CC2420_FIFOP_IS_1 (!!(CC2420_FIFOP_PORT(IN) & BV(CC2420_FIFOP_PIN)))
#define CC2420_FIFO_IS_1  (!!(CC2420_FIFO_PORT(IN) & BV(CC2420_FIFO_PIN)))
#define CC2420_CCA_IS_1   (!!(CC2420_CCA_PORT(IN) & BV(CC2420_CCA_PIN)))
#define CC2420_SFD_IS_1   (!!(CC2420_SFD_PORT(IN) & BV(CC2420_SFD_PIN)))

/* The CC2420 reset pin. */
#define SET_RESET_INACTIVE()   (CC2420_RESET_PORT(OUT) |=  BV(CC2420_RESET_PIN))
#define SET_RESET_ACTIVE()     (CC2420_RESET_PORT(OUT) &= ~BV(CC2420_RESET_PIN))

/* CC2420 voltage regulator enable pin. */
#define SET_VREG_ACTIVE()       (CC2420_VREG_PORT(OUT) |=  BV(CC2420_VREG_PIN))
#define SET_VREG_INACTIVE()     (CC2420_VREG_PORT(OUT) &= ~BV(CC2420_VREG_PIN))

/* CC2420 rising edge trigger for external interrupt (FIFOP). */
#define CC2420_FIFOP_INT_INIT() do {                  \
    CC2420_FIFOP_PORT(IES) &= ~BV(CC2420_FIFOP_PIN);  \
    CC2420_CLEAR_FIFOP_INT();                         \
  } while(0)

/* FIFOP on external interrupt 0. */
#define CC2420_ENABLE_FIFOP_INT()  do {CC2420_FIFOP_PORT(IE) |= BV(CC2420_FIFOP_PIN);} while(0)
#define CC2420_DISABLE_FIFOP_INT() do {CC2420_FIFOP_PORT(IE) &= ~BV(CC2420_FIFOP_PIN);} while(0)
#define CC2420_CLEAR_FIFOP_INT()   do {CC2420_FIFOP_PORT(IFG) &= ~BV(CC2420_FIFOP_PIN);} while(0)


/*
 * Enables/disables CC2420 access to the SPI bus (not the bus).
 * (Chip Select)
 */

 /* ENABLE CSn (active low) */
#define CC2420_SPI_ENABLE()     (CC2420_CSN_PORT(OUT) &= ~BV(CC2420_CSN_PIN))
 /* DISABLE CSn (active low) */
#define CC2420_SPI_DISABLE()    (CC2420_CSN_PORT(OUT) |=  BV(CC2420_CSN_PIN))
#define CC2420_SPI_IS_ENABLED() ((CC2420_CSN_PORT(OUT) & BV(CC2420_CSN_PIN)) != BV(CC2420_CSN_PIN))

#endif /* CC2420_DEF_H_ */
