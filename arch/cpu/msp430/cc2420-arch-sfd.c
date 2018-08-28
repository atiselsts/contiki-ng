/*
 * Copyright (c) 2009, Swedish Institute of Computer Science
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include "contiki.h"
#include "dev/spi-legacy.h"
#include "cc2420.h"
#include "isr_compat.h"

#include <stdio.h>

extern volatile uint8_t cc2420_sfd_counter;
extern volatile uint16_t cc2420_sfd_start_time;
extern volatile uint16_t cc2420_sfd_end_time;

/*---------------------------------------------------------------------------*/
/* SFD interrupt for timestamping radio packets */
ISR(TIMER0_A0, cc2420_timera1_interrupt)
{
  int ta0iv;

  printf("a0 int\n");

  /* always read TAIV to clear IFG */
  ta0iv = TA0IV;
  /* read and discard ta0iv to avoid "variable set but not used" warning */
  (void)ta0iv;
  if(CC2420_SFD_IS_1) {
    cc2420_sfd_counter++;
    cc2420_sfd_start_time = TA0CCR2;
  } else {
    cc2420_sfd_counter = 0;
    cc2420_sfd_end_time = TA0CCR2;
  }

  TA0CCTL2 &= ~CCIFG;
}
/*---------------------------------------------------------------------------*/
void
cc2420_arch_sfd_init(void)
{
  printf("cc2420_arch_sfd_init\n");

  /* Need to select the special function! */
//  CC2420_SFD_PORT(SEL) |= BV(CC2420_SFD_PIN);
  P1SEL |= (1 << 3); /* as function */
  P1DIR &= ~(1 << 3); /* as input */

  /* start timer B - 32768 ticks per second */
  TA0CTL = TACLR;
  TA0CTL = TASSEL_1;

  /* CM_3 = capture mode - capture on both edges */
  TA0CCTL2 = CM_3 | CAP | SCS;
  TA0CCTL2 |= CCIE;
  
  /* Start Timer_B in continuous mode. */
  TA0CTL |= MC1;

  TA0R = RTIMER_NOW();
}
/*---------------------------------------------------------------------------*/



/* TAxCCTLx Control Bits */
#define CM1                 (0x8000)  /* Capture mode 1 */
#define CM0                 (0x4000)  /* Capture mode 0 */
#define CCIS1               (0x2000)  /* Capture input select 1 */
#define CCIS0               (0x1000)  /* Capture input select 0 */
#define SCS                 (0x0800)  /* Capture sychronize */
#define SCCI                (0x0400)  /* Latched capture signal (read) */
#define CAP                 (0x0100)  /* Capture mode: 1 /Compare mode : 0 */
#define OUTMOD2             (0x0080)  /* Output mode 2 */
#define OUTMOD1             (0x0040)  /* Output mode 1 */
#define OUTMOD0             (0x0020)  /* Output mode 0 */
#define CCIE                (0x0010)  /* Capture/compare interrupt enable */
#define CCI                 (0x0008)  /* Capture input signal (read) */
#define OUT                 (0x0004)  /* PWM Output signal if output mode 0 */
#define COV                 (0x0002)  /* Capture/compare overflow flag */
#define CCIFG               (0x0001)  /* Capture/compare interrupt flag */

#define OUTMOD_0            (0x0000)  /* PWM output mode: 0 - output only */
#define OUTMOD_1            (0x0020)  /* PWM output mode: 1 - set */
#define OUTMOD_2            (0x0040)  /* PWM output mode: 2 - PWM toggle/reset */
#define OUTMOD_3            (0x0060)  /* PWM output mode: 3 - PWM set/reset */
#define OUTMOD_4            (0x0080)  /* PWM output mode: 4 - toggle */
#define OUTMOD_5            (0x00A0)  /* PWM output mode: 5 - Reset */
#define OUTMOD_6            (0x00C0)  /* PWM output mode: 6 - PWM toggle/set */
#define OUTMOD_7            (0x00E0)  /* PWM output mode: 7 - PWM reset/set */
#define CCIS_0              (0x0000) /* Capture input select: 0 - CCIxA */
#define CCIS_1              (0x1000) /* Capture input select: 1 - CCIxB */
#define CCIS_2              (0x2000) /* Capture input select: 2 - GND */
#define CCIS_3              (0x3000) /* Capture input select: 3 - Vcc */
#define CM_0                (0x0000) /* Capture mode: 0 - disabled */
#define CM_1                (0x4000) /* Capture mode: 1 - pos. edge */
#define CM_2                (0x8000) /* Capture mode: 1 - neg. edge */
#define CM_3                (0xC000) /* Capture mode: 1 - both edges */
