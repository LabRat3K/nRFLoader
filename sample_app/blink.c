/**
 * Based on the ...
 * USB 512-Word CDC Bootloader Application Code
 * Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
 * v1.0, February 12, 2015
 * Released under a 3-clause BSD license: see the accompanying LICENSE file.
 *
 * Wireless loader Copyright (c) 2022, Andrew Williams (ratsnest.ca)
 *
 * PIC blinking LED example for SDCC.
 * Blinks an LED connected to pin RC5.
 * Tested with a PIC16F1823.
 *
 * Application code that is compatible with the bootloader has 5 requirements:
 *
 * - a main function named app_main(), NOT main().
 *   Defining main() will confuse the linker. As of version 3.4.0, SDCC forces
 *   code generation at address 0x0000; excluding main() prevents this.
 *
 * - an interrupt handler named app_interrupt(), even if the application does
 *   not use interrupts. (in that case, just use an empty function body.)
 *
 * - a configuration byte defined with the APP_CONFIG() macro.
 *   This macro is provided in nrf_bootloader_config.h.
 *   This specifies a major/minor version number that can be polled by the 
 *   booloader (if time and space exists to do so)
 *
 * - it must be compiled with the accompanying file nrf_bootloader_512.S.
 *
 * - the linker script 16f182x_bootloader_512.lkr must be used.
 */

#include <pic16regs.h>
#include "nrf_bootloader_config.h"

#define VERSION_MAJOR (0)
#define VERSION_MINOR (1)

APP_CONFIG(VERSION_MAJOR,VERSION_MINOR);

static unsigned char blink=0;
void app_interrupt(void)
{
  /* Clear interrupt flag and toggle LED state */
  blink--;

  if (blink==0) {
     LATC ^= 0x20;
     blink |= 0x20;
  }

  PIR1bits.TMR1IF = 0;
}


int app_main(void)
{
  blink = 32;
  /* RC4 is an output */
  TRISCbits.TRISC5 = 0;

  LATCbits.LATC5 =0;

  /* Enable Timer1 with 1:8 prescaler and overflow interrupt */
  T1CON = _T1CKPS1|_T1CKPS0|_TMR1ON;
  PIE1bits.TMR1IE = 1;
  INTCONbits.PEIE = 1;
  INTCONbits.GIE = 1;
  PIR1bits.TMR1IF = 0;

  while (1) {}
}

