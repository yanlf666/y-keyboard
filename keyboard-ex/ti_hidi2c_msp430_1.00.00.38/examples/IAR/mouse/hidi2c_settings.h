/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/**
 * @file hidi2c_settings.h
 * @brief User settings for HIDI2C API. Also includes specific interface
 * settings for I2C interfaces
 */

#ifndef _HIDI2C_SETTINGS_H_
#define _HIDI2C_SETTINGS_H_

#define NON_BLOCKING_MODE   0

/* VID/PID/Version */
#define VENDOR_ID   0x2047
#define PRODUCT_ID  0x0401
#define VERSION_ID  0x0100

/* If ACTIVE_LOW is set to 1, the GPIO is active when it is 0 */
#define ACTIVE_LOW  1

/* Interface Settings  */


/**
 *  USCI settings. If the user wants to enable specific USCI modules, they must
 *  enable the USCI defined listed below. If the user does not enable an USCI
 *  module, the code for that module (ISRs, initialization code, etc) is then
 *  removed from the compiler by use of ifdefs
 */

/* USCIB0 */
#define USCIB0           0xB0
#define USCIB0_ADDR      0x48
#define USCIB0_GPIO_POUT P2OUT
#define USCIB0_GPIO_PDIR P2DIR
#define USCIB0_GPIO_PIN  BIT1
#define USCIB0_PORT      P3SEL
#define USCIB0_PINS      (BIT0 + BIT1)

/* USCIB1 */
//#define USCIB1           0xB1
//#define USCIB1_ADDR      0x49
//#define USCIB1_GPIO_POUT P2OUT
//#define USCIB1_GPIO_PDIR P2DIR
//#define USCIB1_GPIO_PIN  BIT1
//#define USCIB1_PORT      P3SEL
//#define USCIB1_PINS      (BIT0 + BIT1)

/* USCIB2 */
//#define USCIB2           0xA0
//#define USCIB2_ADDR      0x50
//#define USCIB2_GPIO_POUT P2OUT
//#define USCIB2_GPIO_PDIR P2DIR
//#define USCIB2_GPIO_PIN  BIT1
//#define USCIB2_PORT      P3SEL
//#define USCIB2_PINS      (BIT0 + BIT1)

/* USCIB3 */
//#define USCIB3           0xA1
//#define USCIB3_ADDR      0x51
//#define USCIB3_GPIO_POUT P2OUT
//#define USCIB3_GPIO_PDIR P2DIR
//#define USCIB3_GPIO_PIN  BIT1
//#define USCIB3_PORT      P3SEL
//#define USCIB3_PINS     (BIT0 + BIT1)
#endif
