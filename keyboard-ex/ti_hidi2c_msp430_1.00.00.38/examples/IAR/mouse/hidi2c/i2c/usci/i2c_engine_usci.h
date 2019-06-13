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
/*----------------------------------------------------------------------------+
 |                                                                             |
 |                              Texas Instruments                              |
 |                                                                             |
 |                          I2C Engine (HIDI2C)                                |
 |                                                                             |
 +----------------------------------------------------------------------------*/
#ifndef _I2C_ENGINE_H_
#define _I2C_ENGINE_H_

#include <hidi2c_types.h>

/* Functions */
BOOL hidI2C_initializeI2C(BOOL idle);
uint8_t hidI2C_setDataAndInitializeInterrupt(uint8_t interface, uint8_t* data,
        uint16_t length);
uint8_t hidI2C_setDataWithoutInterrupt(uint8_t interface, uint8_t* data,
        uint16_t length);
uint8_t hidI2C_setDataChangeRegWithoutInterrupt(uint8_t interface, uint8_t* data,
        uint16_t length, uint16_t registerIndex);
BOOL hidI2C_isGPIOAsserted(uint8_t interface);

/* Interrupt Handling */
void hidI2C_disableInterrupts(uint8_t interface);
void hidI2C_enableInterrupts(uint8_t interface);
BOOL hidI2C_busyWithTransfer(uint8_t interface);

/* Bit Swap */
uint16_t bitSwap(uint8_t lsb, uint8_t msb);

#endif

