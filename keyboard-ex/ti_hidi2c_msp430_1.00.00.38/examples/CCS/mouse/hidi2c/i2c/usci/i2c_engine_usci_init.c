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
 * @file i2c_engine_usci_init.c
 *
 * Contains all of the initialization functions and structures for the I2C
 * layer of the HIDI2C stack. ifdefs are used to minimize the memory footprint
 * for users not using multiple devices. Specific USCI settings are defined in
 * the @link hidi2c_settings.h @endlink file.
 *
 */
#include <hidi2c_types.h>
#include <i2c_engine_usci_init.h>
#include <hidi2c_settings.h>
#include <msp430.h>


/* Specific USCI initialization functions  */
#ifdef USCIB0
BOOL initializeUSCIB0(BOOL idle);
#endif

#ifdef USCIB1
BOOL initializeUSCIB1(BOOL idle);
#endif

#ifdef USCIB2
BOOL initializeUSCIB2(BOOL idle);
#endif

#ifdef USCIB3
BOOL initializeUSCIB3(BOOL idle);
#endif

BOOL hidI2C_initializeUSCII2C(BOOL idle)
{
#ifdef USCIB0
    initializeUSCIB0(idle);
#endif

#ifdef USCIB1
    initializeUSCIB1(idle);
#endif

#ifdef USCIB2
    initializeUSCIB2(idle);
#endif

#ifdef USCIB3
    initializeUSCIB3(idle);
#endif

    return TRUE;
}

/**
 * Sets the GPIO pin for the given interface in idle state. Accounts for the
 * ACTIVE_LOW setting defined in @link hidi2c_settings.h @endlink file.
 *
 * @param interface Specific interface to set the GPIO pin for as defined in the
 * @link hidi2c_settings.h @endlink file.
 */
void hidI2C_setGPIOPinIdle(uint8_t interface)
{
    switch (interface)
    {
#ifdef USCIB0
    case USCIB0:
            USCIB0_GPIO_PDIR &= ~USCIB0_GPIO_PIN;
            break;
#endif

#ifdef USCIB1
        case USCIB1:
            USCIB1_GPIO_PDIR &= ~USCIB1_GPIO_PIN;
        break;
#endif

#ifdef USCIB2
        case USCIB2:
            USCIB2_GPIO_PDIR &= ~USCIB2_GPIO_PIN;
        break;
#endif

#ifdef USCIB3
        case USCIB3:
            USCIB3_GPIO_PDIR &= ~USCIB3_GPIO_PIN;
        break;
#endif

    default:
        break;
    }
}

/**
 * Sets the GPIO pin for the given interface in active state. Accounts for the
 * ACTIVE_LOW setting defined in @link hidi2c_settings.h @endlink file.
 *
 * @param interface Specific interface to set the GPIO pin for as defined in the
 * @link hidi2c_settings.h @endlink file.
 */
void hidI2C_setGPIOPinActive(uint8_t interface)
{
    switch (interface)
    {
#ifdef USCIB0
    case USCIB0:
            USCIB0_GPIO_PDIR |= (USCIB0_GPIO_PIN);
            break;
#endif

#ifdef USCIB1
        case USCIB1:
            USCIB1_GPIO_PDIR |= (USCIB1_GPIO_PIN);
            break;
#endif

#ifdef USCIB2
        case USCIB2:
            USCIB2_GPIO_PDIR |= (USCIB2_GPIO_PIN);
            break;
#endif

#ifdef USCIB3
        case USCIB3:
            USCIB3_GPIO_PDIR |= (USCIB3_GPIO_PIN);
            break;
#endif

    default:
        break;
    }
}

/** Specific USCI module initialization functions */
#ifdef USCIB0
BOOL initializeUSCIB0(BOOL idle)
{
    USCIB0_PORT |= USCIB0_PINS; // Assign I2C pins to USCI_B0
    UCB0CTL1 |= UCSWRST; // Enable SW reset
    UCB0CTL0 = UCMODE_3 + UCSYNC; // I2C Slave, synchronous mode
    UCB0I2COA = USCIB0_ADDR; // Own Address

    if(!idle)
        UCB0CTL1 &= ~UCSWRST; // Clear SW reset, resume operation

    USCIB0_GPIO_POUT &= ~USCIB0_GPIO_PIN;
    USCIB0_GPIO_PDIR &= ~USCIB0_GPIO_PIN;

    return TRUE;
}
#endif

#ifdef USCIB1
BOOL initializeUSCIB1(BOOL idle)
{
    USCIB1_PORT |= USCIB1_PINS; // Assign I2C pins to USCI_B1
    UCB1CTL1 |= UCSWRST;// Enable SW reset
    UCB1CTL0 = UCMODE_3 + UCSYNC;// I2C Slave, synchronous mode
    UCB1I2COA = USCIB1_ADDR;// Own Address

    if(!idle)
        UCB1CTL1 &= ~UCSWRST;// Clear SW reset, resume operation

    USCIB1_GPIO_POUT &= ~USCIB1_GPIO_PIN;
    USCIB1_GPIO_PDIR &= ~USCIB1_GPIO_PIN;

    return TRUE;
}
#endif

#ifdef USCIB2
BOOL initializeUSCIB2(BOOL idle)
{
    USCIB2_PORT |= USCIB2_PINS; // Assign I2C pins to USCI_B2
    UCB2CTL1 |= UCSWRST;// Enable SW reset
    UCB2CTL0 = UCMODE_3 + UCSYNC;// I2C Slave, synchronous mode
    UCB2I2COA = USCIB2_ADDR;// Own Address

    if(!idle)
        UCB2CTL1 &= ~UCSWRST;// Clear SW reset, resume operation

    // Set as input, and use external
    USCIB2_GPIO_POUT &= ~USCIB2_GPIO_PIN;
    USCIB2_GPIO_PDIR &= ~USCIB2_GPIO_PIN;

    return TRUE;
}
#endif

#ifdef USCIB3
BOOL initializeUSCIB3(BOOL idle)
{
    USCIB3_PORT |= USCIB3_PINS; // Assign I2C pins to USCI_B3
    UCB3CTL1 |= UCSWRST;// Enable SW reset
    UCB3CTL0 = UCMODE_3 + UCSYNC;// I2C Slave, synchronous mode
    UCB3I2COA = USCIB3_ADDR;// Own Address

    if(!idle)
        UCB3CTL1 &= ~UCSWRST;// Clear SW reset, resume operation

    USCIB3_GPIO_POUT &= ~USCIB3_GPIO_PIN;
    USCIB3_GPIO_PDIR &= ~USCIB3_GPIO_PIN;

    return TRUE;
}
#endif

