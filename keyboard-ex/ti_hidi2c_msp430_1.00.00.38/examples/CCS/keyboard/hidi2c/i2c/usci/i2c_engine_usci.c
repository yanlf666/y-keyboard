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
 * @file i2c_engine_usci.c
 * @brief I2C (USCI Implementation) for HIDI2C Specification
 * I2C engine for the HIDI2C implementation. 
 * @details This is the transport layer of the API. In this implementation, USCI 
 * is used for management of the I2C. This layer is abstracted out so another 
 * implementation of I2C can be used. 
 *
 * GPIO ports and specific interface settings are defined in the @link
 * hidi2c_settings.h @endlink file.
 *
 */
#include <msp430.h>

#include <hidi2c_settings.h>
#include <hidi2c_errno.h>
#include "hid_engine.h"

/* Interface Settings */
#include <i2c_engine_usci.h>
#include <i2c_engine_usci_init.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Local Variables */
static volatile BOOL clearLPMBits;

/* External call to HID layer */
extern BOOL hidI2C_handleData(uint8_t interface, uint8_t* data, uint16_t reg);
extern void hidI2C_handleRegisterPointer(uint8_t interface,
        uint16_t currentRegisterIndex);
extern void hidI2C_setNextQueuedData(uint8_t inf);
extern BOOL hidI2C_haveMoreGPIOsPending(uint8_t inf);

#ifdef USCIB0
uint16_t usciB0BytesToXfer;
uint8_t* usciB0CurDataPointer;
uint16_t usciB0xferIndex;
uint8_t usciB0RegisterPointer[2] =
{ 0, 0 };
uint16_t usciB0CurrentRegister;
BOOL usciB0DeviceInitiated;
BOOL usciB0startedOnce;
#endif

#ifdef USCIB1
uint16_t usciB1BytesToXfer;
uint8_t* usciB1CurDataPointer;
uint16_t usciB1xferIndex;
uint8_t usciB1RegisterPointer[2] =
{ 0, 0 };
uint16_t usciB1CurrentRegister;
BOOL usciB1DeviceInitiated;
BOOL usciB1startedOnce;
#endif

#ifdef USCIB2
uint16_t usciB2BytesToXfer;
uint8_t* usciB2CurDataPointer;
uint16_t usciB2xferIndex;
uint8_t usciB2RegisterPointer[2] =
{ 0, 0 };
uint16_t usciB2CurrentRegister;
BOOL usciB2DeviceInitiated;
BOOL usciB2startedOnce;
#endif

#ifdef USCIB3
uint16_t usciB3BytesToXfer;
uint8_t* usciB3CurDataPointer;
uint16_t usciB3xferIndex;
uint8_t usciB3RegisterPointer[2] =
{ 0, 0 };
uint16_t usciB3CurrentRegister;
BOOL usciB3DeviceInitiated;
BOOL usciB3startedOnce;
#endif

/**
 * Initializes the I2C physical layer of the HIDI2C API. ifdefs are used to
 * minimize the memory footprint if the user is not using every interface.
 *
 * @return TRUE if initialization succeeded. FALSE otherwise.
 */
BOOL hidI2C_initializeI2C(BOOL idle)
{
    /* Setting up I2C */
    hidI2C_initializeUSCII2C(idle);

#ifdef USCIB0
    UCB0IE |= UCRXIE + UCTXIE + UCSTTIE; // Enable interrupts
    usciB0BytesToXfer = 0;
    usciB0CurDataPointer = 0;
    usciB0startedOnce = FALSE;
    usciB0DeviceInitiated = FALSE;
    usciB0xferIndex = 0;
    usciB0CurrentRegister = 0;
#endif

#ifdef USCIB1
    UCB1IE |= UCRXIE + UCTXIE + UCSTTIE; // Enable interrupts
    usciB1BytesToXfer = 0;
    usciB1CurDataPointer = 0;
    usciB1startedOnce = FALSE;
    usciB1DeviceInitiated = FALSE;
    usciB1xferIndex = 0;
    usciB1CurrentRegister = 0;
#endif

#ifdef USCIB2
    UCB2IE |= UCRXIE + UCTXIE + UCSTTIE; // Enable interrupts
    usciB2BytesToXfer = 0;
    usciB2CurDataPointer = 0;
    usciB2startedOnce = FALSE;
    usciB2DeviceInitiated = FALSE;
    usciB2xferIndex = 0;
    usciB2CurrentRegister = 0;
#endif

#ifdef USCIB3
    UCB3IE |= UCRXIE + UCTXIE + UCSTTIE; // Enable interrupts
    usciB3BytesToXfer = 0;
    usciB3CurDataPointer = 0;
    usciB3startedOnce = FALSE;
    usciB3DeviceInitiated = FALSE;
    usciB3xferIndex = 0;
    usciB3CurrentRegister = 0;
#endif

    return TRUE;
}

/**
 * Internal API command to transfer data from the host and initialize interrupt.
 * Checking for conflicts concurrent accesses is done in the HID layer.
 *
 * @param interface Specific USCI module to send data over. Defined in the
 *  @link hidi2c_settings.h @endlink file.
 * @param data Pointer to data to "queue" for transfer.
 */
uint8_t hidI2C_setDataAndInitializeInterrupt(uint8_t interface, uint8_t* data,
        uint16_t length)
{
    switch (interface)
    {
#ifdef USCIB0
        case USCIB0:
            usciB0CurDataPointer = data;
            usciB0xferIndex = 0;
            usciB0BytesToXfer = length;
            break;
#endif

#ifdef USCIB1
        case USCIB1:
            usciB1CurDataPointer = data;
            usciB1xferIndex = 0;
            usciB1BytesToXfer = length;
            break;
#endif

#ifdef USCIB2
        case USCIB2:
            usciB2CurDataPointer = data;
            usciB2xferIndex = 0;
            usciB2BytesToXfer = length;
            break;
#endif

#ifdef USCIB3
        case USCIB3:
            usciB3CurDataPointer = data;
            usciB3xferIndex = 0;
            usciB3BytesToXfer = length;
            break;
#endif
    }

    hidI2C_setGPIOPinActive(interface);

    return HIDI2C_SUCCESS;
}

/**
 * Function that sets the current data pointer without changing the register
 * value.
 */
uint8_t hidI2C_setDataWithoutInterrupt(uint8_t interface, uint8_t* data,
        uint16_t length)
{
    return hidI2C_setDataChangeRegWithoutInterrupt(interface, data, length, 0);
}

/**
 * Internal API command to transfer data from the host without interrupt.
 * @param interface Specific USCI module to send data over. Defined in the
 *  @link hidi2c_settings.h @endlink file.
 * @param data Pointer to data to "queue" for transfer.
 */
uint8_t hidI2C_setDataChangeRegWithoutInterrupt(uint8_t interface,
        uint8_t* data, uint16_t length, uint16_t registerIndex)
{
    switch (interface)
    {
#ifdef USCIB0
        case USCIB0:
            usciB0CurDataPointer = data;
            usciB0BytesToXfer = length;
            usciB0xferIndex = 0;

            if (registerIndex != 0)
                usciB0CurrentRegister = registerIndex;

            break;
#endif

#ifdef USCIB1
        case USCIB1:
            usciB1CurDataPointer = data;
            usciB1BytesToXfer = length;
            usciB1xferIndex = 0;

            if (registerIndex != 0)
                usciB1CurrentRegister = registerIndex;

            break;
#endif

#ifdef USCIB2
        case USCIB2:
            usciB2CurDataPointer = data;
            usciB2BytesToXfer = length;
            usciB2xferIndex = 0;

            if (registerIndex != 0)
                usciB2CurrentRegister = registerIndex;

            break;
#endif

#ifdef USCIB3
        case USCIB3:
            usciB3CurDataPointer = data;
            usciB3BytesToXfer = length;
            usciB3xferIndex = 0;

            if (registerIndex != 0)
                usciB3CurrentRegister = registerIndex;

            break;
#endif
    }

    return HIDI2C_SUCCESS;
}

/**
 * Returns true if the GPIO is asserted for the given interface.
 */
BOOL hidI2C_isGPIOAsserted(uint8_t interface)
{
    switch (interface)
    {
#ifdef USCIB0
        case USCIB0:
            return (USCIB0_GPIO_PDIR & USCIB0_GPIO_PIN);
#endif

#ifdef USCIB1
        case USCIB1:
            return (USCIB1_GPIO_PDIR & USCIB1_GPIO_PIN);
#endif

#ifdef USCIB2
        case USCIB2:
            return (USCIB2_GPIO_PDIR & USCIB2_GPIO_PIN);
#endif

#ifdef USCIB3
        case USCIB3:
            return (USCIB3_GPIO_PDIR & USCIB3_GPIO_PIN);
#endif
    }

    return FALSE;
}

/**
 * Function that returns true if there is a transfer in progress on given
 * interface
 * @param interface Specific USCI module to send data over. Defined in the
 *  @link hidi2c_settings.h @endlink file.
 *
 * @return TRUE if a transfer is in progress, false otherwise.
 */
BOOL hidI2C_busyWithTransfer(uint8_t interface)
{
    switch (interface)
    {
#ifdef USCIB0
        case USCIB0:
            return (UCB0STAT & UCBUSY);
#endif

#ifdef USCIB1
        case USCIB1:
            return (UCB1STAT & UCBUSY);
#endif

#ifdef USCIB2
        case USCIB2:
            return (UCB2STAT & UCBUSY);
#endif

#ifdef USCIB3
        case USCIB3:
            return (UCB3STAT & UCBUSY);
#endif
    }

    return FALSE;
}

/**
 * Disables START interrupts from happening on given interface.
 * @param interface Specific USCI module to send data over. Defined in the
 *  @link hidi2c_settings.h @endlink file.
 */
void hidI2C_disableInterrupts(uint8_t interface)
{
    switch (interface)
    {
#ifdef USCIB0
        case USCIB0:
            UCB0IE &= ~(UCSTTIE);
            break;
#endif

#ifdef USCIB1
        case USCIB1:
            UCB1IE &= ~(UCSTTIE);
            break;
#endif

#ifdef USCIB2
        case USCIB2:
            UCB2IE &= ~(UCSTTIE);
            break;
#endif

#ifdef USCIB3
        case USCIB3:
            UCB3IE &= ~(UCSTTIE);
            break;
#endif
    }
}

/**
 * Allows START interrupts to happen on given interface
 * @param interface Specific USCI module to send data over. Defined in the
 *  @link hidi2c_settings.h @endlink file.
 */
void hidI2C_enableInterrupts(uint8_t interface)
{
    switch (interface)
    {
#ifdef USCIB0
        case USCIB0:
            UCB0IE |= (UCSTTIE);
            break;
#endif

#ifdef USCIB1
        case USCIB1:
            UCB1IE |= (UCSTTIE);
            break;
#endif

#ifdef USCIB2
        case USCIB2:
            UCB2IE |= (UCSTTIE);
            break;
#endif

#ifdef USCIB3
        case USCIB3:
            UCB3IE |= (UCSTTIE);
            break;
#endif
    }
}

/**
 * Simple function to take two bytes and combine them into one word. This is
 * useful for HIDI2C because everything comes in from the host MSB first
 * @param msb Most significant byte of data 
 * @param lsb Least significant byte of data
 */
uint16_t bitSwap(uint8_t msb, uint8_t lsb)
{
    uint16_t res = 0;
    uint16_t tmpBuffer = 0;

    res = lsb;
    tmpBuffer = msb;
    tmpBuffer = tmpBuffer << 8;
    res = res | tmpBuffer;

    return res;
}

/*
 * USCI_B0 Data ISR
 */
#ifdef USCIB0
#ifdef __GNUC__
__attribute__((interrupt(USCI_B0_VECTOR)))
#else
#pragma vector=USCI_B0_VECTOR
__interrupt
#endif
void USCI_B0_ISR(void)
{
    switch (UCB0IV)
    {
        case 0x06: /* Start */
        {
            usciB0xferIndex = 0;

            /* Disabling start interrupts for the time being */
            hidI2C_disableInterrupts(USCIB0);

            /* First Start Condition */
            if (!usciB0startedOnce)
            {
                usciB0BytesToXfer = 0;

                /* Initial RXing */
                if (!(UCB0CTL1 & UCTR))
                {
                    usciB0BytesToXfer = 2;
                    usciB0CurDataPointer = usciB0RegisterPointer;
                }
                /* Initial TXing */
                else
                {
                    hidI2C_setNextQueuedData(USCIB0);
                    usciB0DeviceInitiated = TRUE;
                    usciB0xferIndex = 0;
                    UCB0IE |= (UCSTPIE);
                }

                usciB0startedOnce = TRUE;
            }

#ifdef UCS7WORKAROUND
            __delay_cycles(HIDI2CUCS7DELAY);
#endif

            break;
        }
        case 0x08: /* Stop */
        {
            if (!hidI2C_haveMoreGPIOsPending(USCIB0))
            {
                hidI2C_setGPIOPinIdle(USCIB0);
            }

            UCB0IE &= ~(UCSTPIE);
            break;

        }
        case 0x0A: /* Host -> Me */
        {

#ifdef USCI30WORKAROUND
            /* ERATTA USCI30 */
            uint8_t bLastStatus, bCount;
            
            bLastStatus = UCSCLLOW & UCB0STAT;
            bCount = 0;
        
            while (bCount < 12)
            {
                if(bLastStatus != (UCSCLLOW & UCB0STAT))
                {
                    bLastStatus = UCSCLLOW & UCB0STAT;
                    bCount = 0;
                }
                else
                {
                    //(delay is 1 quarter of bit clock)
                    __delay_cycles(HIDI2CUSCI30DELAY);
                    bLastStatus = UCSCLLOW & UCB0STAT;
                    bCount++;
                }
            }
#endif

            usciB0CurDataPointer[usciB0xferIndex] = UCB0RXBUF;
            usciB0xferIndex++;

            /* This is the last byte */
            if (usciB0xferIndex == usciB0BytesToXfer)
            {
                /* First two bytes */
                if (usciB0CurDataPointer == usciB0RegisterPointer)
                {
                    usciB0CurrentRegister = bitSwap(usciB0RegisterPointer[1],
                            usciB0RegisterPointer[0]);
                    hidI2C_handleRegisterPointer(USCIB0, usciB0CurrentRegister);
                }
                /* Something else */
                else
                {
                    usciB0BytesToXfer = 0;

                    clearLPMBits = hidI2C_handleData(USCIB0,
                            usciB0CurDataPointer, usciB0CurrentRegister);

                    if (usciB0BytesToXfer == 0)
                    {
                        usciB0startedOnce = FALSE;
                        hidI2C_enableInterrupts(USCIB0);
                    }
                }
            }
            break;
        }
        case 0x0C: /* Me -> Host */
        {
            UCB0TXBUF = usciB0CurDataPointer[usciB0xferIndex];
            usciB0xferIndex++;

            /* Last Byte to TX */
            if (usciB0xferIndex == usciB0BytesToXfer)
            {
                if (usciB0DeviceInitiated)
                {
                    usciB0DeviceInitiated = FALSE;
                }

                usciB0startedOnce = FALSE;
                hidI2C_enableInterrupts(USCIB0);
            }

            break;
        }
    }

    /* Clearing LPM bits if we need to */
    if (clearLPMBits)
    {
        clearLPMBits = FALSE;
        __bic_SR_register_on_exit(LPM4_bits);
        return;
    }

#ifdef UCS7WORKAROUND
    if (usciB0startedOnce)
    {
        __bic_SR_register_on_exit(LPM4_bits & ~LPM0_bits);
    }
    else
    {
        __bis_SR_register_on_exit(UCS7WORKAROUNDLPM);
    }
#endif

}
#endif

/*
 * USCI_B1 Data ISR
 */
#ifdef USCIB1
#ifdef __GNUC__
__attribute__((interrupt(USCI_B1_VECTOR)))
#else
#pragma vector=USCI_B1_VECTOR
__interrupt
#endif
void USCI_B1_ISR(void)
{
    switch (UCB1IV)
    {
        case 0x06: /* Start */
        {
            usciB1xferIndex = 0;

            /* Disabling start interrupts for the time being */
            hidI2C_disableInterrupts(USCIB1);

            /* First Start Condition */
            if (!usciB1startedOnce)
            {
                usciB1BytesToXfer = 0;

                /* Initial RXing */
                if (!(UCB1CTL1 & UCTR))
                {
                    usciB1BytesToXfer = 2;
                    usciB1CurDataPointer = usciB1RegisterPointer;
                }
                /* Initial TXing */
                else
                {
                    hidI2C_setNextQueuedData(USCIB1);
                    usciB1DeviceInitiated = TRUE;
                    usciB1xferIndex = 0;
                    UCB1IE |= (UCSTPIE);
                }

                usciB1startedOnce = TRUE;
            }

#ifdef UCS7WORKAROUND
            __delay_cycles(HIDI2CUCS7DELAY);
#endif

            break;
        }
        case 0x08: /* Stop */
        {
            if (!hidI2C_haveMoreGPIOsPending(USCIB1))
            {
                hidI2C_setGPIOPinIdle(USCIB1);
            }

            UCB1IE &= ~(UCSTPIE);
            break;

        }
        case 0x0A: /* Host -> Me */
        {

#ifdef USCI30WORKAROUND
            /* ERATTA USCI30 */
            uint8_t bLastStatus, bCount;
            
            bLastStatus = UCSCLLOW & UCB1STAT;
            bCount = 0;
        
            while (bCount < 12)
            {
                if(bLastStatus != (UCSCLLOW & UCB1STAT))
                {
                    bLastStatus = UCSCLLOW & UCB1STAT;
                    bCount = 0;
                }
                else
                {
                    //(delay is 1 quarter of bit clock)
                    __delay_cycles(HIDI2CUSCI30DELAY);
                    bLastStatus = UCSCLLOW & UCB1STAT;
                    bCount++;
                }
            }
#endif

            usciB1CurDataPointer[usciB1xferIndex] = UCB1RXBUF;
            usciB1xferIndex++;

            /* This is the last byte */
            if (usciB1xferIndex == usciB1BytesToXfer)
            {
                /* First two bytes */
                if (usciB1CurDataPointer == usciB1RegisterPointer)
                {
                    usciB1CurrentRegister = bitSwap(usciB1RegisterPointer[1],
                            usciB1RegisterPointer[0]);
                    hidI2C_handleRegisterPointer(USCIB1, usciB1CurrentRegister);
                }
                else
                {
                    usciB1BytesToXfer = 0;

                    clearLPMBits = hidI2C_handleData(USCIB1,
                            usciB1CurDataPointer, usciB1CurrentRegister);

                    if (usciB1BytesToXfer == 0)
                    {
                        usciB1startedOnce = FALSE;
                        hidI2C_enableInterrupts(USCIB1);
                    }
                }
            }
            break;
        }
        case 0x0C: /* Me -> Host */
        {
            UCB1TXBUF = usciB1CurDataPointer[usciB1xferIndex];
            usciB1xferIndex++;

            /* Last Byte to TX */
            if (usciB1xferIndex == usciB1BytesToXfer)
            {
                if (usciB1DeviceInitiated)
                {
                    usciB1DeviceInitiated = FALSE;
                }

                usciB1startedOnce = FALSE;
                hidI2C_enableInterrupts(USCIB1);
            }

            break;
        }
    }

    /* Clearing LPM bits if we need to */
    if (clearLPMBits)
    {
        clearLPMBits = FALSE;
        __bic_SR_register_on_exit(LPM4_bits);
        return;
    }

#ifdef UCS7WORKAROUND
    if (usciB1startedOnce)
    {
        __bic_SR_register_on_exit(LPM4_bits & ~LPM0_bits);
    }
    else
    {
        __bis_SR_register_on_exit(UCS7WORKAROUNDLPM);
    }
#endif

}
#endif

/*
 * USCI_B2 Data ISR
 */
#ifdef USCIB2
#ifdef __GNUC__
__attribute__((interrupt(USCI_B2_VECTOR)))
#else
#pragma vector=USCI_B2_VECTOR
__interrupt
#endif
void USCI_B2_ISR(void)
{
    switch (UCB2IV)
    {
        case 0x06: /* Start */
        {
            usciB2xferIndex = 0;

            /* Disabling start interrupts for the time being */
            hidI2C_disableInterrupts(USCIB2);

            /* First Start Condition */
            if (!usciB2startedOnce)
            {
                usciB2BytesToXfer = 0;

                /* Initial RXing */
                if (!(UCB2CTL1 & UCTR))
                {
                    usciB2BytesToXfer = 2;
                    usciB2CurDataPointer = usciB2RegisterPointer;
                }
                /* Initial TXing */
                else
                {
                    hidI2C_setNextQueuedData(USCIB2);
                    usciB2DeviceInitiated = TRUE;
                    usciB2xferIndex = 0;
                    UCB2IE |= (UCSTPIE);
                }

                usciB2startedOnce = TRUE;
            }

#ifdef UCS7WORKAROUND
            __delay_cycles(HIDI2CUCS7DELAY);
#endif

            break;
        }
        case 0x08: /* Stop */
        {
            if (!hidI2C_haveMoreGPIOsPending(USCIB2))
            {
                hidI2C_setGPIOPinIdle(USCIB2);
            }

            UCB2IE &= ~(UCSTPIE);
            break;

        }
        case 0x0A: /* Host -> Me */
        {

#ifdef USCI30WORKAROUND
            /* ERATTA USCI30 */
            uint8_t bLastStatus, bCount;
            
            bLastStatus = UCSCLLOW & UCB2STAT;
            bCount = 0;
        
            while (bCount < 12)
            {
                if(bLastStatus != (UCSCLLOW & UCB2STAT))
                {
                    bLastStatus = UCSCLLOW & UCB2STAT;
                    bCount = 0;
                }
                else
                {
                    //(delay is 1 quarter of bit clock)
                    __delay_cycles(HIDI2CUSCI30DELAY);
                    bLastStatus = UCSCLLOW & UCB2STAT;
                    bCount++;
                }
            }
#endif

            usciB2CurDataPointer[usciB2xferIndex] = UCB2RXBUF;
            usciB2xferIndex++;

            /* This is the last byte */
            if (usciB2xferIndex == usciB2BytesToXfer)
            {
                /* First two bytes */
                if (usciB2CurDataPointer == usciB2RegisterPointer)
                {
                    usciB2CurrentRegister = bitSwap(usciB2RegisterPointer[1],
                            usciB2RegisterPointer[0]);
                    hidI2C_handleRegisterPointer(USCIB2, usciB2CurrentRegister);
                }
                else
                {
                    usciB2BytesToXfer = 0;

                    clearLPMBits = hidI2C_handleData(USCIB2,
                            usciB2CurDataPointer, usciB2CurrentRegister);

                    if (usciB2BytesToXfer == 0)
                    {
                        usciB2startedOnce = FALSE;
                        hidI2C_enableInterrupts(USCIB2);
                    }
                }
            }
            break;
        }
        case 0x0C: /* Me -> Host */
        {
            UCB2TXBUF = usciB2CurDataPointer[usciB2xferIndex];
            usciB2xferIndex++;

            /* Last Byte to TX */
            if (usciB2xferIndex == usciB2BytesToXfer)
            {
                if (usciB2DeviceInitiated)
                {
                    usciB2DeviceInitiated = FALSE;
                }

                usciB2startedOnce = FALSE;
                hidI2C_enableInterrupts(USCIB2);
            }

            break;
        }
    }

    /* Clearing LPM bits if we need to */
    if (clearLPMBits)
    {
        clearLPMBits = FALSE;
        __bic_SR_register_on_exit(LPM4_bits);
        return;
    }

#ifdef UCS7WORKAROUND
    if (usciB2startedOnce)
    {
        __bic_SR_register_on_exit(LPM4_bits & ~LPM0_bits);
    }
    else
    {
        __bis_SR_register_on_exit(UCS7WORKAROUNDLPM);
    }
#endif

}
#endif

/*
 * USCI_B3 Data ISR
 */
#ifdef USCIB3
#ifdef __GNUC__
__attribute__((interrupt(USCI_B3_VECTOR)))
#else
#pragma vector=USCI_B3_VECTOR
__interrupt
#endif
void USCI_B3_ISR(void)
{
    switch (UCB3IV)
    {
        case 0x06: /* Start */
        {
            usciB3xferIndex = 0;

            /* Disabling start interrupts for the time being */
            hidI2C_disableInterrupts(USCIB3);

            /* First Start Condition */
            if (!usciB3startedOnce)
            {
                usciB3BytesToXfer = 0;

                /* Initial RXing */
                if (!(UCB3CTL1 & UCTR))
                {
                    usciB3BytesToXfer = 2;
                    usciB3CurDataPointer = usciB3RegisterPointer;
                }
                /* Initial TXing */
                else
                {
                    hidI2C_setNextQueuedData(USCIB3);
                    usciB3DeviceInitiated = TRUE;
                    usciB3xferIndex = 0;
                    UCB3IE |= (UCSTPIE);
                }

                usciB3startedOnce = TRUE;
            }

#ifdef UCS7WORKAROUND
            __delay_cycles(HIDI2CUCS7DELAY);
#endif

            break;
        }
        case 0x08: /* Stop */
        {
            if (!hidI2C_haveMoreGPIOsPending(USCIB3))
            {
                hidI2C_setGPIOPinIdle(USCIB3);
            }

            UCB3IE &= ~(UCSTPIE);
            break;

        }
        case 0x0A: /* Host -> Me */
        {
#ifdef USCI30WORKAROUND
            /* ERATTA USCI30 */
            uint8_t bLastStatus, bCount;
            
            bLastStatus = UCSCLLOW & UCB3STAT;
            bCount = 0;
        
            while (bCount < 12)
            {
                if(bLastStatus != (UCSCLLOW & UCB3STAT))
                {
                    bLastStatus = UCSCLLOW & UCB3STAT;
                    bCount = 0;
                }
                else
                {
                    //(delay is 1 quarter of bit clock)
                    __delay_cycles(HIDI2CUSCI30DELAY);
                    bLastStatus = UCSCLLOW & UCB3STAT;
                    bCount++;
                }
            }
#endif

            usciB3CurDataPointer[usciB3xferIndex] = UCB3RXBUF;
            usciB3xferIndex++;

            /* This is the last byte */
            if (usciB3xferIndex == usciB3BytesToXfer)
            {
                /* First two bytes */
                if (usciB3CurDataPointer == usciB3RegisterPointer)
                {
                    usciB3CurrentRegister = bitSwap(usciB3RegisterPointer[1],
                            usciB3RegisterPointer[0]);
                    hidI2C_handleRegisterPointer(USCIB3, usciB3CurrentRegister);
                }
                else
                {
                    usciB3BytesToXfer = 0;

                    clearLPMBits = hidI2C_handleData(USCIB3,
                            usciB3CurDataPointer, usciB3CurrentRegister);

                    if (usciB3BytesToXfer == 0)
                    {
                        usciB3startedOnce = FALSE;
                        hidI2C_enableInterrupts(USCIB3);
                    }
                }
            }
            break;
        }
        case 0x0C: /* Me -> Host */
        {
            UCB3TXBUF = usciB3CurDataPointer[usciB3xferIndex];
            usciB3xferIndex++;

            /* Last Byte to TX */
            if (usciB3xferIndex == usciB3BytesToXfer)
            {
                if (usciB3DeviceInitiated)
                {
                    usciB3DeviceInitiated = FALSE;
                }

                usciB3startedOnce = FALSE;
                hidI2C_enableInterrupts(USCIB3);
            }

            break;
        }
    }

    /* Clearing LPM bits if we need to */
    if (clearLPMBits)
    {
        clearLPMBits = FALSE;
        __bic_SR_register_on_exit(LPM4_bits);
        return;
    }

#ifdef UCS7WORKAROUND
    if (usciB3startedOnce)
    {
        __bic_SR_register_on_exit(LPM4_bits & ~LPM0_bits);
    }
    else
    {
        __bis_SR_register_on_exit(UCS7WORKAROUNDLPM);
    }
#endif

}
#endif
