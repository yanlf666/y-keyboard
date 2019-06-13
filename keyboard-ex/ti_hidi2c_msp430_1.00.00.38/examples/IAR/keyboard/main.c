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
 * @file main.c
 * @brief Keyboard example for HIDI2C API
 * Example program which simulates a "keyboard" with the use of the HIDI2C API.
 * Since we are not actually building a keyboard with this example, we emulate
 * a keyboard with the use of the S1 and S2 buttons on the MSPF5529 
 * Experimenter's board. When S1 is pressed, an input report with the keystrokes
 * "MSP430" are sent to the host. S2 is used to simulate the shift button on a
 * keyboard. LEDs for the keyboard are also simulated to show support for 
 * Output reports from the host. 
 */

#include "msp430.h"
#include "keyboard_descriptors.h"
#include "hidi2c_types.h"
#include "hid_engine.h"
#include "hidi2c_settings.h"

#include <stdint.h>
#include <string.h>

/* Input Report to take care of */
uint8_t inputReport[INPUT_REPORT_SIZE];
BOOL sendZeroedData;
BOOL hidi2cReadyToGo;

/* Event Handler Implementations */
BOOL outputReportEventHandler(uint8_t interface, uint8_t* reportData);
BOOL goToSleepEventHandler();
BOOL wakeUpEventHandler();

volatile uint8_t gotoSleep;

int main(void)
{
    int i;

    /* Stop watchdog timer to prevent time out reset */
    WDTCTL = WDTPW + WDTHOLD;
    gotoSleep = FALSE;
    sendZeroedData = FALSE;

    /* Setting S1 to act as a virtual "key", S2 to act as shift, and the LED
     to act as an the Caps Lock Key */P1OUT |= (BIT6 | BIT7);
    P1REN |= (BIT6 | BIT7);
    P1IFG = 0;
    P1IE |= BIT6;
    P1DIR |= BIT1;
    P1OUT &= ~BIT1;

    /* Clearing out our input report */
    memset(inputReport, 0x00, INPUT_REPORT_SIZE);

    /* Creating our settings array */
    tHidI2CSetting settings[1];
    settings[0].bInterface = USCIB0;
    settings[0].pReportDescriptor = (uint8_t*) report_desc_HID0;
    settings[0].wReportLength = 63;

    /* Initializing HID and I2C */
    if (initializeHidI2CStack(settings, 1))
    {
        return -1;
    }

    /* Enter LPM0 and enable interrupts */
    while (1)
    {
        if (gotoSleep)
            __bis_SR_register(LPM3_bits + GIE);
        else
            __bis_SR_register(LPM0_bits + GIE);

        P1IE &= ~BIT6;

        /* Delay Loop */
        for (i = 0x1FFF; i > 0; i--)
        {
        }

        /* When we wake from LPM, send a zeroed report for the sake of
         * a key release (otherwise the host will think you are holding down
         * the button */
        inputReport[1] = 0x00;
        inputReport[2] = 0x00;
        inputReport[3] = 0x00;
        inputReport[4] = 0x00;
        inputReport[5] = 0x00;
        inputReport[6] = 0x00;
        inputReport[7] = 0x00;

        if (sendZeroedData)
            setInputReportAndInitiateSend(USCIB0, inputReport);

        P1IE |= BIT6;
    }
}

/**
 * Output report handler.
 *
 * @return TRUE if the user wants to clear LPM bits after this interrupt, FALSE
 * if the user wants the LPM bits not to be touched \
 */
BOOL outputReportEventHandler(uint8_t interface, uint8_t* outputReport)
{
    /* We are only expecting an output report to tell us if an LED should 
     be on with the keyboard. For this example, we will only check for
     CapsLock and set the LED accordingly */

    /* This should only be one byte long */
    uint8_t bSize = outputReport[0];

    if (bSize != 1)
        return FALSE;

    /* According to the HID usage tables, Caps Lock is 02h */
    if (outputReport[1] & 0x02)
        P1OUT ^= BIT4;

    return FALSE;

}

/**
 * Event handler called when a SET_REPORT command is issued by the host. For
 * keyboards/mice, feature reports are not supported so this is a dummy handler
 *
 *
 * @param interface Specific interface being used
 * @param reportid Report ID of the feature report that was set
 * @param feature New feature report
 *
 * @return TRUE if the user wants to clear LPM bits after this interrupt, FALSE
 * if the user wants the LPM bits not to be touched
 */
BOOL handleSetReportCalled(uint8_t interface, uint8_t reportid,
        uint8_t* feature)
{
    return FALSE;
}

/**
 * Event handler that are called when the host receives a command to put the
 * device to sleep or wake it up. It is important to note that these handlers 
 * are called in interrupt context and should be very brief in nature (set a
 * flag
 *
 * @return TRUE if the user wants to clear LPM bits after this interrupt, FALSE
 * if the user wants the LPM bits not to be touched
 */
BOOL goToSleepEventHandler()
{
    gotoSleep = TRUE;
    return TRUE;
}

/**
 * Because LPM bits need to be cleared at the end of the calling interrupt.
 * Returning TRUE from this function will clear the LPM bits on the MSP430 and
 * cause it to wake up from sleep.
 *
 * @return TRUE if the user wants to clear LPM bits after this interrupt, FALSE
 * if the user wants the LPM bits not to be touched
 */
BOOL wakeUpEventHandler()
{
    gotoSleep = FALSE;
    return TRUE;
}

/**
 * Grabs the latest input report from the device.
 *
 * @return uint8_t* Pointer to array with the latest input report.
 */
uint8_t* handleGetReportCalled(uint8_t interface, uint8_t reportid)
{
    return inputReport;
}

/*  
 * ======== Port1_ISR ========
 */
#ifdef __GNUC__
__attribute__((interrupt(PORT1_VECTOR)))
#else
#pragma vector=PORT1_VECTOR
__interrupt
#endif
void Port1_ISR(void)
{
    uint16_t i;

    if (P1IFG & BIT6)
    {
        /* Delay Loop */
        for (i = 0xFFF; i > 0; i--)
        {
        }

        if (P1IN & BIT6)
        {
            /* Filling out a "fake" input report. Since we don't really have a
             keyboard here we are just writng MSP430. The size is constant */

            /* Checking if our virtual "shift" key is pressed */
            if (!(P1IN & BIT7))
            {
                inputReport[0] = 0x02;
            }
            else
            {
                inputReport[0] = 0x00;
            }

            inputReport[1] = 0x00;
            inputReport[2] = usbUsageM;
            inputReport[3] = usbUsageS;
            inputReport[4] = usbUsageP;
            inputReport[5] = usbUsage4;
            inputReport[6] = usbUsage3;
            inputReport[7] = usbUsage0;

            /* Clear LPM bits after interrupt ends */
            __bic_SR_register_on_exit(LPM3_bits);
            sendZeroedData = TRUE;

            /* Sending the report to this host */
            setInputReportAndInitiateSend(USCIB0, inputReport);
        }

        P1IFG = 0;
    }
}
