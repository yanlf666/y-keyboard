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
 * @brief Mouse example for HIDI2C API
 * Sending/Receiving Reports:
 * This application acts as a mouse on the host, moving the pointer in a 
 * circular motion on the screen.  To re-gain control of the mouse, you can 
 * either remove the device or type the key sequence to exit debug mode 
 * (Ctrl+Shift+D in IAR).
 *
 * A custom report descriptor is defined in descriptors.c.  It defines a 
 * top-level application collection of "Generic Desktop", and a usage of 
 * "Mouse".  Windows recognizes this configuration as a mouse, and begins 
 * servicing it directly. In this way, Windows acts as the "application", rather
 * than using a separate one.
 */

#include "hidi2c_types.h"
#include "mouse_descriptors.h"
#include "hid_engine.h"
#include "hidi2c_settings.h"

#include "msp430.h"

MOUSE_REPORT mouseReport = { 0, 0, 0, 0 }; // HID report, to be sent to the PC.
const int tableSinCosLookUp[93][2]; // Lookup table for mouse data;
uint8_t index = 1; // Index for lookup table
uint8_t sendNewMousePosition = FALSE; // Main loop flag
volatile BOOL gotoSleep = FALSE;

/*  
 * ======== main ========
 */
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;

    /* Making our settings */
    tHidI2CSetting settings[1];
    settings[0].bInterface = USCIB0;
    settings[0].pReportDescriptor = (uint8_t*)report_desc_HID0;
    settings[0].wReportLength = 52;


    /* Allocating Settings Buffer */
    /* Configuring HIDI2C */
    if (initializeHidI2CStack(settings,1))
    {
        return -1;
    }

    /* Configure an LED to blink when a report is sent */P1DIR |= 0x01;
    P1OUT &= ~0x01;

    /* Timer_A will repeatedly run to 547, using 32kHz clock, and then generate
     an interrupt that wakes the main loop (every 1/60 sec) */
    TA0CCTL0 = CCIE; //CCR0 interrupt enabled
    TA0CCR0 = 547; //547/32768 = a period of 16.7ms
    TA0CTL = TASSEL_1 + TACLR; //ACLK, up mode

    __enable_interrupt();

    while (1)
    {
        /* Start timer */
        TA0CTL |= MC_1;

        __bis_SR_register(LPM0_bits + GIE);

        if (sendNewMousePosition)
        {
            /* Fill out report */
            mouseReport.dX = (tableSinCosLookUp[index][0]
                    - tableSinCosLookUp[index - 1][0]) >> 1;
            mouseReport.dY = (tableSinCosLookUp[index][1]
                    - tableSinCosLookUp[index - 1][1]) >> 1;

            /* Sending to HIDI2C */
             setInputReportAndInitiateSend(USCIB0, (uint8_t*) &mouseReport);

            /* Toggle LED on P1.0 */P1OUT ^= 0x01;

            if (index++ >= 90)
            {
                index = 1;
            }
        }

        if (gotoSleep)
            __bis_SR_register(LPM3_bits + GIE);
        else
            __bis_SR_register(LPM0_bits + GIE);
    }
}

/* Event handlers */

/**
 * Output report handler.
 *
 * @return TRUE if the user wants to clear LPM bits after this interrupt, FALSE
 * if the user wants the LPM bits not to be touched \
 */
BOOL outputReportEventHandler(uint8_t interface, uint8_t* data)
{
    /* Do nothing. We don't support output reports */
    return FALSE;
}

/**
 * Event handlers that are called when the host receives a command to put the
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
    return (uint8_t*)&mouseReport;
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
BOOL handleSetReportCalled(uint8_t interface, uint8_t reportid, uint8_t* feature)
{
    return FALSE;
}

/*  
 * ======== TIMER0_A0_ISR ========
 */
#ifdef __GNUC__
__attribute__((interrupt(TIMER0_A0_VECTOR)))
#else
#pragma vector=TIMER0_A0_VECTOR
__interrupt
#endif
void TIMER0_A0_ISR(void)
{
    /* Set flag telling main loop to send a report */
    sendNewMousePosition = TRUE;
    __bic_SR_register_on_exit(LPM0_bits);
}

