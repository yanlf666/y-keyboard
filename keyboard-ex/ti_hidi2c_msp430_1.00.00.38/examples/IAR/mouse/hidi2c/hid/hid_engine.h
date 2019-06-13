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
 |                              HID Engine (HIDI2C)                            |
 |                                                                             |
 +----------------------------------------------------------------------------*/
#ifndef _HID_ENGINE_H_
#define _HID_ENGINE_H_

#include "hid_reportparser.h"
#include "hidi2c_types.h"
#include <stdint.h>

#define SIZEOF_HID_DESCRIPTOR   30
#define HIDI2C_VERSION          0x0100
#define INPUT_REPORT_TYPE       0x10
#define FEATURE_REPORT_TYPE     0x30
#define OUTPUT_REPORT_TYPE      0x20

/* Mandatory Class Requests */
#define HID_RESET               0x01
#define HID_GET_REPORT          0x02
#define HID_SET_REPORT          0x03
#define HID_SET_POWER           0x08

/* Optional Class Requests */
#define HID_GET_IDLE            0x04
#define HID_SET_IDLE            0x05
#define HID_GET_PROTOCOL        0x06
#define HID_SET_PROTOCOL        0x07

/* HID Register Indexes */
#define HID_RES_REG             0x00
#define HID_DESC_REG            0x01
#define HID_REPORT_DESC_REG     0x02
#define HID_INPUT_REPORT_REG    0x03
#define HID_OUTPUT_REPORT_REG   0x04
#define HID_COMMAND_REG         0x05
#define HID_DATA_REG            0x06

#define SIZE_OF_INPUT_QUEUE     8

/**
 * Settings structure for HIDI2C stack.
 */
typedef struct HidI2CSetting
{
    uint8_t bInterface; /** Physical interface (ie. USCIB0) defined in settings.*/
    uint8_t* pReportDescriptor; /** Pointer to report descriptor */
    uint16_t wReportLength; /** Length of report descriptor */

} tHidI2CSetting;

/**
 *  HIDI2C Descriptor - Pg. 16 of HIDI2C Spec
 */
typedef struct HidDescriptor
{
    uint16_t wHIDDescLength; /** Length (30 Bytes for 1.0.0) */
    uint8_t bcdVersion_H; /** Version Number */
    uint8_t bcdVersion_L; /** Version Number */
    uint16_t wReportDescLength; /** Length of Report Descriptor */
    uint16_t wReportDescRegister; /** Register Index for Report Descriptor */
    uint16_t wInputRegister; /** Register Index for Input Descriptor */
    uint16_t wMaxInputLength; /** Max length for input register */
    uint16_t wOutputRegister; /** Register Index for Output Register */
    uint16_t wMaxOutputLength; /** Max length for output register */
    uint16_t wCommandRegister; /** Register Index for Command Register */
    uint16_t wDataRegister; /** Register Indxed for Data Register */
    uint16_t wVendorID; /** Vendor ID */
    uint16_t wProductID; /** Product ID */
    uint16_t wVersionID; /** Version ID */
    uint16_t wReserved0; /** Reserved -- Always 0 */
    uint16_t wReserved1; /** Reserved -- Always 0 */
} tHidDescriptor;

/**
 * Interface structure for HIDI2C. Contains all of the allocated registers as
 * well as the HID descriptor for a specific physical interface.
 */
typedef struct HidI2CInterface
{
    uint8_t bInterface; /** Interface (ie. USCIBO) defined in settings file */
    tHidDescriptor hidDescriptor; /** HID descriptor */
    uint8_t* reportDescriptor; /** Report descriptor (usually const) */
    uint8_t* inputReportRegister; /** Register for input reports. Can support
     multiple TLCs */
    uint8_t* resetRegister;
    uint8_t* outputReportRegister; /** Register for output reports */
    uint8_t* dataRegister; /** Register used for storing commands/information */
    uint8_t setReportCalled; /** Internal API variable for current report */
    uint8_t numOfReportIDs; /** Number of report IDs (0 if no report IDs) */
    uint16_t sizeOfDataReg; /** Size in bytes of data register */
    uint8_t currentQueuePos;
    uint8_t** featureReports; /** Storage for feature reports */
    uint8_t** featureReportsDefault; /** Storage for feature reports */
    uint8_t** inputReportQueue;
    uint8_t   curOpCode;
    uint8_t   inputOffset;
    tReportDescriptorInformation info;

} tHidI2CInterface;

/* Prototypes */
BOOL initializeHidI2CStack(tHidI2CSetting* settings, uint8_t numOfInstances);
BOOL initializeHidI2CStackIdle(tHidI2CSetting* settings, uint8_t numOfInstances,
        BOOL idle);
uint8_t setInputReportAndInitiateSend(uint8_t interface,
        uint8_t* report);
void setFeatureReport(uint8_t interface, uint8_t* report);

#endif
