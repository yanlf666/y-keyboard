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
 * @file hid_engine.c
 * @brief HID Engine for the HIDI2C Specification
 * @details This file is responsible for the HID level of the HIDI2C engine. 
 * All of the descriptors are managed here and commands/data are processed from 
 * the I2C layer. The goal of this layer is to abstract out the 
 * I2C/Communication layer. Relevant functions that communicate with the I2C 
 * layer are declared external and leave the programmer with the liberty of 
 * using their own I2C communication method.
 *
 * Sending HIDI2C reports over multiple physical interfaces in parallel is also
 * supported. This is accomplished by use of an interface parameter for API
 * calls. This interface parameter is interface implementation specific and is
 * defined in the @link hidi2c_settings.h @endlink file.
 *
 */

#include <hid_engine.h>
#include <hidi2c_errno.h>
#include <hidi2c_settings.h>
#include <hid_reportparser.h>

#include <i2c_engine_usci.h>
#include <i2c_engine_usci_init.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Static Functions and variables */
static void clearRegisters();
static int getIndexForReport(uint8_t interface, uint8_t rid);
int getIndexForInterface(uint8_t interface);
BOOL handleHIDCommand(uint8_t interface, uint8_t* data);
static uint16_t numOfInterfaces;
uint8_t* hidI2C_getNextQueuedData(uint8_t inf);
void setGPIOPinActive(uint8_t interface);

/* Queue of Input Reports */
uint8_t* inputReportQueue[SIZE_OF_INPUT_QUEUE];

/* Array of HID Interfaces */
tHidI2CInterface* interfaces;

/**
 * Event handler propagated from the HIDI2C API. With this function, the host 
 * has sent an output report (the HOST wants to write to the DEVICE). The 
 * output report is in standard HID Output Report format as specified by the
 * HID Spec.
 *
 *  For variables with multiple top level collections:
 *
 *      <ul><li>Report ID (1 Bytes) + Report (Variable)</ul>
 *
 *
 *  For variables with only one top level collection and no report ID:
 *      
 *      <ul><li>Report (variable)</ul>
 *
 * @param interface Interface from where the output report originated
 * @param data Pointer to output report
 */
extern BOOL outputReportEventHandler(uint8_t interface, uint8_t* data);
extern BOOL goToSleepEventHandler();
extern BOOL wakeUpEventHandler();
extern BOOL handleSetReportCalled(uint8_t interface, uint8_t reportid,
        uint8_t* feature);
extern uint8_t* handleGetReportCalled(uint8_t interface, uint8_t reportid);

int flip = 0;
/**
 * Initializes the HIDI2C Stack. This setting initializes both the I2C and HID
 * layer of the stack. Memory allocation and initialization of the HID
 * descriptor is done within this function. The give report descriptor is
 * automatically parsed and the maximum sizes are automatically calculated.
 * Support for multiple HID TLCs is also supported and calculated from the
 * report descriptor parser function.
 *
 * Each interface provided will allocate its own set of registers and memory
 * space. This allows for devices that have multiple interfaces (and larger
 * memory spaces) to send out multiple HIDI2C packets in parallel.
 *
 * @param settings Pointer to an array of HIDI2C settings. This data structure
 * is a list of all the physical interfaces as well as the report descriptor
 * that is associated with that specific interface.
 *
 * @param numOfInstances Number of interfaces that are included within the
 * settings array.
 */
BOOL initializeHidI2CStack(tHidI2CSetting* settings, uint8_t numOfInstances)
{
    return initializeHidI2CStackIdle(settings, numOfInstances, FALSE);
}

BOOL initializeHidI2CStackIdle(tHidI2CSetting* settings, uint8_t numOfInstances,
        BOOL idle)
{
    uint8_t ii, jj;
    uint8_t numOfReportIDs;
    uint8_t offset;
    uint16_t sizeOfDataReg;
    uint16_t maxFeatureSize;

    numOfInterfaces = numOfInstances;

    hidI2C_initializeI2C(idle);

    /* Allocating our structures */
    interfaces = malloc(sizeof(tHidI2CInterface) * numOfInstances);

    for (ii = 0; ii < numOfInstances; ii++)
    {
        interfaces[ii].bInterface = settings[ii].bInterface;

        tReportDescriptorInformation info = getReportInformation(
                settings[ii].pReportDescriptor, settings[ii].wReportLength);

        interfaces[ii].info = info;

        numOfReportIDs = info.bNumOfReportIDs;

        if (numOfReportIDs == 0)
        {
            interfaces[ii].inputOffset = 2;
        }
        else
            interfaces[ii].inputOffset = 3;

        /* Finding Max Feature Report Length */
        maxFeatureSize = 0;
        for (jj = 0; jj < numOfReportIDs; jj++)
        {
            if (interfaces[ii].info.reportIDs[jj].featureSize > maxFeatureSize)
            {
                maxFeatureSize = interfaces[ii].info.reportIDs[jj].featureSize;
            }
        }

        if (info.bMaxOutputSize >= info.bMaxInputSize)
        {
            if (info.bMaxOutputSize >= maxFeatureSize)
                sizeOfDataReg = info.bMaxOutputSize + 4;
            else
                sizeOfDataReg = maxFeatureSize + 4;
        }
        else
        {
            if (info.bMaxInputSize >= maxFeatureSize)
                sizeOfDataReg = info.bMaxInputSize + 4;
            else
                sizeOfDataReg = maxFeatureSize + 4;
        }

        interfaces[ii].inputReportQueue = malloc(
                sizeof(uint8_t*) * SIZE_OF_INPUT_QUEUE);
        interfaces[ii].curOpCode = 0;

        if (!interfaces[ii].inputReportQueue)
            return HIDI2C_NO_MEMORY;

        interfaces[ii].currentQueuePos = 0;

        /* Allocating the Queue */
        for (jj = 0; jj < SIZE_OF_INPUT_QUEUE; jj++)
        {
            interfaces[ii].inputReportQueue[jj] =
                    malloc(
                            sizeof(uint8_t)
                                    * (info.bMaxInputSize
                                            + interfaces[ii].inputOffset));

            if (!interfaces[ii].inputReportQueue[jj])
                return HIDI2C_NO_MEMORY;
        }

        /* Propagating the HID Descriptor */
        interfaces[ii].hidDescriptor.wHIDDescLength = SIZEOF_HID_DESCRIPTOR;
        interfaces[ii].hidDescriptor.bcdVersion_H = 0x00;
        interfaces[ii].hidDescriptor.bcdVersion_L = 0x01;
        interfaces[ii].hidDescriptor.wReportDescLength =
                settings[ii].wReportLength;
        interfaces[ii].hidDescriptor.wReportDescRegister = HID_REPORT_DESC_REG;
        interfaces[ii].hidDescriptor.wInputRegister = HID_INPUT_REPORT_REG;

        if (numOfReportIDs > 0)
            interfaces[ii].hidDescriptor.wMaxInputLength = info.bMaxInputSize
                    + 3;
        else
            interfaces[ii].hidDescriptor.wMaxInputLength = info.bMaxInputSize
                    + 2;

        interfaces[ii].hidDescriptor.wOutputRegister = HID_OUTPUT_REPORT_REG;

        interfaces[ii].hidDescriptor.wMaxOutputLength = info.bMaxOutputSize
                + interfaces[ii].inputOffset;

        interfaces[ii].hidDescriptor.wCommandRegister = HID_COMMAND_REG;
        interfaces[ii].hidDescriptor.wDataRegister = HID_DATA_REG;
        interfaces[ii].hidDescriptor.wVendorID = VENDOR_ID;
        interfaces[ii].hidDescriptor.wProductID = PRODUCT_ID;
        interfaces[ii].hidDescriptor.wVersionID = VERSION_ID;
        interfaces[ii].hidDescriptor.wReserved0 = 0x0000;
        interfaces[ii].hidDescriptor.wReserved1 = 0x0000;

        interfaces[ii].reportDescriptor = settings[ii].pReportDescriptor;

        if (numOfReportIDs == 0)
        {
            interfaces[ii].featureReports = malloc(sizeof(uint8_t*));
            interfaces[ii].featureReportsDefault = malloc(sizeof(uint8_t*));

            interfaces[ii].featureReports[0] = malloc(
                    interfaces[ii].info.reportIDs[0].featureSize);
            interfaces[ii].featureReportsDefault[0] = malloc(
                    interfaces[ii].info.reportIDs[0].featureSize);
            memset(interfaces[ii].featureReports[0], 0x00,
                    interfaces[ii].info.reportIDs[0].featureSize);
            memset(interfaces[ii].featureReportsDefault[0], 0x00,
                    interfaces[ii].info.reportIDs[0].featureSize);
        }
        else
        {
            interfaces[ii].featureReports = malloc(
                    sizeof(uint8_t*) * numOfReportIDs);
            interfaces[ii].featureReportsDefault = malloc(
                    sizeof(uint8_t*) * numOfReportIDs);

            /* Allocating Feature Report Registers */
            for (jj = 0; jj < numOfReportIDs; jj++)
            {
                if (interfaces[ii].info.reportIDs[jj].featureSize != 0)
                {
                    interfaces[ii].featureReports[jj] = malloc(
                            interfaces[ii].info.reportIDs[jj].featureSize);
                    interfaces[ii].featureReportsDefault[jj] = malloc(
                            interfaces[ii].info.reportIDs[jj].featureSize);
                    memset(interfaces[ii].featureReports[jj], 0x00,
                            interfaces[ii].info.reportIDs[jj].featureSize);
                    memset(interfaces[ii].featureReportsDefault[jj], 0x00,
                            interfaces[ii].info.reportIDs[jj].featureSize);
                }
            }
        }

        /* Allocating all of our registers */
        if (numOfReportIDs == 0)
        {
            offset = 0;
            interfaces[ii].inputReportRegister = malloc(
                    (info.bMaxInputSize + 2));
            interfaces[ii].inputReportRegister[1] = (info.bMaxInputSize + 2)
                    >> 8;
            interfaces[ii].inputReportRegister[0] = (info.bMaxInputSize + 2)
                    & 0x00FF;

        }
        else
        {
            interfaces[ii].inputReportRegister = malloc(info.bMaxInputSize + 3);
            interfaces[ii].inputReportRegister[1] = (info.reportIDs[ii].inSize
                    + 3) >> 8;
            interfaces[ii].inputReportRegister[0] = (info.reportIDs[ii].inSize
                    + 3) & 0x00FF;
            offset = 1;
        }

        interfaces[ii].resetRegister = malloc(info.bMaxInputSize + 3);
        memset(interfaces[ii].resetRegister, 0x00, info.bMaxInputSize + 3);

        if (info.bMaxOutputSize > 0)
            interfaces[ii].outputReportRegister = malloc(
                    info.bMaxOutputSize + offset);

        interfaces[ii].dataRegister = malloc(sizeOfDataReg);

        interfaces[ii].numOfReportIDs = numOfReportIDs;
        interfaces[ii].sizeOfDataReg = sizeOfDataReg;

        if (!interfaces[ii].dataRegister || !interfaces[ii].inputReportRegister
                || (!interfaces[ii].outputReportRegister
                        && info.bMaxOutputSize > 0))
            return HIDI2C_NO_MEMORY;
    }

    clearRegisters();

    return HIDI2C_SUCCESS;

}

/**
 * Sets the next queued data on the device to be read for a device initiated
 * transfer
 */
void hidI2C_setNextQueuedData(uint8_t inf)
{
    int infIndex;
    uint8_t i;
    uint8_t queuePos;

    uint8_t *queueData;
    uint8_t *tempQueue;

    uint16_t curSize;

    infIndex = getIndexForInterface(inf);

    queuePos = interfaces[infIndex].currentQueuePos;

    curSize = interfaces[infIndex].info.bMaxInputSize
            + interfaces[infIndex].inputOffset;

    if (queuePos == 0)
    {
        memset(interfaces[infIndex].inputReportRegister, 0x00, curSize);

        hidI2C_setDataAndInitializeInterrupt(
                inf,
                interfaces[infIndex].inputReportRegister,
                interfaces[infIndex].info.bMaxInputSize
                        + interfaces[infIndex].inputOffset);

        return;
    }

    queueData = interfaces[infIndex].inputReportQueue[0];

    interfaces[infIndex].currentQueuePos = queuePos - 1;

    memcpy(interfaces[infIndex].inputReportRegister, queueData, curSize);

    if (queuePos > 1)
    {
        tempQueue = interfaces[infIndex].inputReportQueue[0];
        
        for (i = 0; i < SIZE_OF_INPUT_QUEUE - 1; i++)
        {
            interfaces[infIndex].inputReportQueue[i] =
                    interfaces[infIndex].inputReportQueue[i + 1];
        }

        interfaces[infIndex].inputReportQueue[SIZE_OF_INPUT_QUEUE - 1] =
            tempQueue;
        
    }

    hidI2C_setDataAndInitializeInterrupt(
            inf,
            interfaces[infIndex].inputReportRegister,
            interfaces[infIndex].info.bMaxInputSize
                    + interfaces[infIndex].inputOffset);

}

/**
 * Function that returns true if there is another input report queued that needs
 * to be sent to the host
 */
BOOL hidI2C_haveMoreGPIOsPending(uint8_t inf)
{
    int infIndex;
    infIndex = getIndexForInterface(inf);

    if (interfaces[infIndex].currentQueuePos == 0)
        return FALSE;
    else
        return TRUE;
}

/**
 * Clears the input, output, and data registers to a state suitable for
 * pre-enumeration stages. Preserves TLCs and report that have already been 
 * added to API during initialization phases. Clears registers across multiple
 * interfaces
 */
void clearRegisters()
{
    uint8_t ii, jj;

    tHidI2CInterface curInterface;

    for (ii = 0; ii < numOfInterfaces; ii++)
    {
        curInterface = interfaces[ii];
        memset(curInterface.inputReportRegister, 0x00,
                curInterface.hidDescriptor.wMaxInputLength);

        /* Clearing the Output Report Register */
        if (curInterface.numOfReportIDs == 0
                && curInterface.hidDescriptor.wMaxOutputLength > 0)
            memset(curInterface.outputReportRegister, 0x00,
                    curInterface.hidDescriptor.wMaxOutputLength);
        else if (curInterface.hidDescriptor.wMaxOutputLength > 0)
            memset(curInterface.outputReportRegister, 0x00,
                    curInterface.hidDescriptor.wMaxOutputLength + 1);

        /* Setting the data register */
        memset(curInterface.dataRegister, 0x00, curInterface.sizeOfDataReg);

        if (curInterface.numOfReportIDs == 0)
        {
            memcpy(curInterface.featureReports[0],
                    curInterface.featureReportsDefault[0],
                    curInterface.info.reportIDs[0].featureSize);
        }
        else
        {
            for (jj = 0; jj < curInterface.numOfReportIDs; jj++)
            {
                /* Resetting the feature report register */
                memcpy(curInterface.featureReports[jj],
                        curInterface.featureReportsDefault[jj],
                        curInterface.info.reportIDs[jj].featureSize);
            }
        }
    }
}

/**
 * Sets a feature report in the HIDI2C API. If feature reports are used by the
 * device, this function could be called in conjunction with the
 * initializeHidI2CStackIdle command and an initial report should be passed to
 * the API. This also sets the "default" featuyre report.
 *
 * @param interface I2C interface being used
 * @param report Feature report to send (include report ID if multiple TLCs are
 * used.
 */
void setFeatureReport(uint8_t interface, uint8_t* report)
{
    int infIndex, index;
    infIndex = getIndexForInterface(interface);
    index = getIndexForReport(interface, report[0]);

    /* Setting the report */

    if (interfaces[infIndex].info.bNumOfReportIDs > 0)
    {
        memcpy(interfaces[infIndex].featureReports[index], report,
                interfaces[infIndex].info.reportIDs[index].featureSize + 1);

        memcpy(interfaces[infIndex].featureReportsDefault[index], report,
                interfaces[infIndex].info.reportIDs[index].featureSize + 1);
    }
    else
    {
        {
            memcpy(interfaces[infIndex].featureReports[index], report,
                    interfaces[infIndex].info.reportIDs[index].featureSize);

            memcpy(interfaces[infIndex].featureReportsDefault[index], report,
                    interfaces[infIndex].info.reportIDs[index].featureSize);
        }
    }
}

/**
 * Used in the case of multiple report IDs. Goes through our input register
 * array and finds the report register with the provided ID and returns the
 * index. This function should not be used in the case of a report descriptor
 * with no report IDs.
 *
 * @param rid The report ID to find the index of. Returns -1 if not found. 
 * @return The index of the report id in inputReportRegister
 */
int getIndexForReport(uint8_t interface, uint8_t rid)
{
    uint8_t ii;
    int infIndex;

    infIndex = getIndexForInterface(interface);

    if (interfaces[infIndex].numOfReportIDs <= 1)
        return 0;

    for (ii = 0; ii < interfaces[infIndex].numOfReportIDs; ii++)
    {
        if (rid == interfaces[infIndex].info.reportIDs[ii].rid)
            return ii;
    }

    return -1;
}

/**
 * Finds the index of the physical interfaces in the interfaces array.
 *
 * @param rid The interface to find the index of. Returns -1 if not found.
 * @return The index of the interface in interfaces
 */
int getIndexForInterface(uint8_t interface)
{
    uint8_t ii;

    for (ii = 0; ii < numOfInterfaces; ii++)
    {
        if (interfaces[ii].bInterface == interface)
            return ii;
    }

    return -1;
}

/**
 *  Sets an input report and initializes the send. Depending on if the user 
 *  defined NON_BLOCKING_MODE has false in the hidi2c_settings.h file, this
 *  function will return HIDI2C_BUSY if there is a transfer in progress. If the
 *  is in blocking mode, interrupts will be disables and the function will wait
 *  for ending pending transfer to finish before scheduling the transfer and 
 *  asserting the GPIO pin.
 *
 * @param interface Specific interface to send input report over
 * @param report Input report to set
 *
 * @return HIDI2C_SUCCESS on successful copying of the provided report into
 *  the local input report register. See hidi2c_errno.h for other error codes.
 */
uint8_t setInputReportAndInitiateSend(uint8_t interface, uint8_t* report)
{
    uint16_t curSize;
    uint8_t queueIndex;
    int index, infIndex;

    index = getIndexForReport(interface, report[0]);

    if (index < 0)
        return HIDI2C_INVALID_REPORTID;

    infIndex = getIndexForInterface(interface);
    curSize = interfaces[infIndex].info.reportIDs[index].inSize;

    /* Disabling Interrupts and Queuing if necessary */
    hidI2C_disableInterrupts(interface);

    queueIndex = interfaces[infIndex].currentQueuePos;

    if (queueIndex == SIZE_OF_INPUT_QUEUE)
    {
        hidI2C_enableInterrupts(interface);
        return HIDI2C_NO_MEMORY;
    }

    if (interfaces[infIndex].numOfReportIDs > 0)
    {
        interfaces[infIndex].inputReportQueue[queueIndex][1] =
                (uint8_t) ((curSize + 3) >> 8);
        interfaces[infIndex].inputReportQueue[queueIndex][0] = (uint8_t) (0x00FF
                & (curSize + 3));

        memcpy(interfaces[infIndex].inputReportQueue[queueIndex] + 2, report,
                curSize + 1);
    }
    else
    {
        interfaces[infIndex].inputReportQueue[queueIndex][1] =
                (uint8_t) ((curSize + 2) >> 8);
        interfaces[infIndex].inputReportQueue[queueIndex][0] = (uint8_t) (0x00FF
                & (curSize + 2));

        memcpy(interfaces[infIndex].inputReportQueue[queueIndex] + 2, report,
                curSize);
    }

    interfaces[infIndex].currentQueuePos = queueIndex + 1;

    hidI2C_setGPIOPinActive(interface);
    hidI2C_enableInterrupts(interface);

    return HIDI2C_SUCCESS;
}

/**
 * Provides the I2C layer with the necessary register pointer for the given 
 * index. This register can either be DATA, INPUT, OUTPUT, REPORT, or HID 
 * DESCRIPTOR.
 *
 * @param interface Specific interface that is requesting the data pointer
 * @param currentRegisterIndex Register index that the I2C layer needs the 
 * corresponding pointer for.
 *
 */
void hidI2C_handleRegisterPointer(uint8_t interface,
        uint16_t currentRegisterIndex)
{
    uint8_t infIndex;

    infIndex = getIndexForInterface(interface);

    switch (currentRegisterIndex)
    {
        case HID_OUTPUT_REPORT_REG:
            hidI2C_setDataWithoutInterrupt(
                    interface,
                    interfaces[infIndex].outputReportRegister,
                    interfaces[infIndex].info.bMaxOutputSize
                            + interfaces[infIndex].inputOffset);
            break;
        case HID_DATA_REG:
            hidI2C_setDataWithoutInterrupt(interface,
                    interfaces[infIndex].dataRegister,
                    interfaces[infIndex].sizeOfDataReg);
            break;
        case HID_COMMAND_REG:
            hidI2C_setDataWithoutInterrupt(interface,
                    interfaces[infIndex].dataRegister, 2);
            interfaces[infIndex].setReportCalled = FALSE;
            break;
        case HID_REPORT_DESC_REG:
            hidI2C_setDataWithoutInterrupt(interface,
                    interfaces[infIndex].reportDescriptor,
                    interfaces[infIndex].hidDescriptor.wReportDescLength);
            hidI2C_enableInterrupts(interface);
            break;
        case HID_DESC_REG:

            if (flip == 0)
            {
                hidI2C_setDataWithoutInterrupt(interface,
                        ((uint8_t*) &(interfaces[infIndex].hidDescriptor)), 2);
                hidI2C_enableInterrupts(interface);
                flip++;

            }
            else
            {
                hidI2C_setDataWithoutInterrupt(interface,
                        ((uint8_t*) &(interfaces[infIndex].hidDescriptor)),
                        SIZEOF_HID_DESCRIPTOR);
                hidI2C_enableInterrupts(interface);
            }
            break;
        case HID_INPUT_REPORT_REG:
        case HID_RES_REG:
        default:
            hidI2C_setDataWithoutInterrupt(
                    interface,
                    ((uint8_t*) &(interfaces[infIndex].resetRegister)),
                    interfaces[infIndex].info.bMaxInputSize
                            + interfaces[infIndex].inputOffset);
            hidI2C_enableInterrupts(interface);
            break;
    }
}

/**
 * Handles data received from the I2C layer. When this function is called, data
 * has been received and needs to be processed past the point of obtaining the
 * register pointer. The three cases this can happen are: OUTPUT REPORT, 
 * HID COMMAND, and DATA COMMAND (for SET_REPORT) 
 *
 * @param interface Specific interface to handle to data for
 * @param data Data received from the host
 * @param reg The current register index (either OUTPUT, COMMAND, or DATA)
 */
BOOL hidI2C_handleData(uint8_t interface, uint8_t* data, uint16_t reg)
{
    uint8_t infIndex;

    switch (reg)
    {
        case HID_OUTPUT_REPORT_REG:
            return outputReportEventHandler(interface, data + 2);
        case HID_COMMAND_REG:
        {
            infIndex = getIndexForInterface(interface);

            if (interfaces[infIndex].curOpCode == 0)
                interfaces[infIndex].curOpCode = data[1] & 0x0F;

            switch (interfaces[infIndex].curOpCode)
            {
                case HID_SET_POWER:
                case HID_RESET:
                    interfaces[infIndex].curOpCode = 0;
                    return handleHIDCommand(interface, data);
                case HID_GET_REPORT:
                    if ((data[0] & 0x0F) == 0x0F)
                    {
                        hidI2C_setDataChangeRegWithoutInterrupt(interface,
                                interfaces[infIndex].dataRegister + 2, 3,
                                HID_DATA_REG);
                    }
                    else
                    {
                        hidI2C_setDataChangeRegWithoutInterrupt(interface,
                                interfaces[infIndex].dataRegister + 2, 2,
                                HID_DATA_REG);
                    }
                    break;
                case HID_SET_REPORT:
                    if (!interfaces[infIndex].setReportCalled)
                    {
                        if ((data[0] & 0x0F) == 0x0F)
                            hidI2C_setDataWithoutInterrupt(interface,
                                    interfaces[infIndex].dataRegister + 2, 5);
                        else
                            hidI2C_setDataWithoutInterrupt(interface,
                                    interfaces[infIndex].dataRegister + 2, 4);

                        interfaces[infIndex].setReportCalled = TRUE;
                    }
                    else
                    {
                        if ((interfaces[infIndex].dataRegister[0] & 0x0F)
                                == 0x0F)
                            hidI2C_setDataChangeRegWithoutInterrupt(
                                    interface,
                                    interfaces[infIndex].dataRegister + 7,
                                    bitSwap(
                                            interfaces[infIndex].dataRegister[6],
                                            interfaces[infIndex].dataRegister[5])
                                            - 2, HID_DATA_REG);
                        else
                            hidI2C_setDataChangeRegWithoutInterrupt(
                                    interface,
                                    interfaces[infIndex].dataRegister + 6,
                                    bitSwap(
                                            interfaces[infIndex].dataRegister[5],
                                            interfaces[infIndex].dataRegister[4])
                                            - 2, HID_DATA_REG);
                        interfaces[infIndex].setReportCalled = FALSE;

                    }
                    break;
                default:
                    break;
            }

            break;
        }
        case HID_DATA_REG:
            infIndex = getIndexForInterface(interface);
            return handleHIDCommand(interface,
                    interfaces[infIndex].dataRegister);
        default:
            return FALSE;
    }

    return FALSE;
}

/**
 * Queues a dummy "reset" report for the host to read. This can also be used as
 * a device initiated reset to the host. The value of this report is a normal
 * input report with a sentinel value of 0x0000 for the length.
 */
void queueResetReport(uint8_t interface)
{
    uint8_t infIndex;

    infIndex = getIndexForInterface(interface);

    memset(
            interfaces[infIndex].inputReportQueue[0],
            0x00,
            interfaces[infIndex].hidDescriptor.wMaxInputLength
                    + interfaces[infIndex].inputOffset);

    interfaces[infIndex].currentQueuePos = 1;

    hidI2C_setGPIOPinActive(interface);
}

/**
 * Internal function that takes a command and decodes/processes the command.
 * Currently, only the mandatory class commands for HIDI2C devices are
 * implemented.
 *
 * @param interface Specific interface to handle the command for
 * @param data Data for the command to handle
 */
BOOL handleHIDCommand(uint8_t interface, uint8_t* data)
{
    uint8_t opCode;
    uint8_t reportType;
    uint8_t reportID;
    uint8_t powerState;
    uint8_t ridOffset;
    uint8_t* getReportData;
    uint16_t reportSize;
    int infIndex, index;

    infIndex = getIndexForInterface(interface);
    hidI2C_enableInterrupts(interface);

    opCode = data[1] & 0x0F;

    switch (opCode)
    {
        case HID_RESET:
            interfaces[infIndex].currentQueuePos = 0;
            clearRegisters();
            queueResetReport(interface);
            break;
        case HID_GET_REPORT:
            /* Extracting the report type and ID from command code */
            reportType = data[0] & 0x30;
            reportID = data[0] & 0x0F;

            interfaces[infIndex].curOpCode = 0;

            /* If this is value is 0xFF, there is another uint8_t hiding from us */
            if (reportID == 0x0F)
            {
                reportID = data[2];
                ridOffset = 1;
            }

            /* Fetching the feature report and queuing it */
            index = getIndexForReport(interface, reportID);

            /* Input Report Type */
            if (reportType == INPUT_REPORT_TYPE)
            {
                getReportData = handleGetReportCalled(interface, reportID);

                reportSize = interfaces[infIndex].info.reportIDs[index].inSize;

                memcpy(interfaces[infIndex].dataRegister + 2, getReportData,
                        reportSize);
            }
            /* Output Report Type */
            else if (reportType == OUTPUT_REPORT_TYPE)
            {
                reportSize = interfaces[infIndex].sizeOfDataReg;
                memset(interfaces[infIndex].dataRegister + 2, 0x00, reportSize);
            }
            /* Feature Report Type */
            else
            {
                reportSize =
                        interfaces[infIndex].info.reportIDs[index].featureSize;

                if (!reportSize)
                {
                    reportSize = interfaces[infIndex].sizeOfDataReg;
                    memset(interfaces[infIndex].dataRegister + 2, 0x00,
                            reportSize);
                }
                else
                {
                    memcpy(interfaces[infIndex].dataRegister + 2,
                            interfaces[infIndex].featureReports[index],
                            reportSize);
                }
            }

            interfaces[infIndex].dataRegister[0] = (reportSize
                    + interfaces[infIndex].inputOffset) & 0x00FF;
            interfaces[infIndex].dataRegister[1] = (reportSize
                    + interfaces[infIndex].inputOffset) >> 8;

            hidI2C_setDataWithoutInterrupt(interface,
                    interfaces[infIndex].dataRegister,
                    reportSize + interfaces[infIndex].inputOffset);

            break;
        case HID_SET_REPORT:
            /* Extracting the report type and ID from command code */
            reportType = data[0] & 0x30;
            reportID = data[0] & 0x0F;

            interfaces[infIndex].curOpCode = 0;

            /* We can ignore input reports */
            if (reportType == INPUT_REPORT_TYPE)
            {
                return FALSE;

            }
            /* If this is value is 0xFF, there is another uint8_t hiding from us */
            if (reportID == 0x0F)
            {
                reportID = data[2];
                ridOffset = 1;
            }
            else
                ridOffset = 0;

            /* Output Report Type */
            if (reportType == OUTPUT_REPORT_TYPE)
            {
                return outputReportEventHandler(interface, data + 6 + ridOffset);
            }

            /* Finding the index and updating the report */
            index = getIndexForReport(interface, reportID);

            reportSize = bitSwap(data[5 + ridOffset], data[4 + ridOffset]);

            memcpy(interfaces[infIndex].featureReports[index],
                    data + ridOffset + 6, reportSize - 2);

            /* Call a set report handler? */
            return handleSetReportCalled(interface, reportID,
                    data + ridOffset + 6);

        case HID_SET_POWER:
            powerState = data[0] & 0x01;

            /* Call the correct event handler */
            if (powerState)
                return goToSleepEventHandler();
            else
            {
                return wakeUpEventHandler();
            }

        case HID_GET_IDLE:
        case HID_SET_IDLE:
        case HID_GET_PROTOCOL:
        case HID_SET_PROTOCOL:
        default:
            break;
    }

    return FALSE;
}
