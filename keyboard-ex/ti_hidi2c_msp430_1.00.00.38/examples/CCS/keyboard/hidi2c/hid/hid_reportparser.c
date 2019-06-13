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
 * @file hid_reportparser.c
 * @brief Parses relevant information from HID report descriptor
 * @details This file contains the function necessary to extract all relevant
 * information from a report descriptor. As far as the API is concerned, the
 * relevant information includes:
 *      <ul><li>Maximum input report length
 *      <li>Maximum output report length
 *      <li>Number of report IDs</ul>
 *
 * If the report descriptor contains multiple report IDs, we care about the
 * following information:
 *      <ul><li>List of report IDs
 *      <li>Size of each input report section in each report ID</ul>
 * @author Timothy Logan
 */
#include "hid_reportparser.h"
#include <stdlib.h>
#include <stdio.h>

/**
 * Function that parses the report descriptor and extracts all the relevant 
 * information needed for the HIDI2C API. This is mainly to estimate the 
 * appropriate memory to allocate for our buffers and maximize memory usage. It
 * is important to note that no validation of the report descriptor is currently
 * done in this function. If the report descriptor is invalid, bad things will
 * happen.
 * 
 * @param report HID Report descriptor as defined in HID1_11.pdf
 * @param size Size of the descriptors in uint8_ts
 **/
tReportDescriptorInformation getReportInformation(uint8_t* report, uint16_t size)
{
    uint16_t ii = 0;
    uint16_t jj = 0;
    uint8_t byteOffset = 0;
    uint16_t kk = 0;

    uint32_t currentReportSize = 0;
    uint32_t currentReportCount = 0;
    uint32_t currentInputSize = 0;
    uint32_t currentOutputSize = 0;
    uint32_t currentFeatureReportSize = 0;
    uint8_t currentReportID = 0;

    /* Information Structure */
    tReportDescriptorInformation reportInformation;
    reportInformation.bNumOfReportIDs = 0;
    reportInformation.bMaxOutputSize = 0;
    reportInformation.bMaxInputSize = 0;

    /* Make one pass to extract maximums and find out if there are multiple 
     report IDs */
    while (ii < size)
    {
        /* Long Item Tag - skip it (pg27 of HID1_11.pdf) */
        if (report[ii] == LONG_ITEM_TAG)
        {
            ii += ii + report[ii + 1] + 1;
            continue;
        }
        /* Short Item Tag */
        else if ((report[ii] & 0xFC) == REPORT_SIZE_TAG)
        {
            currentReportSize = 0;
            for (jj = (report[ii] & 0x03); jj > 0; jj--)
            {
                currentReportSize = currentReportSize | report[ii + jj];
                currentReportSize = currentReportSize << (jj - 1);
            }
        } else if ((report[ii] & 0xFC) == REPORT_COUNT_TAG)
        {
            currentReportCount = 0;
            for (jj = (report[ii] & 0x03); jj > 0; jj--)
            {
                currentReportCount = currentReportCount | report[ii + jj];
                currentReportCount = currentReportCount << (jj - 1);
            }
        } else if ((report[ii] & 0xFC) == INPUT_ITEM_TAG)
        {
            currentInputSize += currentReportSize * currentReportCount;
        } else if ((report[ii] & 0xFC) == OUTPUT_ITEM_TAG)
        {
            currentOutputSize += currentReportSize * currentReportCount;
        } else if ((report[ii] & 0xFC) == REPORT_ID_TAG)
        {
            reportInformation.bNumOfReportIDs =
                    reportInformation.bNumOfReportIDs + 1;

            if (currentOutputSize > reportInformation.bMaxOutputSize)
                reportInformation.bMaxOutputSize = currentOutputSize;

            if (currentInputSize > reportInformation.bMaxInputSize)
                reportInformation.bMaxInputSize = currentInputSize;

            currentOutputSize = 0;
            currentInputSize = 0;
        }

        byteOffset = (report[ii] & 0x03);

        if(byteOffset == 3)
            byteOffset = 4;

        ii += ( byteOffset + 1);
    }

    /* Checking the last report case */
    if (currentOutputSize > reportInformation.bMaxOutputSize)
        reportInformation.bMaxOutputSize = currentOutputSize;

    if (currentInputSize > reportInformation.bMaxInputSize)
        reportInformation.bMaxInputSize = currentInputSize;

    /* Converting Bits to Bytes (Output) (and rounding up) */
    if ((reportInformation.bMaxOutputSize % 8) == 0)
        reportInformation.bMaxOutputSize = reportInformation.bMaxOutputSize / 8;
    else
        reportInformation.bMaxOutputSize =
                (reportInformation.bMaxOutputSize / 8) + 1;

    /* Converting Bits to Bytes (Input) (and rounding up) */
    if ((reportInformation.bMaxInputSize % 8) == 0)
        reportInformation.bMaxInputSize = reportInformation.bMaxInputSize / 8;
    else
        reportInformation.bMaxInputSize = (reportInformation.bMaxInputSize / 8)
                + 1;

    /* Doing one more pass to get the Report IDs and link their sizes */
    if(reportInformation.bNumOfReportIDs == 0)
        reportInformation.reportIDs = malloc(
                    sizeof(tReportIdNote));
    else
        reportInformation.reportIDs = malloc(
            sizeof(tReportIdNote)
                    * reportInformation.bNumOfReportIDs);
    ii = 0;
    jj = 0;
    currentReportSize = 0;
    currentReportCount = 0;
    currentInputSize = 0;
    currentOutputSize = 0;

    while (ii < size)
    {
        /* Long Item Tag - skip it (pg27 of HID1_11.pdf) */
        if (report[ii] == LONG_ITEM_TAG)
        {
            ii +=  report[ii + 1] + 1;
            continue;
        }
        /* Report Size Tag */
        else if ((report[ii] & 0xFC) == REPORT_SIZE_TAG)
        {
            currentReportSize = 0;
            for (jj = (report[ii] & 0x03); jj > 0; jj--)
            {
                currentReportSize = currentReportSize | report[ii + jj];
                currentReportSize = currentReportSize << (jj - 1);
            }
        }
        /* Report Count Tag */
        else if ((report[ii] & 0xFC) == REPORT_COUNT_TAG)
        {
            currentReportCount = 0;
            for (jj = (report[ii] & 0x03); jj > 0; jj--)
            {
                currentReportCount = currentReportCount | report[ii + jj];
                currentReportCount = currentReportCount << (jj - 1);
            }
        }
        /* Input Report Item Tag */
        else if ((report[ii] & 0xFC) == INPUT_ITEM_TAG)
        {
            currentInputSize += currentReportSize * currentReportCount;
            currentReportCount = 0;
            currentReportSize = 0;
        }
        /* Feature Report Item Tag */
        else if ((report[ii] & 0xFC) == FEATURE_ITEM_TAG)
        {
            currentFeatureReportSize += currentReportSize * currentReportCount;
            currentReportCount = 0;
            currentReportSize = 0;
        }
        /* Report ID Tag */
        else if ((report[ii] & 0xFC) == REPORT_ID_TAG)
        {
            if (currentReportID != 0)
            {
                /* According to HIDI2C spec, length of two is required here
                 if no input reports are there */
                if (currentInputSize == 0)
                    currentInputSize = 2;
                else if ((currentInputSize % 8) == 0)
                    currentInputSize = currentInputSize / 8;
                else
                    currentInputSize = (currentInputSize / 8) + 1;

                if(currentFeatureReportSize == 0)
                    currentFeatureReportSize = 0;
                else if((currentFeatureReportSize % 8) == 0)
                    currentFeatureReportSize = currentFeatureReportSize/8;
                else
                    currentFeatureReportSize = (currentFeatureReportSize/8) + 1;

                reportInformation.reportIDs[kk].rid = currentReportID;
                reportInformation.reportIDs[kk].inSize = currentInputSize;
                reportInformation.reportIDs[kk].featureSize =
                        currentFeatureReportSize;
                kk++;
            }
            currentInputSize = 0;
            currentFeatureReportSize = 0;
            currentReportID = report[ii + 1];
        }

        byteOffset = (report[ii] & 0x03);

        if(byteOffset == 3)
            byteOffset = 4;

        ii += ( byteOffset + 1);
    }

    /* Filling out the last report id */
    if (currentInputSize == 0)
        currentInputSize = 2;
    else if ((currentInputSize % 8) == 0)
        currentInputSize = currentInputSize / 8;
    else
        currentInputSize = (currentInputSize / 8) + 1;

    if ((currentFeatureReportSize % 8) == 0)
        currentFeatureReportSize = currentFeatureReportSize / 8;
    else
        currentFeatureReportSize = (currentFeatureReportSize / 8) + 1;

    reportInformation.reportIDs[kk].rid = currentReportID;
    reportInformation.reportIDs[kk].inSize = currentInputSize;
    reportInformation.reportIDs[kk].featureSize = currentFeatureReportSize;

    return reportInformation;
}
