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
#ifndef _HID_REPORT_PARSER_H_
#define _HID_REPORT_PARSER_H_

#include <stdint.h>

#define LONG_ITEM_TAG       0xFE
#define REPORT_SIZE_TAG     0x74
#define INPUT_ITEM_TAG      0x80
#define FEATURE_ITEM_TAG    0xB0
#define OUTPUT_ITEM_TAG     0x90
#define REPORT_ID_TAG       0x84
#define REPORT_COUNT_TAG    0x94

typedef struct sReportIdNode
{
    uint8_t rid;
    uint16_t inSize;
    uint16_t featureSize;
} tReportIdNote;

typedef struct sReportDescriptorInformation
{
    uint8_t bMaxOutputSize;
    uint8_t bMaxInputSize;
    uint8_t bNumOfReportIDs;
    tReportIdNote* reportIDs;

} tReportDescriptorInformation;

tReportDescriptorInformation getReportInformation(uint8_t* report, uint16_t size);

#endif
