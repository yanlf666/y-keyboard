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
 * @file mouse_descriptors.h
 * @brief HID report descriptor and other variables for mouse example
 */

#ifndef MOUSE_DESCRIPTORS_H
#define MOUSE_DESCRIPTORS_H

#include "hidi2c_types.h"
#include <stdint.h>

typedef struct 
{
    uint8_t buttons;
    uint8_t dX;
    uint8_t dY;
    uint8_t dZ;
} MOUSE_REPORT;

const uint8_t report_desc_HID0[]=
{
    0x05, 0x01,                // Usage Pg (Generic Desktop)
    0x09, 0x02,                // Usage (Mouse)
    0xA1, 0x01,                // Collection: (Application)
    0x09, 0x01,                // Usage Page (Vendor Defined)
    0xA1, 0x00,                // Collection (Linked)
    0x05, 0x09,                // Usage (Button)
    0x19, 0x01,                // Usage Min (#)
    0x29, 0x05,                // Usage Max (#)
    0x15, 0x00,                // Log Min (0)
    0x25, 0x01,                // Usage Maximum
    0x95, 0x05,                // Report count (5)
    0x75, 0x01,                // Report Size (1)
    0x81, 0x02,                // Input (Data,Var,Abs)
    0x95, 0x01,                // Report Count (1)
    0x75, 0x03,                // Report Size (3)
    0x81, 0x01,                // Input: (Constant)
    0x05, 0x01,                // Usage Pg (Generic Desktop)
    0x09, 0x30,                // Usage (X)
    0x09, 0x31,                // Usage (Y)
    0x09, 0x38,                // Usage (Wheel)
    0x15, 0x81,                // Log Min (-127)
    0x25, 0x7F,                // Log Max (127)
    0x75, 0x08,                // Report Size
    0x95, 0x03,                // Report Count (3)
    0x81, 0x06,                // Input: (Data, Variable, Relative)
    0xC0,                      // End Collection
    0xC0                       // End Collection
};

/* Lookup table for mouse position values.  "const" indicates it will be stored
  in flash. */
const int tableSinCosLookUp[93][2] = 
{
    0,200,
    14,200,
    28,198,
    42,196,
    55,192,
    68,188,
    81,183,
    94,177,
    106,170,
    118,162,
    129,153,
    139,144,
    149,134,
    158,123,
    166,112,
    173,100,
    180,88,
    185,75,
    190,62,
    194,48,
    197,35,
    199,21,
    200,7,
    200,-7,
    199,-21,
    197,-35,
    194,-48,
    190,-62,
    185,-75,
    180,-88,
    173,-100,
    166,-112,
    158,-123,
    149,-134,
    139,-144,
    129,-153,
    118,-162,
    106,-170,
    94,-177,
    81,-183,
    68,-188,
    55,-192,
    42,-196,
    28,-198,
    14,-200,
    0,-200,
    -14,-200,
    -28,-198,
    -42,-196,
    -55,-192,
    -68,-188,
    -81,-183,
    -94,-177,
    -106,-170,
    -118,-162,
    -129,-153,
    -139,-144,
    -149,-134,
    -158,-123,
    -166,-112,
    -173,-100,
    -180,-88,
    -185,-75,
    -190,-62,
    -194,-48,
    -197,-35,
    -199,-21,
    -200,-7,
    -200,7,
    -199,21,
    -197,35,
    -194,48,
    -190,62,
    -185,75,
    -180,88,
    -173,100,
    -166,112,
    -158,123,
    -149,134,
    -139,144,
    -129,153,
    -118,162,
    -106,170,
    -94,177,
    -81,183,
    -68,188,
    -55,192,
    -42,196,
    -28,198,
    -14,200,
    0,200
};

#endif


