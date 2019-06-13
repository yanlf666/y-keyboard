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
 * @file keyboard_descriptors.h
 * @brief HID report descriptor and other variables for keyboard example
 */

/*----------------------------------------------------------------------------+
 |                                                                             |
 |                              Texas Instruments                              |
 |                                                                             |
 |                          HID Keyboard Example Include                       |
 |                                                                             |
 +----------------------------------------------------------------------------*/
#include "hidi2c_types.h"
#include <stdint.h>

/* Report Descriptor for Keyboard */
const uint8_t report_desc_HID0[] =
{ 
    0x05, 0x01, 	    // Usage Page (Generic Desktop)
    0x09, 0x06,		    // Usage (Keyboard)
    0xA1, 0x01,		    // Collection (Application)
    0x05, 0x07,		    // Usage Page (Key Codes)
    0x19, 0xE0,		    // Usage Minimum (224)
    0x29, 0xE7,		    // Usage Maximum (231)
    0x15, 0x00,		    // Logical Minimum (0)
    0x25, 0x01,		    // Logical Maximum (1)
    0x75, 0x01,		    // Report Size (1)
    0x95, 0x08,		    // Report Count (8)
    0x81, 0x02,		    // Input (Data, Variable, Absolute) -- Modifier byte
    0x95, 0x01,		    // Report Count (1)
    0x75, 0x08,	 	    // Report Size (8)
    0x81, 0x03,             // (81 01) Input (Constant) -- Reserved byte
    0x95, 0x05,             // Report Count (5)
    0x75, 0x01,             // Report Size (1)
    0x05, 0x08,             // Usage Page (Page# for LEDs)
    0x19, 0x01,             // Usage Minimum (1)
    0x29, 0x05,	            // Usage Maximum (5)
    0x91, 0x02,	            // Output (Data, Variable, Absolute) -- LED report
    0x95, 0x01,	            // Report Count (1)
    0x75, 0x03,	            // Report Size (3)
    0x91, 0x03,	            // (91 03) Output (Constant) -- LED report padding
    0x95, 0x06,             // Report Count (6)
    0x75, 0x08,             // Report Size (8)
    0x15, 0x00,	            // Logical Minimum (0)
    0x25, 0x66,             // Logical Maximum(102)  // was 0x65
    0x05, 0x07,             // Usage Page (Key Codes)
    0x19, 0x00,             // Usage Minimum (0)
    0x29, 0x66,	            // Usage Maximum (102) // was 0x65
    0x81, 0x00,	            // Input (Data, Array) -- Key arrays (6 bytes)
    0xC0                    // End Collection
};

/* Size of input report (including size field) */
#define INPUT_REPORT_SIZE              8

/*  The following section defines the HID Usage codes for each defined key on 
the keyboard.  These are defined in the HID "Usage Tag" (HUT) specification. */
#define usbUsageReserved            0x00
#define usbUsageErrorRollOver       0x01
#define usbUsagePOSTFail            0x02
#define usbUsageErrorUndefined      0x03
#define usbUsageA                   0x04
#define usbUsageB                   0x05
#define usbUsageC                   0x06
#define usbUsageD                   0x07
#define usbUsageE                   0x08
#define usbUsageF                   0x09
#define usbUsageG                   0x0A
#define usbUsageH                   0x0B
#define usbUsageI                   0x0C
#define usbUsageJ                   0x0D
#define usbUsageK                   0x0E
#define usbUsageL                   0x0F
#define usbUsageM                   0x10
#define usbUsageN                   0x11
#define usbUsageO                   0x12
#define usbUsageP                   0x13
#define usbUsageQ                   0x14
#define usbUsageR                   0x15
#define usbUsageS                   0x16
#define usbUsageT                   0x17
#define usbUsageU                   0x18
#define usbUsageV                   0x19
#define usbUsageW                   0x1A
#define usbUsageX                   0x1B
#define usbUsageY                   0x1C
#define usbUsageZ                   0x1D
#define usbUsage1                   0x1E
#define usbUsage2                   0x1F
#define usbUsage3                   0x20
#define usbUsage4                   0x21
#define usbUsage5                   0x22
#define usbUsage6                   0x23
#define usbUsage7                   0x24
#define usbUsage8                   0x25
#define usbUsage9                   0x26
#define usbUsage0                   0x27
#define usbUsageEnter               0x28
#define usbUsageEscape              0x29
#define usbUsageBackspace           0x2A
#define usbUsageTab                 0x2B
#define usbUsageSpacebar            0x2C
#define usbUsageMinus               0x2D
#define usbUsageEqual               0x2E
#define usbUsageLeftBracket         0x2F
#define usbUsageRightBracket        0x30
#define usbUsageBackslash           0x31
#define usbUsageVerticalBar         0x32
#define usbUsageSemicolon           0x33
#define usbUsageApostrophe          0x34
#define usbUsageTilde               0x35
#define usbUsageComma               0x36
#define usbUsagePeriod              0x37
#define usbUsageSlash               0x38
#define usbUsageCapsLock            0x39
#define usbUsageF1                  0x3A
#define usbUsageF2                  0x3B
#define usbUsageF3                  0x3C
#define usbUsageF4                  0x3D
#define usbUsageF5                  0x3E
#define usbUsageF6                  0x3F
#define usbUsageF7                  0x40
#define usbUsageF8                  0x41
#define usbUsageF9                  0x42
#define usbUsageF10                 0x43
#define usbUsageF11                 0x44
#define usbUsageF12                 0x45
#define usbUsagePrintScreen         0x46
#define usbUsageScrollLock          0x47
#define usbUsagePause               0x48
#define usbUsageInsert              0x49
#define usbUsageHome                0x4A
#define usbUsagePageUp              0x4B
#define usbUsageDeleteForward       0x4C
#define usbUsageEnd                 0x4D
#define usbUsagePageDown            0x4E
#define usbUsageRightArrow          0x4F
#define usbUsageLeftArrow           0x50
#define usbUsageDownArrow           0x51
#define usbUsageUpArrow             0x52
#define usbUsageKeypadNumlock       0x53
#define usbUsageKeypadSlash         0x54
#define usbUsageKeypadAsterisk      0x55
#define usbUsageKeypadMinus         0x56
#define usbUsageKeypadPlus          0x57
#define usbUsageKeypadEnter         0x58
#define usbUsageKeypad1             0x59
#define usbUsageKeypad2             0x5A
#define usbUsageKeypad3             0x5B
#define usbUsageKeypad4             0x5C
#define usbUsageKeypad5             0x5D
#define usbUsageKeypad6             0x5E
#define usbUsageKeypad7             0x5F
#define usbUsageKeypad8             0x60
#define usbUsageKeypad9             0x61
#define usbUsageKeypad0             0x62
#define usbUsageKeypadPeriod        0x63
#define usbUsageNonUsBackslash      0x64
#define usbUsageWindowsKey          0x65
#define usbUsagePower               0x66
#define usbUsageKeypadEqual         0x67
#define usbUsageF13                 0x68
#define usbUsageF14                 0x69
#define usbUsageF15                 0x6A
#define usbUsageF16                 0x6B
#define usbUsageF17                 0x6C
#define usbUsageF18                 0x6D
#define usbUsageF19                 0x6E
#define usbUsageF20                 0x6F
#define usbUsageF21                 0x70
#define usbUsageF22                 0x71
#define usbUsageF23                 0x72
#define usbUsageF24                 0x73
#define usbUsageExecute             0x74
#define usbUsageHelp                0x75
#define usbUsageMenu                0x76
#define usbUsageSelect              0x77
#define usbUsageStop                0x78
#define usbUsageAgain               0x79
#define usbUsageUndo                0x7A
#define usbUsageCut                 0x7B
#define usbUsageCopy                0x7C
#define usbUsagePaste               0x7D
#define usbUsageFind                0x7E
#define usbUsageMute                0x7F
#define usbUsageVolumeUp            0x80
#define usbUsageVolumneDown         0x81
#define usbUsageLockingCapsLock     0x82
#define usbUsageLockingNumLock      0x83
#define usbUsageLockingScrollLock   0x84
#define usbUsageKeypadComma         0x85
#define usbUsageAS400KeypadEqual    0x86
#define usbUsageInternational1      0x87
#define usbUsageInternational2      0x88
#define usbUsageInternational3      0x89
#define usbUsageInternational4      0x8A
#define usbUsageInternational5      0x8B
#define usbUsageInternational6      0x8C
#define usbUsageInternational7      0x8D
#define usbUsageInternational8      0x8E
#define usbUsageInternational9      0x8F
#define usbUsageLang1               0x90
#define usbUsageLang2               0x91
#define usbUsageLang3               0x92
#define usbUsageLang4               0x93
#define usbUsageLang5               0x94
#define usbUsageLang6               0x95
#define usbUsageLang7               0x96
#define usbUsageLang8               0x97
#define usbUsageLang9               0x98
#define usbUsageAlternateErase      0x99
#define usbUsageSysReq              0x9A
#define usbUsageCancel              0x9B
#define usbUsageClear               0x9C
#define usbUsagePrior               0x9D
#define usbUsageReturn              0x9E
#define usbUsageSeparator           0x9F
#define usbUsageOut                 0xA0
#define usbUsageOper                0xA1
#define usbUsageClearAgain          0xA2
#define usbUsageCrSelProps          0xA3
#define usbUsageExSel               0xA4
#define usbUsageLeftControl         0xE0
#define usbUsageLeftShift           0xE1
#define usbUsageLeftAlt             0xE2
#define usbUsageLeftGUI             0xE3
#define usbUsageRightControl        0xE4
#define usbUsageRightShift          0xE5
#define usbUsageRightAlt            0xE6
#define usbUsageRightGUI            0xE7
