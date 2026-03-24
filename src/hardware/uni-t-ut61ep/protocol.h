/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2026 Dzerik <dzerik@github.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_UNI_T_UT61EP_PROTOCOL_H
#define LIBSIGROK_HARDWARE_UNI_T_UT61EP_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "uni-t-ut61ep"

/*
 * UNI-T UT61E+ / UT161 series protocol over WCH CH9329 USB/HID.
 *
 * VID:PID 1a86:e429, vendor-specific HID (Usage Page 0xFFA0),
 * 64-byte reports, no Report ID.
 *
 * Request (host -> device): 64 bytes via interrupt OUT EP4.
 *   [0x06, 0xab, 0xcd, 0x03, 0x5e, 0x01, 0xd9, 0x00...]
 *
 * Response (device -> host): 64 bytes via interrupt IN EP4.
 *   Byte layout:
 *     [0-3]   Header: 0x13 0xab 0xcd 0x10
 *     [4]     Unit code (see UT61EP_UNIT_*)
 *     [5]     Range indicator (ASCII digit)
 *     [6-12]  Value: 7 ASCII chars including sign (e.g. " 7.720" or "-0.054")
 *     [13]    Flags byte 1 (overload, battery)
 *     [14]    Flags byte 2 (HOLD, REL, AUTO, MAX, MIN, PEAK)
 *     [15-17] Status: 3 ASCII digits
 *     [18]    Packet trailer (0x03)
 *     [19]    Checksum
 */

#define UT61EP_HID_REPORT_SIZE	64

/* USB VID/PID for WCH CH9329 used in UT61E+/UT161 series */
#define UT61EP_USB_VID		0x1a86
#define UT61EP_USB_PID		0xe429

/* Response packet offsets */
#define UT61EP_UNIT_OFFSET	4
#define UT61EP_RANGE_OFFSET	5
#define UT61EP_DATA_OFFSET	6
#define UT61EP_DATA_LEN		7
#define UT61EP_FLAGS1_OFFSET	13
#define UT61EP_FLAGS2_OFFSET	14
#define UT61EP_STATUS_OFFSET	15
#define UT61EP_STATUS_LEN	3

/* Flags byte 2 (byte[14]) — observed bit fields */
#define UT61EP_FLAG2_HOLD	(1 << 0)
#define UT61EP_FLAG2_REL	(1 << 1)
#define UT61EP_FLAG2_AUTO	(1 << 2)

/* Unit codes */
#define UT61EP_UNIT_V_AC	0
#define UT61EP_UNIT_MV_AC	1
#define UT61EP_UNIT_V_DC	2
#define UT61EP_UNIT_MV_DC	3
#define UT61EP_UNIT_HZ		4
#define UT61EP_UNIT_DUTY	5
#define UT61EP_UNIT_OHM		6
#define UT61EP_UNIT_OHM2	7
#define UT61EP_UNIT_V_DIODE	8
#define UT61EP_UNIT_CAP		9
#define UT61EP_UNIT_TEMP_C	10
#define UT61EP_UNIT_TEMP_F	11
#define UT61EP_UNIT_UA_DC	12
#define UT61EP_UNIT_UA_AC	13
#define UT61EP_UNIT_MA_DC	14
#define UT61EP_UNIT_MA_AC	15
#define UT61EP_UNIT_A_DC	16
#define UT61EP_UNIT_A_AC	17
#define UT61EP_UNIT_NCV		20
#define UT61EP_UNIT_LOZ_V	21

/* Magic request command (7 bytes, padded to 64 with zeros) */
static const uint8_t ut61ep_request_cmd[7] = {
	0x06, 0xab, 0xcd, 0x03, 0x5e, 0x01, 0xd9
};

struct dev_context {
	struct sr_sw_limits limits;
};

SR_PRIV int ut61ep_receive_data(int fd, int revents, void *cb_data);

#endif
