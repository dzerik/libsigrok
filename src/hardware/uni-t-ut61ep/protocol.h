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
 * UNI-T UT61E+ / UT161 series protocol.
 *
 * These meters use a WCH CH9329 UART-to-USB/HID chip (VID:PID 1a86:e429)
 * which presents as a vendor-specific HID device with 64-byte reports.
 *
 * Protocol:
 * - Send a 64-byte HID output report with magic bytes:
 *   [0x06, 0xab, 0xcd, 0x03, 0x5e, 0x01, 0xd9, 0x00...]
 *   The first byte (0x06) is consumed by hidraw as report-id-like prefix.
 * - Receive a 64-byte HID input report with measurement data:
 *   [0x13, 0xab, 0xcd, 0x10, UNIT, D0..D7, FLAGS..., CHECKSUM]
 *   Where:
 *     byte[4]    = unit code (0=VAC, 1=mVAC, 2=VDC, 3=mVDC, 4=Hz, etc.)
 *     byte[5..12]= ASCII measurement value (8 chars, e.g. " 0.0545" or "-7.792 ")
 *     byte[13..] = flags and status bytes
 */

#define UT61EP_HID_REPORT_SIZE	64

/* USB VID/PID for WCH CH9329 used in UT61E+/UT161 series */
#define UT61EP_USB_VID		0x1a86
#define UT61EP_USB_PID		0xe429

/* Offsets in the response packet */
#define UT61EP_UNIT_OFFSET	4
#define UT61EP_RANGE_OFFSET	5
#define UT61EP_DATA_OFFSET	6
#define UT61EP_DATA_LEN		7

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
#define UT61EP_UNIT_MAX		21

/* Magic request command */
static const uint8_t ut61ep_request_cmd[7] = {
	0x06, 0xab, 0xcd, 0x03, 0x5e, 0x01, 0xd9
};

struct dev_context {
	struct sr_sw_limits limits;
	gboolean first_run;
};

SR_PRIV int ut61ep_receive_data(int fd, int revents, void *cb_data);

#endif
