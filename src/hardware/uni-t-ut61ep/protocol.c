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

/*
 * Driver for UNI-T UT61E+ and UT161 series digital multimeters.
 *
 * These meters use a WCH CH9329 UART-to-USB/HID chip (VID:PID 1a86:e429).
 * Communication is via 64-byte HID reports (vendor-specific, Usage Page 0xFFA0).
 *
 * To request a measurement, the host sends a 64-byte output report with
 * magic bytes [0x06, 0xab, 0xcd, 0x03, 0x5e, 0x01, 0xd9] (padded with zeros).
 * The meter responds with a 64-byte input report containing the measurement
 * as ASCII text at a known offset.
 *
 * Protocol decoded from the unit161d project by Pontus Fuchs:
 *   https://github.com/pof2/unit161d
 */

#include <config.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

static int send_request(struct sr_usb_dev_inst *usb)
{
	uint8_t buf[UT61EP_HID_REPORT_SIZE];
	int ret, len;

	memset(buf, 0, sizeof(buf));
	memcpy(buf, ut61ep_request_cmd, sizeof(ut61ep_request_cmd));

	/*
	 * Send via interrupt OUT transfer to EP4 (0x04).
	 * The CH9329 in custom HID mode requires interrupt transfers,
	 * not control transfers, for bidirectional communication.
	 * If interrupt OUT fails, fall back to HID SET_REPORT.
	 */
	ret = libusb_interrupt_transfer(
		usb->devhdl,
		LIBUSB_ENDPOINT_OUT | 4, /* EP4 OUT (0x04) */
		buf,
		sizeof(buf),
		&len,
		1000);

	if (ret == 0 && len == (int)sizeof(buf))
		return SR_OK;

	sr_dbg("Interrupt OUT failed (%s), trying SET_REPORT.",
	       libusb_error_name(ret));

	/* Fallback: HID SET_REPORT (Output report) */
	ret = libusb_control_transfer(
		usb->devhdl,
		LIBUSB_REQUEST_TYPE_CLASS |
		LIBUSB_RECIPIENT_INTERFACE |
		LIBUSB_ENDPOINT_OUT,
		0x09,   /* HID SET_REPORT */
		0x0200, /* Output report, Report ID 0 */
		0,      /* Interface 0 */
		buf,
		sizeof(buf),
		1000);

	if (ret < 0) {
		sr_err("Failed to send request: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int read_response(struct sr_usb_dev_inst *usb, uint8_t *buf)
{
	int ret, len;

	/*
	 * Read response via interrupt transfer from EP4 IN (0x84).
	 * The CH9329 sends measurement data as 64-byte interrupt IN reports.
	 */
	ret = libusb_interrupt_transfer(
		usb->devhdl,
		LIBUSB_ENDPOINT_IN | 4, /* EP4 IN (0x84) */
		buf,
		UT61EP_HID_REPORT_SIZE,
		&len,
		2000);

	if (ret < 0) {
		sr_dbg("USB read error: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	if (len != UT61EP_HID_REPORT_SIZE) {
		sr_dbg("Short read: got %d/%d bytes.", len, UT61EP_HID_REPORT_SIZE);
		return SR_ERR;
	}

	/* Verify response header: should start with 0x13 0xab 0xcd */
	if (buf[0] != 0x13 || buf[1] != 0xab || buf[2] != 0xcd) {
		sr_dbg("Invalid response header: %02x %02x %02x.",
		       buf[0], buf[1], buf[2]);
		return SR_ERR;
	}

	return SR_OK;
}

static void parse_unit(uint8_t unit_code, enum sr_unit *unit,
		       enum sr_mqflag *mqflags, int *exponent)
{
	*mqflags = 0;
	*exponent = 0;

	switch (unit_code) {
	case UT61EP_UNIT_V_AC:
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		break;
	case UT61EP_UNIT_MV_AC:
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		*exponent = -3;
		break;
	case UT61EP_UNIT_V_DC:
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_DC;
		break;
	case UT61EP_UNIT_MV_DC:
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_DC;
		*exponent = -3;
		break;
	case UT61EP_UNIT_HZ:
		*unit = SR_UNIT_HERTZ;
		break;
	case UT61EP_UNIT_DUTY:
		*unit = SR_UNIT_PERCENTAGE;
		break;
	case UT61EP_UNIT_OHM:
	case UT61EP_UNIT_OHM2:
		*unit = SR_UNIT_OHM;
		break;
	case UT61EP_UNIT_V_DIODE:
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_DIODE | SR_MQFLAG_DC;
		break;
	case UT61EP_UNIT_CAP:
		*unit = SR_UNIT_FARAD;
		break;
	case UT61EP_UNIT_TEMP_C:
		*unit = SR_UNIT_CELSIUS;
		break;
	case UT61EP_UNIT_TEMP_F:
		*unit = SR_UNIT_FAHRENHEIT;
		break;
	case UT61EP_UNIT_UA_DC:
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_DC;
		*exponent = -6;
		break;
	case UT61EP_UNIT_UA_AC:
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		*exponent = -6;
		break;
	case UT61EP_UNIT_MA_DC:
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_DC;
		*exponent = -3;
		break;
	case UT61EP_UNIT_MA_AC:
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		*exponent = -3;
		break;
	case UT61EP_UNIT_A_DC:
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_DC;
		break;
	case UT61EP_UNIT_A_AC:
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		break;
	case UT61EP_UNIT_NCV:
		*unit = SR_UNIT_VOLT;
		break;
	case UT61EP_UNIT_LOZ_V:
		*unit = SR_UNIT_VOLT;
		break;
	default:
		sr_dbg("Unknown unit code: %d.", unit_code);
		*unit = SR_UNIT_UNITLESS;
		break;
	}
}

static int count_digits(const char *s, int len)
{
	const char *dot;
	int i;

	dot = memchr(s, '.', len);
	if (!dot)
		return 0;

	/* Count digits after decimal point */
	i = 0;
	dot++;
	while (dot < s + len && isdigit((unsigned char)*dot)) {
		i++;
		dot++;
	}
	return i;
}

static int decode_packet(struct sr_dev_inst *sdi, const uint8_t *buf)
{
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	float floatval;
	enum sr_unit unit;
	enum sr_mqflag mqflags;
	int exponent, digits;
	char valstr[UT61EP_DATA_LEN + 1];

	devc = sdi->priv;

	/* Extract ASCII value string */
	memcpy(valstr, buf + UT61EP_DATA_OFFSET, UT61EP_DATA_LEN);
	valstr[UT61EP_DATA_LEN] = '\0';

	sr_spew("UT61E+ raw: unit=%d range=%c val='%s'",
	        buf[UT61EP_UNIT_OFFSET], buf[UT61EP_RANGE_OFFSET], valstr);

	/* Parse float value */
	floatval = (float)strtod(valstr, NULL);

	/* Parse unit */
	parse_unit(buf[UT61EP_UNIT_OFFSET], &unit, &mqflags, &exponent);

	/* Apply unit prefix (mV, uA, mA) */
	if (exponent != 0)
		floatval *= powf(10.0f, (float)exponent);

	/* Count decimal digits for precision */
	digits = count_digits(valstr, UT61EP_DATA_LEN);
	if (exponent < 0)
		digits += (-exponent);

	/* Build analog packet */
	sr_analog_init(&analog, &encoding, &meaning, &spec, digits);
	analog.meaning->mq = SR_MQ_VOLTAGE; /* Default */
	analog.meaning->mqflags = mqflags;

	switch (unit) {
	case SR_UNIT_VOLT:
		analog.meaning->mq = SR_MQ_VOLTAGE;
		break;
	case SR_UNIT_AMPERE:
		analog.meaning->mq = SR_MQ_CURRENT;
		break;
	case SR_UNIT_OHM:
		analog.meaning->mq = SR_MQ_RESISTANCE;
		break;
	case SR_UNIT_FARAD:
		analog.meaning->mq = SR_MQ_CAPACITANCE;
		break;
	case SR_UNIT_HERTZ:
		analog.meaning->mq = SR_MQ_FREQUENCY;
		break;
	case SR_UNIT_CELSIUS:
	case SR_UNIT_FAHRENHEIT:
		analog.meaning->mq = SR_MQ_TEMPERATURE;
		break;
	case SR_UNIT_PERCENTAGE:
		analog.meaning->mq = SR_MQ_DUTY_CYCLE;
		break;
	default:
		analog.meaning->mq = SR_MQ_VOLTAGE;
		break;
	}

	analog.meaning->unit = unit;
	analog.meaning->channels = sdi->channels;
	analog.num_samples = 1;
	analog.data = &floatval;

	packet.type = SR_DF_ANALOG;
	packet.payload = &analog;
	sr_session_send(sdi, &packet);

	sr_sw_limits_update_samples_read(&devc->limits, 1);

	return SR_OK;
}

SR_PRIV int ut61ep_receive_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	uint8_t buf[UT61EP_HID_REPORT_SIZE];
	int ret;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;
	usb = sdi->conn;

	/* Send measurement request */
	if (send_request(usb) != SR_OK)
		return FALSE;

	/* Read response */
	if (read_response(usb, buf) != SR_OK)
		return TRUE; /* Non-fatal, retry next time */

	/* Decode and send to session */
	decode_packet(sdi, buf);

	/* Check limits */
	if (sr_sw_limits_check(&devc->limits))
		sr_dev_acquisition_stop(sdi);

	return TRUE;
}
