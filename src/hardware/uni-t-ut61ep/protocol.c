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
	 * The CH9329 in custom HID mode requires interrupt transfers.
	 * If interrupt OUT fails, fall back to HID SET_REPORT.
	 */
	ret = libusb_interrupt_transfer(
		usb->devhdl,
		LIBUSB_ENDPOINT_OUT | 4,
		buf,
		sizeof(buf),
		&len,
		1000);

	if (ret == 0 && len == (int)sizeof(buf))
		return SR_OK;

	sr_dbg("Interrupt OUT failed (%s), trying SET_REPORT.",
	       libusb_error_name(ret));

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

	ret = libusb_interrupt_transfer(
		usb->devhdl,
		LIBUSB_ENDPOINT_IN | 4,
		buf,
		UT61EP_HID_REPORT_SIZE,
		&len,
		2000);

	if (ret < 0) {
		sr_dbg("USB read error: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	if (len != UT61EP_HID_REPORT_SIZE) {
		sr_dbg("Short read: got %d/%d bytes.", len,
		       UT61EP_HID_REPORT_SIZE);
		return SR_ERR;
	}

	if (buf[0] != 0x13 || buf[1] != 0xab || buf[2] != 0xcd) {
		sr_dbg("Invalid response header: %02x %02x %02x.",
		       buf[0], buf[1], buf[2]);
		return SR_ERR;
	}

	return SR_OK;
}

static void parse_unit(uint8_t unit_code, enum sr_mq *mq,
		       enum sr_unit *unit, enum sr_mqflag *mqflags,
		       int *exponent)
{
	*mqflags = 0;
	*exponent = 0;

	switch (unit_code) {
	case UT61EP_UNIT_V_AC:
		*mq = SR_MQ_VOLTAGE;
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		break;
	case UT61EP_UNIT_MV_AC:
		*mq = SR_MQ_VOLTAGE;
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		*exponent = -3;
		break;
	case UT61EP_UNIT_V_DC:
		*mq = SR_MQ_VOLTAGE;
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_DC;
		break;
	case UT61EP_UNIT_MV_DC:
		*mq = SR_MQ_VOLTAGE;
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_DC;
		*exponent = -3;
		break;
	case UT61EP_UNIT_HZ:
		*mq = SR_MQ_FREQUENCY;
		*unit = SR_UNIT_HERTZ;
		break;
	case UT61EP_UNIT_DUTY:
		*mq = SR_MQ_DUTY_CYCLE;
		*unit = SR_UNIT_PERCENTAGE;
		break;
	case UT61EP_UNIT_OHM:
	case UT61EP_UNIT_OHM2:
		*mq = SR_MQ_RESISTANCE;
		*unit = SR_UNIT_OHM;
		break;
	case UT61EP_UNIT_V_DIODE:
		*mq = SR_MQ_VOLTAGE;
		*unit = SR_UNIT_VOLT;
		*mqflags = SR_MQFLAG_DIODE | SR_MQFLAG_DC;
		break;
	case UT61EP_UNIT_CAP:
		*mq = SR_MQ_CAPACITANCE;
		*unit = SR_UNIT_FARAD;
		break;
	case UT61EP_UNIT_TEMP_C:
		*mq = SR_MQ_TEMPERATURE;
		*unit = SR_UNIT_CELSIUS;
		break;
	case UT61EP_UNIT_TEMP_F:
		*mq = SR_MQ_TEMPERATURE;
		*unit = SR_UNIT_FAHRENHEIT;
		break;
	case UT61EP_UNIT_UA_DC:
		*mq = SR_MQ_CURRENT;
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_DC;
		*exponent = -6;
		break;
	case UT61EP_UNIT_UA_AC:
		*mq = SR_MQ_CURRENT;
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		*exponent = -6;
		break;
	case UT61EP_UNIT_MA_DC:
		*mq = SR_MQ_CURRENT;
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_DC;
		*exponent = -3;
		break;
	case UT61EP_UNIT_MA_AC:
		*mq = SR_MQ_CURRENT;
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		*exponent = -3;
		break;
	case UT61EP_UNIT_A_DC:
		*mq = SR_MQ_CURRENT;
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_DC;
		break;
	case UT61EP_UNIT_A_AC:
		*mq = SR_MQ_CURRENT;
		*unit = SR_UNIT_AMPERE;
		*mqflags = SR_MQFLAG_AC | SR_MQFLAG_RMS;
		break;
	case UT61EP_UNIT_NCV:
		*mq = SR_MQ_VOLTAGE;
		*unit = SR_UNIT_VOLT;
		break;
	case UT61EP_UNIT_LOZ_V:
		*mq = SR_MQ_VOLTAGE;
		*unit = SR_UNIT_VOLT;
		break;
	default:
		sr_dbg("Unknown unit code: %d.", unit_code);
		*mq = SR_MQ_VOLTAGE;
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

	i = 0;
	dot++;
	while (dot < s + len && isdigit((unsigned char)*dot)) {
		i++;
		dot++;
	}
	return i;
}

static gboolean value_is_ol(const char *s, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (s[i] == 'O' || s[i] == 'L')
			return TRUE;
	}
	return FALSE;
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
	enum sr_mq mq;
	enum sr_unit unit;
	enum sr_mqflag mqflags;
	int exponent, digits;
	uint8_t flags2;
	char valstr[UT61EP_DATA_LEN + 1];

	devc = sdi->priv;

	memcpy(valstr, buf + UT61EP_DATA_OFFSET, UT61EP_DATA_LEN);
	valstr[UT61EP_DATA_LEN] = '\0';

	sr_spew("UT61E+ raw: unit=%d range='%c' val='%s' f1=%02x f2=%02x",
	        buf[UT61EP_UNIT_OFFSET], buf[UT61EP_RANGE_OFFSET],
	        valstr, buf[UT61EP_FLAGS1_OFFSET], buf[UT61EP_FLAGS2_OFFSET]);

	parse_unit(buf[UT61EP_UNIT_OFFSET], &mq, &unit, &mqflags, &exponent);

	/* Check for overload (OL) */
	if (value_is_ol(valstr, UT61EP_DATA_LEN)) {
		sr_spew("Overload detected.");
		floatval = INFINITY;
		digits = 0;
	} else {
		floatval = (float)strtod(valstr, NULL);
		if (exponent != 0)
			floatval *= powf(10.0f, (float)exponent);
		digits = count_digits(valstr, UT61EP_DATA_LEN);
		if (exponent < 0)
			digits += (-exponent);
	}

	/* Parse flags byte 2 */
	flags2 = buf[UT61EP_FLAGS2_OFFSET];
	if (flags2 & UT61EP_FLAG2_HOLD)
		mqflags |= SR_MQFLAG_HOLD;
	if (flags2 & UT61EP_FLAG2_REL)
		mqflags |= SR_MQFLAG_RELATIVE;
	if (flags2 & UT61EP_FLAG2_AUTO)
		mqflags |= SR_MQFLAG_AUTORANGE;

	/* Build and send analog packet */
	sr_analog_init(&analog, &encoding, &meaning, &spec, digits);
	analog.meaning->mq = mq;
	analog.meaning->unit = unit;
	analog.meaning->mqflags = mqflags;
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

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;
	usb = sdi->conn;

	if (send_request(usb) != SR_OK)
		return FALSE;

	if (read_response(usb, buf) != SR_OK)
		return TRUE;

	decode_packet(sdi, buf);

	if (sr_sw_limits_check(&devc->limits))
		sr_dev_acquisition_stop(sdi);

	return TRUE;
}
