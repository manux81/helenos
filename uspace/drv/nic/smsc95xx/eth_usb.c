/*
 * Copyright (c) 2020 Manuele Conti
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file eth_usb.c
 *
 * Driver for SMSC95XX USB Ethernet.
 *
 */

#include <usb/classes/classes.h>
#include <usb/dev/request.h>
#include <usb/dev/poll.h>
#include <usb/debug.h>
#include <byteorder.h>

#include "eth_usb.h"
#include "driver.h"

/* Endpoint 0: Control */
static const usb_endpoint_description_t usb_smsc95xx_in_ctr_endpoint_description = {
	.transfer_type = USB_TRANSFER_CONTROL,
	.direction = USB_DIRECTION_IN,
	.interface_class = USB_CLASS_VENDOR_SPECIFIC,
	.interface_subclass = 0x0,
	.interface_protocol = 0x0,
	.flags = 0
};

/* Endpoint 1: Bulk In */
static const usb_endpoint_description_t usb_smsc95xx_in_bulk_endpoint_description = {
	.transfer_type = USB_TRANSFER_BULK,
	.direction = USB_DIRECTION_IN,
	.interface_class = USB_CLASS_VENDOR_SPECIFIC,
	.interface_subclass = 0x0,
	.interface_protocol = 0x0,
	.flags = 0
};

/* Endpoint 1: Bulk Out */
static const usb_endpoint_description_t usb_smsc95xx_out_bulk_endpoint_description = {
	.transfer_type = USB_TRANSFER_BULK,
	.direction = USB_DIRECTION_OUT,
	.interface_class = USB_CLASS_VENDOR_SPECIFIC,
	.interface_subclass = 0x0,
	.interface_protocol = 0x0,
	.flags = 0
};

/* Endpoint 2: Interrupt */
static const usb_endpoint_description_t usb_smsc95xx_out_int_endpoint_description = {
	.transfer_type = USB_TRANSFER_INTERRUPT,
	.direction = USB_DIRECTION_OUT,
	.interface_class = USB_CLASS_VENDOR_SPECIFIC,
	.interface_subclass = 0x0,
	.interface_protocol = 0x0,
	.flags = 0
};

/* Array of endpoints expected on the device, NULL terminated. */
const usb_endpoint_description_t *endpoints[] = {
	&usb_smsc95xx_in_ctr_endpoint_description,
	&usb_smsc95xx_in_bulk_endpoint_description,
	&usb_smsc95xx_out_bulk_endpoint_description,
	&usb_smsc95xx_out_int_endpoint_description,
	NULL
};

/** Initialize SMSC95xx USB device.
 *
 * @param smsc95xx    SMSC95xx device structure.
 * @param usb_device  Connected USB device.
 *
 * @return EOK if succeed, error code otherwise.
 *
 */
errno_t smsc95xx_usb_init(smsc95xx_t *smsc95xx, usb_device_t *usb_device, const usb_endpoint_description_t **endpoints)
{
	smsc95xx_usb_t *smsc95xx_usb = NULL;
	int rc = EOK;

	smsc95xx_usb = malloc(sizeof(smsc95xx_usb_t));
	if (!smsc95xx_usb) {
		usb_log_error("Failed to allocate memory for smsc95xx usb device "
		    "structure.\n");
		rc = ENOMEM;
		goto exit;
	}

	for (int p = 0; p < SMSC95XX_NUM_ENDPOINTS; p++) {
		usb_endpoint_mapping_t *epm = usb_device_get_mapped_ep_desc(usb_device, endpoints[p]);
		if (!epm || !epm->present) {
			usb_log_error("Failed to map endpoint: %d.", p);
			rc = ENOENT;
			free(smsc95xx_usb);
			goto exit;
		}
		smsc95xx_usb->endpoint_pipe[p] = &epm->pipe;
	}

	smsc95xx_usb->usb_device =  usb_device;
	smsc95xx->smsc95xx_usb = smsc95xx_usb;

exit:
	return rc;
}

/** Write SMC95XX register.
 *
 * @param smsc95xx    SMSC95XX device structure.
 * @param index       Value of wIndex field of setup packet.
 * @param data        Data to be sent during DATA stage.
 *
 * @return EOK if succeed, error code otherwise.
 *
 */
errno_t smsc95xx_usb_write_reg(smsc95xx_t *smsc95xx, uint16_t index, uint32_t data)
{
	uint32_t tmpbuf = 0U;
	const usb_device_request_setup_packet_t setup_packet = {
		.request_type = 0x40,
		.request = 0xA0,
		.value = 0x00,
		.index = index,
		.length = sizeof(data),
	};
	smsc95xx_usb_t *smsc95xx_usb = smsc95xx->smsc95xx_usb;

	tmpbuf = host2int32_t_le(data);
	return usb_pipe_control_write(smsc95xx_usb->endpoint_pipe[ctrl_in_ep],
	    &setup_packet, sizeof(setup_packet), &tmpbuf, sizeof(tmpbuf));
}

/** Read SMSC95XX register.
 *
 * @param smsc95xx    SMSC95XX device structure.
 * @param index       Value of wIndex field of setup packet.
 * @param data        Data to be sent during DATA stage.
 *
 * @return EOK if succeed, error code otherwise.
 *
 */
errno_t smsc95xx_usb_read_reg(smsc95xx_t *smsc95xx, uint16_t index, uint32_t *data)
{
	size_t len = 0;
	uint32_t tmpbuf = 0U;
	smsc95xx_usb_t *smsc95xx_usb = NULL;
	const usb_device_request_setup_packet_t setup_packet = {
		.request_type = 0xC0,
		.request = 0xA1,
		.value = 0x00,
		.index = index,
		.length = sizeof(*data),
	};

	assert(smsc95xx);
	smsc95xx_usb = smsc95xx->smsc95xx_usb;
	int rc = usb_pipe_control_read(smsc95xx_usb->endpoint_pipe[ctrl_in_ep],
	    &setup_packet, sizeof(setup_packet), &tmpbuf, sizeof(tmpbuf), &len);

	if (rc != EOK) {
		goto error;
	}

	if (len != sizeof(*data)) {
		usb_log_error("smsc95xx_read_reg failed: index:=%d, len=%zd",
		    index, len);
		rc = EIO;
		goto error;
	}

	*data = uint32_t_le2host(tmpbuf);

	return EOK;

error:
	return rc;
}


