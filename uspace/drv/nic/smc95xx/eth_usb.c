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
 * Driver for SMC95XX USB Ethernet.
 *
 */

#include <usb/classes/classes.h>
#include <usb/dev/request.h>
#include <usb/dev/poll.h>
#include <usb/debug.h>

#include "eth_usb.h"
#include "driver.h"

/* Endpoint 0: Control */
static const usb_endpoint_description_t usb_smc95xx_in_ctr_endpoint_description = {
	.transfer_type = USB_TRANSFER_CONTROL,
	.direction = USB_DIRECTION_IN,
	.interface_class = USB_CLASS_VENDOR_SPECIFIC,
	.interface_subclass = 0x0,
	.interface_protocol = 0x0,
	.flags = 0
};

/* Endpoint 1: Bulk In */
static const usb_endpoint_description_t usb_smc95xx_in_bulk_endpoint_description = {
	.transfer_type = USB_TRANSFER_BULK,
	.direction = USB_DIRECTION_IN,
	.interface_class = USB_CLASS_VENDOR_SPECIFIC,
	.interface_subclass = 0x0,
	.interface_protocol = 0x0,
	.flags = 0
};

/* Endpoint 1: Bulk Out */
static const usb_endpoint_description_t usb_smc95xx_out_bulk_endpoint_description = {
	.transfer_type = USB_TRANSFER_BULK,
	.direction = USB_DIRECTION_OUT,
	.interface_class = USB_CLASS_VENDOR_SPECIFIC,
	.interface_subclass = 0x0,
	.interface_protocol = 0x0,
	.flags = 0
};

/* Endpoint 2: Interrupt */
static const usb_endpoint_description_t usb_smc95xx_out_int_endpoint_description = {
	.transfer_type = USB_TRANSFER_INTERRUPT,
	.direction = USB_DIRECTION_OUT,
	.interface_class = USB_CLASS_VENDOR_SPECIFIC,
	.interface_subclass = 0x0,
	.interface_protocol = 0x0,
	.flags = 0
};

/* Array of endpoints expected on the device, NULL terminated. */
const usb_endpoint_description_t *endpoints[] = {
	&usb_smc95xx_in_ctr_endpoint_description,
	&usb_smc95xx_in_bulk_endpoint_description,
	&usb_smc95xx_out_bulk_endpoint_description,
	&usb_smc95xx_out_int_endpoint_description,
	NULL
};

/** Initialize SMSC95xx USB device.
 *
 * @param smc95xx smc96xx device structure.
 * @param usb_device  Connected USB device.
 *
 * @return EOK if succeed, error code otherwise.
 *
 */
errno_t smc95xx_usb_init(smc95xx_t *smc95xx, usb_device_t *usb_device, const usb_endpoint_description_t **endpoints)
{
	smc95xx_usb_t *smc95xx_usb = NULL;
	int rc = EOK;

	smc95xx_usb = malloc(sizeof(smc95xx_usb_t));
	if (!smc95xx_usb) {
		usb_log_error("Failed to allocate memory for smc95xx usb device "
		    "structure.\n");
		rc = ENOMEM;
		goto exit;
	}

	for (int p = 0; p < SMC95XX_NUM_ENDPOINTS; p++) {
		usb_endpoint_mapping_t *epm = usb_device_get_mapped_ep_desc(usb_device, endpoints[p]);
		if (!epm || !epm->present) {
			usb_log_error("Failed to map endpoint: %d.", p);
			rc = ENOENT;
			free(smc95xx_usb);
			goto exit;
		}
		smc95xx_usb->endpoint_pipe[p] = &epm->pipe;
	}

	smc95xx_usb->usb_device =  usb_device;
	smc95xx->smc95xx_usb = smc95xx_usb;

exit:
	return rc;
}

/** Send control message.
 *
 * @param smc95xx     SMC95XX device structure.
 * @param buffer      Buffer with data to send.
 * @param buffer_size Buffer size.
 *
 * @return EOK if succeed, error code otherwise.
 *
 */
errno_t smc95xx_usb_send_ctrl_message(smc95xx_t *smc95xx, void *buffer,
    size_t buffer_size)
{
	smc95xx_usb_t *smc95xx_usb = smc95xx->smc95xx_usb;
	return usb_pipe_write(smc95xx_usb->endpoint_pipe[ctrl_in_ep], buffer, buffer_size);
}

/** Read control message.
 *
 * @param smc95xx          SMC95XX device structure.
 * @param buffer           Buffer with data to send.
 * @param buffer_size      Buffer size.
 * @param transferred_size Real size of read data.
 *
 * @return EOK if succeed, error code otherwise.
 *
 */
errno_t smc95xx_usb_read_ctrl_message(smc95xx_t *smc95xx, void *buffer,
    size_t buffer_size, size_t *transferred_size)
{
	smc95xx_usb_t *smc95xx_usb = smc95xx->smc95xx_usb;
	return usb_pipe_read(smc95xx_usb->endpoint_pipe[ctrl_in_ep], buffer, buffer_size, transferred_size);
}



