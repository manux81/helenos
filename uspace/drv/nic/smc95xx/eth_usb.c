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
errno_t smc95xx_usb_init(smc95xx_usb_t *smc95xx, usb_device_t *usb_device, const usb_endpoint_description_t **endpoints)
{
	return EOK;
}


