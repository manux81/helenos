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

#include <assert.h>
#include <async.h>
#include <errno.h>
#include <align.h>
#include <byteorder.h>
#include <barrier.h>
#include <as.h>
#include <ddf/log.h>
#include <ddf/interrupt.h>
#include <device/hw_res.h>
#include <io/log.h>
#include <nic.h>
#include <stdio.h>
#include <str.h>
#include <usb/classes/classes.h>
#include <usb/dev/request.h>
#include <usb/dev/poll.h>
#include <usb/debug.h>
#include <str_error.h>

#include "driver.h"
#include "eth_usb.h"

static errno_t smsc95xx_get_device_info(ddf_fun_t *fun, nic_device_info_t *info);

/** Network interface options for SMC95XX card driver */
static nic_iface_t smsc95xx_nic_iface = {
	.get_device_info = &smsc95xx_get_device_info,
};

/** Basic device operations for SMC95XX driver */
static ddf_dev_ops_t smsc95xx_dev_ops;

/** Get device information.
 *
 */
static errno_t smsc95xx_get_device_info(ddf_fun_t *dev, nic_device_info_t *info)
{
	assert(dev);
	assert(info);

	memset(info, 0, sizeof(nic_device_info_t));
	info->vendor_id = 0x0424;
	info->device_id = 0xec00;
	str_cpy(info->vendor_name, NIC_VENDOR_MAX_LENGTH,
	    "Microchip Technology, Inc.");
	str_cpy(info->model_name, NIC_MODEL_MAX_LENGTH,
	    "LAN9512/LAN9514");

	return EOK;
}


/** Create driver data structure.
 * @param dev The device structure
 *
 * @return Initialized device data structure or NULL if error occured
 */
static smsc95xx_t *smc95xx_create_dev_data(ddf_dev_t *dev)
{
	errno_t rc = EOK;
	const char *err_msg = NULL;

	/* USB framework initialization. */
	rc = usb_device_create_ddf(dev, endpoints, &err_msg);
	if (rc != EOK) {
		usb_log_error("Failed to create USB device: %s, "
		    "ERR_NUM = %s\n", err_msg, str_error_name(rc));
		return NULL;
	}

	/* SMC95XX structure initialization. */
	smsc95xx_t *smsc95xx = calloc(1, sizeof(smsc95xx_t));
	if (!smsc95xx) {
		usb_log_error("Failed to initialize SMSC95XX structure: %s\n",
		    str_error_name(rc));
		return NULL;
	}

	smsc95xx->ddf_dev = dev;

	fibril_mutex_initialize(&smsc95xx->lock);

	rc = smsc95xx_usb_init(smsc95xx, usb_device_get(dev), endpoints);
	if (rc != EOK) {
		free(smsc95xx);
		usb_log_error("Failed to initialize SMC95XX structure: %s\n",
		    str_error_name(rc));
		return NULL;
	}

	return smsc95xx;
}

static errno_t smsc95xx_dev_add(ddf_dev_t *dev);

/** Basic driver operations for SMC95XX driver */
static driver_ops_t smsc95xx_driver_ops = {
	.dev_add = &smsc95xx_dev_add,
};

/** Driver structure for RTL8139 driver */
static driver_t smsc95xx_driver = {
	.name = NAME,
	.driver_ops = &smsc95xx_driver_ops
};

/** The add_device callback of SMC95XX callback
 *
 * Probe and initialize the newly added device.
 *
 * @param dev  The SMSC95XX device.
 *
 * @return EOK if added successfully, error code otherwise
 */
errno_t smsc95xx_dev_add(ddf_dev_t *dev)
{
	assert(dev);

	smsc95xx_t *smsc95xx = smc95xx_create_dev_data(dev);
	if (smsc95xx == NULL) {
		usb_log_error("Unable to allocate device softstate.\n");
		return ENOMEM;
	}

	usb_log_info("HelenOS SMSC95XX device initialized.\n");

	return EOK;
}

/** Main function of SMSC95XX driver
 *
 *  Just initialize the driver structures and
 *  put it into the device drivers interface
 */
int main(void)
{
	printf("%s: HelenOS SMSC95XX network adapter driver\n", NAME);

	errno_t rc = nic_driver_init(NAME);
	if (rc != EOK)
		return rc;

	nic_driver_implement(&smsc95xx_driver_ops, &smsc95xx_dev_ops,
	    &smsc95xx_nic_iface);

	ddf_log_init(NAME);

	return ddf_driver_main(&smsc95xx_driver);
}

