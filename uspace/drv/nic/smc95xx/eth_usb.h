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

#ifndef SMC95XX_ETH_USB_H_
#define SMC95XX_ETH_USB_H_

#include <usb/dev/device.h>
#include <usb/dev/pipes.h>

#include "driver.h"

#define SMC95XX_NUM_ENDPOINTS 4

enum smc95xx_endpoints {
	ctrl_in_ep,
	bulk_in_ep,
	bulk_out_ep,
	intr_in_ep
};

/** SMC95xx USB device structure */
typedef struct {
	/** USB pipes indexes */
	usb_pipe_t *endpoint_pipe[SMC95XX_NUM_ENDPOINTS];

	/** Pointer to connected USB device. */
	usb_device_t *usb_device;
} smc95xx_usb_t;

extern const usb_endpoint_description_t *endpoints[];

extern errno_t smc95xx_usb_init(smc95xx_t *, usb_device_t *, const usb_endpoint_description_t **endpoints);
extern errno_t smc95xx_usb_send_ctrl_message(smc95xx_t *, void *, size_t);
extern errno_t smc95xx_usb_read_ctrl_message(smc95xx_t *, void *, size_t, size_t *);

#endif
