/*
 * Copyright (c) 2011 Jan Vesely
 * Copyright (c) 2011 Vojtech Horky
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
/** @addtogroup drvusbohci
 * @{
 */
/** @file
 * Main routines of OHCI driver.
 */
#include <ddf/driver.h>
#include <ddf/interrupt.h>
#include <device/hw_res.h>
#include <errno.h>
#include <str_error.h>

#include <usb_iface.h>
#include <usb/ddfiface.h>
#include <usb/debug.h>

#include "pci.h"
#include "iface.h"
#include "hc.h"

static int ohci_add_device(ddf_dev_t *device);
static int get_hc_handle(ddf_fun_t *fun, devman_handle_t *handle)
{
	assert(handle);
  assert(fun != NULL);

  *handle = fun->handle;
  return EOK;
}
/*----------------------------------------------------------------------------*/
static int get_address(
    ddf_fun_t *fun, devman_handle_t handle, usb_address_t *address)
{
	assert(fun);
	usb_device_keeper_t *manager = &fun_to_hc(fun)->manager;
  usb_address_t addr = usb_device_keeper_find(manager, handle);
  if (addr < 0) {
    return addr;
  }

  if (address != NULL) {
    *address = addr;
  }

  return EOK;
}
/*----------------------------------------------------------------------------*/
/** IRQ handling callback, identifies device
 *
 * @param[in] dev DDF instance of the device to use.
 * @param[in] iid (Unused).
 * @param[in] call Pointer to the call that represents interrupt.
 */
static void irq_handler(ddf_dev_t *dev, ipc_callid_t iid, ipc_call_t *call)
{
	assert(dev);
	hc_t *hc = (hc_t*)dev->driver_data;
	assert(hc);
	hc_interrupt(hc, 0);
}
/*----------------------------------------------------------------------------*/
static driver_ops_t ohci_driver_ops = {
	.add_device = ohci_add_device,
};
/*----------------------------------------------------------------------------*/
static driver_t ohci_driver = {
	.name = NAME,
	.driver_ops = &ohci_driver_ops
};
/*----------------------------------------------------------------------------*/
static usb_iface_t hc_usb_iface = {
	.get_address = get_address,
	.get_hc_handle = get_hc_handle,
};
/*----------------------------------------------------------------------------*/
static ddf_dev_ops_t hc_ops = {
	.interfaces[USB_DEV_IFACE] = &hc_usb_iface,
	.interfaces[USBHC_DEV_IFACE] = &hc_iface,
};
/*----------------------------------------------------------------------------*/
/** Initializes a new ddf driver instance of OHCI hcd.
 *
 * @param[in] device DDF instance of the device to initialize.
 * @return Error code.
 */
static int ohci_add_device(ddf_dev_t *device)
{
	assert(device);
#define CHECK_RET_RETURN(ret, message...) \
if (ret != EOK) { \
	usb_log_error(message); \
	return ret; \
}

	uintptr_t mem_reg_base = 0;
	size_t mem_reg_size = 0;
	int irq = 0;

	int ret =
	    pci_get_my_registers(device, &mem_reg_base, &mem_reg_size, &irq);
	CHECK_RET_RETURN(ret,
	    "Failed(%d) to get memory addresses:.\n", ret, device->handle);
	usb_log_info("Memory mapped regs at 0x%X (size %zu), IRQ %d.\n",
	    mem_reg_base, mem_reg_size, irq);

	ret = pci_disable_legacy(device);
	CHECK_RET_RETURN(ret,
	    "Failed(%d) disable legacy USB: %s.\n", ret, str_error(ret));

	hc_t *hcd = malloc(sizeof(hc_t));
	if (hcd == NULL) {
		usb_log_error("Failed to allocate OHCI driver.\n");
		return ENOMEM;
	}

	ddf_fun_t *hc_fun = ddf_fun_create(device, fun_exposed, "ohci-hc");
	if (hc_fun == NULL) {
		usb_log_error("Failed to create OHCI function.\n");
		free(hcd);
		return ENOMEM;
	}


	bool interrupts = false;
#ifdef CONFIG_USBHC_NO_INTERRUPTS
	usb_log_warning("Interrupts disabled in OS config, " \
	    "falling back to polling.\n");
#else
	ret = pci_enable_interrupts(device);
	if (ret != EOK) {
		usb_log_warning("Failed to enable interrupts: %s.\n",
		    str_error(rc));
		usb_log_info("HW interrupts not available, " \
		    "falling back to polling.\n");
	} else {
		usb_log_debug("Hw interrupts enabled.\n");
		interrupts = true;
	}
#endif

	ret = hc_init(hcd, hc_fun, device, mem_reg_base, mem_reg_size, interrupts);
	if (ret != EOK) {
		usb_log_error("Failed to initialize OHCI driver.\n");
		free(hcd);
		return ret;
	}

	ret = register_interrupt_handler(device, irq, irq_handler, NULL);

	hc_fun->ops = &hc_ops;
	ret = ddf_fun_bind(hc_fun);
	if (ret != EOK) {
		usb_log_error("Failed to bind OHCI function.\n");
		ddf_fun_destroy(hc_fun);
		free(hcd);
		return ret;
	}
	hc_fun->driver_data = hcd;

	fid_t later = fibril_create((int(*)(void*))hc_register_hub, hcd);
	fibril_add_ready(later);

	usb_log_info("Controlling new OHCI device `%s' (handle %llu).\n",
	    device->name, device->handle);

	return EOK;
#undef CHECK_RET_RETURN
}
/*----------------------------------------------------------------------------*/
/** Initializes global driver structures (NONE).
 *
 * @param[in] argc Nmber of arguments in argv vector (ignored).
 * @param[in] argv Cmdline argument vector (ignored).
 * @return Error code.
 *
 * Driver debug level is set here.
 */
int main(int argc, char *argv[])
{
	usb_log_enable(USB_LOG_LEVEL_DEFAULT, NAME);
	sleep(5);
	return ddf_driver_main(&ohci_driver);
}
/**
 * @}
 */
