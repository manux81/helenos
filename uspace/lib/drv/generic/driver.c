/*
 * Copyright (c) 2010 Lenka Trochtova
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

/**
 * @defgroup libdrv generic device driver support.
 * @brief HelenOS generic device driver support.
 * @{
 */

/** @file
 */

#include <assert.h>
#include <ipc/services.h>
#include <ipc/ns.h>
#include <async.h>
#include <stdio.h>
#include <errno.h>
#include <bool.h>
#include <fibril_synch.h>
#include <stdlib.h>
#include <str.h>
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>

#include <ipc/driver.h>

#include "dev_iface.h"
#include "driver.h"

/** Driver structure */
static driver_t *driver;

/** Devices */
LIST_INITIALIZE(functions);
FIBRIL_MUTEX_INITIALIZE(functions_mutex);

/** Interrupts */
static interrupt_context_list_t interrupt_contexts;

static irq_cmd_t default_cmds[] = {
	{
		.cmd = CMD_ACCEPT
	}
};

static irq_code_t default_pseudocode = {
	sizeof(default_cmds) / sizeof(irq_cmd_t),
	default_cmds
};


static void driver_irq_handler(ipc_callid_t iid, ipc_call_t *icall)
{
	int id = (int)IPC_GET_IMETHOD(*icall);
	interrupt_context_t *ctx;
	
	ctx = find_interrupt_context_by_id(&interrupt_contexts, id);
	if (ctx != NULL && ctx->handler != NULL)
		(*ctx->handler)(ctx->dev, iid, icall);
}

interrupt_context_t *create_interrupt_context(void)
{
	interrupt_context_t *ctx;
	
	ctx = (interrupt_context_t *) malloc(sizeof(interrupt_context_t));
	if (ctx != NULL)
		memset(ctx, 0, sizeof(interrupt_context_t));
	
	return ctx;
}

void delete_interrupt_context(interrupt_context_t *ctx)
{
	if (ctx != NULL)
		free(ctx);
}

void init_interrupt_context_list(interrupt_context_list_t *list)
{
	memset(list, 0, sizeof(interrupt_context_list_t));
	fibril_mutex_initialize(&list->mutex);
	list_initialize(&list->contexts);
}

void
add_interrupt_context(interrupt_context_list_t *list, interrupt_context_t *ctx)
{
	fibril_mutex_lock(&list->mutex);
	ctx->id = list->curr_id++;
	list_append(&ctx->link, &list->contexts);
	fibril_mutex_unlock(&list->mutex);
}

void remove_interrupt_context(interrupt_context_list_t *list,
    interrupt_context_t *ctx)
{
	fibril_mutex_lock(&list->mutex);
	list_remove(&ctx->link);
	fibril_mutex_unlock(&list->mutex);
}

interrupt_context_t *
find_interrupt_context_by_id(interrupt_context_list_t *list, int id)
{
	fibril_mutex_lock(&list->mutex);
	
	link_t *link = list->contexts.next;
	interrupt_context_t *ctx;
	
	while (link != &list->contexts) {
		ctx = list_get_instance(link, interrupt_context_t, link);
		if (ctx->id == id) {
			fibril_mutex_unlock(&list->mutex);
			return ctx;
		}
		link = link->next;
	}
	
	fibril_mutex_unlock(&list->mutex);
	return NULL;
}

interrupt_context_t *
find_interrupt_context(interrupt_context_list_t *list, device_t *dev, int irq)
{
	fibril_mutex_lock(&list->mutex);
	
	link_t *link = list->contexts.next;
	interrupt_context_t *ctx;
	
	while (link != &list->contexts) {
		ctx = list_get_instance(link, interrupt_context_t, link);
		if (ctx->irq == irq && ctx->dev == dev) {
			fibril_mutex_unlock(&list->mutex);
			return ctx;
		}
		link = link->next;
	}
	
	fibril_mutex_unlock(&list->mutex);
	return NULL;
}


int
register_interrupt_handler(device_t *dev, int irq, interrupt_handler_t *handler,
    irq_code_t *pseudocode)
{
	interrupt_context_t *ctx = create_interrupt_context();
	
	ctx->dev = dev;
	ctx->irq = irq;
	ctx->handler = handler;
	
	add_interrupt_context(&interrupt_contexts, ctx);
	
	if (pseudocode == NULL)
		pseudocode = &default_pseudocode;
	
	int res = register_irq(irq, dev->handle, ctx->id, pseudocode);
	if (res != EOK) {
		remove_interrupt_context(&interrupt_contexts, ctx);
		delete_interrupt_context(ctx);
	}

	return res;
}

int unregister_interrupt_handler(device_t *dev, int irq)
{
	interrupt_context_t *ctx = find_interrupt_context(&interrupt_contexts,
	    dev, irq);
	int res = unregister_irq(irq, dev->handle);
	
	if (ctx != NULL) {
		remove_interrupt_context(&interrupt_contexts, ctx);
		delete_interrupt_context(ctx);
	}
	
	return res;
}

static void add_to_functions_list(function_t *fun)
{
	fibril_mutex_lock(&functions_mutex);
	list_append(&fun->link, &functions);
	fibril_mutex_unlock(&functions_mutex);
}

static void remove_from_functions_list(function_t *fun)
{
	fibril_mutex_lock(&functions_mutex);
	list_remove(&fun->link);
	fibril_mutex_unlock(&functions_mutex);
}

static function_t *driver_get_function(link_t *functions, devman_handle_t handle)
{
	function_t *fun = NULL;
	printf("driver_get_function handle=%" PRIun "\n", handle);
	
	fibril_mutex_lock(&functions_mutex);
	link_t *link = functions->next;
	
	while (link != functions) {
		fun = list_get_instance(link, function_t, link);
		printf(" - fun handle %" PRIun "\n", fun->handle);
		if (fun->handle == handle) {
			fibril_mutex_unlock(&functions_mutex);
			return fun;
		}
		
		link = link->next;
	}
	
	fibril_mutex_unlock(&functions_mutex);
	
	return NULL;
}

static void driver_add_device(ipc_callid_t iid, ipc_call_t *icall)
{
	char *dev_name = NULL;
	int res;
	
	devman_handle_t dev_handle = IPC_GET_ARG1(*icall);
    	devman_handle_t parent_fun_handle = IPC_GET_ARG2(*icall);
	
	device_t *dev = create_device();
	dev->handle = dev_handle;

	async_data_write_accept((void **) &dev_name, true, 0, 0, 0, 0);
	dev->name = dev_name;

	/*
	 * Currently not used, parent fun handle is stored in context
	 * of the connection to the parent device driver.
	 */
	(void) parent_fun_handle;
	
	res = driver->driver_ops->add_device(dev);
	if (res == EOK) {
		printf("%s: new device with handle=%" PRIun " was added.\n",
		    driver->name, dev_handle);
	} else {
		printf("%s: failed to add a new device with handle = %" PRIun ".\n",
		    driver->name, dev_handle);
		delete_device(dev);
	}
	
	async_answer_0(iid, res);
}

static void driver_connection_devman(ipc_callid_t iid, ipc_call_t *icall)
{
	/* Accept connection */
	async_answer_0(iid, EOK);
	
	bool cont = true;
	while (cont) {
		ipc_call_t call;
		ipc_callid_t callid = async_get_call(&call);
		
		switch (IPC_GET_IMETHOD(call)) {
		case IPC_M_PHONE_HUNGUP:
			cont = false;
			continue;
		case DRIVER_ADD_DEVICE:
			driver_add_device(callid, &call);
			break;
		default:
			async_answer_0(callid, ENOENT);
		}
	}
}

/**
 * Generic client connection handler both for applications and drivers.
 *
 * @param drv		True for driver client, false for other clients
 *			(applications, services etc.).
 */
static void driver_connection_gen(ipc_callid_t iid, ipc_call_t *icall, bool drv)
{
	/*
	 * Answer the first IPC_M_CONNECT_ME_TO call and remember the handle of
	 * the device to which the client connected.
	 */
	devman_handle_t handle = IPC_GET_ARG2(*icall);
	function_t *fun = driver_get_function(&functions, handle);

	if (fun == NULL) {
		printf("%s: driver_connection_gen error - no function with handle"
		    " %" PRIun " was found.\n", driver->name, handle);
		async_answer_0(iid, ENOENT);
		return;
	}
	
	
	/*
	 * TODO - if the client is not a driver, check whether it is allowed to
	 * use the device.
	 */
	
	int ret = EOK;
	/* Open device function */
	if (fun->ops != NULL && fun->ops->open != NULL)
		ret = (*fun->ops->open)(fun);
	
	async_answer_0(iid, ret);
	if (ret != EOK)
		return;
	
	while (1) {
		ipc_callid_t callid;
		ipc_call_t call;
		callid = async_get_call(&call);
		sysarg_t method = IPC_GET_IMETHOD(call);
		int iface_idx;
		
		switch  (method) {
		case IPC_M_PHONE_HUNGUP:
			/* Close device function */
			if (fun->ops != NULL && fun->ops->close != NULL)
				(*fun->ops->close)(fun);
			async_answer_0(callid, EOK);
			return;
		default:
			/* convert ipc interface id to interface index */
			
			iface_idx = DEV_IFACE_IDX(method);
			
			if (!is_valid_iface_idx(iface_idx)) {
				remote_handler_t *default_handler =
				    function_get_default_handler(fun);
				if (default_handler != NULL) {
					(*default_handler)(fun, callid, &call);
					break;
				}
				
				/*
				 * Function has no such interface and
				 * default handler is not provided.
				 */
				printf("%s: driver_connection_gen error - "
				    "invalid interface id %d.",
				    driver->name, iface_idx);
				async_answer_0(callid, ENOTSUP);
				break;
			}
			
			/* calling one of the function's interfaces */
			
			/* Get the interface ops structure. */
			void *ops = function_get_ops(fun, iface_idx);
			if (ops == NULL) {
				printf("%s: driver_connection_gen error - ",
				    driver->name);
				printf("Function with handle %" PRIun " has no interface "
				    "with id %d.\n", handle, iface_idx);
				async_answer_0(callid, ENOTSUP);
				break;
			}
			
			/*
			 * Get the corresponding interface for remote request
			 * handling ("remote interface").
			 */
			remote_iface_t *rem_iface = get_remote_iface(iface_idx);
			assert(rem_iface != NULL);
			
			/* get the method of the remote interface */
			sysarg_t iface_method_idx = IPC_GET_ARG1(call);
			remote_iface_func_ptr_t iface_method_ptr =
			    get_remote_method(rem_iface, iface_method_idx);
			if (iface_method_ptr == NULL) {
				// the interface has not such method
				printf("%s: driver_connection_gen error - "
				    "invalid interface method.", driver->name);
				async_answer_0(callid, ENOTSUP);
				break;
			}
			
			/*
			 * Call the remote interface's method, which will
			 * receive parameters from the remote client and it will
			 * pass it to the corresponding local interface method
			 * associated with the function by its driver.
			 */
			(*iface_method_ptr)(fun, ops, callid, &call);
			break;
		}
	}
}

static void driver_connection_driver(ipc_callid_t iid, ipc_call_t *icall)
{
	driver_connection_gen(iid, icall, true);
}

static void driver_connection_client(ipc_callid_t iid, ipc_call_t *icall)
{
	driver_connection_gen(iid, icall, false);
}

/** Function for handling connections to device driver. */
static void driver_connection(ipc_callid_t iid, ipc_call_t *icall)
{
	/* Select interface */
	switch ((sysarg_t) (IPC_GET_ARG1(*icall))) {
	case DRIVER_DEVMAN:
		/* Handle request from device manager */
		driver_connection_devman(iid, icall);
		break;
	case DRIVER_DRIVER:
		/* Handle request from drivers of child devices */
		driver_connection_driver(iid, icall);
		break;
	case DRIVER_CLIENT:
		/* Handle request from client applications */
		driver_connection_client(iid, icall);
		break;
	default:
		/* No such interface */
		async_answer_0(iid, ENOENT);
	}
}

/** Create new device structure.
 *
 * @return		The device structure.
 */
device_t *create_device(void)
{
	device_t *dev;

	dev = malloc(sizeof(device_t));
	if (dev == NULL)
		return NULL;

	memset(dev, 0, sizeof(device_t));
	return dev;
}

/** Create new function structure.
 *
 * @return		The device structure.
 */
function_t *create_function(void)
{
	function_t *fun;

	fun = malloc(sizeof(function_t));
	if (fun == NULL)
		return NULL;

	memset(fun, 0, sizeof(device_t));

	init_match_ids(&fun->match_ids);
	link_initialize(&fun->link);

	return fun;
}

/** Delete device structure.
 *
 * @param dev		The device structure.
 */
void delete_device(device_t *dev)
{
	free(dev);
}

/** Delete device structure.
 *
 * @param dev		The device structure.
 */
void delete_function(function_t *fun)
{
	clean_match_ids(&fun->match_ids);
	if (fun->name != NULL)
		free(fun->name);
	free(fun);
}

void *function_get_ops(function_t *fun, dev_inferface_idx_t idx)
{
	assert(is_valid_iface_idx(idx));
	if (fun->ops == NULL)
		return NULL;
	return fun->ops->interfaces[idx];
}

int register_function(function_t *fun, device_t *dev)
{
	assert(fun->name != NULL);
	
	int res;
	
	fun->dev = dev;
	
	add_to_functions_list(fun);
	res = devman_add_function(fun->name, fun->ftype, &fun->match_ids,
	    dev->handle, &fun->handle);
	if (res != EOK) {
		remove_from_functions_list(fun);
		return res;
	}
	
	return res;
}

/** Wrapper for child_device_register for devices with single match id.
 *
 * @param parent Parent device.
 * @param child_name Child device name.
 * @param child_match_id Child device match id.
 * @param child_match_score Child device match score.
 * @return Error code.
 */
int register_function_wrapper(device_t *dev, const char *fun_name,
    const char *match_id, int match_score)
{
	function_t *fun = NULL;
	match_id_t *m_id = NULL;
	int rc;
	
	fun = create_function();
	if (fun == NULL) {
		rc = ENOMEM;
		goto failure;
	}
	
	fun->dev = dev;
	fun->name = fun_name;
	fun->ftype = fun_inner;
	
	m_id = create_match_id();
	if (m_id == NULL) {
		rc = ENOMEM;
		goto failure;
	}
	
	m_id->id = match_id;
	m_id->score = match_score;
	add_match_id(&fun->match_ids, m_id);
	
	rc = register_function(fun, dev);
	if (rc != EOK)
		goto failure;
	
	return EOK;
	
failure:
	if (m_id != NULL) {
		m_id->id = NULL;
		delete_match_id(m_id);
	}
	
	if (fun != NULL) {
		fun->name = NULL;
		delete_function(fun);
	}
	
	return rc;
}

/** Get default handler for client requests */
remote_handler_t *function_get_default_handler(function_t *fun)
{
	if (fun->ops == NULL)
		return NULL;
	return fun->ops->default_handler;
}

int add_function_to_class(function_t *fun, const char *class_name)
{
	return devman_add_device_to_class(fun->handle, class_name);
}

int driver_main(driver_t *drv)
{
	/*
	 * Remember the driver structure - driver_ops will be called by generic
	 * handler for incoming connections.
	 */
	driver = drv;
	
	/* Initialize the list of interrupt contexts. */
	init_interrupt_context_list(&interrupt_contexts);
	
	/* Set generic interrupt handler. */
	async_set_interrupt_received(driver_irq_handler);
	
	/*
	 * Register driver by device manager with generic handler for incoming
	 * connections.
	 */
	devman_driver_register(driver->name, driver_connection);
	
	async_manager();
	
	/* Never reached. */
	return 0;
}

/**
 * @}
 */
