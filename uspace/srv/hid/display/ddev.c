/*
 * Copyright (c) 2019 Jiri Svoboda
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

/** @addtogroup display
 * @{
 */
/**
 * @file Display server display device
 */

#include <adt/list.h>
#include <ddev.h>
#include <errno.h>
#include <io/log.h>
#include <stdio.h>
#include <stdlib.h>
#include "display.h"
#include "ddev.h"

/** Open display device.
 *
 * @param display Parent display
 * @param svc_id Service ID
 * @param rddev Place to store pointer to new display device.
 * @return EOK on success, ENOMEM if out of memory
 */
errno_t ds_ddev_open(ds_display_t *display, service_id_t svc_id,
    ds_ddev_t **rddev)
{
	ds_ddev_t *ddev;
	ddev_info_t info;
	gfx_context_t *gc;
	ddev_t *dd = NULL;
	char *name = NULL;
	errno_t rc;

	rc = loc_service_get_name(svc_id, &name);
	if (rc != EOK) {
		printf("Error resolving name of service %lu.\n",
		    (unsigned long) svc_id);
		return rc;
	}

	rc = ddev_open(name, &dd);
	if (rc != EOK) {
		printf("Error opening display device '%s'.\n", name);
		free(name);
		return rc;
	}

	rc = ddev_get_info(dd, &info);
	if (rc != EOK) {
		printf("Error getting information for display device '%s'.\n",
		    name);
		free(name);
		ddev_close(dd);
		return rc;
	}

	log_msg(LOG_DEFAULT, LVL_NOTE, "Device rectangle for '%s': "
	    "%d,%d,%d,%d\n", name, info.rect.p0.x, info.rect.p0.y,
	    info.rect.p1.x, info.rect.p1.y);

	rc = ddev_get_gc(dd, &gc);
	if (rc != EOK) {
		printf("Error getting device context for '%s'.\n", name);
		ddev_close(dd);
		free(name);
		return rc;
	}

	ddev = calloc(1, sizeof(ds_ddev_t));
	if (ddev == NULL) {
		free(name);
		ddev_close(dd);
		return ENOMEM;
	}

	ddev->svc_name = name;
	ddev->svc_id = svc_id;
	ddev->dd = dd;
	ddev->gc = gc;
	ddev->info = info;

	ds_display_add_ddev(display, ddev);

	rc = ds_display_paint_bg(display, NULL);
	if (rc != EOK)
		return rc;

	*rddev = ddev;
	return EOK;
}

/** Destroy display device.
 *
 * @param ddev Display device
 */
void ds_ddev_close(ds_ddev_t *ddev)
{
	ds_display_remove_ddev(ddev);
	free(ddev);
}

/** @}
 */
