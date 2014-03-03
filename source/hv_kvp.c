/*-
 * Copyright (c) 2009-2012 Microsoft Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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

/*-
 * Copyright (c) 2007 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Dieter Baron.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* An implementation of key value pair (KVP) functionality for FreeBSD */

/*
 * Code for handling all KVP related messages
 */

/* Headers */
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/reboot.h>
#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/kthread.h>
#include <sys/socket.h>
#include <sys/syscallsubr.h>
#include <sys/sysproto.h>
#include <sys/un.h>
#include <sys/endian.h>
#include <net/if_arp.h>

/* hyper-v headers */
#include <hyperv.h>
#include <dev/hyperv/netvsc/hv_net_vsc.h>
#include "hv_kvp.h"

/* Unicode Conversions */
#include <sys/cdefs.h>
#include <sys/_null.h>


/*
 * hv_kvp probe function 
 */
static int
hv_kvp_probe(device_t dev)
{
	int rtn_value = ENXIO;
	
	const char *p = vmbus_get_type(dev);

	if (service_table_kvp.enabled && 
	    !memcmp(p, &service_table_kvp.guid, sizeof(hv_guid))) {
		device_set_softc(dev, (void *)(&service_table_kvp));
		rtn_value = 0;
	}
	return (rtn_value);
}

/*
 * hv_kvp attach function 
 */
static int
hv_kvp_attach(device_t dev)
{
	struct hv_device        *hv_dev;
	struct hv_vmbus_service *service;
	int    ret;
	size_t receive_buffer_offset;

	hv_dev  = vmbus_get_devctx(dev);
	service = device_get_softc(dev);
	receive_buffer_offset = service - &service_table_kvp;
	device_printf(dev, "Hyper-V Service attaching: %s\n", service->name);
	receive_buffer[receive_buffer_offset] =
	    malloc(4 * PAGE_SIZE, M_DEVBUF, M_WAITOK | M_ZERO);
	if (service->init != NULL) {
		ret = service->init(service);
		if (ret) {
			ret = ENODEV;
			goto error0;
		}
	}
	ret = hv_vmbus_channel_open(hv_dev->channel, 4 * PAGE_SIZE,
		4 * PAGE_SIZE, NULL, 0,
		service->callback, hv_dev->channel);

	if (ret) {
		goto error0;
	}

	return (0);

error0:

	free(receive_buffer[receive_buffer_offset], M_DEVBUF);
	receive_buffer[receive_buffer_offset] = NULL;

	return (ret);
}

/*
 * hv_kvp detach function 
 */
static int
hv_kvp_detach(device_t dev)
{
	struct hv_device        *hv_dev;
	struct hv_vmbus_service *service;
	size_t receive_buffer_offset;

	hv_dev = vmbus_get_devctx(dev);

	hv_vmbus_channel_close(hv_dev->channel);
	service = device_get_softc(dev);
	receive_buffer_offset = service - &service_table_kvp;

	if (service->work_queue != NULL) {
		hv_work_queue_close(service->work_queue);
	}

	free(receive_buffer[receive_buffer_offset], M_DEVBUF);
	receive_buffer[receive_buffer_offset] = NULL;

	return (0);
}

/* KVP main function */
static void hv_kvp_main(void)
{
}

/* KVP module event */
static int hv_kvp_modevent(module_t mod, int event, void *arg)
{
	switch (event)
	{
	case MOD_LOAD:
		break;

	case MOD_UNLOAD:
		break;

	default:
		break;
	}
	return (0);
}

/* Device Interface Methods */
static device_method_t kvp_methods[] =
{
	/* Device interface */
	DEVMETHOD(device_probe,    hv_kvp_probe),
	DEVMETHOD(device_attach,   hv_kvp_attach),
	DEVMETHOD(device_detach,   hv_kvp_detach),
	DEVMETHOD(device_shutdown, bus_generic_shutdown),
	{ 0, 0 }
}

static driver_t kvp_driver = { "hyperv-kvp", kvp_methods, 0 };

static devclass_t kvp_devclass;

DRIVER_MODULE(hv_kvp, vmbus, kvp_driver, kvp_devclass, hv_kvp_modevent, 0);
MODULE_VERSION(hv_kvp, 1);
MODULE_DEPEND(hv_kvp, vmbus, 1, 1, 1);

SYSINIT(hv_kvp_initx, SI_SUB_KTHREAD_IDLE, SI_ORDER_MIDDLE + 1, hv_kvp_main, NULL);
