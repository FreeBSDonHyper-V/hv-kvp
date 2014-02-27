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

DRIVER_MODULE(hv_kvp, vmbus, kvp_driver, kvp_devclass, hv_kvp_modevent, 0);
MODULE_VERSION(hv_kvp, 1);
MODULE_DEPEND(hv_kvp, vmbus, 1, 1, 1);

SYSINIT(hv_kvp_initx, SI_SUB_KTHREAD_IDLE, SI_ORDER_MIDDLE + 1, hv_kvp_main, NULL);
