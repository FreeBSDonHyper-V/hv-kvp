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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/reboot.h>
#include <sys/lock.h>
#include <sys/taskqueue.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/kthread.h>
#include <sys/syscallsubr.h>
#include <sys/sysproto.h>
#include <sys/un.h>
#include <sys/endian.h>
#include <sys/_null.h>

#include <net/if_arp.h>

#include <dev/hyperv/include/hyperv.h>
#include <dev/hyperv/netvsc/hv_net_vsc.h>

#include "hv_kvp.h"

/* hv_kvp defines */
#define BUFFERSIZE	sizeof(struct hv_kvp_msg)
#define KVP_SUCCESS	0
#define KVP_ERROR	1
#define kvp_hdr		hdr.kvp_hdr

/* prototypes */
static d_open_t hv_kvp_dev_open;
static d_close_t hv_kvp_dev_close;
static d_read_t hv_kvp_dev_daemon_read;
static d_write_t hv_kvp_dev_daemon_write;
static size_t hv_kvp_convert8_to_16(uint16_t *, size_t, const char *, size_t, int *);
static size_t hv_kvp_convert16_to_8(char *, size_t, const uint16_t *, size_t, int *);
static int hv_kvp_transaction_active(void);
static void hv_kvp_transaction_init(uint32_t, hv_vmbus_channel *, uint64_t, uint8_t *);
static void hv_kvp_process_hostmsg(void);

/* hv_kvp character device structure */
static struct cdevsw hv_kvp_cdevsw =
{
	.d_version	= D_VERSION,
	.d_open		= hv_kvp_dev_open,
	.d_close	= hv_kvp_dev_close,
	.d_read		= hv_kvp_dev_daemon_read,
	.d_write	= hv_kvp_dev_daemon_write,
	.d_name		= "hv_kvp_dev",
};

static struct cdev *hv_kvp_dev;
static struct hv_kvp_msg *hv_kvp_dev_buf;

/*
 * We maintain a global state, assuming only one transaction can be active
 * at any point in time.
 * Inited by the kvp callback routine (utils file) when a valid message is
 * received from the host;
 */
static struct {
	boolean_t		kvp_ready;      /* indicates if kvp module is ready or not */
	boolean_t		in_progress;    /* transaction status - active or not */
	uint32_t		host_msg_len;   /* length of host message */
	hv_vmbus_channel *	channelp;       /* pointer to channel */
	uint64_t		host_msg_id;    /* host message id */
	struct hv_kvp_msg *	host_kvp_msg;   /* current message from the host */
	uint8_t *		rcv_buf;        /* rcv buffer for communicating with the host*/
	struct sema		dev_sema;       /* device semaphore to control communication */
	struct timeout_task	delay_task;     /* delayed task structure */
} kvp_msg_state;

/* global vars */
MALLOC_DECLARE(M_HV_KVP_DEV_BUF);
MALLOC_DEFINE(M_HV_KVP_DEV_BUF, "hv_kvp_dev buffer", "buffer for hv_kvp_dev module");

/*
 * Data Buffer used by kernel for to/from communication with user daemon
 */
static struct hv_kvp_msg hv_user_kvp_msg;

/* indicates daemon registered with driver */
static boolean_t register_done;

/*
 * hv_kvp low level functions
 */

/*
 * Check if kvp routines are ready to receive and respond
 */
static int
hv_kvp_ready(void)
{
	return (kvp_msg_state.kvp_ready);
}


/*
 * Check if kvp transaction is in progres
 */
static int
hv_kvp_transaction_active(void)
{
	return (kvp_msg_state.in_progress);
}


/*
 * This routine is called whenever a message is received from the host
 */
static void
hv_kvp_transaction_init(uint32_t rcv_len, hv_vmbus_channel *rcv_channel,
    uint64_t request_id, uint8_t *rcv_buf)
{
	/* Store all the relevant message details in the global structure */
	kvp_msg_state.in_progress = TRUE;
	kvp_msg_state.host_msg_len = rcv_len;
	kvp_msg_state.channelp = rcv_channel;
	kvp_msg_state.host_msg_id = request_id;
	kvp_msg_state.rcv_buf = rcv_buf;
	kvp_msg_state.host_kvp_msg = (struct hv_kvp_msg *)&rcv_buf[
		sizeof(struct hv_vmbus_pipe_hdr) +
		sizeof(struct hv_vmbus_icmsg_hdr)];
}


/*
 * hv_kvp - version neogtiation function
 */
static void
hv_kvp_negotiate_version(
	struct hv_vmbus_icmsg_hdr *icmsghdrp,
	struct hv_vmbus_icmsg_negotiate *negop,
	uint8_t *buf
	)
{
	int icframe_vercnt;
	int icmsg_vercnt;

	icmsghdrp->icmsgsize = 0x10;

	negop = (struct hv_vmbus_icmsg_negotiate *)&buf[
		sizeof(struct hv_vmbus_pipe_hdr) +
		sizeof(struct hv_vmbus_icmsg_hdr)];
	icframe_vercnt = negop->icframe_vercnt;
	icmsg_vercnt = negop->icmsg_vercnt;

	/*
	 * Select the framework version number we will support
	 */

	if ((icframe_vercnt >= 2) && (negop->icversion_data[1].major == 3)) {
		icframe_vercnt = 3;
		if (icmsg_vercnt >= 2) {
			icmsg_vercnt = 4;
		} else {
			icmsg_vercnt = 3;
		}
	} else {
		icframe_vercnt = 1;
		icmsg_vercnt = 1;
	}

	negop->icframe_vercnt = 1;
	negop->icmsg_vercnt = 1;
	negop->icversion_data[0].major = icframe_vercnt;
	negop->icversion_data[0].minor = 0;
	negop->icversion_data[1].major = icmsg_vercnt;
	negop->icversion_data[1].minor = 0;
}


static size_t
hv_kvp_convert8_to_16(uint16_t *dst, size_t dst_len,
    const char *src, size_t src_len,
    int *errp)
{
	const unsigned char *s;
	size_t spos, dpos;
	int error, flags = 1;
	uint16_t c;

#define IS_CONT(c)    (((c) & 0xc0) == 0x80)

	error = 0;
	s = (const unsigned char *)src;
	spos = dpos = 0;

	while (spos < src_len) {
		if (s[spos] < 0x80) {
			c = s[spos++];
		} else if ((flags & 0x03) &&
		    ((spos >= src_len) || !IS_CONT(s[spos + 1])) &&
		    (s[spos] >= 0xa0)) {
			c = s[spos++];
		} else if ((s[spos] < 0xc0) || (s[spos] >= 0xf5)) {
			error++;
			spos++;
			continue;
		} else if (s[spos] < 0xe0) {
			if ((spos >= src_len) || !IS_CONT(s[spos + 1])) {
				spos++;
				error++;
				continue;
			}
			c = ((s[spos] & 0x3f) << 6) | (s[spos + 1] & 0x3f);
			spos += 2;
			if (c < 0x80) {
				error++;
				continue;
			}
		} else if (s[spos] < 0xf0) {
			if ((spos >= src_len - 2) ||
			    !IS_CONT(s[spos + 1]) || !IS_CONT(s[spos + 2])) {
				spos++;
				error++;
				continue;
			}
			c = ((s[spos] & 0x0f) << 12) | ((s[spos + 1] & 0x3f) << 6) |
			    (s[spos + 2] & 0x3f);
			spos += 3;
			if ((c < 0x800) || ((c & 0xdf00) == 0xd800)) {
				error++;
				continue;
			}
		} else {
			uint32_t cc;
			if ((spos >= src_len - 3) || !IS_CONT(s[spos + 1]) ||
			    !IS_CONT(s[spos + 2]) || !IS_CONT(s[spos + 3])) {
				spos++;
				error++;
				continue;
			}
			cc = ((s[spos] & 0x03) << 18) | ((s[spos + 1] & 0x3f) << 12) |
			    ((s[spos + 2] & 0x3f) << 6) | (s[spos + 3] & 0x3f);
			spos += 4;
			if (cc < 0x10000) {
				/* overlong encoding */
				error++;
				continue;
			}
			if (dst && (dpos < dst_len)) {
				dst[dpos] = (0xd800 | ((cc - 0x10000) >> 10));
			}
			dpos++;
			c = 0xdc00 | ((cc - 0x10000) & 0x3ffff);
		}

		if (dst && (dpos < dst_len)) {
			dst[dpos] = c;
		}
		dpos++;
	}

	if (errp) {
		*errp = error;
	}

	return (dpos);

#undef IS_CONT
}


static size_t
hv_kvp_convert16_to_8(char *dst, size_t dst_len,
    const uint16_t *src, size_t src_len,
    int *errp)
{
	uint16_t spos, dpos;
	int error;

#define CHECK_LENGTH(l)		(dpos > dst_len - (l) ? dst = NULL : NULL)
#define ADD_BYTE(b)		(dst ? dst[dpos] = (b) : 0, dpos++)

	error = 0;
	dpos = 0;
	for (spos = 0; spos < src_len; spos++) {
		if (src[spos] < 0x80) {
			CHECK_LENGTH(1);
			ADD_BYTE(src[spos]);
		} else if (src[spos] < 0x800) {
			CHECK_LENGTH(2);
			ADD_BYTE(0xc0 | (src[spos] >> 6));
			ADD_BYTE(0x80 | (src[spos] & 0x3f));
		} else if ((src[spos] & 0xdc00) == 0xd800) {
			uint32_t c;
			/* first surrogate */
			if ((spos == src_len - 1) || ((src[spos] & 0xdc00) != 0xdc00)) {
				/* no second surrogate present */
				error++;
				continue;
			}
			spos++;
			CHECK_LENGTH(4);
			c = (((src[spos] & 0x3ff) << 10) | (src[spos + 1] & 0x3ff)) + 0x10000;
			ADD_BYTE(0xf0 | (c >> 18));
			ADD_BYTE(0x80 | ((c >> 12) & 0x3f));
			ADD_BYTE(0x80 | ((c >> 6) & 0x3f));
			ADD_BYTE(0x80 | (c & 0x3f));
		} else if ((src[spos] & 0xdc00) == 0xdc00) {
			/* second surrogate without preceding first surrogate */
			error++;
		} else {
			CHECK_LENGTH(3);
			ADD_BYTE(0xe0 | src[spos] >> 12);
			ADD_BYTE(0x80 | ((src[spos] >> 6) & 0x3f));
			ADD_BYTE(0x80 | (src[spos] & 0x3f));
		}
	}

	if (errp) {
		*errp = error;
	}
	return (dpos);

#undef ADD_BYTE
#undef CHECK_LENGTH
}


/*
 * Convert ip related info in umsg from utf8 to utf16 and store in hmsg
 */
static int
hv_kvp_ipinfo_utf8_utf16(struct hv_kvp_msg *umsg, struct hv_kvp_ip_msg *host_ip_msg)
{
	int err_ip, err_subnet, err_gway, err_dns, err_adap;

	size_t len = 0;

	len = hv_kvp_convert8_to_16((uint16_t *)host_ip_msg->kvp_ip_val.ip_addr,
		MAX_IP_ADDR_SIZE,
		(char *)umsg->body.kvp_ip_val.ip_addr,
		strlen((char *)umsg->body.kvp_ip_val.ip_addr),
		&err_ip);
	len = hv_kvp_convert8_to_16((uint16_t *)host_ip_msg->kvp_ip_val.sub_net,
		MAX_IP_ADDR_SIZE,
		(char *)umsg->body.kvp_ip_val.sub_net,
		strlen((char *)umsg->body.kvp_ip_val.sub_net),
		&err_subnet);
	len = hv_kvp_convert8_to_16((uint16_t *)host_ip_msg->kvp_ip_val.gate_way,
		MAX_GATEWAY_SIZE,
		(char *)umsg->body.kvp_ip_val.gate_way,
		strlen((char *)umsg->body.kvp_ip_val.gate_way),
		&err_gway);
	len = hv_kvp_convert8_to_16((uint16_t *)host_ip_msg->kvp_ip_val.dns_addr,
		MAX_IP_ADDR_SIZE,
		(char *)umsg->body.kvp_ip_val.dns_addr,
		strlen((char *)umsg->body.kvp_ip_val.dns_addr),
		&err_dns);
	len = hv_kvp_convert8_to_16((uint16_t *)host_ip_msg->kvp_ip_val.adapter_id,
		MAX_IP_ADDR_SIZE,
		(char *)umsg->body.kvp_ip_val.adapter_id,
		strlen((char *)umsg->body.kvp_ip_val.adapter_id),
		&err_adap);
	host_ip_msg->kvp_ip_val.dhcp_enabled = umsg->body.kvp_ip_val.dhcp_enabled;
	host_ip_msg->kvp_ip_val.addr_family = umsg->body.kvp_ip_val.addr_family;

	return (err_ip | err_subnet | err_gway | err_dns | err_adap);
}


/*
 * Convert ip related info in hmsg from utf16 to utf8 and store in umsg
 */
static int
ipinfo_utf16_utf8(struct hv_kvp_ip_msg *host_ip_msg, struct hv_kvp_msg *umsg)
{
	int err_ip, err_subnet, err_gway, err_dns, err_adap;
	int guid_index;
	struct hv_device *hv_dev;       /* GUID Data Structure */
	hn_softc_t *sc;                 /* hn softc structure  */
	char if_name[4];
	unsigned char guid_instance[40];
	char *guid_data = NULL;
	char buf[39];
	int len = 16;

	struct guid_extract {
		char	a1[2];
		char	a2[2];
		char	a3[2];
		char	a4[2];
		char	b1[2];
		char	b2[2];
		char	c1[2];
		char	c2[2];
		char	d[4];
		char	e[12];
	};

	struct guid_extract *id;
	device_t *devs;
	int devcnt;

	/* IP Address */
	len = hv_kvp_convert16_to_8((char *)umsg->body.kvp_ip_val.ip_addr,
		MAX_IP_ADDR_SIZE,
		(uint16_t *)host_ip_msg->kvp_ip_val.ip_addr,
		MAX_IP_ADDR_SIZE, &err_ip);

	/* Adapter ID : GUID */
	len = hv_kvp_convert16_to_8((char *)umsg->body.kvp_ip_val.adapter_id,
		MAX_ADAPTER_ID_SIZE,
		(uint16_t *)host_ip_msg->kvp_ip_val.adapter_id,
		MAX_ADAPTER_ID_SIZE, &err_adap);

	if (devclass_get_devices(devclass_find("hn"), &devs, &devcnt) == 0) {
		for (devcnt = devcnt - 1; devcnt >= 0; devcnt--) {
			sc = device_get_softc(devs[devcnt]);

			/* Trying to find GUID of Network Device */
			hv_dev = sc->hn_dev_obj;

			for (guid_index = 0; guid_index < 16; guid_index++) {
				sprintf(&guid_instance[guid_index * 2], "%02x",
				    hv_dev->device_id.data[guid_index]);
			}

			guid_data = (char *)guid_instance;
			id = (struct guid_extract *)guid_data;
			snprintf(buf, sizeof(buf), "{%.2s%.2s%.2s%.2s-%.2s%.2s-%.2s%.2s-%.4s-%s}",
			    id->a4, id->a3, id->a2, id->a1,
			    id->b2, id->b1, id->c2, id->c1, id->d, id->e);
			guid_data = NULL;
			sprintf(if_name, "%s%d", "hn", device_get_unit(devs[devcnt]));

			if (strncmp(buf, (char *)umsg->body.kvp_ip_val.adapter_id, 39) == 0) {
				strcpy((char *)umsg->body.kvp_ip_val.adapter_id, if_name);
				break;
			}
		}
		free(devs, M_TEMP);
	}

	/* Address Family , DHCP , SUBNET, Gateway, DNS */
	umsg->kvp_hdr.operation = host_ip_msg->operation;
	umsg->body.kvp_ip_val.addr_family = host_ip_msg->kvp_ip_val.addr_family;
	umsg->body.kvp_ip_val.dhcp_enabled = host_ip_msg->kvp_ip_val.dhcp_enabled;
	hv_kvp_convert16_to_8((char *)umsg->body.kvp_ip_val.sub_net, MAX_IP_ADDR_SIZE,
	    (uint16_t *)host_ip_msg->kvp_ip_val.sub_net,
	    MAX_IP_ADDR_SIZE, &err_subnet);
	hv_kvp_convert16_to_8((char *)umsg->body.kvp_ip_val.gate_way, MAX_GATEWAY_SIZE,
	    (uint16_t *)host_ip_msg->kvp_ip_val.gate_way,
	    MAX_GATEWAY_SIZE, &err_gway);

	hv_kvp_convert16_to_8((char *)umsg->body.kvp_ip_val.dns_addr, MAX_IP_ADDR_SIZE,
	    (uint16_t *)host_ip_msg->kvp_ip_val.dns_addr,
	    MAX_IP_ADDR_SIZE, &err_dns);

	return (err_ip | err_subnet | err_gway | err_dns | err_adap);
}


/*
 * Prepare a user kvp msg based on host kvp msg (utf16 to utf8)
 * Ensure utf16_utf8 takes care of the additional string terminating char!!
 */
static void
conv_hostmsg_to_usermsg(void)
{
	int utf_err = 0;
	uint32_t value_type;
	struct hv_kvp_ip_msg *host_ip_msg = (struct hv_kvp_ip_msg *)kvp_msg_state.host_kvp_msg;

	struct hv_kvp_msg *hmsg = kvp_msg_state.host_kvp_msg;
	struct hv_kvp_msg *umsg = &hv_user_kvp_msg;

	memset(umsg, 0, sizeof(struct hv_kvp_msg));

	umsg->kvp_hdr.operation = hmsg->kvp_hdr.operation;
	umsg->kvp_hdr.pool = hmsg->kvp_hdr.pool;

	switch (umsg->kvp_hdr.operation) {
	case HV_KVP_OP_SET_IP_INFO:
		ipinfo_utf16_utf8(host_ip_msg, umsg);
		break;

	case HV_KVP_OP_GET_IP_INFO:

		hv_kvp_convert16_to_8((char *)umsg->body.kvp_ip_val.adapter_id,
		    MAX_ADAPTER_ID_SIZE,
		    (uint16_t *)host_ip_msg->kvp_ip_val.adapter_id,
		    MAX_ADAPTER_ID_SIZE, &utf_err);

		umsg->body.kvp_ip_val.addr_family =
		    host_ip_msg->kvp_ip_val.addr_family;
		break;

	case HV_KVP_OP_SET:
		value_type = hmsg->body.kvp_set.data.value_type;

		switch (value_type) {
		case HV_REG_SZ:
			umsg->body.kvp_set.data.value_size =
			    hv_kvp_convert16_to_8(
				(char *)umsg->body.kvp_set.data.msg_value.value,
				HV_KVP_EXCHANGE_MAX_VALUE_SIZE - 1,
				(uint16_t *)hmsg->body.kvp_set.data.msg_value.value,
				hmsg->body.kvp_set.data.value_size,
				&utf_err);
			/* utf8 encoding */
			umsg->body.kvp_set.data.value_size =
			    umsg->body.kvp_set.data.value_size / 2;
			break;

		case HV_REG_U32:
			umsg->body.kvp_set.data.value_size =
			    sprintf(umsg->body.kvp_set.data.msg_value.value, "%d",
				hmsg->body.kvp_set.data.msg_value.value_u32) + 1;
			break;

		case HV_REG_U64:
			umsg->body.kvp_set.data.value_size =
			    sprintf(umsg->body.kvp_set.data.msg_value.value, "%llu",
				(unsigned long long)
				hmsg->body.kvp_set.data.msg_value.value_u64) + 1;
			break;
		}

		umsg->body.kvp_set.data.key_size =
		    hv_kvp_convert16_to_8(
			umsg->body.kvp_set.data.key,
			HV_KVP_EXCHANGE_MAX_KEY_SIZE - 1,
			(uint16_t *)hmsg->body.kvp_set.data.key,
			hmsg->body.kvp_set.data.key_size,
			&utf_err);

		/* utf8 encoding */
		umsg->body.kvp_set.data.key_size =
		    umsg->body.kvp_set.data.key_size / 2;
		break;

	case HV_KVP_OP_GET:
		umsg->body.kvp_get.data.key_size =
		    hv_kvp_convert16_to_8(umsg->body.kvp_get.data.key,
			HV_KVP_EXCHANGE_MAX_KEY_SIZE - 1,
			(uint16_t *)hmsg->body.kvp_get.data.key,
			hmsg->body.kvp_get.data.key_size,
			&utf_err);
		/* utf8 encoding */
		umsg->body.kvp_get.data.key_size =
		    umsg->body.kvp_get.data.key_size / 2;
		break;

	case HV_KVP_OP_DELETE:
		umsg->body.kvp_delete.key_size =
		    hv_kvp_convert16_to_8(umsg->body.kvp_delete.key,
			HV_KVP_EXCHANGE_MAX_KEY_SIZE - 1,
			(uint16_t *)hmsg->body.kvp_delete.key,
			hmsg->body.kvp_delete.key_size,
			&utf_err);
		/* utf8 encoding */
		umsg->body.kvp_delete.key_size =
		    umsg->body.kvp_delete.key_size / 2;
		break;

	case HV_KVP_OP_ENUMERATE:
		umsg->body.kvp_enum_data.index =
		    hmsg->body.kvp_enum_data.index;
		break;

	default:
		printf("host_user_kvp_msg: Invalid operation : %d\n",
		    umsg->kvp_hdr.operation);
	}
}


/*
 * Prepare a host kvp msg based on user kvp msg (utf8 to utf16)
 */
static int
conv_usermsg_to_hostmsg(void)
{
	int hkey_len = 0, hvalue_len = 0, utf_err = 0;
	struct hv_kvp_exchg_msg_value *host_exchg_data;
	char *key_name, *value;

	struct hv_kvp_msg *umsg = &hv_user_kvp_msg;
	struct hv_kvp_msg *hmsg = kvp_msg_state.host_kvp_msg;
	struct hv_kvp_ip_msg *host_ip_msg = (struct hv_kvp_ip_msg *)hmsg;

	switch (kvp_msg_state.host_kvp_msg->kvp_hdr.operation) {
	case HV_KVP_OP_GET_IP_INFO:
		return (hv_kvp_ipinfo_utf8_utf16(umsg, host_ip_msg));

	case HV_KVP_OP_SET_IP_INFO:
	case HV_KVP_OP_SET:
	case HV_KVP_OP_DELETE:
		return (KVP_SUCCESS);

	case HV_KVP_OP_ENUMERATE:
		host_exchg_data = &hmsg->body.kvp_enum_data.data;
		key_name = umsg->body.kvp_enum_data.data.key;
		hkey_len = hv_kvp_convert8_to_16((uint16_t *)host_exchg_data->key,
			((HV_KVP_EXCHANGE_MAX_KEY_SIZE / 2) - 2),
			key_name, strlen(key_name),
			&utf_err);
		/* utf16 encoding */
		host_exchg_data->key_size = 2 * (hkey_len + 1);
		value = umsg->body.kvp_enum_data.data.msg_value.value;
		hvalue_len =
		    hv_kvp_convert8_to_16((uint16_t *)host_exchg_data->msg_value.value,
			((HV_KVP_EXCHANGE_MAX_VALUE_SIZE / 2) - 2),
			value, strlen(value),
			&utf_err);
		host_exchg_data->value_size = 2 * (hvalue_len + 1);
		host_exchg_data->value_type = HV_REG_SZ;

		if ((hkey_len < 0) || (hvalue_len < 0)) {
			return (HV_KVP_E_FAIL);
		}
		return (KVP_SUCCESS);

	case HV_KVP_OP_GET:
		host_exchg_data = &hmsg->body.kvp_get.data;
		value = umsg->body.kvp_get.data.msg_value.value;
		hvalue_len = hv_kvp_convert8_to_16(
			(uint16_t *)host_exchg_data->msg_value.value,
			((HV_KVP_EXCHANGE_MAX_VALUE_SIZE / 2) - 2),
			value, strlen(value),
			&utf_err);
		/* Convert value size to uft16 */
		host_exchg_data->value_size = 2 * (hvalue_len + 1);
		/* Use values by string */
		host_exchg_data->value_type = HV_REG_SZ;

		if ((hkey_len < 0) || (hvalue_len < 0)) {
			return (HV_KVP_E_FAIL);
		}
		return (KVP_SUCCESS);

	default:
		return (HV_KVP_E_FAIL);
	}
}


/*
 * Send the response back to the host.
 */
static void
hv_kvp_respond_host(int error)
{
	struct hv_vmbus_icmsg_hdr *hv_icmsg_hdrp;

	if (!hv_kvp_transaction_active()) {
		//TODO: Triage why we are here.
		printf("No active transaction returning\n");
		return;
	}

	hv_icmsg_hdrp = (struct hv_vmbus_icmsg_hdr *)
	    &kvp_msg_state.rcv_buf[sizeof(struct hv_vmbus_pipe_hdr)];

	if (error) {
		error = HV_KVP_E_FAIL;
	}

	hv_icmsg_hdrp->status = error;
	hv_icmsg_hdrp->icflags = HV_ICMSGHDRFLAG_TRANSACTION | HV_ICMSGHDRFLAG_RESPONSE;

	/* Now ready to process another transaction */
	kvp_msg_state.in_progress = FALSE;
	
	error = hv_vmbus_channel_send_packet(kvp_msg_state.channelp,
		kvp_msg_state.rcv_buf,
		kvp_msg_state.host_msg_len, kvp_msg_state.host_msg_id,
		HV_VMBUS_PACKET_TYPE_DATA_IN_BAND, 0);

	if (error) {
		printf("hv_kvp_respond_host: sendpacket error:%d\n", error);
	}
}


/**
 * This is the main kvp kernel process that interacts with both user daemon
 * and the host
 */
static void
hv_kvp_process_hostmsg(void)
{
	/* Check for daemon registertion */
	if (!register_done) {
		return;
	}

	/* Prepare kvp_msg to be sent to user */
	conv_hostmsg_to_usermsg();

	/* Send the msg to user via function deamon_read - setting sema */
	sema_post(&kvp_msg_state.dev_sema);
}


static void
hv_kvp_enqueue_timeout(void *arg, int pending __unused)
{
	taskqueue_enqueue_timeout(taskqueue_thread, &kvp_msg_state.delay_task, 5000);
}


/*
 * Callback routine that gets called whenever there is a message from host
 */
void hv_kvp_callback(void *context)
{
	uint8_t *kvp_buf;
	hv_vmbus_channel *channel = context;
	uint32_t recvlen;
	uint64_t requestid;
	int ret = 0;

	struct hv_vmbus_icmsg_hdr *icmsghdrp;

	kvp_buf = receive_buffer[HV_KVP];

	/*
	 * Check if already one transaction is under process
	 */
	if (!hv_kvp_transaction_active()) {
		ret = hv_vmbus_channel_recv_packet(channel, kvp_buf, 2 * PAGE_SIZE,
			&recvlen, &requestid);

		if ((ret == 0) && (recvlen > 0)) {
			icmsghdrp = (struct hv_vmbus_icmsg_hdr *)
			    &kvp_buf[sizeof(struct hv_vmbus_pipe_hdr)];
			hv_kvp_transaction_init(recvlen, channel, requestid, kvp_buf);

			if (icmsghdrp->icmsgtype == HV_ICMSGTYPE_NEGOTIATE) {
				hv_kvp_negotiate_version(icmsghdrp, NULL, kvp_buf);
				hv_kvp_respond_host(ret);
			}else {
				hv_kvp_process_hostmsg();
				/* TIMEOUT Work */
				TIMEOUT_TASK_INIT(taskqueue_thread, &kvp_msg_state.delay_task, 0, hv_kvp_enqueue_timeout, NULL);
			}
		} else {
			printf(" invaild msg \n");
			ret = HV_KVP_E_FAIL;
		}
	} else {
		ret = HV_KVP_E_FAIL;
	}

	if (ret != 0) {
		hv_kvp_respond_host(ret);
	}
}


/*
 * hv_kvp_dev initialized
 *
 */
static int
hv_kvp_dev_init(void)
{
	int error = 0;

	/* initialize semaphore */
	sema_init(&kvp_msg_state.dev_sema, 0, "hv_kvp device semaphore");
	/* create character device */
	error = make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
		&hv_kvp_dev,
		&hv_kvp_cdevsw,
		0,
		UID_ROOT,
		GID_WHEEL,
		0600,
		"hv_kvp_dev");
	if (error != 0) {
		return (error);
	}

	hv_kvp_dev_buf = malloc(sizeof(*hv_kvp_dev_buf), M_HV_KVP_DEV_BUF, M_WAITOK |
		M_ZERO);

	return (error);
}


/*
 * This function is called by the hv_kvp_init -
 * initialize character device
 */
static void
hv_kvp_dev_destroy(void)
{
	destroy_dev(hv_kvp_dev);
	free(hv_kvp_dev_buf, M_HV_KVP_DEV_BUF);
}


static int
hv_kvp_dev_open(struct cdev *dev __unused, int oflags __unused, int devtype __unused,
    struct thread *td __unused)
{
	uprintf("Opened device \"hv_kvp_device\" successfully.\n");
	return (0);
}


static int
hv_kvp_dev_close(struct cdev *dev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{
	uprintf("Closing device \"hv_kvp_device\".\n");
	return (0);
}


static int
hv_kvp_dev_daemon_read(struct cdev *dev __unused, struct uio *uio, int ioflag __unused)
{
	size_t amt;
	int error;

	if (register_done == FALSE) {
		return (0);
	}

	sema_wait(&kvp_msg_state.dev_sema);

	memcpy(hv_kvp_dev_buf, &hv_user_kvp_msg, sizeof(struct hv_kvp_msg));

	amt = MIN(uio->uio_resid, uio->uio_offset >= BUFFERSIZE + 1 ? 0 :
		BUFFERSIZE + 1 - uio->uio_offset);

	if ((error = uiomove(hv_kvp_dev_buf, amt, uio)) != 0) {
		uprintf("uiomove read failed!\n");
	}

	return (error);
}


/*
 * hv_kvp_daemon write invokes this function
 * this function replaces - receive
 *
 */
static int
hv_kvp_dev_daemon_write(struct cdev *dev __unused, struct uio *uio, int ioflag __unused)
{
	size_t amt;
	int error = 0;

	uio->uio_offset = 0;

	amt = MIN(uio->uio_resid, BUFFERSIZE);
	error = uiomove(hv_kvp_dev_buf, amt, uio);

	if (error != 0) {
		return (error);
	}

	memcpy(&hv_user_kvp_msg, hv_kvp_dev_buf, sizeof(struct hv_kvp_msg));

	if (register_done == FALSE) {
		if (hv_user_kvp_msg.kvp_hdr.operation == HV_KVP_OP_REGISTER) {
			register_done = TRUE;
			kvp_msg_state.kvp_ready = TRUE;
		}else {
			printf(" KVP Registeration Failed\n");
			return (error);
		}
	}

	conv_usermsg_to_hostmsg();

	if (taskqueue_cancel_timeout(taskqueue_thread, &kvp_msg_state.delay_task, NULL) == 0) {
		printf("in tine message\n");
		hv_kvp_respond_host(KVP_SUCCESS);
	}else {
		printf("timed out task \n");
	}
	return (error);
}


int
hv_kvp_init(hv_vmbus_service *srv)
{
	int error = 0;
	hv_work_queue *work_queue = NULL;

	work_queue = hv_work_queue_create("KVP Service");
	if (work_queue == NULL) {
		printf("hv_kvp_init: Work queue alloc failed\n");
		error = ENOMEM;
		goto Finish;
	}
	srv->work_queue = work_queue;

	hv_kvp_dev_init();
Finish:
	return (error);
}


void
hv_kvp_deinit(void)
{
	hv_kvp_dev_destroy();
}
