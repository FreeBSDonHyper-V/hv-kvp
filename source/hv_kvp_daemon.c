/*-
 * Copyright (C) 2010, Novell, Inc.
 * Author : K. Y. Srinivasan <ksrinivasan@novell.com>
 *
 * An implementation of key value pair (KVP) functionality for FreeBSD.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
 
 /* Headers */
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <sys/utsname.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <syslog.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <net/if_dl.h>
#include <net/if_types.h>

/* hv_kvp header */
#include "hv_kvp.h"

/* 
 * hv_kvp_daemon - main function
 */
int main(void)
{
	int                        fd, len, cl;
	int                        error;
	struct hv_kvp_msg          *hv_msg;
	char                       *key_value;
	char                       *key_name;
	int                        op;
	int                        pool;
	char                       *if_name;
	struct hv_kvp_ipaddr_value *kvp_ip_val;

	/* hv_kvp_daemon runs in background */
	daemon(1, 0);
	openlog("KVP", 0, LOG_USER);
	syslog(LOG_INFO, "KVP starting; pid is:%d", getpid());
}