/*-
 * Copyright (c) 2014 Microsoft Corp.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 *      Author: Sainath Varanasi.
 *      Date:   4/2012
 */


/* header */
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <sys/utsname.h>
#include <sys/stat.h>
#include <sys/un.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <syslog.h>
#include <fcntl.h>
#include <dirent.h>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>

#include <netinet/in.h>
#include <net/if_dl.h>
#include <net/if_types.h>

#include "hv_kvp.h"

typedef uint8_t		__u8;
typedef uint16_t	__u16;
typedef uint32_t	__u32;
typedef uint64_t	__u64;

/*
 * ENUM Data
 *
 */

/* Global Variables */
enum key_index {
	FullyQualifiedDomainName = 0,
	IntegrationServicesVersion, /*This key is serviced in the kernel*/
	NetworkAddressIPv4,
	NetworkAddressIPv6,
	OSBuildNumber,
	OSName,
	OSMajorVersion,
	OSMinorVersion,
	OSVersion,
	ProcessorArchitecture
};


enum {
	IPADDR = 0,
	NETMASK,
	GATEWAY,
	DNS
};


/* Global buffers for KVP communication */

static char *os_name = "";
static char *os_major = "";
static char *os_minor = "";
static char *processor_arch;
static char *os_build;
static char *lic_version = "BSD Pre-Release version";
static struct utsname uts_buf;

/*
 * The location of the interface configuration file.
 */

#define MAX_FILE_NAME		100
#define ENTRIES_PER_BLOCK	50

struct kvp_record {
	char	key[HV_KVP_EXCHANGE_MAX_KEY_SIZE];
	char	value[HV_KVP_EXCHANGE_MAX_VALUE_SIZE];
};

struct kvp_file_state {
	int			fd;
	int			num_blocks;
	struct kvp_record *	records;
	int			num_records;
	char			fname[MAX_FILE_NAME];
};

static struct kvp_file_state kvp_file_info[HV_KVP_POOL_COUNT];

static void kvp_acquire_lock(int pool)
{
	struct flock fl = { 0, 0, 0, F_WRLCK, SEEK_SET, 0 };

	fl.l_pid = getpid();

	if (fcntl(kvp_file_info[pool].fd, F_SETLKW, &fl) == -1) {
		syslog(LOG_ERR, "Failed to acquire the lock pool: %d", pool);
		exit(EXIT_FAILURE);
	}
}


static void kvp_release_lock(int pool)
{
	struct flock fl = { 0, 0, 0, F_UNLCK, SEEK_SET, 0 };

	fl.l_pid = getpid();

	if (fcntl(kvp_file_info[pool].fd, F_SETLK, &fl) == -1) {
		perror("fcntl");
		syslog(LOG_ERR, "Failed to release the lock pool: %d", pool);
		exit(EXIT_FAILURE);
	}
}


static void kvp_update_file(int pool)
{
	FILE *filep;
	size_t bytes_written;

	/*
	 * We are going to write our in-memory registry out to
	 * disk; acquire the lock first.
	 */
	kvp_acquire_lock(pool);

	filep = fopen(kvp_file_info[pool].fname, "w");
	if (!filep) {
		kvp_release_lock(pool);
		syslog(LOG_ERR, "Failed to open file, pool: %d", pool);
		exit(EXIT_FAILURE);
	}

	bytes_written = fwrite(kvp_file_info[pool].records,
		sizeof(struct kvp_record),
		kvp_file_info[pool].num_records, filep);

	if (ferror(filep) || fclose(filep)) {
		kvp_release_lock(pool);
		syslog(LOG_ERR, "Failed to write file, pool: %d", pool);
		exit(EXIT_FAILURE);
	}

	kvp_release_lock(pool);
}


static void kvp_update_mem_state(int pool)
{
	FILE *filep;
	size_t records_read = 0;
	struct kvp_record *record = kvp_file_info[pool].records;
	struct kvp_record *readp;
	int num_blocks = kvp_file_info[pool].num_blocks;
	int alloc_unit = sizeof(struct kvp_record) * ENTRIES_PER_BLOCK;

	kvp_acquire_lock(pool);

	filep = fopen(kvp_file_info[pool].fname, "r");
	if (!filep) {
		kvp_release_lock(pool);
		syslog(LOG_ERR, "Failed to open file, pool: %d", pool);
		exit(EXIT_FAILURE);
	}
	for ( ; ; )
	{
		readp = &record[records_read];
		records_read += fread(readp, sizeof(struct kvp_record),
			ENTRIES_PER_BLOCK * num_blocks,
			filep);

		if (ferror(filep)) {
			syslog(LOG_ERR, "Failed to read file, pool: %d", pool);
			exit(EXIT_FAILURE);
		}

		if (!feof(filep)) {
			/*
			 * We have more data to read.
			 */
			num_blocks++;
			record = realloc(record, alloc_unit * num_blocks);

			if (record == NULL) {
				syslog(LOG_ERR, "malloc failed");
				exit(EXIT_FAILURE);
			}
			continue;
		}
		break;
	}

	kvp_file_info[pool].num_blocks = num_blocks;
	kvp_file_info[pool].records = record;
	kvp_file_info[pool].num_records = records_read;

	fclose(filep);
	kvp_release_lock(pool);
}


static int kvp_file_init(void)
{
	int fd;
	FILE *filep;
	size_t records_read;
	char *fname;
	struct kvp_record *record;
	struct kvp_record *readp;
	int num_blocks;
	int i;
	int alloc_unit = sizeof(struct kvp_record) * ENTRIES_PER_BLOCK;

	if (access("/usr/local/hyperv/pool", F_OK)) {
		if (mkdir("/usr/local/hyperv/pool", S_IRUSR | S_IWUSR | S_IROTH)) {
			syslog(LOG_ERR, " Failed to create /usr/local/hyperv/pool");
			exit(EXIT_FAILURE);
		}
	}

	for (i = 0; i < HV_KVP_POOL_COUNT; i++)
	{
		fname = kvp_file_info[i].fname;
		records_read = 0;
		num_blocks = 1;
		sprintf(fname, "/usr/local/hyperv/pool/.kvp_pool_%d", i);
		fd = open(fname, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IROTH);

		if (fd == -1) {
			return (1);
		}


		filep = fopen(fname, "r");
		if (!filep) {
			return (1);
		}

		record = malloc(alloc_unit * num_blocks);
		if (record == NULL) {
			fclose(filep);
			return (1);
		}
		for ( ; ; )
		{
			readp = &record[records_read];
			records_read += fread(readp, sizeof(struct kvp_record),
				ENTRIES_PER_BLOCK,
				filep);

			if (ferror(filep)) {
				syslog(LOG_ERR, "Failed to read file, pool: %d",
				    i);
				exit(EXIT_FAILURE);
			}

			if (!feof(filep)) {
				/*
				 * We have more data to read.
				 */
				num_blocks++;
				record = realloc(record, alloc_unit *
					num_blocks);
				if (record == NULL) {
					fclose(filep);
					return (1);
				}
				continue;
			}
			break;
		}
		kvp_file_info[i].fd = fd;
		kvp_file_info[i].num_blocks = num_blocks;
		kvp_file_info[i].records = record;
		kvp_file_info[i].num_records = records_read;
		fclose(filep);
	}

	return (0);
}


static int kvp_key_delete(int pool, __u8 *key, int key_size)
{
	int i;
	int j, k;
	int num_records;
	struct kvp_record *record;

	/*
	 * First update the in-memory state.
	 */
	kvp_update_mem_state(pool);

	num_records = kvp_file_info[pool].num_records;
	record = kvp_file_info[pool].records;

	for (i = 0; i < num_records; i++)
	{
		if (memcmp(key, record[i].key, key_size)) {
			continue;
		}

		/*
		 * Found a match; just move the remaining
		 * entries up.
		 */
		if (i == num_records) {
			kvp_file_info[pool].num_records--;
			kvp_update_file(pool);
			return (0);
		}

		j = i;
		k = j + 1;
		for ( ; k < num_records; k++)
		{
			strcpy(record[j].key, record[k].key);
			strcpy(record[j].value, record[k].value);
			j++;
		}
		kvp_file_info[pool].num_records--;
		kvp_update_file(pool);
		return (0);
	}
	return (1);
}


static int
kvp_key_add_or_modify(int pool, __u8 *key, __u32 key_size, __u8 *value,
    __u32 value_size)
{
	int i;
	int num_records;
	struct kvp_record *record;
	int num_blocks;

	if ((key_size > HV_KVP_EXCHANGE_MAX_KEY_SIZE) ||
	    (value_size > HV_KVP_EXCHANGE_MAX_VALUE_SIZE)) {
		printf("kvp_key_add_or_modify: returning 1\n");
		return (1);
	}

	/*
	 * First update the in-memory state.
	 */
	kvp_update_mem_state(pool);

	num_records = kvp_file_info[pool].num_records;
	record = kvp_file_info[pool].records;
	num_blocks = kvp_file_info[pool].num_blocks;

	for (i = 0; i < num_records; i++)
	{
		if (memcmp(key, record[i].key, key_size)) {
			continue;
		}

		/*
		 * Found a match; just update the value -
		 * this is the modify case.
		 */
		memcpy(record[i].value, value, value_size);
		kvp_update_file(pool);
		return (0);
	}

	/*
	 * Need to add a new entry;
	 */
	if (num_records == (ENTRIES_PER_BLOCK * num_blocks)) {
		/* Need to allocate a larger array for reg entries. */
		record = realloc(record, sizeof(struct kvp_record) *
			ENTRIES_PER_BLOCK * (num_blocks + 1));

		if (record == NULL) {
			return (1);
		}
		kvp_file_info[pool].num_blocks++;
	}
	memcpy(record[i].value, value, value_size);
	memcpy(record[i].key, key, key_size);
	kvp_file_info[pool].records = record;
	kvp_file_info[pool].num_records++;
	kvp_update_file(pool);
	return (0);
}


static int kvp_get_value(int pool, __u8 *key, int key_size, __u8 *value,
    int value_size)
{
	int i;
	int num_records;
	struct kvp_record *record;

	if ((key_size > HV_KVP_EXCHANGE_MAX_KEY_SIZE) ||
	    (value_size > HV_KVP_EXCHANGE_MAX_VALUE_SIZE)) {
		return (1);
	}

	/*
	 * First update the in-memory state.
	 */
	kvp_update_mem_state(pool);

	num_records = kvp_file_info[pool].num_records;
	record = kvp_file_info[pool].records;

	for (i = 0; i < num_records; i++)
	{
		if (memcmp(key, record[i].key, key_size)) {
			continue;
		}

		/*
		 * Found a match; just copy the value out.
		 */
		memcpy(value, record[i].value, value_size);
		return (0);
	}

	return (1);
}


static int kvp_pool_enumerate(int pool, int index, __u8 *key, int key_size,
    __u8 *value, int value_size)
{
	struct kvp_record *record;

	/*
	 * First update our in-memory database.
	 */
	kvp_update_mem_state(pool);
	record = kvp_file_info[pool].records;

	if (index >= kvp_file_info[pool].num_records) {
		return (1);
	}

	memcpy(key, record[index].key, key_size);
	memcpy(value, record[index].value, value_size);
	return (0);
}


void kvp_get_os_info(void)
{
	FILE *file;
	char *p, buf[512];

	uname(&uts_buf);
	os_build = uts_buf.release;
	os_name = uts_buf.sysname;
	processor_arch = uts_buf.machine;

	/*
	 * The current windows host (win7) expects the build
	 * string to be of the form: x.y.z
	 * Strip additional information we may have.
	 */
	p = strchr(os_build, '-');
	if (p) {
		*p = '\0';
	}

	/*
	 * Parse the /etc/os-release file if present:
	 * http://www.freedesktop.org/software/systemd/man/os-release.html
	 */
	file = fopen("/etc/os-release", "r");
	if (file != NULL) {
		while (fgets(buf, sizeof(buf), file))
		{
			char *value, *q;

			/* Ignore comments */
			if (buf[0] == '#') {
				continue;
			}

			/* Split into name=value */
			p = strchr(buf, '=');
			if (!p) {
				continue;
			}
			*p++ = 0;

			/* Remove quotes and newline; un-escape */
			value = p;
			q = p;
			while (*p)
			{
				if (*p == '\\') {
					++p;
					if (!*p) {
						break;
					}
					*q++ = *p++;
				} else if ((*p == '\'') || (*p == '"') ||
				    (*p == '\n')) {
					++p;
				} else {
					*q++ = *p++;
				}
			}
			*q = 0;

			if (!strcmp(buf, "NAME")) {
				p = strdup(value);
				if (!p) {
					break;
				}
				os_name = p;
			} else if (!strcmp(buf, "VERSION_ID")) {
				p = strdup(value);
				if (!p) {
					break;
				}
				os_major = p;
			}
		}
		fclose(file);
		return;
	}

	/* Fallback for older RH/SUSE releases */
	file = fopen("/etc/SuSE-release", "r");
	if (file != NULL) {
		goto kvp_osinfo_found;
	}
	file = fopen("/etc/redhat-release", "r");
	if (file != NULL) {
		goto kvp_osinfo_found;
	}

	/*
	 * We don't have information about the os.
	 */
	return;

kvp_osinfo_found:
	/* up to three lines */
	p = fgets(buf, sizeof(buf), file);
	if (p) {
		p = strchr(buf, '\n');
		if (p) {
			*p = '\0';
		}
		p = strdup(buf);
		if (!p) {
			goto done;
		}
		os_name = p;

		/* second line */
		p = fgets(buf, sizeof(buf), file);
		if (p) {
			p = strchr(buf, '\n');
			if (p) {
				*p = '\0';
			}
			p = strdup(buf);
			if (!p) {
				goto done;
			}
			os_major = p;

			/* third line */
			p = fgets(buf, sizeof(buf), file);
			if (p) {
				p = strchr(buf, '\n');
				if (p) {
					*p = '\0';
				}
				p = strdup(buf);
				if (p) {
					os_minor = p;
				}
			}
		}
	}

done:
	fclose(file);
}


/*
 * Retrieve an interface name corresponding to the specified guid.
 * If there is a match, the function returns a pointer
 * to the interface name and if not, a NULL is returned.
 * If a match is found, the caller is responsible for
 * freeing the memory.
 */
static char *kvp_if_name_to_mac(char *);

static char *get_mac_address(const char *sdlstring)
{
	char octet[4];
	char buf[256] = "\0";
	int i;
	char *mac = NULL;

	for (i = 0; i < 6; i++)
	{
		if (i != 5) {
			snprintf(octet, sizeof(octet), "%02x:", (unsigned char)sdlstring[i]);
		} else{
			snprintf(octet, sizeof(octet), "%02x", (unsigned char)sdlstring[i]);
		}
		strcat(buf, octet);
	}
	mac = strdup(buf);
	return (mac);
}


/*
 * Retrieve the MAC address given the interface name.
 */
static char *kvp_if_name_to_mac(char *if_name)
{
	char *mac_addr = NULL;
	struct ifaddrs *ifaddrs_ptr;
	struct ifaddrs *head_ifaddrs_ptr;
	struct sockaddr_dl *sdl;
	int status;

	status = getifaddrs(&ifaddrs_ptr);

	if (status >= 0) {
		head_ifaddrs_ptr = ifaddrs_ptr;
		do
		{
			sdl = (struct sockaddr_dl *)ifaddrs_ptr->ifa_addr;
			if ((sdl->sdl_type == IFT_ETHER) &&
			    (strcmp(ifaddrs_ptr->ifa_name, if_name) == 0)) {
				mac_addr = get_mac_address(LLADDR(sdl));
				break;
			}
		} while ((ifaddrs_ptr = ifaddrs_ptr->ifa_next) != NULL);
		freeifaddrs(head_ifaddrs_ptr);
	}

	return (mac_addr);
}


/*
 * Retrieve the interface name given tha MAC address.
 */
static char *kvp_mac_to_if_name(char *mac)
{
	char *if_name = NULL;
	struct ifaddrs *ifaddrs_ptr;
	struct ifaddrs *head_ifaddrs_ptr;
	struct sockaddr_dl *sdl;
	int status, i;
	char *buf_ptr;

	status = getifaddrs(&ifaddrs_ptr);

	if (status >= 0) {
		head_ifaddrs_ptr = ifaddrs_ptr;
		do
		{
			sdl = (struct sockaddr_dl *)ifaddrs_ptr->ifa_addr;
			if (sdl->sdl_type == IFT_ETHER) {
				buf_ptr = get_mac_address(LLADDR(sdl));
				for (i = 0; i < strlen(buf_ptr); i++)
				{
					buf_ptr[i] = toupper(buf_ptr[i]);
				}

				if (strncmp(buf_ptr, mac, strlen(mac)) == 0) {
					/* Caller will free the memory */
					if_name = strdup(ifaddrs_ptr->ifa_name);
					free(buf_ptr);
					break;
				}else if (buf_ptr != NULL) {
					free(buf_ptr);
				}
			}
		} while ((ifaddrs_ptr = ifaddrs_ptr->ifa_next) != NULL);
		freeifaddrs(head_ifaddrs_ptr);
	}
	return (if_name);
}


static void kvp_process_ipconfig_file(char *cmd,
    char *config_buf, int len,
    int element_size, int offset)
{
	char buf[256];
	char *p;
	char *x;
	FILE *file;

	/*
	 * First execute the command.
	 */
	file = popen(cmd, "r");
	if (file == NULL) {
		return;
	}

	if (offset == 0) {
		memset(config_buf, 0, len);
	}
	while ((p = fgets(buf, sizeof(buf), file)) != NULL)
	{
		if ((len - strlen(config_buf)) < (element_size + 1)) {
			break;
		}

		x = strchr(p, '\n');
		*x = '\0';
		strcat(config_buf, p);
		strcat(config_buf, ";");
	}
	pclose(file);
}


static void kvp_get_ipconfig_info(char *if_name,
    struct hv_kvp_ipaddr_value *buffer)
{
	char cmd[512];
	char dhcp_info[128];
	char *p;
	FILE *file;

	/*
	 * Get the address of default gateway (ipv4).
	 */

	sprintf(cmd, "%s %s", "netstat -rn | grep", if_name);
	strcat(cmd, " | awk '/default/ {print $2 }'");

	/*
	 * Execute the command to gather gateway info.
	 */
	kvp_process_ipconfig_file(cmd, (char *)buffer->gate_way,
	    (MAX_GATEWAY_SIZE * 2), INET_ADDRSTRLEN, 0);

	/*
	 * Get the address of default gateway (ipv6).
	 */
	sprintf(cmd, "%s %s", "netstat -rn inet6 | grep", if_name);
	strcat(cmd, " | awk '/default/ {print $2 }'");

	/*
	 * Execute the command to gather gateway info (ipv6).
	 */
	kvp_process_ipconfig_file(cmd, (char *)buffer->gate_way,
	    (MAX_GATEWAY_SIZE * 2), INET6_ADDRSTRLEN, 1);

	/*
	 * Gather the DNS  state.
	 * Since there is no standard way to get this information
	 * across various distributions of interest; we just invoke
	 * an external script that needs to be ported across distros
	 * of interest.
	 *
	 * Following is the expected format of the information from the script:
	 *
	 * ipaddr1 (nameserver1)
	 * ipaddr2 (nameserver2)
	 * .
	 * .
	 */
	/* Scripts are stored in /usr/local/hyperv/scripts/ directory */
	sprintf(cmd, "%s", "sh /usr/local/hyperv/scripts/hv_get_dns_info");

	/*
	 * Execute the command to gather DNS info.
	 */
	kvp_process_ipconfig_file(cmd, (char *)buffer->dns_addr,
	    (MAX_IP_ADDR_SIZE * 2), INET_ADDRSTRLEN, 0);

	/*
	 * Gather the DHCP state.
	 * We will gather this state by invoking an external script.
	 * The parameter to the script is the interface name.
	 * Here is the expected output:
	 *
	 * Enabled: DHCP enabled.
	 */


	sprintf(cmd, "%s %s", "sh /usr/local/hyperv/scripts/hv_get_dhcp_info", if_name);

	file = popen(cmd, "r");
	if (file == NULL) {
		return;
	}

	p = fgets(dhcp_info, sizeof(dhcp_info), file);
	if (p == NULL) {
		pclose(file);
		return;
	}

	if (!strncmp(p, "Enabled", 7)) {
		buffer->dhcp_enabled = 1;
	} else{
		buffer->dhcp_enabled = 0;
	}

	pclose(file);
}


static unsigned int hweight32(unsigned int *w)
{
	unsigned int res = *w - ((*w >> 1) & 0x55555555);

	res = (res & 0x33333333) + ((res >> 2) & 0x33333333);
	res = (res + (res >> 4)) & 0x0F0F0F0F;
	res = res + (res >> 8);
	return ((res + (res >> 16)) & 0x000000FF);
}


static int kvp_process_ip_address(void *addrp,
    int family, char *buffer,
    int length, int *offset)
{
	struct sockaddr_in *addr;
	struct sockaddr_in6 *addr6;
	int addr_length;
	char tmp[50];
	const char *str;

	if (family == AF_INET) {
		addr = (struct sockaddr_in *)addrp;
		str = inet_ntop(family, &addr->sin_addr, tmp, 50);
		addr_length = INET_ADDRSTRLEN;
	} else {
		addr6 = (struct sockaddr_in6 *)addrp;
		str = inet_ntop(family, &addr6->sin6_addr.s6_addr, tmp, 50);
		addr_length = INET6_ADDRSTRLEN;
	}

	if ((length - *offset) < addr_length + 1) {
		return (HV_KVP_E_FAIL);
	}
	if (str == NULL) {
		strcpy(buffer, "inet_ntop failed\n");
		return (HV_KVP_E_FAIL);
	}
	if (*offset == 0) {
		strcpy(buffer, tmp);
	} else{
		strcat(buffer, tmp);
	}
	strcat(buffer, ";");

	*offset += strlen(str) + 1;
	return (0);
}


static int
kvp_get_ip_info(int family, char *if_name, int op,
    void *out_buffer, int length)
{
	struct ifaddrs *ifap;
	struct ifaddrs *curp;
	int offset = 0;
	int sn_offset = 0;
	int error = 0;
	char *buffer;
	struct hv_kvp_ipaddr_value *ip_buffer;
	char cidr_mask[5];                       /* /xyz */
	int weight;
	int i;
	unsigned int *w = NULL;
	char *sn_str;
	struct sockaddr_in6 *addr6;

	if (op == HV_KVP_OP_ENUMERATE) {
		buffer = out_buffer;
	} else {
		ip_buffer = out_buffer;
		buffer = (char *)ip_buffer->ip_addr;
		ip_buffer->addr_family = 0;
	}

	/*
	 * On entry into this function, the buffer is capable of holding the
	 * maximum key value.
	 */

	if (getifaddrs(&ifap)) {
		strcpy(buffer, "getifaddrs failed\n");
		return (HV_KVP_E_FAIL);
	}

	curp = ifap;
	while (curp != NULL)
	{
		if (curp->ifa_addr == NULL) {
			curp = curp->ifa_next;
			continue;
		}

		if ((if_name != NULL) &&
		    (strncmp(curp->ifa_name, if_name, strlen(if_name)))) {
			/*
			 * We want info about a specific interface;
			 * just continue.
			 */
			curp = curp->ifa_next;
			continue;
		}

		/*
		 * We only support two address families: AF_INET and AF_INET6.
		 * If a family value of 0 is specified, we collect both
		 * supported address families; if not we gather info on
		 * the specified address family.
		 */
		if ((family != 0) && (curp->ifa_addr->sa_family != family)) {
			curp = curp->ifa_next;
			continue;
		}
		if ((curp->ifa_addr->sa_family != AF_INET) &&
		    (curp->ifa_addr->sa_family != AF_INET6)) {
			curp = curp->ifa_next;
			continue;
		}

		if (op == HV_KVP_OP_GET_IP_INFO) {
			/*
			 * Gather info other than the IP address.
			 * IP address info will be gathered later.
			 */
			if (curp->ifa_addr->sa_family == AF_INET) {
				ip_buffer->addr_family |= ADDR_FAMILY_IPV4;

				/*
				 * Get subnet info.
				 */
				error = kvp_process_ip_address(
					curp->ifa_netmask,
					AF_INET,
					(char *)
					ip_buffer->sub_net,
					length,
					&sn_offset);
				if (error) {
					goto gather_ipaddr;
				}
			} else {
				ip_buffer->addr_family |= ADDR_FAMILY_IPV6;

				/*
				 * Get subnet info in CIDR format.
				 */
				weight = 0;
				sn_str = (char *)ip_buffer->sub_net;
				addr6 = (struct sockaddr_in6 *)
				    curp->ifa_netmask;
				w = (unsigned int *)addr6->sin6_addr.s6_addr;

				for (i = 0; i < 4; i++)
				{
					weight += hweight32(&w[i]);
				}

				sprintf(cidr_mask, "/%d", weight);
				if ((length - sn_offset) <
				    (strlen(cidr_mask) + 1)) {
					goto gather_ipaddr;
				}

				if (sn_offset == 0) {
					strcpy(sn_str, cidr_mask);
				} else{
					strcat(sn_str, cidr_mask);
				}
				strcat((char *)ip_buffer->sub_net, ";");
				sn_offset += strlen(sn_str) + 1;
			}

			/*
			 * Collect other ip related configuration info.
			 */

			kvp_get_ipconfig_info(if_name, ip_buffer);
		}

gather_ipaddr:
		error = kvp_process_ip_address(curp->ifa_addr,
			curp->ifa_addr->sa_family,
			buffer,
			length, &offset);
		if (error) {
			goto getaddr_done;
		}

		curp = curp->ifa_next;
	}

getaddr_done:
	freeifaddrs(ifap);
	return (error);
}


static int kvp_write_file(FILE *f, char *s1, char *s2, char *s3)
{
	int ret;

	ret = fprintf(f, "%s%s%s%s\n", s1, s2, "=", s3);

	if (ret < 0) {
		return (HV_KVP_E_FAIL);
	}

	return (0);
}


static int kvp_set_ip_info(char *if_name, struct hv_kvp_ipaddr_value *new_val)
{
	int error = 0;
	char if_file[128];
	FILE *file;
	char cmd[512];
	char *mac_addr;

	/*
	 * FreeBSD - Configuration File
	 */
	snprintf(if_file, sizeof(if_file), "%s%s", "/usr/local/hyperv/", "hv_set_ip_data");
	file = fopen(if_file, "w");

	if (file == NULL) {
		syslog(LOG_ERR, "FreeBSD Failed to open config file");
		return (HV_KVP_E_FAIL);
	}

	/*
	 * First write out the MAC address.
	 */

	mac_addr = kvp_if_name_to_mac(if_name);
	if (mac_addr == NULL) {
		error = HV_KVP_E_FAIL;
		goto setval_error;
	}
	/* MAC Address */
	error = kvp_write_file(file, "HWADDR", "", mac_addr);
	if (error) {
		goto setval_error;
	}

	/* Interface Name  */
	error = kvp_write_file(file, "IF_NAME", "", if_name);
	if (error) {
		goto setval_error;
	}

	/* IP - Address  */
	error = kvp_write_file(file, "IP_ADDR", "", (char *)new_val->ip_addr);
	if (error) {
		goto setval_error;
	}

	/* Subnet Mask */
	error = kvp_write_file(file, "SUBNET", "", (char *)new_val->sub_net);
	if (error) {
		goto setval_error;
	}


	/* Gateway */
	error = kvp_write_file(file, "GATEWAY", "", (char *)new_val->gate_way);
	if (error) {
		goto setval_error;
	}

	/* DNS */
	error = kvp_write_file(file, "DNS", "", (char *)new_val->dns_addr);
	if (error) {
		goto setval_error;
	}

	/* DHCP */
	if (new_val->dhcp_enabled) {
		error = kvp_write_file(file, "DHCP", "", "1");
	} else{
		error = kvp_write_file(file, "DHCP", "", "0");
	}

	if (error) {
		goto setval_error;
	}

	goto setval_done;

	/*
	 * We are done!.
	 */

setval_done:
	free(mac_addr);
	fclose(file);

	/*
	 * Now that we have populated the configuration file,
	 * invoke the external script to do its magic.
	 */

	snprintf(cmd, sizeof(cmd), "%s %s", "sh /usr/local/hyperv/scripts/hv_set_ifconfig", if_file);
	system(cmd);
	return (0);

setval_error:
	syslog(LOG_ERR, "Failed to write config file");
	free(mac_addr);
	fclose(file);
	return (error);
}


static int
kvp_get_domain_name(char *buffer, int length)
{
	struct addrinfo hints, *info;
	int error = 0;

	gethostname(buffer, length);
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;    /* Get only ipv4 addrinfo. */
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_CANONNAME;

	error = getaddrinfo(buffer, NULL, &hints, &info);
	if (error != 0) {
		strcpy(buffer, "getaddrinfo failed\n");
		return (error);
	}
	strcpy(buffer, info->ai_canonname);
	freeaddrinfo(info);
	return (error);
}


int main(void)
{
	char *key_value, *key_name, *if_name;
	int op, pool;
	int hv_kvp_dev_fd, error, len;
	struct hv_kvp_msg *hv_kvp_dev_buf;      /* communication buffer */
	struct hv_kvp_ipaddr_value *kvp_ip_val; /* IP address buf */
	struct hv_kvp_msg *hv_msg;              /* operation buffer */

	/* Allocate memmory of hv_kvp_dev_buf */
	hv_kvp_dev_buf = malloc(sizeof(*hv_kvp_dev_buf));
	hv_msg = malloc(sizeof(*hv_msg));

	/* Memory allocation failed*/

	daemon(1, 0);
	openlog("HV_KVP", 0, LOG_USER);
	syslog(LOG_INFO, "HV_KVP starting; pid is:%d", getpid());


	/* Basics of KVP */
	kvp_get_os_info();

	if (kvp_file_init()) {
		syslog(LOG_ERR, "Failed to initialize the pools");
		exit(EXIT_FAILURE);
	}

	/* Open the Character Device */
	hv_kvp_dev_fd = open("/dev/hv_kvp_dev", O_RDWR);

	if (hv_kvp_dev_fd < 0) {
		syslog(LOG_ERR, "open /dev/hv_kvp_dev failed; error: %d %s", errno, strerror(errno));
		close(hv_kvp_dev_fd);
		exit(EXIT_FAILURE);
	}

	/* First Register the Daemon */
	hv_kvp_dev_buf->hdr.kvp_hdr.operation = HV_KVP_OP_REGISTER;
	len = write(hv_kvp_dev_fd, hv_kvp_dev_buf, sizeof(*hv_kvp_dev_buf));

	while (1)
	{
		/* Read from character device */
		len = pread(hv_kvp_dev_fd, hv_kvp_dev_buf, sizeof(*hv_kvp_dev_buf), 0);

		if (len != sizeof(struct hv_kvp_msg)) {
			syslog(LOG_ERR, "read len is: %d", len);
			continue;
		}

		/* Copy hv_kvp_dev_buf to hv_msg */
		memcpy(hv_msg, hv_kvp_dev_buf, sizeof(*hv_msg));

		/*
		 * We will use the KVP header information to pass back
		 * the error from this daemon. So, first copy the state
		 * and set the error code to success.
		 */

		op = hv_msg->hdr.kvp_hdr.operation;
		pool = hv_msg->hdr.kvp_hdr.pool;
		hv_msg->hdr.error = HV_KVP_S_OK;

		switch (op)
		{
		case HV_KVP_OP_GET_IP_INFO:

			kvp_ip_val = &hv_msg->body.kvp_ip_val;

			if_name =
			    kvp_mac_to_if_name((char *)kvp_ip_val->adapter_id);

			if (if_name == NULL) {
				/*
				 * We could not map the mac address to an
				 * interface name; return error.
				 */
				hv_msg->hdr.error = HV_KVP_E_FAIL;
				break;
			}
			error = kvp_get_ip_info(
				0, if_name, HV_KVP_OP_GET_IP_INFO,
				kvp_ip_val,
				(MAX_IP_ADDR_SIZE * 2));
			if (error) {
				hv_msg->hdr.error = error;
			}

			free(if_name);
			break;

		case HV_KVP_OP_SET_IP_INFO:

			kvp_ip_val = &hv_msg->body.kvp_ip_val;
			if_name = (char *)kvp_ip_val->adapter_id;
			if (if_name == NULL) {
				/*
				 * We could not map the guid to an
				 * interface name; return error.
				 */
				hv_msg->hdr.error = HV_KVP_GUID_NOTFOUND;
				break;
			}
			error = kvp_set_ip_info(if_name, kvp_ip_val);
			if (error) {
				hv_msg->hdr.error = error;
			}
			break;

		case HV_KVP_OP_SET:

			if (kvp_key_add_or_modify(pool,
			    hv_msg->body.kvp_set.data.key,
			    hv_msg->body.kvp_set.data.key_size,
			    hv_msg->body.kvp_set.data.msg_value.value,
			    hv_msg->body.kvp_set.data.value_size)) {
				hv_msg->hdr.error = HV_KVP_S_CONT;
			}
			break;

		case HV_KVP_OP_GET:

			if (kvp_get_value(pool,
			    hv_msg->body.kvp_set.data.key,
			    hv_msg->body.kvp_set.data.key_size,
			    hv_msg->body.kvp_set.data.msg_value.value,
			    hv_msg->body.kvp_set.data.value_size)) {
				hv_msg->hdr.error = HV_KVP_S_CONT;
			}
			break;

		case HV_KVP_OP_DELETE:

			if (kvp_key_delete(pool,
			    hv_msg->body.kvp_delete.key,
			    hv_msg->body.kvp_delete.key_size)) {
				hv_msg->hdr.error = HV_KVP_S_CONT;
			}
			break;

		default:
			break;
		}

		if (op != HV_KVP_OP_ENUMERATE) {
			goto hv_kvp_done;
		}

		/*
		 * If the pool is HV_KVP_POOL_AUTO, dynamically generate
		 * both the key and the value; if not read from the
		 * appropriate pool.
		 */
		if (pool != HV_KVP_POOL_AUTO) {
			if (kvp_pool_enumerate(pool,
			    hv_msg->body.kvp_enum_data.index,
			    hv_msg->body.kvp_enum_data.data.key,
			    HV_KVP_EXCHANGE_MAX_KEY_SIZE,
			    hv_msg->body.kvp_enum_data.data.msg_value.value,
			    HV_KVP_EXCHANGE_MAX_VALUE_SIZE)) {
				hv_msg->hdr.error = HV_KVP_S_CONT;
			}
			goto hv_kvp_done;
		}

		key_name = (char *)hv_msg->body.kvp_enum_data.data.key;
		key_value = (char *)hv_msg->body.kvp_enum_data.data.msg_value.value;

		switch (hv_msg->body.kvp_enum_data.index)
		{
		case FullyQualifiedDomainName:
			kvp_get_domain_name(key_value,
			    HV_KVP_EXCHANGE_MAX_VALUE_SIZE);
			strcpy(key_name, "FullyQualifiedDomainName");
			break;

		case IntegrationServicesVersion:
			strcpy(key_name, "IntegrationServicesVersion");
			strcpy(key_value, lic_version);
			break;

		case NetworkAddressIPv4:
			kvp_get_ip_info(AF_INET, NULL, HV_KVP_OP_ENUMERATE,
			    key_value, HV_KVP_EXCHANGE_MAX_VALUE_SIZE);
			strcpy(key_name, "NetworkAddressIPv4");
			break;

		case NetworkAddressIPv6:
			kvp_get_ip_info(AF_INET6, NULL, HV_KVP_OP_ENUMERATE,
			    key_value, HV_KVP_EXCHANGE_MAX_VALUE_SIZE);
			strcpy(key_name, "NetworkAddressIPv6");
			break;

		case OSBuildNumber:
			strcpy(key_value, os_build);
			strcpy(key_name, "OSBuildNumber");
			break;

		case OSName:
			strcpy(key_value, os_name);
			strcpy(key_name, "OSName");
			break;

		case OSMajorVersion:
			strcpy(key_value, os_major);
			strcpy(key_name, "OSMajorVersion");
			break;

		case OSMinorVersion:
			strcpy(key_value, os_minor);
			strcpy(key_name, "OSMinorVersion");
			break;

		case OSVersion:
			strcpy(key_value, os_build);
			strcpy(key_name, "OSVersion");
			break;

		case ProcessorArchitecture:
			strcpy(key_value, processor_arch);
			strcpy(key_name, "ProcessorArchitecture");
			break;

		default:
			hv_msg->hdr.error = HV_KVP_S_CONT;
			break;
		}

		/*
		 * Send the value back to the kernel. The response is
		 * already in the receive buffer.
		 */

		/* copy the hv_msg to hv_kvp_dev_buf to send the data */

hv_kvp_done:
		len = pwrite(hv_kvp_dev_fd, hv_msg, sizeof(*hv_kvp_dev_buf), 0);

		if (len != sizeof(struct hv_kvp_msg)) {
			syslog(LOG_ERR, "write len is: %d", len);
			goto hv_kvp_done;
		}
	}
}
