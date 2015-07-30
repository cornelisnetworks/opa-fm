/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

/*
 * This is a lightweight test driver for the SM's Job Management
 * functionality, with no API dependencies except OFED's UMAD
 * interface and QS's libpublic, and is intended to exercise basic
 * functionality and verify RMPP through the kernel.
 *
 * Full integration testing is presented as part of the opasadb
 * library effort.
 */

#include <arpa/inet.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "infiniband/umad.h"
#include "ibyteswap.h"

#define N 1024

#define FAIL(v, msg) { perror("errno"); printf("ERROR (%d): %s\n", v, msg); return 1; }
#define LZ_CHECK(v, msg) if (v < 0) { perror("errno"); printf("ERROR (%d): %s\n", v, msg); return 1; }
#define NZ_CHECK(v, msg) if (v != 0) { perror("errno"); printf("ERROR (%d): %s\n", v, msg); return 1; }
#define NULL_CHECK(v, msg) if (v == NULL) { printf("ERROR: %s\n", msg); return 1; }

struct ib_mad_hdr
{
	uint8_t  base_version;
	uint8_t  mgmt_class;
	uint8_t  class_version;
	uint8_t  method;
	uint16_t status;
	uint16_t class_specific;
	uint64_t tid;
	uint16_t attr_id;
	uint16_t resv;
	uint32_t attr_mod;
} __attribute__ ((packed));

struct ib_rmpp_hdr
{
	uint8_t  rmpp_version;
	uint8_t  rmpp_type;
	uint8_t  rmpp_rtime_flags;
	uint8_t  rmpp_status;
	uint32_t seg_num;
	uint32_t paylen_newwin;
} __attribute__ ((packed));

static void
dump_mad(uint8_t *buf, int len, char *prefix)
{
	int i;
	for (i = 0; i < len; ++i) {
		if (i % 16 == 0) printf(prefix);
		printf("%02x", *(buf + i));
		if (i % 4 == 3) printf(" ");
		if (i % 16 == 15) printf("\n");
	}
	if (i % 16 != 0) printf("\n");
}

static void
dump_create_resp(uint8_t *buf, int num_guids, char *prefix)
{
	int i, j, k;

	uint32_t resp_status = *buf;
	printf("%sstatus   : 0x%02x\n", prefix, resp_status);
	if ((resp_status & 0xfe) == 0)
	{
		uint64_t resp_id = ntohl(*(uint32_t *)(buf + 1));
		resp_id = (resp_id << 32) | ntohl(*(uint32_t *)(buf + 5));
		printf("%sid       : 0x%016"PRIu64"\n", prefix, resp_id);
		uint16_t resp_portcount = ntohs(*(uint16_t *)(buf + 9));
		printf("%sportcnt  : 0x%04x\n", prefix, resp_portcount);
		uint16_t resp_swcount = ntohs(*(uint16_t *)(buf + 11));
		printf("%sswcnt    : 0x%04x\n", prefix, resp_swcount);
		printf("%smap      :\n", prefix);
		for (i = 0; i < num_guids; ++i)
			printf("    %05d -> %05d\n", i, ntohs(*(uint16_t *)(buf + 13 + i * 2)));
		uint16_t resp_swcount2 = ntohs(*(uint16_t *)(buf + 13 + resp_portcount * 2));
		printf("%sswcnt2   : 0x%04x\n", prefix, resp_swcount2);
		printf("%scost     :\n", prefix);
		for (i = 0, k = 0; i < resp_swcount; ++i) {
			printf("%s  ", prefix);
			for (j = 0; j <= i; ++j)
				printf("----- ");
			for (j = i + 1; j < resp_swcount; ++j, ++k)
				printf("%05d ", ntohs(*(uint16_t *)(buf + 13 + resp_portcount * 2 + 2 + i * 2)));
			printf("\n");
		}
	}
}

static void
dump_get_jobs_resp(uint8_t *buf, char *prefix)
{
	uint32_t resp_status = *buf;
	printf("%sstatus   : 0x%02x\n", prefix, resp_status);
	if ((resp_status & 0xfe) == 0)
	{
		uint16_t resp_count = ntohs(*(uint16_t *)(buf + 1));
		printf("%scount    : 0x%04x\n", prefix, resp_count);
	}
}

static void
dump_set_use_matrix_resp(uint8_t *buf, char *prefix)
{
	uint32_t resp_status = *buf;
	printf("%sstatus   : 0x%02x\n", prefix, resp_status);
}

static void
dump_complete_resp(uint8_t *buf, char *prefix)
{
	uint32_t resp_status = *buf;
	printf("%sstatus   : 0x%02x\n", prefix, resp_status);
}

static void
dump_get_use_matrix_resp(uint8_t *buf, char *prefix)
{
	uint32_t resp_status = *buf;
	printf("%sstatus   : 0x%02x\n", prefix, resp_status);
}

static int
send_message(int pid, int aid, int lid, int msg, char *buf, int len, char **outbuf, int *outlen)
{
	static uint32_t tid = 1;

	int rc;
	void *rbuf;
	int rlen;

	struct ib_mad_hdr madh =
		{ .base_version   = 0x01
		, .mgmt_class     = 0x03
		, .class_version  = 0x02
		, .method         = 0x14
		, .status         = 0x00
		, .class_specific = 0x00
		, .tid            = (uint64_t)htonl(tid++) << 32
		, .attr_id        = 0xb2ff
		, .resv           = 0x00
		, .attr_mod       = htonl(msg)
		};

	struct ib_rmpp_hdr rmpph;
	memset(&rmpph, 0, sizeof(rmpph));
	if (len > 200)
		rmpph.rmpp_rtime_flags |= 1;

	void *mad = umad_alloc(1, umad_size() + 56 + len);
	NULL_CHECK(mad, "alloc send mad failed");

	memset(mad, 0, umad_size() + 56 + len);
	memcpy(umad_get_mad(mad), &madh, sizeof(madh));
	memcpy(umad_get_mad(mad) + 24, &rmpph, sizeof(rmpph));
	if (len) memcpy(umad_get_mad(mad) + 56, buf, len);
	umad_set_addr(mad, lid, 1, 0, 0x80010000);

	printf("raw send mad:\n");
	dump_mad((uint8_t *)mad + umad_size(), 56 + len, "  ");

	rc = umad_send(pid, aid, mad, 56 + len, 1000, 0);
	umad_free(mad);
	LZ_CHECK(rc, "send mad failed");

	rlen = 256;
	rbuf = umad_alloc(1, umad_size() + rlen);
	NULL_CHECK(rbuf, "receive buffer alloc failed");
	rc = umad_recv(pid, rbuf, &rlen, 1000);
	if (rc < 0) {
		umad_free(rbuf);
		if (rlen <= 256)
			FAIL(rc, "recv mad failed (1)");
		rbuf = umad_alloc(1, umad_size() + rlen);
		NULL_CHECK(rbuf, "receive buffer alloc failed");
		rc = umad_recv(pid, rbuf, &rlen, 1000);
		LZ_CHECK(rc, "recv mad failed (2)");
	}

	*outbuf = rbuf;
	*outlen = rlen;

	return 0;
}

static int
test_create_message(int pid, int aid, int lid)
{
	int rc, i;
	int num_guids = 32;
	int blen = 146 + num_guids * 8;
	char *rbuf = NULL;
	int rlen = 0;

	char buf[blen];
	strncpy(buf, "job name", 64);
	strncpy(buf + 64, "application name", 64);
	*(uint32_t *)(buf + 128) = htonl(0x01234567);
	*(uint32_t *)(buf + 132) = htonl(0x89abcdef);
	*(uint32_t *)(buf + 136) = htonl(0x01234567);
	*(uint32_t *)(buf + 140) = htonl(0x89abcdef);
	*(uint16_t *)(buf + 144) = htons(num_guids);
	for (i = 0; i < num_guids; ++i) {
		*(uint32_t *)(buf + 146 + i * 8    ) = htonl(0x00117500);
		*(uint32_t *)(buf + 146 + i * 8 + 4) = htonl(i * 8);
	}

	rc = send_message(pid, aid, lid, 2, buf, blen, &rbuf, &rlen);
	NZ_CHECK(rc, "send message failed");

	printf("raw response mad:\n");
	dump_mad((uint8_t *)rbuf + umad_size(), rlen, "  ");

	printf("message response (create):\n");
	dump_create_resp((uint8_t *)rbuf + umad_size() + 56, num_guids, "  ");

	umad_free(rbuf);

	return 0;
}

static int
test_get_jobs_message(int pid, int aid, int lid)
{
	int rc;
	char *rbuf = NULL;
	int rlen = 0;

	rc = send_message(pid, aid, lid, 10, NULL, 0, &rbuf, &rlen);
	NZ_CHECK(rc, "send message failed");

	printf("raw response mad:\n");
	dump_mad((uint8_t *)rbuf + umad_size(), rlen, "  ");

	printf("message response (get jobs):\n");
	dump_get_jobs_resp((uint8_t *)rbuf + umad_size() + 56, "  ");

	umad_free(rbuf);

	return 0;
}

static uint64_t
get_first_job_id(uint8_t * buf)
{
	uint32_t status = *buf;
	if ((status & 0xfe) == 0)
	{
		uint16_t count = ntohs(*(uint16_t *)(buf + 1));
		if (count > 0)
		{
			uint64_t guid = 0;
			guid = ((uint64_t)ntohl(*(uint32_t *)(buf + 3)) << 32)
			     | (uint64_t)ntohl(*(uint32_t *)(buf + 7));
			return guid;
		}
	}

	return 0;
}

static int
test_set_use_matrix_message(int pid, int aid, int lid)
{
	int rc;
	char *buf = NULL;
	char *rbuf = NULL;
	int blen = 0, rlen = 0;
	uint64_t id;

	rc = send_message(pid, aid, lid, 10, NULL, 0, &rbuf, &rlen);
	NZ_CHECK(rc, "send message failed");

	printf("raw response mad:\n");
	dump_mad((uint8_t *)rbuf + umad_size(), rlen, "  ");

	printf("message response (get jobs):\n");
	dump_get_jobs_resp((uint8_t *)rbuf + umad_size() + 56, "  ");

	id = get_first_job_id((uint8_t *)rbuf + umad_size() + 56);

	umad_free(rbuf);

	if (id == 0)
	{
		printf("no jobs found\n");
		return 0;
	}

	buf = (char *)malloc(23);
	NULL_CHECK(buf, "failed to allocate mad buffer");

	*(uint32_t *)(buf +  0) = htonl(id >> 32);
	*(uint32_t *)(buf +  4) = htonl(id & 0x00000000ffffffffull);
	*(uint16_t *)(buf +  8) = htons(0x0000);
	*(uint8_t  *)(buf + 10) = 0x00;
	*(uint16_t *)(buf + 11) = htons(0x0002);
	*(uint16_t *)(buf + 13) = htons(0x0000);
	*(uint16_t *)(buf + 15) = htons(0x0001);
	*(uint8_t  *)(buf + 17) = 0x02;
	*(uint16_t *)(buf + 18) = htons(0x0003);
	*(uint16_t *)(buf + 20) = htons(0x0004);
	*(uint8_t  *)(buf + 22) = 0x05;
	blen = 23;

	rc = send_message(pid, aid, lid, 3, buf, blen, &rbuf, &rlen);
	free(buf);
	NZ_CHECK(rc, "send message failed");

	printf("raw response mad:\n");
	dump_mad((uint8_t *)rbuf + umad_size(), rlen, "  ");

	printf("message response (set use matrix):\n");
	dump_set_use_matrix_resp((uint8_t *)rbuf + umad_size() + 56, "  ");

	return 0;
}

static int
test_complete_message(int pid, int aid, int lid)
{
	int rc;
	char *buf = NULL;
	char *rbuf = NULL;
	int blen = 0, rlen = 0;
	uint64_t id;

	rc = send_message(pid, aid, lid, 10, NULL, 0, &rbuf, &rlen);
	NZ_CHECK(rc, "send message failed");

	printf("raw response mad:\n");
	dump_mad((uint8_t *)rbuf + umad_size(), rlen, "  ");

	printf("message response (get jobs):\n");
	dump_get_jobs_resp((uint8_t *)rbuf + umad_size() + 56, "  ");

	id = get_first_job_id((uint8_t *)rbuf + umad_size() + 56);

	umad_free(rbuf);

	if (id == 0)
	{
		printf("no jobs found\n");
		return 0;
	}

	buf = (char *)malloc(sizeof(uint64_t));
	NULL_CHECK(buf, "failed to allocate mad buffer");

	*(uint32_t *)(buf + 0) = htonl(id >> 32);
	*(uint32_t *)(buf + 4) = htonl(id & 0x00000000ffffffffull);
	blen = sizeof(uint64_t);

	rc = send_message(pid, aid, lid, 5, buf, blen, &rbuf, &rlen);
	free(buf);
	NZ_CHECK(rc, "send message failed");

	printf("raw response mad:\n");
	dump_mad((uint8_t *)rbuf + umad_size(), rlen, "  ");

	printf("message response (complete):\n");
	dump_complete_resp((uint8_t *)rbuf + umad_size() + 56, "  ");

	return 0;
}

static int
test_get_use_matrix_message(int pid, int aid, int lid)
{
	int rc;
	char *buf = NULL;
	char *rbuf = NULL;
	int blen = 0, rlen = 0;
	uint64_t id;

	rc = send_message(pid, aid, lid, 10, NULL, 0, &rbuf, &rlen);
	NZ_CHECK(rc, "send message failed");

	printf("raw response mad:\n");
	dump_mad((uint8_t *)rbuf + umad_size(), rlen, "  ");

	printf("message response (get jobs):\n");
	dump_get_jobs_resp((uint8_t *)rbuf + umad_size() + 56, "  ");

	id = get_first_job_id((uint8_t *)rbuf + umad_size() + 56);

	umad_free(rbuf);

	if (id == 0)
	{
		printf("no jobs found\n");
		return 0;
	}

	buf = (char *)malloc(sizeof(uint64_t));
	NULL_CHECK(buf, "failed to allocate mad buffer");

	*(uint32_t *)(buf + 0) = htonl(id >> 32);
	*(uint32_t *)(buf + 4) = htonl(id & 0x00000000ffffffffull);
	blen = sizeof(uint64_t);

	rc = send_message(pid, aid, lid, 9, buf, blen, &rbuf, &rlen);
	free(buf);
	NZ_CHECK(rc, "send message failed");

	printf("raw response mad:\n");
	dump_mad((uint8_t *)rbuf + umad_size(), rlen, "  ");

	printf("message response (get use matrix):\n");
	dump_get_use_matrix_resp((uint8_t *)rbuf + umad_size() + 56, "  ");

	return 0;
}

int main(void)
{
	int rc;

	rc = umad_init();
	LZ_CHECK(rc, "umad init failed");

	char cas[1][UMAD_CA_NAME_LEN];
	memset(cas, 0, UMAD_CA_NAME_LEN);

	rc = umad_get_cas_names((void*)cas, 1);
	LZ_CHECK(rc, "get ca name failed");

	int pid = umad_open_port(cas[0], 1);
	LZ_CHECK(pid, "open port failed");
	
	uint32_t mask[4] = {0, 0, 0, 0};
	int aid = umad_register(pid, 3, 2, 1, (void *)mask);
	LZ_CHECK(aid, "registration failed");

	umad_port_t port;
	memset(&port, 0, sizeof(port));
	rc = umad_get_port(cas[0], 1, &port);
	LZ_CHECK(rc, "get port failed");

	int lid = port.base_lid;
	umad_release_port(&port);

	printf("TEST: get jobs\n");
	rc = test_get_jobs_message(pid, aid, lid);
	NZ_CHECK(rc, "'get jobs' test failed");

	printf("TEST: create\n");
	rc = test_create_message(pid, aid, lid);
	NZ_CHECK(rc, "'create' test failed");

	printf("TEST: get jobs\n");
	rc = test_get_jobs_message(pid, aid, lid);
	NZ_CHECK(rc, "'get jobs' test failed");

	printf("TEST: set use matrix\n");
	rc = test_set_use_matrix_message(pid, aid, lid);
	NZ_CHECK(rc, "'set use matrix' test failed");

	printf("TEST: get use matrix\n");
	rc = test_get_use_matrix_message(pid, aid, lid);
	NZ_CHECK(rc, "'get use matrix' test failed");

	printf("TEST: complete\n");
	rc = test_complete_message(pid, aid, lid);
	NZ_CHECK(rc, "'complete' test failed");

	printf("TEST: get jobs\n");
	rc = test_get_jobs_message(pid, aid, lid);
	NZ_CHECK(rc, "'get jobs' test failed");

	return 0;
}

