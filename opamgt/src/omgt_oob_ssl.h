/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef __OMGT_OOB_SSL_H
#define __OMGT_OOB_SSL_H

extern OMGT_STATUS_T omgt_oob_ssl_init(struct omgt_port *port);
extern void *omgt_oob_ssl_client_open(struct omgt_port *port, const char *ffDir,
	const char *ffCertificate, const char *ffPrivateKey,
	const char *ffCaCertificate, uint32_t ffCertChainDepth,
	const char *ffDHParameters, uint32_t ffCaCrlEnabled, const char *ffCaCrl);
extern void *omgt_oob_ssl_connect(struct omgt_port *port, void *context, int serverfd);
extern int omgt_oob_ssl_read(struct omgt_port *port, void *session,
	unsigned char *buffer, int bufferLength);
extern int omgt_oob_ssl_write(struct omgt_port *port, void *session,
	unsigned char *buffer, int bufferLength);

#endif

