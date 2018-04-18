/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

 * ** END_ICS_COPYRIGHT5   ****************************************/
/* [ICS VERSION STRING: unknown] */

#include <sys/types.h>
#include <unistd.h>
#include <openssl/rand.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/conf.h>
#include <openssl/x509v3.h>
#include <openssl/pem.h>
#include <openssl/obj_mac.h>
#include <openssl/ec.h>
#include <string.h>


#define OPAMGT_PRIVATE 1
#include "ib_utils_openib.h"
#include "omgt_oob_ssl.h"

#define DNS_NAME_MAX_LENGTH     256
#define FE_SSL_FM_DNS_NAME     "<sever-name>.com"

#define FE_SSL_CIPHER_LIST    "ECDHE-ECDSA-AES128-GCM-SHA256:DHE-DSS-AES256-SHA"

// Include all workaround patches. Exclude versions SSLv2, SSLv3, and lower.
// Include Diffie-Hellman protocol for Ephemeral Keying, in order to take
// advantage of Forward Secrecy property of TLS
#define FE_SSL_OPTIONS      SSL_OP_ALL | SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3 | SSL_OP_SINGLE_DH_USE

#define FE_SSL_IS_VALID_FN(d, f, b)   ((strlen(d) + strlen(f) + 1) < (sizeof(b) - 1))
#define FE_SSL_GET_FN(d, f, b) { \
    if (d[strlen(d)] == '/') \
        snprintf(b, sizeof(b), "%s%s", d, f); \
    else \
        snprintf(b, sizeof(b), "%s/%s", d, f); \
}

static void omgt_oob_ssl_print_ciphers(struct omgt_port *port, SSL *ssl)
{
	int index = 0;
	const char *next = NULL;
	do {
		next = SSL_get_cipher_list(ssl, index);
		if (next != NULL) {
			OMGT_DBGPRINT(port, "CIPHER[%d] %s\n", index, next);
			index++;
		}
	}while (next != NULL);
}

static void omgt_oob_ssl_print_error_stack(struct omgt_port *port)
{
	unsigned long error, i;

	for (i = 0; i < 5; i++) {
		if ((error = ERR_get_error())) {
			OMGT_OUTPUT_ERROR(port, "\tStackErr[%d] %s\n", (int)i, ERR_error_string(error, NULL));
		}
	}
}

static int omgt_oob_ssl_ctx_pem_password_cb(char *buf, int size, int rwflag, void *userdata)
{
	int len = 0;

	if (buf && userdata) {
		memset(buf, 0, size);
		strncpy(buf, userdata, size - 1);
		buf[size - 1] = 0;
		len = strlen(buf);
	}

	return (len);
}

static OMGT_STATUS_T omgt_oob_ssl_post_connection_cert_check(struct omgt_port *port, X509 *cert)
{
	X509_STORE_CTX *x509StoreContext;
	int err = 0;

	if (!cert)
		return OMGT_STATUS_ERROR;

	if (port->is_x509_store_initialized) {
		// create the X509 store context
		if (!(x509StoreContext = X509_STORE_CTX_new())) {
			OMGT_OUTPUT_ERROR(port, "Failed to allocate x509 store context\n");
			return OMGT_STATUS_INSUFFICIENT_MEMORY;
		}

		// initialize the X509 store context with the global X509 store and certificate
		// of the peer.
		if (X509_STORE_CTX_init(x509StoreContext, port->x509_store, cert, NULL) != 1) {
			OMGT_OUTPUT_ERROR(port, "Failed to initialize x509 store context\n");
			return OMGT_STATUS_ERROR;
		}

		// validate the certificate of the peer against the CRLs in the global X509 store
		if (X509_verify_cert(x509StoreContext) != 1) {
			err = X509_STORE_CTX_get_error(x509StoreContext);
			OMGT_OUTPUT_ERROR(port, "Failed to verify certificate: %s (%d)\n",
				X509_verify_cert_error_string((long)err), err);
			return OMGT_STATUS_REJECT;
		}

		X509_STORE_CTX_cleanup(x509StoreContext);
	}

	return OMGT_STATUS_SUCCESS;
}

OMGT_STATUS_T omgt_oob_ssl_init(struct omgt_port *port)
{
	OMGT_STATUS_T status = OMGT_STATUS_SUCCESS;

	if (!port->is_ssl_initialized) {
		port->is_ssl_initialized = 1;
		// initialize the OpenSSL library and error strings.
		SSL_library_init();
		SSL_load_error_strings();

		// allocate the method that will be used to create a SSL/TLS connection.
		// The method defines which protocol version to be used by the connection.
		port->ssl_client_method = port->oob_input.is_esm_fe ?
			(SSL_METHOD *)TLSv1_client_method() : (SSL_METHOD *)TLSv1_2_client_method();

		if (!port->ssl_client_method) {
			port->is_ssl_initialized = 0;
			status = OMGT_STATUS_INSUFFICIENT_MEMORY;
			OMGT_OUTPUT_ERROR(port, "failed to allocate SSL method\n");
		}
	}

	return status;
}

void* omgt_oob_ssl_client_open(struct omgt_port *port, const char *ffDir,
	const char *ffCertificate, const char *ffPrivateKey,
	const char *ffCaCertificate, uint32_t ffCertChainDepth,
	const char *ffDHParameters, uint32_t ffCaCrlEnabled, const char *ffCaCrl)
{
	SSL_CTX *context = NULL;
	X509_LOOKUP *x509StoreLookup;
	EC_KEY *ecdhPrime256v1 = NULL;
	char fileName[OMGT_OOB_SSL_PATH_SIZE], ffCertificateFn[OMGT_OOB_SSL_PATH_SIZE],
		ffCaCertificateFn[OMGT_OOB_SSL_PATH_SIZE];
	const char *bogusPassword = "boguspswd";

	if (!port->is_ssl_initialized) {
		OMGT_OUTPUT_ERROR(port, "TLS/SSL is not initialized\n");
		return (void *)context;
	}

	if (!ffDir || !ffCertificate || !ffPrivateKey || !ffCaCertificate || !ffCaCrl) {
		OMGT_OUTPUT_ERROR(port, "invalid parameter\n");
		return (void *)context;
	}

	if (!FE_SSL_IS_VALID_FN(ffDir, ffCertificate, fileName) ||
		!FE_SSL_IS_VALID_FN(ffDir, ffPrivateKey, fileName) ||
		!FE_SSL_IS_VALID_FN(ffDir, ffCaCertificate, fileName) ||
		!FE_SSL_IS_VALID_FN(ffDir, ffCaCrl, fileName)) {
		OMGT_OUTPUT_ERROR(port, "invalid file name\n");
		return (void *)context;
	}

	// allocate a new SSL/TLS connection context
	context = SSL_CTX_new(port->ssl_client_method);
	if (!context) {
		OMGT_OUTPUT_ERROR(port, "failed to allocate SSL context\n");
		return (void *)context;
	}

	// Usage of the passphrase would require the admin to manual enter
	// a password each time a FF utility is ran, so set the context
	// callback to override the passphrases prompt stage of SSL/TLS by
	// specifying a bogus password.
	SSL_CTX_set_default_passwd_cb(context, omgt_oob_ssl_ctx_pem_password_cb);
	SSL_CTX_set_default_passwd_cb_userdata(context, (void *)bogusPassword);

	// set the location and name of certificate file to be used by the context.
	FE_SSL_GET_FN(ffDir, ffCertificate, ffCertificateFn);
	if (SSL_CTX_use_certificate_file(context, ffCertificateFn, SSL_FILETYPE_PEM) <= 0) {
		OMGT_OUTPUT_ERROR(port, "failed to set up the certificate file: %s\n", ffCertificateFn);
		omgt_oob_ssl_print_error_stack(port);
		goto bail;
	}

	// set the location and name of the private key file to be used by the context.
	FE_SSL_GET_FN(ffDir, ffPrivateKey, fileName);
	if (SSL_CTX_use_PrivateKey_file(context, fileName, SSL_FILETYPE_PEM) <= 0) {
		OMGT_OUTPUT_ERROR(port, "failed to set up the private key file: %s\n", fileName);
		omgt_oob_ssl_print_error_stack(port);
		goto bail;
	}

	// check the consistency of the private key with the certificate
	if (SSL_CTX_check_private_key(context) != 1) {
		OMGT_OUTPUT_ERROR(port, "private key and certificate do not match\n");
		omgt_oob_ssl_print_error_stack(port);
		goto bail;
	}

	// set the location and name of the trusted CA certificates file to be used
	// by the context.
	FE_SSL_GET_FN(ffDir, ffCaCertificate, ffCaCertificateFn);
	if (!SSL_CTX_load_verify_locations(context, ffCaCertificateFn, NULL)) {
		OMGT_OUTPUT_ERROR(port, "failed to set up the CA certificates file: %s\n", ffCaCertificateFn);
		omgt_oob_ssl_print_error_stack(port);
		goto bail;
	}

	// Certificate Revocation Lists (CRL).  CRLs are an addtional mechanism
	// used to verify certificates, in order to ensure that they have not
	// expired or been revoked.
	if (ffCaCrlEnabled && !port->is_x509_store_initialized) {
		if (!(port->x509_store = X509_STORE_new())) {
			OMGT_OUTPUT_ERROR(port, "Failed to allocate X509 store for CRLs\n");
			goto bail;
		}

		if (X509_STORE_load_locations(port->x509_store, ffCaCertificateFn, ffDir) != 1) {
			OMGT_OUTPUT_ERROR(port, "Failed to load FF CA certificate file into X509 store\n");
			goto bail;
		}

		if (X509_STORE_set_default_paths(port->x509_store) != 1) {
			OMGT_OUTPUT_ERROR(port, "Failed to set the default paths for X509 store\n");
			goto bail;
		}

		if (!(x509StoreLookup = X509_STORE_add_lookup(port->x509_store, X509_LOOKUP_file()))) {
			OMGT_OUTPUT_ERROR(port, "Failed add file lookup object to X509 store\n");
			goto bail;
		}

		FE_SSL_GET_FN(ffDir, ffCaCrl, fileName);
		if (X509_load_crl_file(x509StoreLookup, fileName, X509_FILETYPE_PEM) != 1) {
			OMGT_OUTPUT_ERROR(port, "Failed to load the FF CRL file into X509 store: file %s\n", fileName);
			goto bail;
		}

		X509_STORE_set_flags(port->x509_store, X509_V_FLAG_CRL_CHECK | X509_V_FLAG_CRL_CHECK_ALL);

		if (!(x509StoreLookup = X509_STORE_add_lookup(port->x509_store, X509_LOOKUP_hash_dir()))) {
			OMGT_OUTPUT_ERROR(port, "Failed add hash directory lookup object to X509 store\n");
			goto bail;
		}

		if (!X509_LOOKUP_add_dir(x509StoreLookup, ffDir, X509_FILETYPE_PEM)) {
			OMGT_OUTPUT_ERROR(port, "Failed add FF CRL directory to X509 store\n");
			goto bail;
		}

		port->is_x509_store_initialized = 1;
	}

	// set options to be enforced upon the SSL/TLS connection.
	SSL_CTX_set_verify(context, SSL_VERIFY_PEER, NULL);
	SSL_CTX_set_verify_depth(context, ffCertChainDepth);
	SSL_CTX_set_options(context, FE_SSL_OPTIONS);

	// initialize Diffie-Hillmen parameters
	if (!port->is_dh_params_initialized) {
		FILE *dhParmsFile;

		// set the location and name of the Diffie-Hillmen parameters file to be used
		// by the context.
		FE_SSL_GET_FN(ffDir, ffDHParameters, fileName);

		dhParmsFile = fopen(fileName, "r");
		if (!dhParmsFile) {
			OMGT_OUTPUT_ERROR(port, "failed to open Diffie-Hillmen parameters PEM file: %s\n", fileName);
			goto bail;
		} else {
			port->dh_params = PEM_read_DHparams(dhParmsFile, NULL, NULL, NULL);
			fclose(dhParmsFile);
			if (!port->dh_params) {
				OMGT_OUTPUT_ERROR(port, "failed to read Diffie-Hillmen parameters PEM file: %s\n", fileName);
				goto bail;
			} else {
				port->is_dh_params_initialized = 1;
				// set DH for all SSL sesions in this context (should only be one per port)
				SSL_CTX_set_tmp_dh(context, port->dh_params);
			}
		}
	}

	// set up the cipher list with the cipher suites that are allowed
	// to be used in establishing the connection.
	if (!SSL_CTX_set_cipher_list(context, FE_SSL_CIPHER_LIST)) {
		OMGT_OUTPUT_ERROR(port, "failed to set up the cipher list\n");
		goto bail;
	}

	// set up the elliptic curve to be used in establishing the connection.
	ecdhPrime256v1 = EC_KEY_new_by_curve_name(NID_X9_62_prime256v1);
	if (ecdhPrime256v1 == NULL) {
		OMGT_OUTPUT_ERROR(port, "Failed to to create curve: prime256v1\n");
		goto bail;
	}

	SSL_CTX_set_tmp_ecdh(context, ecdhPrime256v1);
	EC_KEY_free(ecdhPrime256v1);

	return (void *)context;

bail:

	if (context) {
		SSL_CTX_free(context);
	}

	return NULL;
}

void* omgt_oob_ssl_connect(struct omgt_port *port, void *context, int serverfd)
{
	OMGT_STATUS_T status = OMGT_STATUS_SUCCESS;
	int rc;
	SSL *session = NULL;
	X509 *cert = NULL;

	if (!port || !context) {
		OMGT_OUTPUT_ERROR(port, "invalid context parameter\n");
		return (void *)session;
	}

	// allocate a new session to be associated with the SSL/TLS connection
	if (!(session = SSL_new((SSL_CTX *)context))) {
		OMGT_OUTPUT_ERROR(port, "Failed to allocate new SSL/TLS session for socket fd %d\n", serverfd);
		return (void *)session;
	}

	// assign peer file descriptor to the SSL/TLS session
	SSL_set_fd(session, serverfd);
	if (port->dbg_file) {
		omgt_oob_ssl_print_ciphers(port, session);
	}

	// attempt to establish SSL/TLS connection to the server
	if ((rc = SSL_connect(session)) != 1) {
		OMGT_OUTPUT_ERROR(port, "SSL/TLS handshake failed for connect: err %d, rc %d, %s\n", SSL_get_error(session, rc), rc, strerror(errno));
		omgt_oob_ssl_print_error_stack(port);
	} else {
		OMGT_DBGPRINT(port, "ACTIVE cipher suite: %s\n", SSL_CIPHER_get_name(SSL_get_current_cipher(session)));

		// request and verify the certificate of the server.  Typically, it is
		// required for a server to present a certificate.  Strict security
		// enforcement is to be done for the STL FM, so if a certificate
		// is not presented by the server the connection will be rejected.
		if (!(cert = SSL_get_peer_certificate(session))) {
			status = OMGT_STATUS_NOT_FOUND;
			OMGT_OUTPUT_ERROR(port, "server has no certifcate to verfiy\n");
		} else {
			long result;

			// post-connection verification is very important, because it allows for
			// much finer grained control over the certificate that is presented by
			// the server, beyond the basic certificate verification that is done by
			// the SSL/TLS protocol proper during the connect operation.
			if (!(status = omgt_oob_ssl_post_connection_cert_check(port, cert))) {
				// do the final phase of post-connection verification of the
				// certificate presented by the server.
				result = SSL_get_verify_result(session);
				if (result != X509_V_OK) {
					status = OMGT_STATUS_REJECT;
					OMGT_OUTPUT_ERROR(port, "verification of server certificate failed: err %ld\n", result);
				}
			}

			// deallocate the cert
			X509_free(cert);
		}
	}
	if (status && session) {
		SSL_free(session);
		session = NULL;
	}

	return (void *)session;
}

int omgt_oob_ssl_read(struct omgt_port *port, void *session, unsigned char *buffer, int bufferLength)
{
	int erc, bytesRead = -1;

	if (buffer) {
		bytesRead = SSL_read((SSL *)session, (void *)buffer, bufferLength);
		OMGT_DBGPRINT(port, "Received %d bytes from SSL/TLS server\n", bytesRead);

		// FIXME: cjking - Implement retry logic code for certain return error conditions
		erc = SSL_get_error((SSL *)session, bytesRead);
		switch (erc) {
		case SSL_ERROR_NONE:
			break;
		case SSL_ERROR_ZERO_RETURN:
			OMGT_DBGPRINT(port, "Warning, SSL read failed with SSL_ERROR_ZERO_RETURN error, retrying read\n");
			break;
		case SSL_ERROR_WANT_READ:
			OMGT_DBGPRINT(port, "Warning, SSL read failed with SSL_ERROR_WANT_READ error, retrying read\n");
			break;
		case SSL_ERROR_WANT_WRITE:
			OMGT_DBGPRINT(port, "Warning, SSL read failed with SSL_ERROR_WANT_WRITE error, retrying read\n");
			break;
		default:
			OMGT_OUTPUT_ERROR(port, "SSL read failed with rc %d: %s\n", erc, strerror(errno));
			break;
		}
	}

	return bytesRead;
}

int omgt_oob_ssl_write(struct omgt_port *port, void *session, unsigned char *buffer, int bufferLength)
{
	int erc, bytesWritten = -1;

	if (buffer) {
		bytesWritten = SSL_write((SSL *)session, (void *)buffer, bufferLength);
		OMGT_DBGPRINT(port, "Sent %d bytes to SSL/TLS server\n", bytesWritten);

		// FIXME: cjking - Implement retry logic code for certain return error conditions
		erc = SSL_get_error((SSL *)session, bytesWritten);
		switch (erc) {
		case SSL_ERROR_NONE:
			break;
		case SSL_ERROR_ZERO_RETURN:
			OMGT_DBGPRINT(port, "Warning, SSL read failed with SSL_ERROR_ZERO_RETURN error, retrying read\n");
			break;
		case SSL_ERROR_WANT_READ:
			OMGT_DBGPRINT(port, "Warning, SSL read failed with SSL_ERROR_WANT_READ error, retrying read\n");
			break;
		case SSL_ERROR_WANT_WRITE:
			OMGT_DBGPRINT(port, "Warning, SSL read failed with SSL_ERROR_WANT_WRITE error, retrying read\n");
			break;
		default:
			OMGT_OUTPUT_ERROR(port, "SSL read failed with rc %d: %s\n", erc, strerror(errno));
			break;
		}
	}

	return bytesWritten;
}

