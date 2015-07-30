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

 * ** END_ICS_COPYRIGHT5   ****************************************/
/* [ICS VERSION STRING: unknown] */

#include <openssl/rand.h> 
#include <openssl/ssl.h> 
#include <openssl/err.h> 
#include <openssl/conf.h>
#include <openssl/x509v3.h>
#include <openssl/pem.h>
 #include <openssl/obj_mac.h>
 #include <openssl/ec.h>
#include "fe_ssl.h"

extern int g_verbose;
extern int g_feESM;
 
#define DNS_NAME_MAX_LENGTH     256
#define FE_SSL_FM_DNS_NAME     "<sever-name>.com"

static int g_fe_ssl_inited = 0;
static SSL_METHOD * g_fe_ssl_client_method = NULL; 
static int g_fe_ssl_dh_params_inited = 0;
static DH *g_fe_ssl_dh_parms = NULL;
static int g_fe_ssl_x509_store_inited=0;
static X509_STORE *g_fe_ssl_x509_store = NULL;

#define FE_SSL_CIPHER_LIST	"ECDHE-ECDSA-AES128-GCM-SHA256:DHE-DSS-AES256-SHA"

// Include all workaround patches. Exclude versions SSLv2, SSLv3, and lower.
// Include Diffie-Hellman protocol for Ephemeral Keying, in order to take
// advantage of Forward Secrecy property of TLS
#define FE_SSL_OPTIONS      SSL_OP_ALL | SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3 | SSL_OP_SINGLE_DH_USE

#define FE_SSL_IS_VALID_FN(d, f, b)   ((strlen(d) + strlen(f) + 1) < (sizeof(b) - 1))
#define FE_SSL_GET_FN(d, f, b) { \
    if (d[strlen(d)] == '/') \
        sprintf(b, "%s%s", d, f); \
    else \
        sprintf(b, "%s/%s", d, f); \
}

static void fe_ssl_print_ciphers(SSL *ssl)
{
  int index = 0;
  const char *next = NULL;
  do {
    next = SSL_get_cipher_list(ssl,index);
    if (next != NULL) {
      fprintf(stderr, "%s: CIPHER[%d] %s\n", __func__, index, next);
      index++;
    }
  }
  while (next != NULL);
}

static void fe_ssl_print_error_stack()
{
    unsigned long error, i;

    for (i=0; i < 5; i++) {
        if ((error = ERR_get_error())) {
            fprintf(stderr, "\tStackErr[%d] %s\n", (int)i, ERR_error_string (error, NULL));
        }
    }
}

static int fe_ssl_ctx_pem_password_cb(char *buf, int size, int rwflag, void *userdata)
{
    int len = 0;

    if (buf && userdata) {
        memset(buf, 0, size);
        strncpy(buf, userdata, size - 1);
		buf[size-1] = 0;
        len = strlen(buf);
    }

    return(len);
}

static DH * fe_ssl_tmp_dh_callback(SSL *ssl, int is_export, int keylength)
{
    return g_fe_ssl_dh_parms;
}

static int fe_ssl_x509_verify_callback(int ok, X509_STORE_CTX *ctx)
{
    if (!ok) {
        char *str = (char *)X509_verify_cert_error_string(ctx->error);
        fprintf(stderr, "%s: Peer certificate failed CRL verification: %s\n", __func__, str);
    }

    return ok;
}

static Status_t fe_ssl_post_connection_cert_check(X509 *cert)
{
    X509_STORE_CTX *x509StoreContext;
#ifdef FE_SSL_VALIDATE_CERT_DNS_FIELD
    int i, cnt;
    X509_NAME *subjectName;
    char dnsCommonName[DNS_NAME_MAX_LENGTH];
#endif

    if (!cert)
        return VSTATUS_BAD;

    if (g_fe_ssl_x509_store_inited) {
        // create the X509 store context 
        if (!(x509StoreContext = X509_STORE_CTX_new())) {
            fprintf(stderr, "%s: Failed to allocate x509 store context\n", __func__);
            return VSTATUS_NOMEM;
        }

        // initialize the X509 store context with the global X509 store and certificate
        // of the peer.
        if (X509_STORE_CTX_init(x509StoreContext, g_fe_ssl_x509_store, cert, NULL) != 1) {
            fprintf(stderr, "%s: Failed to initialize x509 store context\n", __func__);
            return VSTATUS_BAD;
        }

        // validate the certificate of the peer against the CRLs in the global X509 store
        if (X509_verify_cert(x509StoreContext) != 1)
            return VSTATUS_MISMATCH;

        X509_STORE_CTX_cleanup(x509StoreContext);
    }


#ifdef FE_SSL_VALIDATE_CERT_DNS_FIELD
    // check for the fully qualified domain name (DNS name) specified in
    // the certificate. Fist check the dNSName field of the subjectAltName
    // extension.
    if ((cnt = X509_get_ext_count(cert)) > 0) {
        for (i = 0; i < cnt; i++) {
            char *extnObjStr;
            X509_EXTENSION *extn;

            extn = X509_get_ext(cert, i);
            extnObjStr = (char *)OBJ_nid2sn(OBJ_obj2nid(X509_EXTENSION_get_object(extn)));
            if (strcmp(extnObjStr, "subjectAltName") == 0) {
                int field;
                const unsigned char *extnData;
                STACK_OF(CONF_VALUE) *skVal;
                CONF_VALUE *confVal;
                X509V3_EXT_METHOD *extnMeth;

                //
                // found the subjectAltName field, now look for the dNSName field
                if (!(extnMeth = (X509V3_EXT_METHOD *)X509V3_EXT_get(extn)))
                    break;
                extnData = extn->value->data;
                skVal = extnMeth->i2v(extnMeth, extnMeth->d2i(NULL, (const unsigned char **)&extnData, extn->value->length), NULL);

                for (field = 0; field < sk_CONF_VALUE_num(skVal); field++) {
                    confVal = sk_CONF_VALUE_value(skVal, field);
                    if (strcmp(confVal->name, "DNS") == 0) {
                        // validate the dNSName field against the DNS name assigned to STL FM.

                        // FIXME: cjking - temporary patch DNS name should be a configuration parameter.
                        return (strcmp(confVal->value, FE_SSL_FM_DNS_NAME)) ? VSTATUS_MISMATCH : VSTATUS_OK;
                        break;
                    }
                }
            }
        }
    }

    // 
    // if the DNS name is found in the subjectAltName extension, then check the
    // commonName field (this is a legacy certificate field that is deprecated
    // in the X509 standard).
    memset(dnsCommonName, 0, sizeof(dnsCommonName));
    subjectName = X509_get_subject_name(cert);
    if (subjectName && X509_NAME_get_text_by_NID(subjectName, NID_commonName, dnsCommonName, DNS_NAME_MAX_LENGTH) > 0) {
        // validate the commonName field against the DNS name assigned to STL FM.

        // FIXME: cjking - temporary patch DNS name should be a configuration parameter.
        return (strcmp(dnsCommonName, FE_SSL_FM_DNS_NAME)) ? VSTATUS_MISMATCH : VSTATUS_OK;
    } else {
        if (g_verbose) fprintf(stderr, "%s: Error, no subject in the peer's certificate\n", __func__);
    }

    return VSTATUS_NOT_FOUND;
#else
    return VSTATUS_OK;
#endif
}

Status_t fe_ssl_init(void) 
{
    Status_t status = VSTATUS_OK;
    
    if (!g_fe_ssl_inited) {
        g_fe_ssl_inited = 1;
        // initialize the OpenSSL library and error strings.   
        SSL_library_init(); 
        SSL_load_error_strings();

        // allocate the method that will be used to create a SSL/TLS connection.
        // The method defines which protocol version to be used by the connection.
        g_fe_ssl_client_method = (g_feESM) ? (SSL_METHOD *)TLSv1_client_method() : 
                                             (SSL_METHOD *)TLSv1_2_client_method(); 
        if (!g_fe_ssl_client_method) {
            g_fe_ssl_inited = 0;
            status = VSTATUS_NOMEM;
            fprintf(stderr, "%s: Error, failed to allocate SSL method\n", __func__);
        }
    }
     
    return status;
}

void* fe_ssl_client_open(const char *ffDir, 
                         const char *ffCertificate, 
                         const char *ffPrivateKey, 
                         const char *ffCaCertificate, 
                         unsigned int ffCertChainDepth, 
                         const char *ffDHParameters, 
                         unsigned int ffCaCrlEnabled,
                         const char *ffCaCrl)
{
    SSL_CTX *context = NULL;
    X509_LOOKUP *x509StoreLookup;
    EC_KEY *ecdhPrime256v1 = NULL; 
    char fileName[100], ffCertificateFn[100], ffCaCertificateFn[100];
    const char *bogusPassword = "boguspswd"; 
    
    if (!g_fe_ssl_inited) {
        fprintf(stderr, "%s: Error, TLS/SSL is not initialized\n", __func__);
        return (void *)context; 
    }

    if (!ffDir || !ffCertificate || !ffPrivateKey || !ffCaCertificate || !ffCaCrl) {
        fprintf(stderr, "%s: Error, invalid parameter\n", __func__);
        return (void *)context; 
    }

    if (!FE_SSL_IS_VALID_FN(ffDir, ffCertificate, fileName)   ||
        !FE_SSL_IS_VALID_FN(ffDir, ffPrivateKey, fileName)    ||
        !FE_SSL_IS_VALID_FN(ffDir, ffCaCertificate, fileName) ||
        !FE_SSL_IS_VALID_FN(ffDir, ffCaCrl, fileName)) {
        fprintf(stderr, "%s: Error, invalid file name\n", __func__);
        return (void *)context; 
    }

    // allocate a new SSL/TLS connection context
    context = SSL_CTX_new(g_fe_ssl_client_method); 
    if (!context) {
        fprintf(stderr, "%s: Error, failed to allocate SSL context\n", __func__);
        return (void *)context; 
    }
    
    // Usage of the passphrase would require the admin to manual enter
    // a password each time a FF utility is ran, so set the context
    // callback to override the passphrases prompt stage of SSL/TLS by
    // specifying a bogus password.
    SSL_CTX_set_default_passwd_cb(context, fe_ssl_ctx_pem_password_cb); 
    SSL_CTX_set_default_passwd_cb_userdata(context, (void *)bogusPassword); 

    // set the location and name of certificate file to be used by the context.     
    FE_SSL_GET_FN(ffDir, ffCertificate, ffCertificateFn);
    if (SSL_CTX_use_certificate_file(context, ffCertificateFn, SSL_FILETYPE_PEM) <= 0) {
        fprintf(stderr, "%s: Error, failed to set up the certificate file: %s\n", __func__, ffCertificateFn);
        goto bail; 
    }

    // set the location and name of the private key file to be used by the context.
    FE_SSL_GET_FN(ffDir, ffPrivateKey, fileName);
    if (SSL_CTX_use_PrivateKey_file(context, fileName, SSL_FILETYPE_PEM) <= 0) {
        fprintf(stderr, "%s: Error, failed to set up the private key file: %s\n", __func__, fileName);
        (void)fe_ssl_print_error_stack();
        goto bail; 
    }

    // check the consistency of the private key with the certificate
    if (SSL_CTX_check_private_key(context) != 1) {
        fprintf(stderr, "%s: Error, private key and certificate do not match\n", __func__);
        goto bail; 
    }

    // set the location and name of the trusted CA certificates file to be used
    // by the context.
    FE_SSL_GET_FN(ffDir, ffCaCertificate, ffCaCertificateFn);
    if (!SSL_CTX_load_verify_locations(context, ffCaCertificateFn, NULL)) {
        fprintf(stderr, "%s: Error, failed to set up the CA certificates file: %s\n", __func__, ffCaCertificateFn);
        goto bail; 
    }

    // Certificate Revocation Lists (CRL).  CRLs are an addtional mechanism
    // used to verify certificates, in order to ensure that they have not
    // expired or been revoked.
    if (ffCaCrlEnabled && !g_fe_ssl_x509_store_inited) {
        if (!(g_fe_ssl_x509_store = X509_STORE_new())) {
            fprintf(stderr, "%s: Failed to allocate X509 store for CRLs\n", __func__);
            goto bail; 
        }

        X509_STORE_set_verify_cb_func(g_fe_ssl_x509_store, fe_ssl_x509_verify_callback);

        /*--if (X509_STORE_load_locations(g_fe_ssl_x509_store, ffCertificateFn, ffDir) != 1) {
            fprintf(stderr, "%s: Failed to load FF certificate file into X509 store", __func__);
            goto bail; 
        }--*/
        if (X509_STORE_load_locations(g_fe_ssl_x509_store, ffCaCertificateFn, ffDir) != 1) {
            fprintf(stderr, "%s: Failed to load FF CA certificate file into X509 store\n", __func__);
            goto bail; 
        }

        if (X509_STORE_set_default_paths(g_fe_ssl_x509_store) != 1) {
            fprintf(stderr, "%s: Failed to set the default paths for X509 store\n", __func__);
            goto bail; 
        }

        if (!(x509StoreLookup = X509_STORE_add_lookup(g_fe_ssl_x509_store, X509_LOOKUP_file()))) {
            fprintf(stderr, "%s: Failed add file lookup object to X509 store\n", __func__);
            goto bail; 
        }

        FE_SSL_GET_FN(ffDir, ffCaCrl, fileName);
        if (X509_load_crl_file(x509StoreLookup, fileName, X509_FILETYPE_PEM) != 1) {
            fprintf(stderr, "%s: Failed to load the FF CRL file into X509 store: file %s\n", __func__, fileName);
            goto bail; 
        }

        X509_STORE_set_flags(g_fe_ssl_x509_store, X509_V_FLAG_CRL_CHECK | X509_V_FLAG_CRL_CHECK_ALL);

        if (!(x509StoreLookup = X509_STORE_add_lookup(g_fe_ssl_x509_store, X509_LOOKUP_hash_dir()))) {
            fprintf(stderr, "%s: Failed add hash directory lookup object to X509 store\n", __func__);
            goto bail; 
        }

        if (!X509_LOOKUP_add_dir(x509StoreLookup, ffDir, X509_FILETYPE_PEM)) {
            fprintf(stderr, "%s: Failed add FF CRL directory to X509 store\n", __func__);
            goto bail; 
        }

        g_fe_ssl_x509_store_inited = 1;
    }
    
    // set options to be enforced upon the SSL/TLS connection. 		
    SSL_CTX_set_verify(context, SSL_VERIFY_PEER, NULL); 
    SSL_CTX_set_verify_depth(context, ffCertChainDepth);
    SSL_CTX_set_options(context,FE_SSL_OPTIONS);

    // initialize Diffie-Hillmen parameters
    if (!g_fe_ssl_dh_params_inited) {
        FILE *dhParmsFile;

        // set the location and name of the Diffie-Hillmen parameters file to be used
        // by the context.
        FE_SSL_GET_FN(ffDir, ffDHParameters, fileName);

        dhParmsFile = fopen(fileName, "r");
        if (!dhParmsFile) {
            fprintf(stderr, "%s: failed to open Diffie-Hillmen parameters PEM file: %s\n", __func__, fileName);
            goto bail; 
        } else {
            g_fe_ssl_dh_parms = PEM_read_DHparams(dhParmsFile, NULL, NULL, NULL);
			fclose(dhParmsFile);
            if (!g_fe_ssl_dh_parms) {
                fprintf(stderr, "%s: failed to read Diffie-Hillmen parameters PEM file: %s\n", __func__, fileName);
                goto bail; 
            } else {
                g_fe_ssl_dh_params_inited = 1;
                // set DH callback 
                SSL_CTX_set_tmp_dh_callback(context, fe_ssl_tmp_dh_callback);
            }
        }
    }

    // set up the cipher list with the cipher suites that are allowed
    // to be used in establishing the connection.
    if (!SSL_CTX_set_cipher_list(context, FE_SSL_CIPHER_LIST)) {
        fprintf(stderr, "%s: Error, failed to set up the cipher list\n", __func__);
        goto bail; 
    }

    // set up the elliptic curve to be used in establishing the connection.
    ecdhPrime256v1 = EC_KEY_new_by_curve_name(NID_X9_62_prime256v1); 
    if (ecdhPrime256v1 == NULL) {
        fprintf(stderr, "%s: Error, Failed to to create curve: prime256v1\n", __func__);
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

void * fe_ssl_connect(void *context, int serverfd) 
{ 
    Status_t status = VSTATUS_OK;
    int rc; 
    SSL *session = NULL; 
    X509 *cert = NULL; 
    
    if (!context) {
        fprintf(stderr, "%s: Error, invalid context parameter\n", __func__);
        return (void *)session; 
    }

    // allocate a new session to be associated with the SSL/TLS connection
    if (!(session = SSL_new((SSL_CTX *)context))) {
        fprintf(stderr, "%s: Error, Failed to allocate new SSL/TLS session for socket fd %d\n", __func__, serverfd);
        return (void *)session; 
    }

    // assign peer file descriptor to the SSL/TLS session 
    SSL_set_fd(session, serverfd);
    if (g_verbose)
        (void)fe_ssl_print_ciphers(session);

    // attempt to establish SSL/TLS connection to the server  
    if ((rc = SSL_connect(session)) != 1) {
        fprintf(stderr, "%s: Error, SSL/TLS handshake failed for connect: err %d, rc %d, %s\n", __func__, SSL_get_error(session, rc), rc, strerror(errno));
        (void)fe_ssl_print_error_stack();
    } else {
        if (g_verbose)
            fprintf(stderr, "%s: ACTIVE cipher suite: %s\n", __func__, SSL_CIPHER_get_name(SSL_get_current_cipher(session)));

        // request and verify the certificate of the server.  Typically, it is
        // required for a server to present a certificate.  Strict security
        // enforcement is to be done for the STL FM, so if a certificate
        // is not presented by the server the connection will be rejected. 
        if (!(cert = SSL_get_peer_certificate(session))) {
            status = VSTATUS_NOT_FOUND;
            fprintf(stderr, "%s: Error, server has no certifcate to verfiy\n", __func__);
        } else {
            long result;

            // post-connection verification is very important, because it allows for
            // much finer grained control over the certificate that is presented by
            // the server, beyond the basic certificate verification that is done by
            // the SSL/TLS protocol proper during the connect operation.   
            if (!(status = fe_ssl_post_connection_cert_check(cert))) {
                // do the final phase of post-connection verification of the
                // certificate presented by the server.
                result = SSL_get_verify_result(session); 
                if (result != X509_V_OK) {
                    status = VSTATUS_MISMATCH;
                    fprintf(stderr, "%s: Error, verification of server certificate failed: err %ld\n", __func__, result);
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

int fe_ssl_read(void *session, unsigned char *buffer, int bufferLength) 
{ 
    int erc, bytesRead = -1;
 
    if (buffer) {
        bytesRead = SSL_read((SSL *)session, (void *)buffer, bufferLength); 
        if (g_verbose) fprintf(stderr, "%s: Received %d bytes from SSL/TLS server\n", __func__, bytesRead);

        // FIXME: cjking - Implement retry logic code for certain return error conditions
        erc = SSL_get_error((SSL *)session, bytesRead);
        switch (erc) {
        case SSL_ERROR_NONE:
            break;
        case SSL_ERROR_ZERO_RETURN:
            fprintf(stderr, "%s: Warning, SSL read failed with SSL_ERROR_ZERO_RETURN error, retrying read\n", __func__);
            break;
        case SSL_ERROR_WANT_READ:
            fprintf(stderr, "%s: Warning, SSL read failed with SSL_ERROR_WANT_READ error, retrying read\n", __func__);
            break;
        case SSL_ERROR_WANT_WRITE:
            fprintf(stderr, "%s: Warning, SSL read failed with SSL_ERROR_WANT_WRITE error, retrying read\n", __func__);
            break;
        default:
            fprintf(stderr, "%s: Error, SSL read failed with rc %d: %s\n", __func__, erc, strerror(errno));
            break;
        }
    }

    return bytesRead;
}

int fe_ssl_write(void *session, unsigned char *buffer, int bufferLength) 
{
    int erc, bytesWritten = -1;
     
    if (buffer) {
        bytesWritten = SSL_write((SSL *)session, (void *)buffer, bufferLength); 
        if (g_verbose) fprintf(stderr, "%s: Sent %d bytes to SSL/TLS server\n", __func__, bytesWritten);

        // FIXME: cjking - Implement retry logic code for certain return error conditions
        erc = SSL_get_error((SSL *)session, bytesWritten);
        switch (erc) {
        case SSL_ERROR_NONE:
            break;
        case SSL_ERROR_ZERO_RETURN:
            fprintf(stderr, "%s: Warning, SSL read failed with SSL_ERROR_ZERO_RETURN error, retrying read\n", __func__);
            break;
        case SSL_ERROR_WANT_READ:
            fprintf(stderr, "%s: Warning, SSL read failed with SSL_ERROR_WANT_READ error, retrying read\n", __func__);
            break;
        case SSL_ERROR_WANT_WRITE:
            fprintf(stderr, "%s: Warning, SSL read failed with SSL_ERROR_WANT_WRITE error, retrying read\n", __func__);
            break;
        default:
            //IB_LOG_ERROR_FMT(__func__, "SSL read failed with rc %d: %s", erc, strerror(errno));
            fprintf(stderr, "%s: Error, SSL read failed with rc %d: %s\n", __func__, erc, strerror(errno));
            break;
        }
    }

    return bytesWritten;
}

void fe_ssl_conn_close(void *context) 
{
    if (context) {
        SSL_CTX_free((SSL_CTX *)context);
        context = NULL; 
    }
}

void fe_ssl_sess_close(void *session) 
{
    if (session) {
        SSL_free((SSL *)session);
        session = NULL; 
    }
}
   
