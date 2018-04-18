/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

#include <stdio.h>
#include <stdlib.h>

#include <openssl/rand.h> 
#include <openssl/ssl.h> 
#include <openssl/err.h> 
#include <openssl/conf.h>
#include <openssl/x509v3.h>
#include <openssl/x509_vfy.h>
#include <openssl/pem.h>

#include <cs_g.h>
#include <cs_log.h>

#include "if3_ssl.h"

#define DNS_NAME_MAX_LENGTH     256

#ifdef IF3_SSL_VALIDATE_CERT_DNS_FIELD
#define IF3_SSL_FM_DNS_NAME     "<sever-name>.com"
#endif

#ifdef __VXWORKS__
// The vxWorks version used by the STL ESM does not support the
// ECDHE-ECDSA-AES128-GCM-SHA256 cipher; the strongest cipher
// supported is DHE-DSS-AES256-SHA 
#define IF3_SSL_CIPHER_LIST     "DHE-DSS-AES256-SHA"
// vxWorks has a bug with the SSL_VERIFY_PEER options, which requests and
// verifies the certificate of the peer multiple times during the SSL/TLS
// handshake.  The SSL_VERIFY_CLIENT_ONCE options does this only once.
#define IF3_SSL_SSL_VERIFY      SSL_VERIFY_CLIENT_ONCE
#else
// ciphers to support for the HSM, ESM, and FV
// HSM - ECDHE-ECDSA-AES128-GCM-SHA256
// ESM - DHE-DSS-AES256-SHA
// FV -  ECDHE-ECDSA-AES128-GCM-SHA256 or ECDHE-ECDSA-AES128-SHA256 
#define IF3_SSL_CIPHER_LIST     "ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES128-SHA256:DHE-DSS-AES256-SHA"
#define IF3_SSL_SSL_VERIFY      SSL_VERIFY_PEER
#endif

// Include all workaround patches. Exclude versions SSLv2, SSLv3, and lower.
// Include Diffie-Hellman protocol for Ephemeral Keying, in order to take
// advantage of Forward Secrecy property of TLS
#define IF3_SSL_OPTIONS      SSL_OP_ALL | SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3 | SSL_OP_SINGLE_DH_USE

#define IF3_SSL_IS_VALID_FN(d, f, b)   ((strlen(d) + strlen(f) + 1) < (sizeof(b) - 1))
#define IF3_SSL_GET_FN(d, f, b) { \
    if (d[strlen(d)] == '/') \
        snprintf(b, sizeof(b), "%s%s", d, f); \
    else \
        snprintf(b, sizeof(b), "%s/%s", d, f); \
}

static int g_if3_ssl_inited = 0;
static Pool_t * g_if3_ssl_thread_pool = NULL;
static SSL_METHOD * g_if3_ssl_server_method = NULL; 

static int g_if3_ssl_dh_params_inited = 0;
static DH *g_if3_ssl_dh_parms = NULL;

static int g_if3_ssl_x509_store_inited=0;
static X509_STORE *g_if3_ssl_x509_store = NULL;

#ifdef __VXWORKS__
static Lock_t * g_if3_thread_mutexs = NULL;
#else
static int g_if3_ssl_ec_inited = 0;
static pthread_mutex_t * g_if3_thread_mutexs = NULL;
#endif

void if3_ssl_print_ciphers(SSL *ssl)
{
  int index = 0;
  const char *next = NULL;
  do {
    next = SSL_get_cipher_list(ssl,index);
    if (next != NULL) {
      IB_LOG_INFINI_INFO_FMT(__func__, "CIPHER[%d] %s", index, next);
      index++;
    }
  }
  while (next != NULL);
}

static void if3_ssl_print_error_stack(void)
{
    unsigned long error, i;

    for (i = 0; i < 5; i++) {
        if ((error = ERR_get_error())) {
            IB_LOG_ERROR_FMT(__func__, "StackErr[%d] %s", (int)i, ERR_error_string (error, NULL));
        }
    }
}

static int if3_ssl_ctx_pem_password_cb(char *buf, int size, int rwflag, void *userdata)
{
    int len = 0;

    if (buf && userdata) {
        strncpy(buf, userdata, size - 1);
        buf[size-1] = 0;
        len = strlen(buf);
    }

    return(len);
}

#if defined(__LINUX__) || (defined(__VXWORKS__) && !defined(OPENSSL_NO_ENGINE))
static unsigned long if3_ssl_id_callback(void)
{
#ifdef __LINUX__
    return((unsigned long) pthread_self());
#else
    return((unsigned long)taskIdSelf());
#endif
}
#endif

static void if3_ssl_locking_callback(int mode, int type, const char *file, int line)
{ 
#ifdef __LINUX__
    if (mode & CRYPTO_LOCK) {
        pthread_mutex_lock(&g_if3_thread_mutexs[type]);
    } else {
        pthread_mutex_unlock(&g_if3_thread_mutexs[type]);
    }
#else
    if (mode & CRYPTO_LOCK) {
        vs_lock(&g_if3_thread_mutexs[type]);
    } else {
        vs_unlock(&g_if3_thread_mutexs[type]);
    }
#endif
}

static Status_t if3_ssl_thread_setup(Pool_t *pool)
{
    Status_t status = VSTATUS_OK; 
    int rc, mutex, mutexsLength;
    int totalMutexs = CRYPTO_num_locks();     

    // allocate and initialize the mutex table required by OpenSSL to support
    // a multi-threaded application.
    if (!pool) {
        return VSTATUS_NOHANDLE;
    } else if (totalMutexs <= 0)
        return status;

    // retain pool pointer for later use
    g_if3_ssl_thread_pool = pool;

#ifdef __LINUX__
    mutexsLength = totalMutexs * sizeof(pthread_mutex_t);
#else
    mutexsLength = totalMutexs * sizeof(Lock_t);
#endif

    rc = vs_pool_alloc(g_if3_ssl_thread_pool, mutexsLength, (void *)&g_if3_thread_mutexs); 
    if (rc != VSTATUS_OK) {
        status = VSTATUS_NOMEM; 
        IB_LOG_ERROR_FMT(__func__, "Failed to allocate SSL/TLS mutex table: rc %d", rc);
    } else {
        // initialize the mutexs
        for (mutex = 0; mutex < totalMutexs; mutex++) {
#ifdef __LINUX__
            pthread_mutex_init(&g_if3_thread_mutexs[mutex], NULL);
#else
            status = vs_lock_init(&g_if3_thread_mutexs[mutex], VLOCK_FREE, VLOCK_THREAD);
            if (status != VSTATUS_OK) {
                IB_LOG_ERROR_FMT(__func__, "Failed to initialize SSL/TLS mutex table");
                // deallocate the mutexs table
                vs_pool_free(g_if3_ssl_thread_pool, (void *)g_if3_thread_mutexs);
                g_if3_thread_mutexs = NULL;
                break;
            }
#endif
        }

        // set the callback routines that are used by OpenSSL to interface with
        // the mutexs
        if (!status) {
#if defined(__LINUX__) || (defined(__VXWORKS__) && !defined(OPENSSL_NO_ENGINE))
            CRYPTO_set_id_callback(if3_ssl_id_callback);
#endif
            CRYPTO_set_locking_callback(if3_ssl_locking_callback);
        }
    }
    
    return status;
}
       
static void if3_ssl_seed_prng_init(void)
{
    // FIXME: cjking - Implement this routine that will be responsible for
    // initalizing the Pseudorandom Number Generator (PRNG).
}

static DH * if3_ssl_tmp_dh_callback(SSL *ssl, int is_export, int keylength)
{
    return g_if3_ssl_dh_parms;
}

static int if3_ssl_x509_verify_callback(int ok, X509_STORE_CTX *ctx)
{
    if (!ok) {
        int err = X509_STORE_CTX_get_error(ctx);
        char *str = (char *)X509_verify_cert_error_string(err);
        IB_LOG_ERROR_FMT(__func__, "Peer certificate failed CRL verification: %s", str);
    }

    return ok;
}

static Status_t if3_ssl_post_connection_cert_check(X509 *cert)
{
    X509_STORE_CTX *x509StoreContext;
#ifdef IF3_SSL_VALIDATE_CERT_DNS_FIELD
    int i, cnt;
    X509_NAME *subjectName;
    char dnsCommonName[DNS_NAME_MAX_LENGTH];
#endif

    if (!cert)
        return VSTATUS_BAD;


    if (g_if3_ssl_x509_store_inited) {
        // create the X509 store context 
        if (!(x509StoreContext = X509_STORE_CTX_new())) {
            IB_LOG_ERROR_FMT(__func__, "Failed to allocate x509 store context");
            return VSTATUS_NOMEM;
        }

        // initialize the X509 store context with the global X509 store and certificate
        // of the peer.
        if (X509_STORE_CTX_init(x509StoreContext, g_if3_ssl_x509_store, cert, NULL) != 1) {
            IB_LOG_ERROR_FMT(__func__, "Failed to initialize x509 store context");
            return VSTATUS_BAD;
        }

        // validate the certificate of the peer against the CRLs in the global X509 store
        if (X509_verify_cert(x509StoreContext) != 1)
            return VSTATUS_MISMATCH;

        X509_STORE_CTX_cleanup(x509StoreContext);
    }

#ifdef IF3_SSL_VALIDATE_CERT_DNS_FIELD
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
                unsigned char *extnData;
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
                        return (strcmp(confVal->value, IF3_SSL_FM_DNS_NAME)) ? VSTATUS_MISMATCH : VSTATUS_OK;
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
        return (strcmp(dnsCommonName, IF3_SSL_FM_DNS_NAME)) ? VSTATUS_MISMATCH : VSTATUS_OK;
    } else {
        IB_LOG_INFINI_INFO_FMT(__func__, "Warning: No subject in the peer's certificate");
    }

    return VSTATUS_NOT_FOUND;
#else
    return VSTATUS_OK;
#endif
}

void if3_ssl_thread_cleanup(void)
{
    int mutex, totalMutexs = CRYPTO_num_locks();
         
    if (g_if3_ssl_thread_pool) {
        // reset the callback routines that are used by OpenSSL to interface with
        // the mutexs
#if defined(__LINUX__) || (defined(__VXWORKS__) && !defined(OPENSSL_NO_ENGINE))
        CRYPTO_set_id_callback(NULL);
#endif
        CRYPTO_set_locking_callback(NULL);

        if (g_if3_thread_mutexs) {
            // deallocating the mutexs
            for (mutex = 0; mutex < totalMutexs; mutex++) {
#ifdef __LINUX__
                pthread_mutex_destroy(&g_if3_thread_mutexs[mutex]);
#else
                vs_lock_delete(&g_if3_thread_mutexs[mutex]);
#endif
            }
        }
        // deallocate the mutexs table
        vs_pool_free(g_if3_ssl_thread_pool, (void *)g_if3_thread_mutexs);
        g_if3_thread_mutexs = NULL;
        g_if3_ssl_inited = 0;
    }
}

Status_t if3_ssl_init(Pool_t *pool) 
{
    Status_t status = VSTATUS_OK;
    
    // FIXME: cjking - Determine whether a mutex semiphore is needed.
    // OpenSSL already provides mutex handling for multi-threading
    // applications.
    if (!g_if3_ssl_inited) {
        g_if3_ssl_inited = 1;
        // initialize the OpenSSL library and error strings.   
        SSL_library_init(); 
        SSL_load_error_strings();
#if !defined(__VXWORKS__)
        OpenSSL_add_all_algorithms();
#endif
        // initialize the Pseudorandom Number Generator(PRNG).  OpenSSL uses
        // PRNGs to for many functions (i.e., creating sessions keys, generating
        // public/private key pairs, etc.).
        if3_ssl_seed_prng_init();

        // initialize the data structures required by OpenSSL to support
        // a multi-threaded application.
        if (pool)
            status = if3_ssl_thread_setup(pool);

        if (!status) {
            // allocate the method that will be used to create a SSL/TLS connection.
            // The method defines which protocol version to be used by the connection.
#if defined(__VXWORKS__)
            g_if3_ssl_server_method = (SSL_METHOD *)TLSv1_server_method();
#else
            g_if3_ssl_server_method = (SSL_METHOD *)TLSv1_2_server_method();
#endif
            if (!g_if3_ssl_server_method) {
                g_if3_ssl_inited = 0;
                status = VSTATUS_NOMEM;
                IB_LOG_ERROR_FMT(__func__, "failed to allocate SSL method");
            }
        }
    }
     
    return status;
}

void* if3_ssl_srvr_open(const char *fmDir, 
                        const char *fmCertificate, 
                        const char *fmPrivateKey, 
                        const char *fmCaCertificate, 
                        uint32_t fmCertChainDepth, 
                        const char *fmDHParameters,
                        uint32_t fmCaCrlEnabled,
                        const char *fmCaCrl)
{
    SSL_CTX *context = NULL;
#ifndef __VXWORKS__
    EC_KEY *ecdhPrime256v1 = NULL;
#endif
    X509_LOOKUP *x509StoreLookup;
    char fileName[100], fmCertificateFn[100], fmCaCertificateFn[100];
    const char *bogusPassword = "boguspswd";
    
    if (!g_if3_ssl_inited) {
        IB_LOG_ERROR_FMT(__func__, "TLS/SSL is not initialized");
        return (void *)context; 
    }

    if (!fmDir || !fmCertificate || !fmPrivateKey || !fmCaCertificate || !fmCaCrl) {
        IB_LOG_ERROR_FMT(__func__, "Invalid parameter");
        return (void *)context; 
    }

    if (!IF3_SSL_IS_VALID_FN(fmDir, fmCertificate, fileName)   ||
        !IF3_SSL_IS_VALID_FN(fmDir, fmPrivateKey, fileName)    ||
        !IF3_SSL_IS_VALID_FN(fmDir, fmCaCertificate, fileName) ||
        !IF3_SSL_IS_VALID_FN(fmDir, fmCaCrl, fileName)) {
        IB_LOG_ERROR_FMT(__func__, "Invalid file name");
        return (void *)context; 
    }

    // allocate a new SSL/TLS connection context
    context = SSL_CTX_new(g_if3_ssl_server_method); 
    if (!context) {
        IB_LOG_ERROR_FMT(__func__, "Failed to allocate SSL context");
        return (void *)context; 
    }
    
    // Usage of the passphrase would require the admin to manual enter
    // a password each time the FM/FE start, so set the context callback to override
    // the passphrases prompt stage of SSL/TLS by specifying a bogus password.
    SSL_CTX_set_default_passwd_cb(context, if3_ssl_ctx_pem_password_cb); 
    SSL_CTX_set_default_passwd_cb_userdata(context, (void *)bogusPassword); 

    // set the location and name of certificate file to be used by the context.     
    IF3_SSL_GET_FN(fmDir, fmCertificate, fmCertificateFn);
    if (SSL_CTX_use_certificate_file(context, fmCertificateFn, SSL_FILETYPE_PEM) <= 0) {
        IB_LOG_ERROR_FMT(__func__, "Failed to set up the certificate file: %s", fmCertificateFn);
        goto bail; 
    }

    // set the location and name of the private key file to be used by the context.
    IF3_SSL_GET_FN(fmDir, fmPrivateKey, fileName);
    if (SSL_CTX_use_PrivateKey_file(context, fileName, SSL_FILETYPE_PEM) <= 0) {
        IB_LOG_ERROR_FMT(__func__, "Failed to set up the private key file: %s", fileName);
        goto bail; 
    }

    // check the consistency of the private key with the certificate
    if (SSL_CTX_check_private_key(context) != 1) {
        IB_LOG_ERROR_FMT(__func__, "Private key and certificate do not match");
        goto bail; 
    }

    // set the location and name of the trusted CA certificates file to be used
    // by the context.
    IF3_SSL_GET_FN(fmDir, fmCaCertificate, fmCaCertificateFn);
    if (!SSL_CTX_load_verify_locations(context, fmCaCertificateFn, NULL)) {
        IB_LOG_ERROR_FMT(__func__, "Failed to set up the CA certificates file: %s", fmCaCertificateFn);
        goto bail; 
    }
    
    // Certificate Revocation Lists (CRL).  CRLs are an addtional mechanism
    // used to verify certificates, in order to ensure that they have not
    // expired or been revoked.
    if (fmCaCrlEnabled && !g_if3_ssl_x509_store_inited) {
        g_if3_ssl_x509_store_inited = 1;
        if (!(g_if3_ssl_x509_store = X509_STORE_new())) {
            IB_LOG_ERROR_FMT(__func__, "Failed to allocate X509 store for CRLs");
            goto bail; 
        }

        X509_STORE_set_verify_cb_func(g_if3_ssl_x509_store, if3_ssl_x509_verify_callback);

        if (X509_STORE_load_locations(g_if3_ssl_x509_store, fmCaCertificateFn, fmDir) != 1) {
            IB_LOG_ERROR_FMT(__func__, "Failed to load FM FI certificate file into X509 store");
            goto bail; 
        }

        if (X509_STORE_set_default_paths(g_if3_ssl_x509_store) != 1) {
            IB_LOG_ERROR_FMT(__func__, "Failed to set the default paths for X509 store");
            goto bail; 
        }

        if (!(x509StoreLookup = X509_STORE_add_lookup(g_if3_ssl_x509_store, X509_LOOKUP_file()))) {
            IB_LOG_ERROR_FMT(__func__, "Failed add file lookup object to X509 store");
            goto bail; 
        }

        IF3_SSL_GET_FN(fmDir, fmCaCrl, fileName);
        if (X509_load_crl_file(x509StoreLookup, fileName, X509_FILETYPE_PEM) != 1) {
            IB_LOG_ERROR_FMT(__func__, "Failed to load the FM CRL file into X509 store: file %s", fileName);
            goto bail; 
        }

        X509_STORE_set_flags(g_if3_ssl_x509_store, X509_V_FLAG_CRL_CHECK | X509_V_FLAG_CRL_CHECK_ALL);

        if (!(x509StoreLookup = X509_STORE_add_lookup(g_if3_ssl_x509_store, X509_LOOKUP_hash_dir()))) {
            IB_LOG_ERROR_FMT(__func__, "Failed add hash directory lookup object to X509 store");
            goto bail; 
        }

        if (!X509_LOOKUP_add_dir(x509StoreLookup, fmDir, X509_FILETYPE_PEM)) {
            IB_LOG_ERROR_FMT(__func__, "Failed add FM CRL directory to X509 store");
            goto bail; 
        }
    }

    // set options to be enforced upon the SSL/TLS connection.
    SSL_CTX_set_verify(context, IF3_SSL_SSL_VERIFY, NULL); 
    SSL_CTX_set_verify_depth(context, fmCertChainDepth);
    SSL_CTX_set_options(context, IF3_SSL_OPTIONS);

    // initialize Diffie-Hillmen parameters
    if (!g_if3_ssl_dh_params_inited) {
        FILE *dhParmsFile;

        // set the location and name of the Diffie-Hillmen parameters file to be used
        // by the context.
        IF3_SSL_GET_FN(fmDir, fmDHParameters, fileName);

        dhParmsFile = fopen(fileName, "r");
        if (!dhParmsFile) {
            IB_LOG_ERROR_FMT(__func__, "failed to open Diffie-Hillmen parameters PEM file: %s", fileName);
            goto bail; 
        } else {
            g_if3_ssl_dh_parms = PEM_read_DHparams(dhParmsFile, NULL, NULL, NULL);
            if (!g_if3_ssl_dh_parms) {
                fclose(dhParmsFile);
                IB_LOG_ERROR_FMT(__func__, "failed to read Diffie-Hillmen parameters PEM file");
                goto bail; 
            } else {
                g_if3_ssl_dh_params_inited = 1;
                fclose(dhParmsFile);
                // set DH callback 
                SSL_CTX_set_tmp_dh_callback(context, if3_ssl_tmp_dh_callback);
            }
        }
    }

    // set up the cipher list with the cipher suites that are allowed
    // to be used in establishing the connection.
    if (!SSL_CTX_set_cipher_list(context, IF3_SSL_CIPHER_LIST)) {
        IB_LOG_ERROR_FMT(__func__, "Failed to set up the cipher list");
        goto bail; 
    }
    
#ifndef __VXWORKS__
    // set up the elliptic curve to be used in establishing the connection.
    if (!g_if3_ssl_ec_inited) {
        ecdhPrime256v1 = EC_KEY_new_by_curve_name(NID_X9_62_prime256v1); 
        if (ecdhPrime256v1 == NULL) {
            IB_LOG_ERROR_FMT(__func__, "Failed to to create curve: prime256v1");
            goto bail; 
        }
        
        g_if3_ssl_ec_inited = 1;        
        SSL_CTX_set_tmp_ecdh(context, ecdhPrime256v1);
        EC_KEY_free(ecdhPrime256v1);
    }
#endif

    return (void *)context;

bail:

    if (context)
        SSL_CTX_free(context);
    if (g_if3_ssl_x509_store)
        X509_STORE_free(g_if3_ssl_x509_store);
     
    return NULL;
}

void * if3_ssl_accept(void *context, int clientfd) 
{ 
    Status_t status = VSTATUS_OK;
    int rc; 
    SSL *session = NULL; 
    X509 *cert = NULL; 
    
    if (!context) {
        IB_LOG_ERROR_FMT(__func__, "Invalid context parameter");
        return (void *)session; 
    }

    // allocate a new session to be associated with the SSL/TLS connection
    if (!(session = SSL_new((SSL_CTX *)context))) {
        IB_LOG_ERROR_FMT(__func__, "Failed to allocate new SSL/TLS session for socket fd %d", clientfd);
        return (void *)session; 
    }

    // assign peer file descriptor to the SSL/TLS session 
    SSL_set_fd(session, clientfd);
    //(void)if3_ssl_print_ciphers(session);

    // wait for the client to initiate the SSL/TLS handshake  
    if ((rc = SSL_accept(session)) != 1) {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, "SSL/TLS handshake failed for accept: err %d, rc %d, %s", SSL_get_error(session, rc), rc, strerror(errno));
        (void)if3_ssl_print_error_stack();
    } else {
        //IB_LOG_INFINI_INFO_FMT(__func__, "ACTIVE cipher suite: %s\n", SSL_CIPHER_get_name(SSL_get_current_cipher(session)));

        // request and verify the certificate of the client.  Typically, it is
        // optional for a client to present a certificate.  If strict security
        // enforcement is to be done for the STL FM, then the client should be
        // required to present a certificate.  For now, strict security
        // enforement is not enforced. 
        if (!(cert = SSL_get_peer_certificate(session))) {
            IB_LOG_INFINI_INFO_FMT(__func__, "Client has no certifcate to verfiy");
        } else {
            long result;

            // post-connection verification is very important, because it allows for
            // much finer grained control over the certificate that is presented by
            // the client, beyond the basic certificate verification that is done by
            // the SSL/TLS protocol proper during the accept operation.   
            if (!(status = if3_ssl_post_connection_cert_check(cert))) {
                // do the final phase of post-connection verification of the
                // certificate presented by the peer.
                result = SSL_get_verify_result(session); 
                if (result != X509_V_OK) {
                    status = VSTATUS_MISMATCH;
                    IB_LOG_ERROR_FMT(__func__, "Verification of client certificate failed: err %ld", result);
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

int if3_ssl_read(void *session, uint8_t *buffer, int bufferLength) 
{ 
    int erc, bytesRead = -1;
 
    if (buffer) {
        bytesRead = SSL_read((SSL *)session, (void *)buffer, bufferLength); 
        //IB_LOG_INFINI_INFO_FMT(__func__, "Received %d bytes from SSL/TLS client", bytesRead);

        // FIXME: cjking - Implement retry logic code for certain return error conditions
        erc = SSL_get_error(session, bytesRead);
        switch (erc) {
        case SSL_ERROR_NONE:
            break;
        case SSL_ERROR_ZERO_RETURN:
            IB_LOG_WARN_FMT(__func__, "SSL read failed with SSL_ERROR_ZERO_RETURN error, retrying read");
            break;
        case SSL_ERROR_WANT_READ:
            IB_LOG_WARN_FMT(__func__, "SSL read failed with SSL_ERROR_WANT_READ error, retrying read");
            break;
        case SSL_ERROR_WANT_WRITE:
            IB_LOG_WARN_FMT(__func__, "SSL read failed with SSL_ERROR_WANT_WRITE error, retrying read");
            break;
        default:
            //IB_LOG_ERROR_FMT(__func__, "SSL read failed with rc %d: %s", erc, strerror(errno));
            break;
        }
    }

    return bytesRead;
}

int if3_ssl_write(void *session, uint8_t *buffer, int bufferLength) 
{
    int erc, bytesWritten = -1;
     
    if (buffer) {
        bytesWritten = SSL_write((SSL *)session, (void *)buffer, bufferLength); 
        //IB_LOG_INFINI_INFO_FMT(__func__, "Sent %d bytes to SSL/TLS client", bytesWritten);

        // FIXME: cjking - Implement retry logic code for certain return error conditions
        erc = SSL_get_error(session, bytesWritten);
        switch (erc) {
        case SSL_ERROR_NONE:
            break;
        case SSL_ERROR_ZERO_RETURN:
            IB_LOG_WARN_FMT(__func__, "SSL write failed with SSL_ERROR_ZERO_RETURN error, retrying read");
            break;
        case SSL_ERROR_WANT_READ:
            IB_LOG_WARN_FMT(__func__, "SSL write failed with SSL_ERROR_WANT_READ error, retrying read");
            break;
        case SSL_ERROR_WANT_WRITE:
            IB_LOG_WARN_FMT(__func__, "SSL write failed with SSL_ERROR_WANT_WRITE error, retrying read");
            break;
        default:
            //IB_LOG_ERROR_FMT(__func__, "SSL write failed with rc %d: %s", erc, strerror(errno));
            break;
        }
    }

    return bytesWritten;
}

void if3_ssl_conn_close(void *context) 
{
    if (context) {
        SSL_CTX_free((SSL_CTX *)context);
        context = NULL; 
    }
}

void if3_ssl_sess_close(void *session) 
{
    if (session) {
        SSL_free((SSL *)session);
        session = NULL; 
    }
}
