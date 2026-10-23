#pragma once
#include <CommonCrypto/CommonHMAC.h>
typedef struct { CCHmacContext c; int setup; } mbedtls_md_context_t;
typedef struct { int dummy; } mbedtls_md_info_t;
typedef enum { MBEDTLS_MD_SHA256 = 9 } mbedtls_md_type_t;
static const mbedtls_md_info_t stubInfo;
static inline const mbedtls_md_info_t *mbedtls_md_info_from_type(mbedtls_md_type_t t) { (void)t; return &stubInfo; }
static inline void mbedtls_md_init(mbedtls_md_context_t *c) { c->setup = 0; }
static inline int mbedtls_md_setup(mbedtls_md_context_t *c, const mbedtls_md_info_t *i, int h) { (void)i; (void)h; c->setup = 1; return 0; }
static inline void mbedtls_md_free(mbedtls_md_context_t *c) { c->setup = 0; }
static inline int mbedtls_md_hmac_starts(mbedtls_md_context_t *c, const unsigned char *k, size_t n) { CCHmacInit(&c->c, kCCHmacAlgSHA256, k, n); return 0; }
static inline int mbedtls_md_hmac_update(mbedtls_md_context_t *c, const unsigned char *d, size_t n) { CCHmacUpdate(&c->c, d, n); return 0; }
static inline int mbedtls_md_hmac_finish(mbedtls_md_context_t *c, unsigned char *o) { CCHmacFinal(&c->c, o); return 0; }
