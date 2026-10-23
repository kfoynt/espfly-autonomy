#pragma once
#include <CommonCrypto/CommonDigest.h>
static inline int mbedtls_sha256(const unsigned char *in, size_t n, unsigned char *out, int is224) { (void)is224; CC_SHA256(in, (CC_LONG)n, out); return 0; }
