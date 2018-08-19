#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../keccak/sha3.h"

#include "contiki.h"
#include "sys/cc.h"

/* ----------------------------------------------------------------------- */
/* -                      DEFINES AND MACROS                             - */
/* ----------------------------------------------------------------------- */

// no catching supported, just abort while printing the code name to stderr
#define THROW(x) \
        ({ printf("EXCEPTION: " #x "\n"); })

#define UNUSED(x) (void)(x)

#ifndef MIN
#define MIN(a,b) \
        ({ __typeof__ (a)_a = (a); \
           __typeof__ (b)_b = (b); \
           _a < _b ? _a : _b; })
#define MAX(a,b) \
        ({ __typeof__ (a)_a = (a); \
           __typeof__ (b)_b = (b); \
           _a > _b ? _a : _b; })
#endif // MIN not defined

/* ----------------------------------------------------------------------- */
/* -                         OS ALTERNATIVES                             - */
/* ----------------------------------------------------------------------- */

#define os_swap_u32 __builtin_bswap32

#define os_memmove memmove
#define os_memcpy memcpy

#define os_memset memset

/* ----------------------------------------------------------------------- */
/* -                          CRYPTO FUNCTIONS                           - */
/* ----------------------------------------------------------------------- */

#define CX_LAST (1 << 0)

typedef SHA3_CTX cx_sha3_t;
typedef SHA3_CTX cx_hash_t;

static inline void cx_keccak_init(SHA3_CTX* hash, int size) {
        UNUSED(size);

        keccak_384_Init(hash);
}

static inline void cx_hash(SHA3_CTX* hash, int mode, const unsigned char *in,
                           unsigned int len, unsigned char *out) {

        if (mode != CX_LAST) {
                // if CX_LAST is not set, add input data to add to current hash
                keccak_Update(hash, in, len);
        } else if (len == 0) {
                // if no input data given, compute and copy the hash
                keccak_Final(hash, out);
        } else {
                // if CX_LAST is set, compute hash for input
                keccak_Update(hash, in, len);
                keccak_Final(hash, out);
        }
}

/* ----------------------------------------------------------------------- */
/* -                                 IO                                  - */
/* ----------------------------------------------------------------------- */

#define IO_ASYNCH_REPLY 1

/* ----------------------------------------------------------------------- */
/* -                            COMMON                                   - */
/* ----------------------------------------------------------------------- */

#define CX_KECCAK384_SIZE 48

#define CEILING(x,y) (((x) + (y) - 1) / (y))

#if 0
#define ASSIGN(dest, src)                                                      \
        ({                                                                     \
                typeof(src)_x = (src);                                         \
                typeof(dest)_y = _x;                                           \
                (_x == _y && ((_x < 1) == (_y < 1)) ? (void)((dest) = _y),     \
                 1 : 0);                                                       \
        })

#define IN_RANGE(x, min, max)                                                  \
        ({                                                                     \
                typeof(x)_x = (x);                                             \
                (_x >= (min) && _x <= (max));                                  \
        })
#else

#define IN_RANGE(x, min, max)                                           \
  ((x) >= (min) && (x) <= (max))                                       \

#endif

cx_sha3_t *get_sha(unsigned index);

#endif // COMMON_H
