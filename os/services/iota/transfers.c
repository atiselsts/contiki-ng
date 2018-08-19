#include "transfers.h"

#include <string.h>
#include <assert.h>
// iota-related stuff
#include "conversion.h"
#include "addresses.h"
#include "bundle.h"
#include "signing.h"
#include "iota/common.h"
#include "iota/kerl.h"

#define ZERO_HASH                                                              \
    "999999999999999999999999999999999999999999999999999999999999999999999999" \
    "999999999"
#define ZERO_TAG "999999999999999999999999999"

typedef struct TX_OBJECT {
//    char signatureMessageFragment[2187];
    char address[81];
    int64_t value;
    char obsoleteTag[27];
    uint32_t timestamp;
    uint32_t currentIndex;
    uint32_t lastIndex;
    char bundle[81];
    char trunkTransaction[81];
    char branchTransaction[81];
    char tag[27];
    uint32_t attachmentTimestamp;
    uint32_t attachmentTimestampLowerBound;
    uint32_t attachmentTimestampUpperBound;
    char nonce[27];
} TX_OBJECT;

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static char signatureMessageFragment[2187];

static const TX_OBJECT DEFAULT_TX = {
//    {0},
    ZERO_HASH, 0,        ZERO_TAG, 0, 0, 0,       ZERO_HASH,
    ZERO_HASH, ZERO_HASH, ZERO_TAG, 0,        0, 0, ZERO_TAG};

// taken from aux.c
#define PAD_CHAR '9'
static void rpad_chars(char *destination, const char *source, unsigned int num_chars)
{
    const size_t len = strnlen(source, num_chars);
    os_memcpy(destination, source, len);
    os_memset(destination + len, PAD_CHAR, num_chars - len);
}

static char *int64_to_chars(const char *debug_msg, int64_t value, char *chars, unsigned int num_trytes)
{
    trit_t trits[num_trytes * 3];
    int64_to_trits(value, trits, num_trytes * 3);
    trits_to_chars(trits, chars, num_trytes * 3);

    PRINTF("  %s: %lu\n", debug_msg, (long unsigned int)value);

    return chars + num_trytes;
}

static void get_address(const unsigned char *seed_bytes, uint32_t idx,
                        unsigned int security, char *address)
{
    unsigned char bytes[48];
    get_public_addr(seed_bytes, idx, security, bytes);
    bytes_to_chars(bytes, address, 48);
}

static char *char_copy(const char *debug_msg, char *destination, const char *source, unsigned int len)
{
    assert(strnlen(source, len) == len);
    memcpy(destination, source, len);

#if DEBUG
    len = MIN(len, 99);
    static char buffer[100];
    memcpy(buffer, source, len);
    buffer[len] = 0;
    PRINTF("  %s: %s\n", debug_msg, buffer);
#endif

    return destination + len;
}

static void get_transaction_chars(const TX_OBJECT tx, char *transaction_chars)
{
    // just to make sure
    memset(transaction_chars, '\0', 2673);

    char *c = transaction_chars;

    PRINTF("tx: \n");
    //c = char_copy("signature", c, tx.signatureMessageFragment, 2187);
    c = char_copy("address", c, tx.address, 81);
    c = int64_to_chars("value", tx.value, c, 27);
    c = char_copy("obsoleteTag", c, tx.obsoleteTag, 27);
    c = int64_to_chars("timestamp", tx.timestamp, c, 9);
    c = int64_to_chars("currentIndex", tx.currentIndex, c, 9);
    c = int64_to_chars("lastIndex", tx.lastIndex, c, 9);
    c = char_copy("bundle", c, tx.bundle, 81);
    c = char_copy("trunk", c, tx.trunkTransaction, 81);
    c = char_copy("branch", c, tx.branchTransaction, 81);
    c = char_copy("tag", c, tx.tag, 27);
    c = int64_to_chars("attachmentTimestamp", tx.attachmentTimestamp, c, 9);
    c = int64_to_chars("attachmentTimestampLowerBound", tx.attachmentTimestampLowerBound, c, 9);
    c = int64_to_chars("attachmentTimestampUpperBound", tx.attachmentTimestampUpperBound, c, 9);

    char_copy("nonce", c, tx.nonce, 27);
}

static void increment_obsolete_tag(unsigned int tag_increment, TX_OBJECT *tx)
{
    char extended_tag[81];
    unsigned char tag_bytes[48];
    rpad_chars(extended_tag, tx->obsoleteTag, NUM_HASH_TRYTES);
    chars_to_bytes(extended_tag, tag_bytes, NUM_HASH_TRYTES);

    bytes_add_u32_mem(tag_bytes, tag_increment);
    bytes_to_chars(tag_bytes, extended_tag, 48);

    // TODO: do we need to increment both? Probably only obsoleteTag...
    memcpy(tx->obsoleteTag, extended_tag, 27);
    memcpy(tx->tag, extended_tag, 27);
}

static void set_bundle_hash(const BUNDLE_CTX *bundle_ctx, TX_OBJECT *txs,
                            unsigned int num_txs)
{
    char bundle[81];
    bytes_to_chars(bundle_get_hash(bundle_ctx), bundle, 48);

    for (unsigned int i = 0; i < num_txs; i++) {
        memcpy(txs[i].bundle, bundle, 81);
    }
}

void prepare_transfers(char *seed, uint8_t security, TX_OUTPUT *outputs,
                       int num_outputs, TX_INPUT *inputs, int num_inputs,
                       char transaction_chars[][2673])
{
    // TODO use a proper timestamp
    const uint32_t timestamp = 0;
    const unsigned int num_txs = num_outputs + num_inputs * security;
    const unsigned int last_tx_index = num_txs - 1;

    static unsigned char seed_bytes[48];
    chars_to_bytes(seed, seed_bytes, 81);

    // first create the transaction objects
    static TX_OBJECT txs[4]; // [num_txs];

    // make sure at least 1
    security = MAX(security, MIN_SECURITY_LEVEL);

    int idx = 0;

    for (unsigned int i = 0; i < num_outputs; i++) {

        // initialize with defaults
        memcpy(&txs[idx], &DEFAULT_TX, sizeof(TX_OBJECT));

        //rpad_chars(txs[idx].signatureMessageFragment, outputs[i].message, 2187);
        memcpy(txs[idx].address, outputs[i].address, 81);
        txs[idx].value = outputs[i].value;
        rpad_chars(txs[idx].obsoleteTag, outputs[i].tag, 27);
        txs[idx].timestamp = timestamp;
        txs[idx].currentIndex = idx;
        txs[idx].lastIndex = last_tx_index;
        rpad_chars(txs[idx].tag, outputs[i].tag, 27);
        idx++;
    }

    for (unsigned int i = 0; i < num_inputs; i++) {

        // initialize with defaults
        memcpy(&txs[idx], &DEFAULT_TX, sizeof(TX_OBJECT));

        char *address = txs[idx].address;
        if(inputs != NULL) {
          get_address(seed_bytes, inputs[i].key_index, security, address);
          txs[idx].value = -inputs[i].balance;
        } else {
          get_address(seed_bytes, 1, security, address);
          txs[idx].value = -10000;
        }
        txs[idx].timestamp = timestamp;
        txs[idx].currentIndex = idx;
        txs[idx].lastIndex = last_tx_index;
        idx++;

        // add meta transactions
        for (unsigned int j = 1; j < security; j++) {

            // initialize with defaults
            memcpy(&txs[idx], &DEFAULT_TX, sizeof(TX_OBJECT));

            memcpy(txs[idx].address, address, 81);
            txs[idx].value = 0;
            txs[idx].timestamp = timestamp;
            txs[idx].currentIndex = idx;
            txs[idx].lastIndex = last_tx_index;
            idx++;
        }
    }

    PRINTF("*** init bundle %u (%u tx)\n", num_kerl_calls, num_txs);

    // create a secure bundle
    static BUNDLE_CTX bundle_ctx;
    bundle_initialize(&bundle_ctx, last_tx_index);

    PRINTF("*** bundle init done\n");

    for (unsigned int i = 0; i < num_txs; i++) {
      PRINTF("*** bundle add tx %u\n", i);
        bundle_set_external_address(&bundle_ctx, txs[i].address);
        bundle_add_tx(&bundle_ctx, txs[i].value, txs[i].tag, txs[i].timestamp);
    }

    PRINTF("*** bundle finalize\n");

    uint32_t tag_increment = bundle_finalize(&bundle_ctx);

    PRINTF("*** incr tag\n");

    // increment the tag in the first transaction object
    increment_obsolete_tag(tag_increment, &txs[0]);

    PRINTF("*** set bundle hash %u\n", num_kerl_calls);

    // set the bundle hash in all transaction objects
    set_bundle_hash(&bundle_ctx, txs, num_txs);

    // sign the inputs
    static tryte_t normalized_bundle_hash[81];
    bundle_get_normalized_hash(&bundle_ctx, normalized_bundle_hash);

    PRINTF("*** sign inputs %u\n", num_kerl_calls);

    for (unsigned int i = 0; i < num_inputs; i++) {
        static SIGNING_CTX signing_ctx;
        unsigned key_index;

        if(inputs) {
          key_index = inputs[i].key_index;
        } else {
          key_index = 1;
        }
        signing_initialize(&signing_ctx, seed_bytes, key_index,
                           security, normalized_bundle_hash);
        unsigned int idx = num_outputs + i * security;

        PRINTF("    signing init done %u\n", num_kerl_calls);
        
        // exactly one fragment for transaction including meta transactions
        for (unsigned int j = 0; j < security; j++) {

            static unsigned char signature_bytes[27 * 48];
            signing_next_fragment(&signing_ctx, signature_bytes);
            bytes_to_chars(signature_bytes,
                //txs[idx++].signatureMessageFragment,
                signatureMessageFragment,
                           27 * 48);
            PRINTF("    signing fragment done %u\n", num_kerl_calls);

//            hw_watchdog_periodic();
        }
    }

    // convert everything into trytes
    for (unsigned int i = 0; i < num_txs; i++) {
      // get_transaction_chars(txs[i], transaction_chars[last_tx_index - i]);
      get_transaction_chars(txs[i], transaction_chars[0]);
    }

    PRINTF("*** finish %u\n", num_kerl_calls);
    num_kerl_calls = 0;
}
