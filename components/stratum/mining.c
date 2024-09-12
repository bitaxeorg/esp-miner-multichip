#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include "mining.h"
#include "utils.h"
#include "mbedtls/sha256.h"

// Function to free bm_job resources
void free_bm_job(bm_job *job)
{
    free(job->jobid);
    free(job->extranonce2);
    free(job);
}

// Construct the coinbase transaction
char *construct_coinbase_tx(const char *coinbase_1, const char *coinbase_2,
                            const char *extranonce, uint32_t extranonce_2, uint32_t extranonce_2_len)
{
    // Preallocate extranonce_2 string
    char extranonce_2_str[extranonce_2_len * 2 + 1];
    extranonce_2_generate(extranonce_2, extranonce_2_len, extranonce_2_str);

    // Calculate the total length for the coinbase transaction
    int coinbase_tx_len = strlen(coinbase_1) + strlen(coinbase_2) + strlen(extranonce) + strlen(extranonce_2_str) + 1;

    // Allocate memory for the coinbase transaction and construct it
    char *coinbase_tx = malloc(coinbase_tx_len);
    strcpy(coinbase_tx, coinbase_1);
    strcat(coinbase_tx, extranonce);
    strcat(coinbase_tx, extranonce_2_str);
    strcat(coinbase_tx, coinbase_2);
    coinbase_tx[coinbase_tx_len - 1] = '\0';

    return coinbase_tx;
}

// Calculate the Merkle root hash
char *calculate_merkle_root_hash(const char *coinbase_tx, const uint8_t merkle_branches[][32], const int num_merkle_branches)
{
    size_t coinbase_tx_bin_len = strlen(coinbase_tx) / 2;
    uint8_t *coinbase_tx_bin = malloc(coinbase_tx_bin_len);
    hex2bin(coinbase_tx, coinbase_tx_bin, coinbase_tx_bin_len);

    uint8_t both_merkles[64];
    uint8_t *new_root = double_sha256_bin(coinbase_tx_bin, coinbase_tx_bin_len);
    free(coinbase_tx_bin);
    memcpy(both_merkles, new_root, 32);
    free(new_root);
    
    for (int i = 0; i < num_merkle_branches; i++)
    {
        memcpy(both_merkles + 32, merkle_branches[i], 32);
        uint8_t *new_root = double_sha256_bin(both_merkles, 64);
        memcpy(both_merkles, new_root, 32);
        free(new_root);
    }

    char *merkle_root_hash = malloc(65);
    bin2hex(both_merkles, 32, merkle_root_hash, 65);
    return merkle_root_hash;
}

// Construct the bm_job from mining_notify parameters
bm_job construct_bm_job(mining_notify *params, const char *merkle_root, const uint32_t version_mask)
{
    bm_job new_job;

    new_job.version = params->version;
    new_job.starting_nonce = 0;
    new_job.target = params->target;
    new_job.ntime = params->ntime;
    new_job.pool_diff = params->difficulty;

    hex2bin(merkle_root, new_job.merkle_root, 32);
    swap_endian_words(merkle_root, new_job.merkle_root_be);
    reverse_bytes(new_job.merkle_root_be, 32);

    swap_endian_words(params->prev_block_hash, new_job.prev_block_hash);
    hex2bin(params->prev_block_hash, new_job.prev_block_hash_be, 32);
    reverse_bytes(new_job.prev_block_hash_be, 32);

    // Make the midstate hash
    uint8_t midstate_data[64];
    memcpy(midstate_data, &new_job.version, 4);               // Copy version
    memcpy(midstate_data + 4, new_job.prev_block_hash, 32);   // Copy prev_block_hash
    memcpy(midstate_data + 36, new_job.merkle_root, 28);      // Copy part of merkle_root

    midstate_sha256_bin(midstate_data, 64, new_job.midstate); // Midstate hash
    reverse_bytes(new_job.midstate, 32);                      // Reverse midstate bytes

    // Version rolling if applicable
    if (version_mask != 0)
    {
        uint32_t rolled_version = increment_bitmask(new_job.version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, new_job.midstate1);
        reverse_bytes(new_job.midstate1, 32);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, new_job.midstate2);
        reverse_bytes(new_job.midstate2, 32);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, new_job.midstate3);
        reverse_bytes(new_job.midstate3, 32);
        new_job.num_midstates = 4;
    }
    else
    {
        new_job.num_midstates = 1;
    }

    return new_job;
}

// Optimized extranonce_2_generate function
void extranonce_2_generate(uint32_t extranonce_2, uint32_t length, char *extranonce_2_str)
{
    memset(extranonce_2_str, '0', length * 2);  // Fill with '0's
    extranonce_2_str[length * 2] = '\0';        // Null-terminate
    bin2hex((uint8_t *)&extranonce_2, sizeof(extranonce_2), extranonce_2_str, length * 2 + 1);

    if (length > 4)
    {
        extranonce_2_str[8] = '0';  // Padding adjustment if extranonce is longer than 4 bytes
    }
}

// Test the nonce value and return difficulty (0 means invalid)
double test_nonce_value(const bm_job *job, const uint32_t nonce, const uint32_t rolled_version)
{
    double d64, s64, ds;
    unsigned char header[80];
    unsigned char hash_buffer[32];
    unsigned char hash_result[32];

    // Use precomputed midstate
    memcpy(hash_buffer, job->midstate, 32);

    // Complete header with (ntime, target, nonce)
    memcpy(header, &job->ntime, 4);
    memcpy(header + 4, &job->target, 4);
    memcpy(header + 8, &nonce, 4);

    // Perform second round SHA-256 hashing
    mbedtls_sha256(header, 12, hash_buffer + 32, 0);  // Hash 12 bytes of data
    mbedtls_sha256(hash_buffer, 64, hash_result, 0);  // Second round SHA-256

    // Compute difficulty from hash result
    d64 = truediffone;
    s64 = le256todouble(hash_result);
    ds = d64 / s64;

    return ds;
}

// Increment bitmask for version rolling
uint32_t increment_bitmask(uint32_t value, uint32_t mask)
{
    if (mask == 0)
        return value;

    while (mask != 0)
    {
        uint32_t carry = (value & mask) + (mask & -mask);
        uint32_t overflow = carry & ~mask;
        value = (value & ~mask) | (carry & mask);
        mask = overflow << 1;  // Propagate carry
    }
    return value;
}
