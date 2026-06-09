// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// crc.c: Mode S CRC calculation and error correction.
//
// Copyright (c) 2014,2015 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// stratux-esp32: the original dump1090 crc.c #include'd "dump1090.h" for these
// few definitions. Provide them locally so the module is self-contained. The
// error-correction syndrome tables (prepareErrorTable et al.) are only built
// when modesChecksumInit() is called with fixBits>0; this port calls it with
// fixBits=0 (we trust the Pong's FEC and only need modesChecksum()), so they
// are compiled but never exercised at runtime.

#include "crc.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#define MODES_LONG_MSG_BITS   112
#define MODES_SHORT_MSG_BITS   56

// Errorinfo for "no errors"
static struct errorinfo NO_ERRORS;

// Generator polynomial for the Mode S CRC:
#define MODES_GENERATOR_POLY 0xfff409U

// CRC values for all single-byte messages;
// used to speed up CRC calculation.
static uint32_t crc_table[256];

// Syndrome values for all single-bit errors;
// used to speed up construction of error-
// correction tables.
static uint32_t single_bit_syndrome[112];

static void initLookupTables()
{
    int i;
    uint8_t msg[112/8];

    for (i = 0; i < 256; ++i) {
        uint32_t c = i << 16;
        int j;
        for (j = 0; j < 8; ++j) {
            if (c & 0x800000)
                c = (c<<1) ^ MODES_GENERATOR_POLY;
            else
                c = (c<<1);
        }

        crc_table[i] = c & 0x00ffffff;
    }

    memset(msg, 0, sizeof(msg));
    for (i = 0; i < 112; ++i) {
        msg[i/8] ^= 1 << (7 - (i & 7));
        single_bit_syndrome[i] = modesChecksum(msg, 112);
        msg[i/8] ^= 1 << (7 - (i & 7));
    }
}

uint32_t modesChecksum(const uint8_t *message, int bits)
{
    uint32_t rem = 0;
    int i;
    int n = bits/8;

    assert(bits % 8 == 0);
    assert(n >= 3);

    for (i = 0; i < n-3; ++i) {
        rem = (rem << 8) ^ crc_table[message[i] ^ ((rem & 0xff0000) >> 16)];
        rem = rem & 0xffffff;
    }

    rem = rem ^ (message[n-3] << 16) ^ (message[n-2] << 8) ^ (message[n-1]);
    return rem;
}

static struct errorinfo *bitErrorTable_short;
static int bitErrorTableSize_short;

static struct errorinfo *bitErrorTable_long;
static int bitErrorTableSize_long;

// compare two errorinfo structures
static int syndrome_compare(const void *x, const void *y) {
    struct errorinfo *ex = (struct errorinfo*)x;
    struct errorinfo *ey = (struct errorinfo*)y;
    return (int)ex->syndrome - (int)ey->syndrome;
}

// (n k), the number of ways of selecting k distinct items from a set of n items
static int combinations(int n, int k)
{
    int result = 1, i;

    if (k == 0 || k == n)
        return 1;

    if (k > n)
        return 0;

    for (i = 1; i <= k; ++i) {
        result = result * n / i;
        n = n - 1;
    }

    return result;
}

// Recursively populates an errorinfo table with error syndromes
static int prepareSubtable(struct errorinfo *table, int n, int maxsize, int offset, int startbit, int endbit, struct errorinfo *base_entry, int error_bit, int max_errors)
{
    int i = 0;

    if (error_bit >= max_errors)
        return n;

    for (i = startbit; i < endbit; ++i) {
        assert(n < maxsize);

        table[n] = *base_entry;
        table[n].syndrome ^= single_bit_syndrome[i + offset];
        table[n].errors = error_bit+1;
        table[n].bit[error_bit] = i;

        ++n;
        n = prepareSubtable(table, n, maxsize, offset, i + 1, endbit, &table[n-1], error_bit + 1, max_errors);
    }

    return n;
}

static int flagCollisions(struct errorinfo *table, int tablesize, int offset, int startbit, int endbit, uint32_t base_syndrome, int error_bit, int first_error, int last_error)
{
    int i = 0;
    int count = 0;

    if (error_bit > last_error)
        return 0;

    for (i = startbit; i < endbit; ++i) {
        struct errorinfo ei;

        ei.syndrome = base_syndrome ^ single_bit_syndrome[i + offset];

        if (error_bit >= first_error) {
            struct errorinfo *collision = bsearch(&ei, table, tablesize, sizeof(struct errorinfo), syndrome_compare);
            if (collision != NULL && collision->errors != -1) {
                ++count;
                collision->errors = -1;
            }
        }

        count += flagCollisions(table, tablesize, offset, i+1, endbit, ei.syndrome, error_bit + 1, first_error, last_error);
    }

    return count;
}


// Allocate and build an error table for messages of length "bits" (max 112)
// returns a pointer to the new table and sets *size_out to the table length
static struct errorinfo *prepareErrorTable(int bits, int max_correct, int max_detect, int *size_out)
{
    int maxsize, usedsize;
    struct errorinfo *table;
    struct errorinfo base_entry;
    int i, j;

    assert (bits >= 0 && bits <= 112);
    assert (max_correct >=0 && max_correct <= MODES_MAX_BITERRORS);
    assert (max_detect >= max_correct);

    if (!max_correct) {
        *size_out = 0;
        return NULL;
    }

    maxsize = 0;
    for (i = 1; i <= max_correct; ++i) {
        maxsize += combinations(bits, i); // space needed for all i-bit errors
    }

    table = malloc(maxsize * sizeof(struct errorinfo));
    base_entry.syndrome = 0;
    base_entry.errors = 0;
    for (i = 0; i < MODES_MAX_BITERRORS; ++i)
        base_entry.bit[i] = -1;

    usedsize = prepareSubtable(table, 0, maxsize, 112 - bits, 0, bits, &base_entry, 0, max_correct);

    qsort(table, usedsize, sizeof(struct errorinfo), syndrome_compare);

    // Handle ambiguous cases, where there is more than one possible error pattern
    // that produces a given syndrome (this happens with >2 bit errors).

    for (i = 0, j = 0; i < usedsize; ++i) {
        if (i < usedsize-1 && table[i+1].syndrome == table[i].syndrome) {
            // skip over this entry and all collisions
            while (i < usedsize && table[i+1].syndrome == table[i].syndrome)
                ++i;

            // now table[i] is the last duplicate
            continue;
        }

        if (i != j)
            table[j] = table[i];
        ++j;
    }

    if (j < usedsize) {
        usedsize = j;
    }

    // Flag collisions we want to detect but not correct
    if (max_detect > max_correct) {
        int flagged;

        flagged = flagCollisions(table, usedsize, 112 - bits, 0, bits, 0, 1, max_correct+1, max_detect);

        if (flagged > 0) {
            for (i = 0, j = 0; i < usedsize; ++i) {
                if (table[i].errors != -1) {
                    if (i != j)
                        table[j] = table[i];
                    ++j;
                }
            }

            usedsize = j;
        }
    }

    *size_out = usedsize;
    return table;
}

// Precompute syndrome tables for 56- and 112-bit messages.
void modesChecksumInit(int fixBits)
{
    initLookupTables();

    switch (fixBits) {
    case 0:
        bitErrorTable_short = bitErrorTable_long = NULL;
        bitErrorTableSize_short = bitErrorTableSize_long = 0;
        break;

    case 1:
        // For 1 bit correction, we have 100% coverage up to 4 bit detection, so don't bother
        // with flagging collisions there.
        bitErrorTable_short = prepareErrorTable(MODES_SHORT_MSG_BITS, 1, 1, &bitErrorTableSize_short);
        bitErrorTable_long = prepareErrorTable(MODES_LONG_MSG_BITS, 1, 1, &bitErrorTableSize_long);
        break;

    default:
        // Detect out to 4 bit errors; this reduces our 2-bit coverage to about 65%.
        bitErrorTable_short = prepareErrorTable(MODES_SHORT_MSG_BITS, 2, 4, &bitErrorTableSize_short);
        bitErrorTable_long = prepareErrorTable(MODES_LONG_MSG_BITS, 2, 4, &bitErrorTableSize_long);
        break;
    }
}

// Given an error syndrome and message length, return
// an error-correction descriptor, or NULL if the
// syndrome is uncorrectable
struct errorinfo *modesChecksumDiagnose(uint32_t syndrome, int bitlen)
{
    struct errorinfo *table;
    int tablesize;

    struct errorinfo ei;

    if (syndrome == 0)
        return &NO_ERRORS;

    assert (bitlen == 56 || bitlen == 112);
    if (bitlen == 56) { table = bitErrorTable_short; tablesize = bitErrorTableSize_short; }
    else { table = bitErrorTable_long; tablesize = bitErrorTableSize_long; }

    if (!table)
        return NULL;

    ei.syndrome = syndrome;
    return bsearch(&ei, table, tablesize, sizeof(struct errorinfo), syndrome_compare);
}

// Given a message and an error-correction descriptor,
// apply the error correction to the given message.
void modesChecksumFix(uint8_t *msg, struct errorinfo *info)
{
    int i;

    if (!info)
        return;

    for (i = 0; i < info->errors; ++i)
        msg[info->bit[i] >> 3] ^= 1 << (7 - (info->bit[i] & 7));
}
