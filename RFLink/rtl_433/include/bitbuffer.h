/** @file
    A two-dimensional bit buffer consisting of bytes.

    Copyright (C) 2015 Tommy Vestermark

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#ifndef INCLUDE_BITBUFFER_H_
#define INCLUDE_BITBUFFER_H_

#include <stdint.h>

// NOTE: Wireless mbus protocol needs at least ((256+16*2+3)*12)/8 => 437 bytes
//       but it will not be included if RTL_433_REDUCE_STACK_USE is defined
#ifdef RTL_433_REDUCE_STACK_USE
#define BITBUF_COLS 40 // Number of bytes in a column
#else
#define BITBUF_COLS 450 // Number of bytes in a column
#endif
#define BITBUF_ROWS 25
#define BITBUF_MAX_PRINT_BITS 50 // Maximum number of bits to print (in addition to hex values)

typedef uint8_t bitrow_t[BITBUF_COLS];
typedef bitrow_t bitarray_t[BITBUF_ROWS];

/// Bit buffer.
typedef struct bitbuffer {
    uint16_t num_rows;                      // Number of active rows
    uint16_t bits_per_row[BITBUF_ROWS];     // Number of active bits per row
    uint16_t syncs_before_row[BITBUF_ROWS]; // Number of sync pulses before row
    bitarray_t bb;                          // The actual bits buffer
} bitbuffer_t;

/// Clear the content of the bitbuffer.
void bitbuffer_clear(bitbuffer_t *bits);

/// Add a single bit at the end of the bitbuffer (MSB first).
void bitbuffer_add_bit(bitbuffer_t *bits, int bit);

/// Add a new row to the bitbuffer.
void bitbuffer_add_row(bitbuffer_t *bits);

/// Increment sync counter, add new row if not empty.
void bitbuffer_add_sync(bitbuffer_t *bits);

/// Extract (potentially unaligned) bytes from the bit buffer. Len is bits.
void bitbuffer_extract_bytes(bitbuffer_t *bitbuffer, unsigned row,
        unsigned pos, uint8_t *out, unsigned len);

/// Invert all bits in the bitbuffer (do not invert the empty bits).
void bitbuffer_invert(bitbuffer_t *bits);

/// Non-Return-to-Zero Space (NRZI) decode the bitbuffer.
/// "One" is represented by no change in level, "Zero" is represented by change in level.
void bitbuffer_nrzs_decode(bitbuffer_t *bits);

/// Non-Return-to-Zero Mark (NRZI) decode the bitbuffer.
/// "One" is represented by change in level, "Zero" is represented by no change in level.
void bitbuffer_nrzm_decode(bitbuffer_t *bits);

/// Print the content of the bitbuffer.
void bitbuffer_print(const bitbuffer_t *bits);

/// Debug the content of the bitbuffer.
void bitbuffer_debug(const bitbuffer_t *bits);

/// Print the content of a bit row (byte buffer).
void bitrow_print(uint8_t const *bitrow, unsigned bit_len);

/// Debug the content of a bit row (byte buffer).
void bitrow_debug(uint8_t const *bitrow, unsigned bit_len);

/// Parse a string into a bitbuffer.
void bitbuffer_parse(bitbuffer_t *bits, const char *code);

/// Search the specified row of the bitbuffer, starting from bit 'start', for
/// the pattern provided. Return the location of the first match, or the end
/// of the row if no match is found.
/// The pattern starts in the high bit. For example if searching for 011011
/// the byte pointed to by 'pattern' would be 0xAC. (011011xx).
unsigned bitbuffer_search(bitbuffer_t *bitbuffer, unsigned row, unsigned start,
        const uint8_t *pattern, unsigned pattern_bits_len);

/// Manchester decoding from one bitbuffer into a bitrow, starting at the
/// specified row and start bit. Decode at most 'max' data bits (i.e. 2*max)
/// bits from the input buffer). Return the bit position in the input row
/// (i.e. returns start + 2*outbuf->bits_per_row[0]).
/// per IEEE 802.3 conventions, i.e. high-low is a 0 bit, low-high is a 1 bit.
unsigned bitbuffer_manchester_decode(bitbuffer_t *inbuf, unsigned row, unsigned start,
        uint8_t *outrow, uint16_t *outrow_num_bits, unsigned max);

/// Differential Manchester decoding from one bitbuffer into another, starting at the
/// specified row and start bit. Decode at most 'max' data bits (i.e. 2*max)
/// bits from the input buffer). Return the bit position in the input row
/// (i.e. returns start + 2*outbuf->bits_per_row[0]).
unsigned bitbuffer_differential_manchester_decode(bitbuffer_t *inbuf, unsigned row, unsigned start,
        bitbuffer_t *outbuf, unsigned max);

/// Function to compare bitbuffer rows and count repetitions.
int compare_rows(bitbuffer_t *bits, unsigned row_a, unsigned row_b);

unsigned count_repeats(bitbuffer_t *bits, unsigned row);

/// Find a repeated row that has a minimum count of bits.
/// Return the row index or -1.
int bitbuffer_find_repeated_row(bitbuffer_t *bits, unsigned min_repeats, unsigned min_bits);

/// Return a single bit from a bitrow at bit_idx position.
static inline uint8_t bitrow_get_bit(const bitrow_t bitrow, unsigned bit_idx)
{
    return bitrow[bit_idx >> 3] >> (7 - (bit_idx & 7)) & 1;
}

/// Return a single byte from a bitrow at bit_idx position (which may be unaligned).
static inline uint8_t bitrow_get_byte(const bitrow_t bitrow, unsigned bit_idx)
{
    return (uint8_t)((bitrow[(bit_idx >> 3)] << (bit_idx & 7)) |
                     (bitrow[(bit_idx >> 3) + 1] >> (8 - (bit_idx & 7))));
}

/// Clear the content of the bitrow and sets bitrow_num_bits to 0.
void bitrow_clear(bitrow_t bitrow, uint16_t *bitrow_num_bits);

/// Add the given bit into the bitrow, at the bit_idx position and increments the position upon return
void bitrow_add_bit(bitrow_t bitrow, uint16_t *bitrow_num_bits, int bit);

/// Extract (potentially unaligned) bytes from the bit row. Len is bits.
void bitrow_extract_bytes(bitrow_t const bitrow, unsigned pos, uint8_t *out, unsigned len);

/// Invert all bits in the bitrow (do not invert the empty bits).
void bitrow_invert(bitrow_t bitrow, uint16_t bitrow_num_bits);

/// Manchester decoding from one bitrow into another, starting at the
/// specified start bit. Decode at most 'max' data bits (i.e. 2*max)
/// bits from the input buffer). Return the bit position in the input row
/// (i.e. returns start + 2*outbuf->bits_per_row[0]).
/// per IEEE 802.3 conventions, i.e. high-low is a 0 bit, low-high is a 1 bit.
unsigned bitrow_manchester_decode(bitrow_t const inrow, uint16_t inrow_num_bits, unsigned start,
        bitrow_t outrow, uint16_t *outrow_num_bits, unsigned max);

#endif /* INCLUDE_BITBUFFER_H_ */