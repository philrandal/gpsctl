//
// Created by Tom Dilatush on 10/24/17.
//


// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L

#include "sl_bits.h"


static uint64_t bitMasks[] = {
        0x0000000000000001, 0x0000000000000002, 0x0000000000000004, 0x0000000000000008, 0x0000000000000010, 0x0000000000000020, 0x0000000000000040, 0x0000000000000080,
        0x0000000000000100, 0x0000000000000200, 0x0000000000000400, 0x0000000000000800, 0x0000000000001000, 0x0000000000002000, 0x0000000000004000, 0x0000000000008000,
        0x0000000000010000, 0x0000000000020000, 0x0000000000040000, 0x0000000000080000, 0x0000000000100000, 0x0000000000200000, 0x0000000000400000, 0x0000000000800000,
        0x0000000001000000, 0x0000000002000000, 0x0000000004000000, 0x0000000008000000, 0x0000000010000000, 0x0000000020000000, 0x0000000040000000, 0x0000000080000000,
        0x0000000100000000, 0x0000000200000000, 0x0000000400000000, 0x0000000800000000, 0x0000001000000000, 0x0000002000000000, 0x0000004000000000, 0x0000008000000000,
        0x0000010000000000, 0x0000020000000000, 0x0000040000000000, 0x0000080000000000, 0x0000100000000000, 0x0000200000000000, 0x0000400000000000, 0x0000800000000000,
        0x0001000000000000, 0x0002000000000000, 0x0004000000000000, 0x0008000000000000, 0x0010000000000000, 0x0020000000000000, 0x0040000000000000, 0x0080000000000000,
        0x0100000000000000, 0x0200000000000000, 0x0400000000000000, 0x0800000000000000, 0x1000000000000000, 0x2000000000000000, 0x4000000000000000, 0x8000000000000000
};

// Return the value of the bit field in the given value, where the bit field is defined by the one bits in the given
// mask.
extern uint64_t getBitField_slBits( uint64_t value, uint64_t mask ) {

    // if no mask, just leave with a zero...
    if( mask == 0 ) return 0;

    // right-justify our field and value...
    while( (mask & 1) == 0) {
        value >>= 1;
        mask >>= 1;
    }

    // return the resulting value...
    return value & mask;
}

// Returns true if the given bit number is set (to a one) in the given value.  Note that only the six LSBs of the bit
// number (for bits 0..63) are actually used; all other bits in that value are ignored.
extern bool isBitSet_slBits( uint64_t value, int bitNum ) {
    return (value & bitMasks[ 0x3F & bitNum ]) != 0;
}


// Sets the given bit number in the given value to the value of state (one for true, zero for false).  Returns the
// resulting value.  Note that only the six LSBs of the bit number (for bits 0..63) are actually used; all other bits
// in that value are ignored.
extern uint64_t setBit_slBits( uint64_t value, int bitNum, bool state ) {
    uint64_t mask = bitMasks[ 0x3F & bitNum];
    return (value & (~mask)) | (state ? mask : 0);
}