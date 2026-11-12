//
// Created by Tom Dilatush on 10/24/17.
//


// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "sl_general.h"
#include "sl_buffer.h"

struct slBuffer {
    size_t size;
    enum Endiness endiness;
    byte buffer[];
};


// Returns a pointer to a newly created and allocated instance of slBuffer with an uninitialized buffer of the given
// size and endiness.
extern slBuffer* create_slBuffer( size_t size, Endiness endiness ) {

    size_t memSize = sizeof( slBuffer ) + size;
    slBuffer* result = safeMalloc( memSize );
    result->size = size;
    result->endiness = endiness;
    return result;
}


// Returns a pointer to a newly created and allocated instance of slBuffer, with a buffer of the given size, and
// endiness, initialized with the given arguments.  If the number of arguments is less than the size, the behavior is
// not predictable.  If the number of arguments is greater than the size, only (size) elements will be initialized.
// Note that while this function MAY be called directly, the better way to call it is indirectly through the
// "init_slBuffer(...)" macro, which automatically calculates the size based on the number of arguments.
extern slBuffer* init_slBuffer_int( size_t size, Endiness endiness, ... ) {

    slBuffer* result = create_slBuffer( size, endiness );

    if( size != 0 ) {
        va_list argPtr;
        va_start( argPtr, endiness );
        for (int i = 0; i < size; i++) {
            byte b = (byte) va_arg( argPtr, int );
            result->buffer[i] = b;
        }
        va_end( argPtr );
    }
    return result;
}


// Returns a pointer to a newly created and allocated instance of slBuffer initialized to a copy of the given
// byte buffer, with the given endiness.
extern slBuffer* copy_slBuffer( size_t size, Endiness endiness, byte *buffer ) {
    slBuffer* result = create_slBuffer( size, endiness );
    memcpy( result->buffer, buffer, size );
    return result;
}


// Prints a representation of the given instance of slBuffer.
extern void print_slBuffer( slBuffer *buffer ) {
    char* endy = (buffer->endiness == LittleEndian) ? "little-endian" : "big-endian";
    printf( "slBuffer, %d bytes, %s:", (int) buffer->size, endy );
    for( int i = 0; i < buffer->size; i++ )
        printf( "%02x:", buffer->buffer[i] );
    printf( "\n" );
}


// Return the size of the given instance of slBuffer.  Note that this is NOT the size of the slBuffer structure
// itself, but rather the size of the byte buffer it contains.
extern size_t size_slBuffer( slBuffer *buffer ) {
    return buffer->size;
}


// Returns a pointer to the start of the byte buffer contained in the given instance of slBuffer.
extern byte* buffer_slBuffer( slBuffer *buffer ) {
    return buffer->buffer;
}


// Puts the given 32 bit unsigned value at the given offset within the given instance of slBuffer.  The endiness of
// the instance controls the byte layout.  Note that if the value will not fit in the allocated buffer, then no action
// is taken.
extern void put_uint32_slBuffer( slBuffer *buffer, size_t offset, uint32_t value ) {

    // if we're about to write outside our buffer, just bail out...
    if( (offset + 4) > buffer->size ) return;

    byte* b = buffer->buffer;
    if( buffer->endiness == LittleEndian ) {
        b[offset++] = (byte)  value;
        b[offset++] = (byte) (value >> 8 );
        b[offset++] = (byte) (value >> 16);
        b[offset  ] = (byte) (value >> 24);
    }
    else {
        b[offset++] = (byte) (value >> 24);
        b[offset++] = (byte) (value >> 16);
        b[offset++] = (byte) (value >> 8 );
        b[offset  ] = (byte)  value;
    }
}


// Gets the 32 bit unsigned value at the given offset within the given instance of slBuffer.  The endiness of the
// instance controls the byte layout.  Note that if the value lies outside the allocated buffer, zero is returned.
extern uint32_t get_uint32_slBuffer( slBuffer *buffer, size_t offset ) {

    // if we're about to write outside our buffer, just bail out...
    if ((offset + 4) > buffer->size) return 0;

    byte *b = buffer->buffer;
    return (buffer->endiness == LittleEndian) ?
           b[offset]        | (b[offset + 1] <<  8) | (b[offset + 2] << 16) | (b[offset + 3] << 24 ) :
          (b[offset] << 24) | (b[offset + 1] << 16) | (b[offset + 2] <<  8) |  b[offset + 3]         ;
}


// Puts the given 32 bit signed value at the given offset within the given instance of slBuffer.  The endiness of
// the instance controls the byte layout.  Note that if the value will not fit in the allocated buffer, then no action
// is taken.
extern void put_int32_slBuffer( slBuffer *buffer, size_t offset, int32_t value ) {
    put_uint32_slBuffer( buffer, offset, (uint32_t) value );
}


// Gets the 32 bit signed value at the given offset within the given instance of slBuffer.  The endiness of the
// instance controls the byte layout.  Note that if the value lies outside the allocated buffer, zero is returned.
extern int32_t get_int32_slBuffer( slBuffer *buffer, size_t offset ) {
    return (int32_t) get_uint32_slBuffer( buffer, offset );
}


// Puts the given 16 bit unsigned value at the given offset within the given instance of slBuffer.  The endiness of
// the instance controls the byte layout.  Note that if the value will not fit in the allocated buffer, then no action
// is taken.
extern void put_uint16_slBuffer( slBuffer *buffer, size_t offset, uint16_t value ) {

    // if we're about to write outside our buffer, just bail out...
    if( (offset + 2) > buffer->size ) return;

    byte* b = buffer->buffer;
    if( buffer->endiness == LittleEndian ) {
        b[offset++] = (byte)  value;
        b[offset  ] = (byte) (value >> 8 );
    }
    else {
        b[offset++] = (byte) (value >> 8 );
        b[offset  ] = (byte)  value;
    }
}


// Gets the 16 bit unsigned value at the given offset within the given instance of slBuffer.  The endiness of the
// instance controls the byte layout.  Note that if the value lies outside the allocated buffer, zero is returned.
extern uint16_t get_uint16_slBuffer( slBuffer *buffer, size_t offset ) {

    // if we're about to write outside our buffer, just bail out...
    if ((offset + 2) > buffer->size) return 0;

    byte *b = buffer->buffer;
    return (uint16_t) ((buffer->endiness == LittleEndian) ?
                       (b[offset] | (b[offset + 1] << 8)) :
                       ((b[offset] << 8) | b[offset + 1]));
}


// Puts the given 16 bit signed value at the given offset within the given instance of slBuffer.  The endiness of
// the instance controls the byte layout.  Note that if the value will not fit in the allocated buffer, then no action
// is taken.
extern void put_int16_slBuffer( slBuffer *buffer, size_t offset, int16_t value ) {
    put_uint16_slBuffer( buffer, offset, (uint16_t) value );
}


// Gets the 16 bit signed value at the given offset within the given instance of slBuffer.  The endiness of the
// instance controls the byte layout.  Note that if the value lies outside the allocated buffer, zero is returned.
extern int16_t get_int16_slBuffer( slBuffer *buffer, size_t offset ) {
    return (int16_t) get_uint16_slBuffer( buffer, offset );
}


// Puts the given 8 bit unsigned value at the given offset within the given instance of slBuffer.  The endiness of
// the instance controls the byte layout.  Note that if the value will not fit in the allocated buffer, then no action
// is taken.
extern void put_uint8_slBuffer( slBuffer *buffer, size_t offset, uint8_t value ) {

    // if we're about to write outside our buffer, just bail out...
    if( (offset + 1) > buffer->size ) return;

    buffer->buffer[offset] = (byte) value;
}


// Gets the 8 bit unsigned value at the given offset within the given instance of slBuffer.  The endiness of the
// instance controls the byte layout.  Note that if the value lies outside the allocated buffer, zero is returned.
extern uint8_t get_uint8_slBuffer( slBuffer *buffer, size_t offset ) {

    // if we're about to read outside our buffer, just bail out...
    if ((offset + 1) > buffer->size) return 0;

    return (uint8_t) buffer->buffer[offset];
}


// Puts the given 8 bit signed value at the given offset within the given instance of slBuffer.  The endiness of
// the instance controls the byte layout.  Note that if the value will not fit in the allocated buffer, then no action
// is taken.
extern void put_int8_slBuffer( slBuffer *buffer, size_t offset, int8_t value ) {
    put_uint8_slBuffer( buffer, offset, (uint8_t) value );
}


// Gets the 8 bit signed value at the given offset within the given instance of slBuffer.  The endiness of the
// instance controls the byte layout.  Note that if the value lies outside the allocated buffer, zero is returned.
extern int8_t get_int8_slBuffer( slBuffer *buffer, size_t offset ) {
    return (int8_t) get_uint8_slBuffer( buffer, offset );
}
