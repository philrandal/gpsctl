//
// Created by Tom Dilatush on 10/19/17.
//

#ifndef GPSCTL_SLIGHTLY_LOONY_H
#define GPSCTL_SLIGHTLY_LOONY_H
#include <memory.h>
#include <stdlib.h>
#include <execinfo.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <time.h>
#include <errno.h>
#include <ctype.h>
#
#define ARRAY_SIZE(x) (sizeof (x) / sizeof *(x))

typedef enum Endiness { BigEndian, LittleEndian } Endiness;

typedef unsigned char byte;

char* onOff( const bool onoff );

char* yesNo( const bool yesno );

void stackDump( void );

void *safeMalloc( size_t size );

bool issprint( const char* str );

bool issgraph( const char* str );

char* concat( const char *s1, const char *s2 );

void append( char** s1, const char* s2 );

void sleep_ns( int ns );

long long currentTimeMs( void );

bool strempty( const char* str );

long long max_ll( long long a, long long b );
int min_i( int a, int b);

int hex2int( char c );

bool isLeapYear( uint32_t year );

uint32_t daysInMonth( uint32_t year, uint32_t month );

char* getAllocatedStringCopy( const char *source );

// Returns the number of arguments that have been passed to it.
#define NUMARGS(...) NUMARGS_(__VA_ARGS__,RSEQ_N())
#define NUMARGS_(...) NUMARGS_N(__VA_ARGS__)
#define NUMARGS_N( \
          _1, _2, _3, _4, _5, _6, _7, _8, _9,_10, \
         _11,_12,_13,_14,_15,_16,_17,_18,_19,_20, \
         _21,_22,_23,_24,_25,_26,_27,_28,_29,_30, \
         _31,_32,_33,_34,_35,_36,_37,_38,_39,_40, \
         _41,_42,_43,_44,_45,_46,_47,_48,_49,_50, \
         _51,_52,_53,_54,_55,_56,_57,_58,_59,_60, \
         _61,_62,_63,N,...) N
#define RSEQ_N() \
         63,62,61,60,                   \
         59,58,57,56,55,54,53,52,51,50, \
         49,48,47,46,45,44,43,42,41,40, \
         39,38,37,36,35,34,33,32,31,30, \
         29,28,27,26,25,24,23,22,21,20, \
         19,18,17,16,15,14,13,12,11,10, \
         9,8,7,6,5,4,3,2,1,0

#endif //GPSCTL_SLIGHTLY_LOONY_H
