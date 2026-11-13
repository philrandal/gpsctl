//
// Created by Tom Dilatush on 10/19/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L

#include "sl_general.h"


// Returns "on" or "off" for the given true or false.
extern char* onOff( const bool onoff ) {
    return onoff ? "on" : "off";
}

// Returns "yes" or "no" for the given true or false.
extern char* yesNo( const bool yesno ) {
    return yesno ? "yes" : "no";
}

// Dumps a stack trace to stderr.
extern void stackDump( void ) {
    #define STACK_DUMP_SIZE 250
    void *stackFrames[STACK_DUMP_SIZE];

    int numFrames = backtrace( stackFrames, STACK_DUMP_SIZE );
    backtrace_symbols_fd( stackFrames, numFrames, STDERR_FILENO );
}


// A safe replacement for malloc that checks the result.  On failure, prints an error message and stack trace
// to stderr.  Otherwise, it behaves EXACTLY like malloc().
extern void *safeMalloc( size_t size ) {
    void *result = malloc( size );
    if( !result ) {
        fprintf( stderr, "ERROR: malloc() failed\n" );
        stackDump();
        exit(1);
    }
    return result;
}


// Returns true if every character in the given string is printable (see isprint() for details).  Returns false if
// the given string is empty.
extern bool issprint( const char* str ) {
    if( strempty( str ) ) return false;
    do {
        if( !isprint( *str ) ) return false;
    } while( *(++str) != 0 );
    return true;
}


// Returns true if every character in the given string is graphical (see isgraph() for details).  Returns false if
// the given string is empty.
extern bool issgraph( const char* str ) {
    if( strempty( str ) ) return false;
    do {
        if( !isgraph( *str ) ) return false;
    } while( *(++str) != 0 );
    return true;
}


// Appends the second given string to the first.  If the first string is NULL, makes a new copy of the second into
// the first.  Otherwise, allocates memory for the concatenation of the two strings, concatenates them, and frees
// the original first string.  This mechanism allows a series of appends to be performed while ending up with only
// one unfreed allocated block of memory: the one with the result.
extern void append( char** s1, const char* s2 ) {
    char* r = concat( *s1, s2 );
    free( *s1 );
    *s1 = r;
}


// Return the concatenation of the two given string in a newly allocated memory on the heap.  If the two
// given strings are null, a null is returned.  If one of the given strings is null, then a COPY (in newly allocated
// memory) of the non-null given string is returned.  Note that this behavior means that ANY non-null return value
// represents a block of memory that must be freed by the caller.  If malloc fails, a stack trace is dumped to stderr
// and the program is aborted.
extern char* concat( const char *s1, const char *s2 ) {

    // if both arguments are NULL, return a NULL...
    if( (s1 == NULL) && (s2 == NULL) ) return NULL;

    // get the lengths just once (optimization)...
    const size_t len1 = s1 ? strlen( s1 ) : 0;
    const size_t len2 = s2 ? strlen( s2 ) : 0;

    // get the memory required for the concatenated strings plus the terminator...
    char *result = safeMalloc( len1 + len2 + 1 );

    // build our result...
    if( s1 != NULL ) memcpy(result, s1, len1);
    if( s2 != NULL ) memcpy(result + len1, s2, len2 );

    // insert the terminator...
    *(result + len1 + len2) = 0;

    return result;
}


// returns true if the given string pointer is NULL or points to an empty string...
extern bool strempty( const char* str ) {
    return (str == NULL) || (*str == 0);
}


extern long long max_ll( long long a, long long b ) {
    return (a > b) ? a : b;
}


extern int min_i( int a, int b ) {
    return (a < b) ? a : b;
}


// Returns the monotonic clock time in milliseconds.  The arithmetic is done in 64 bits: where time_t is 32 bits,
// tv_sec * 1000L overflows a 32-bit long after about 24.8 days of uptime, which breaks every timeout calculation
// that straddles the wrap.
extern long long currentTimeMs() {
    struct timespec tp;
    clock_gettime( CLOCK_MONOTONIC, &tp );
    return (long long) tp.tv_sec * 1000LL + tp.tv_nsec / 1000000LL;
}


// Sleep for the given number of nanoseconds.
extern void sleep_ns( int ns ) {
    struct timespec nanos;
    nanos.tv_sec = 0;
    nanos.tv_nsec = ns;
    nanosleep( &nanos, NULL );
}

// Returns [0..15] if the given character is a valid hex digit, -1 otherwise.
extern int hex2int( char c ) {
    if( !isxdigit( c ) ) return -1;
    return ((c >= '0') && (c <= '9')) ? (c - '0') : (0x0F & (c + 9));
}


// Returns true if the given year is a leap year...
extern bool isLeapYear( uint32_t year ) {
    return ((year % 400) == 0) || (((year % 4) == 0) && ((year % 100) != 0));
}


// Returns the number of days in the given month in the given year.  The month is represented by [1..12].
static uint8_t minDaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
extern uint32_t daysInMonth( uint32_t year, uint32_t month ) {
    if( (month == 0) || (month > 12)) return 0;
    uint32_t result = minDaysInMonth[month - 1];
    if( (month == 2) && isLeapYear( year ) )
        result++;
    return result;
}


// Returns a pointer to a newly allocated copy of the given source string.  If the source is NULL, so is the return
// value.
extern char* getAllocatedStringCopy( const char *source ) {

    if( !source ) return NULL;

    char *result = safeMalloc( 1 + strlen( source ) );
    strcpy( result, source );
    return result;
}
