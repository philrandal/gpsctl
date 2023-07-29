//
// Created by Tom Dilatush on 10/22/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L


#include <memory.h>
#include <stdlib.h>
#include <execinfo.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <errno.h>
#include "sl_general.h"
#include "sl_serial.h"
#include "sl_bits.h"
#include "sl_return.h"


// Returns speed information for the given port.
extern slReturn getSpeedInfo( int fdPort, speedInfo* result ) {
    struct termios options;
    tcgetattr( fdPort, &options );
    slReturn brResp = getBaudRate((int) cfgetispeed( &options ));
    if( isErrorReturn( brResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( brResp ), "could not get baud rate" );
    result->baudRate =  getReturnInfoInt32( brResp );
    result->nsBit = 1000000000 / result->baudRate;
    int stopBits = (options.c_cflag & CSTOPB) ? 2 : 1;
    int dataBits = (int) getBitField_slBits( options.c_cflag, CSIZE );
    int bits = stopBits + dataBits + 1;  // the +1 is for the ever-present start bit...
    result->nsChar = bits * result->nsBit;
    return makeOkReturn();
}


// Tests whether the given device name is non-null, exists, is a device, is a character device, and is a terminal.
extern slReturn verifySerialDevice( const char* deviceName ) {

    if( !deviceName )
        return makeErrorMsgReturn( ERR_ROOT, "attempted to verify a NULL device name" );

    struct stat stats;
    int r = stat( deviceName, &stats );

    if( r != 0 )
        return makeErrorFmtMsgReturn( ERR_ROOT, "stat() returned error on \"%s\": %s", deviceName, strerror(errno) );

    if( ((stats.st_mode & S_IFMT) != S_IFBLK) && ((stats.st_mode & S_IFMT) != S_IFCHR) )
        return makeErrorFmtMsgReturn( ERR_ROOT, "\"%s\" is not a device", deviceName );

    if( (stats.st_mode & S_IFMT) != S_IFCHR )
        return makeErrorFmtMsgReturn( ERR_ROOT, "\"%s\" is not a character mode device", deviceName );;

    int fd = open( deviceName, O_RDWR | O_NOCTTY | O_NONBLOCK );
    if( fd < 0 )
        return makeErrorFmtMsgReturn( ERR_ROOT, "can't open \"%s\": %s", deviceName, strerror(errno) );

    struct termios termios;
    r = tcgetattr( fd, &termios );
    if( r != 0 )
        return makeErrorFmtMsgReturn( ERR_ROOT, "can't retrieve terminal attributes for \"%s\": %s", deviceName,
                                     strerror(errno) );

    close( fd );

    return makeOkReturn();
}


// Reads a single character from the given serial port, waiting at most the given number of milliseconds.  On an Ok
// response, returns the character read as a char in additional info.  Returns a warning if it timed out.
extern slReturn readSerialChar( int fdPort, long long msTimeout ) {

    unsigned char c;
    long long startTime = currentTimeMs();

    int count = 0;

    // figure out how many nanoseconds to sleep between port queries...
    speedInfo si;
    slReturn gsiResp = getSpeedInfo( fdPort, &si );
    if( isErrorReturn( gsiResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( gsiResp ), "could not get speed information" );
    int sleepyNs = si.nsChar >> 1;

    // keep trying unless we run out of time...
    while((currentTimeMs() - startTime) < msTimeout ) {

        count++;

        // try to read a character...
        ssize_t rx_length = read( fdPort, &c, 1 );

        // if we got an error, tell our caller...
        if((rx_length < 0) && (errno != EAGAIN))
            return makeErrorFmtMsgReturn( ERR_ROOT, "read() returned error: %s", strerror( errno ) );

        // if we read a character, return it...
        if( rx_length == 1 )
            return makeOkInfoReturn( char2info(c) );

        // otherwise, we didn't read anything..
        // delay a half character time before we try again...
        sleep_ns( sleepyNs );
    }

    // if we get here, then we timed out...
    return makeWarningReturn();
}


typedef struct baudTrans {
    int baud;
    speed_t cookie;
} baudTrans;
static baudTrans validRates[] = {
        {     50,     B50 },
        {     75,     B75 },
        {    110,    B110 },
        {    134,    B134 },
        {    150,    B150 },
        {    200,    B200 },
        {    300,    B300 },
        {    600,    B600 },
        {   1200,   B1200 },
        {   1800,   B1800 },
        {   2400,   B2400 },
        {   4800,   B4800 },
        {   9600,   B9600 },
        {  19200,  B19200 },
        {  38400,  B38400 },
        {  57600,  B57600 },
        { 115200, B115200 },
        { 230400, B230400 },
        { 460800, B460800 },
        { 921600, B921600 }
};

// Translate the given integer baud rate into the constant needed by termios to specify that baud rate.  If the given
// baud rate can't be translated, it is not a valid baud rate value.  If the given baud rate is valid, the termios
// constant is returned.  Otherwise, -1 is returned.
extern int getBaudRateCookie( int baudRate ) {

    for( int i = 0; i < ARRAY_SIZE( validRates ); i++ ) {
        if( baudRate == validRates[i].baud )
            return (int) validRates[i].cookie;
    }
    return -1;
}


// Translate the given baud rate cookie into the actual baud rate.  If the given cookie can't be translated, it wasn't
// a valid cookie.  If the given cookie is valid, the actual baud rate is returned as an int in info.
extern slReturn getBaudRate( int cookie ) {

    for( int i = 0; i < ARRAY_SIZE( validRates ); i++ ) {
        if( cookie == validRates[i].cookie )
            return makeOkInfoReturn( int2info( validRates[i].baud ) );
    }
    return makeErrorFmtMsgReturn( ERR_ROOT, "unrecognized baud rate cookie: %d", cookie );
}


// Flush the receive buffer.
extern slReturn flushRx( int fdPort ) {
    int result = tcflush( fdPort, TCIFLUSH );
    if( result < 0 )
        return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcflush(): %s", strerror(errno) );
    return makeOkReturn();
}


// Change the baud rate ONLY.  First waits for transmitter to finish sending any buffered data, then flushes any
// received data after changing the rate.
extern slReturn setTermOptionsBaud( int fdPort, int baud ) {

    if( fdPort < 0 ) return makeErrorMsgReturn( ERR_ROOT, "Invalid serial port file descriptor" );
    int baudCookie = getBaudRateCookie( baud );
    if( baudCookie < 0 ) return makeErrorMsgReturn( ERR_ROOT, "Invalid baud rate" );

    struct termios options;
    int result;
    result = tcdrain( fdPort );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcdrain(): %s", strerror(errno) );
    result = tcgetattr( fdPort, &options );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcgetattr(): %s", strerror(errno) );
    result = cfsetispeed( &options, (speed_t) baudCookie );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from cfsetispeed(): %s", strerror(errno) );
    result = cfsetospeed( &options, (speed_t) baudCookie );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from cfsetospeed(): %s", strerror(errno) );
    result = tcflush( fdPort, TCIFLUSH );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcflush(): %s", strerror(errno) );
    result = tcsetattr( fdPort, TCSANOW, &options );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcsetattr(): %s", strerror(errno) );
    return makeOkReturn();
}


// Changes the terminal options on the given serial port to the given baud rate, number of data bits, number of stop
// bits, parity enable, and parity polarity (true for odd, false for even).  The given arguments are first checked for
// validity.
extern slReturn setTermOptions( int fdPort, int baud, int dataBits, int stopBits, bool parityEnable, bool parityOdd ) {

    if( fdPort < 0 ) return makeErrorMsgReturn(ERR_ROOT, "Invalid serial port file descriptor" );;
    int baudCookie = getBaudRateCookie( baud );
    if( baudCookie < 0 ) return makeErrorMsgReturn(ERR_ROOT, "Invalid baud rate" );
    int dataBitCookie = 0;
    switch( dataBits ) {
        case 5:
            dataBitCookie = CS5;
            break;
        case 6:
            dataBitCookie = CS6;
            break;
        case 7:
            dataBitCookie = CS7;
            break;
        case 8:
            dataBitCookie = CS8;
            break;
        default:
            return makeErrorFmtMsgReturn(ERR_ROOT, "Invalid number of data bits: %d", dataBits );
    }
    int stopBitCookie;
    if( stopBits == 1 ) stopBitCookie = 0;
    else if( stopBits == 2 ) stopBitCookie = CSTOPB;
    else return makeErrorFmtMsgReturn(ERR_ROOT, "Invalid number of stop bits: %d", stopBits );
    int parityCookie = (parityEnable ? PARENB | (parityOdd ? PARODD : 0) : 0);

    struct termios options;
    int result;
    result = tcgetattr( fdPort, &options );
    if( result < 0 ) return makeErrorFmtMsgReturn(ERR_ROOT, "error from tcgetattr: %s", strerror(errno));
    options.c_cflag = (tcflag_t) (baudCookie | dataBitCookie | stopBitCookie | parityCookie | CLOCAL | CREAD);
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    result = tcflush( fdPort, TCIFLUSH );
    if( result < 0 ) return makeErrorFmtMsgReturn(ERR_ROOT, "error from tcflush: %s", strerror(errno));
    result = tcsetattr( fdPort, TCSANOW, &options );
    if( result < 0 ) return makeErrorFmtMsgReturn(ERR_ROOT, "error from tcsetattr: %s", strerror(errno));
    return makeOkReturn();
}


// Default baud rate synchronizer. This looks for 25 sequential characters in the normal range of ASCII codes, and
// then declares victory.  If result is Ok, additional info is true for success.
extern slReturn asciiBaudRateSynchronizer( int fdPort, int maxTimeMs, int verbosity ) {

    int count = 0;
    long long start = currentTimeMs();

    // flush any junk we might have in the receiver buffer...
    slReturn frResp = flushRx( fdPort );
    if( isErrorReturn( frResp ) )
        makeErrorMsgReturn( ERR_CAUSE( frResp ), "could not flush receiver prior to synchronization" );

    // read characters until either we appear to be synchronized or we run out of time...
    while( currentTimeMs() < start + maxTimeMs ) {

        slReturn rscResp = readSerialChar( fdPort, start + maxTimeMs - currentTimeMs() );
        if( isErrorReturn( rscResp ) )
            return makeErrorMsgReturn( ERR_CAUSE( rscResp ), "problem reading a character while synchronizing" );
        if( isWarningReturn( rscResp ) )
            return makeOkInfoReturn( bool2info( false ) );

        char c = getReturnInfoChar( rscResp );
        if( ((c >= 32) && (c <= 127)) || (c == 10) || (c == 9) || (c == 13) )
            count++;
        else
            count = 0;

        if( count >= 25 )
            return makeOkInfoReturn( bool2info( true ) );
    }
    return makeOkInfoReturn( bool2info( false ) );
}


// Determines the baud rate on the given serial port, by sequentially trying all the possible baud rates, starting
// with the highest (921600) and working down to the given minimum baud rate.  If it finds a baud rate that
// synchronizes using the given synchronizer, it stops and returns Ok with that baud rate as additional info, leaving
// the host's port set to that baud rate.  If if fails to find a baud rate, returns an error.
extern slReturn autoBaudRate( int fdPort, int minBaud, baudRateSynchronizer synchronizer, int verbosity ) {

    // we're going to allow 250 character times, or 4 seconds, whichever is greater...
    speedInfo si;
    // this code block is in the wrong place, IMHO, and should be in the loop below
    // However, a timeout of 4 seconds is OK for our purposes here, so I'm leaving it - PR
    slReturn gsiResp = getSpeedInfo( fdPort, &si );
    if( isErrorReturn( gsiResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( gsiResp ), "problem getting speed information" );
    int maxMs = (int) max_ll( 4000, si.nsChar * 250 / 1000000 );

    int bauds[] = { 921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600, 4800, 2400,
                    1800, 1200, 600, 300, 200, 150, 134, 110, 75, 50   };

    if( verbosity >= 3 ) printf( "Trying baud rates from 921600 to %d...\n", minBaud );

    for( int i = 0; (i < ARRAY_SIZE( bauds ) ) && (bauds[i] >= minBaud); i++ ) {

        if( verbosity >= 2 ) printf( "Trying %d baud...\n", bauds[i] );

        // set the baud rate for the current attempt...
        slReturn stobResp = setTermOptionsBaud( fdPort, bauds[i] );
        if( isErrorReturn( stobResp ) )
            return makeErrorFmtMsgReturn( ERR_CAUSE( stobResp ), "problem trying to set baud rate to %d", bauds[i] );

        // try to synchronize on it...
        slReturn syncResp = synchronizer( fdPort, maxMs, verbosity );
        if( isErrorReturn( syncResp ) )
            return makeErrorMsgReturn( ERR_CAUSE( syncResp ), "problem while synchronizing" );
        if( getReturnInfoBool( syncResp ) ) {
            if( verbosity >= 2 ) printf( "Synchronized on %d baud...\n", bauds[i] );
            return makeOkInfoReturn(int2info( bauds[i] ));
        }
    }
    printf( "Could not synchronize on any baud rate...\n" );
    return makeErrorFmtMsgReturn( ERR_ROOT, "could not synchronize at any baud rate from 921600 to %d", minBaud );
}
