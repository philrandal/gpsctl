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
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <sys/stat.h>
#include <errno.h>
#include "sl_general.h"
#include "sl_serial.h"
#include "sl_bits.h"
#include "sl_return.h"


// Returns speed information for the given port.
extern slReturn getSpeedInfo( int fdPort, speedInfo* result ) {
    struct termios options;
    if( tcgetattr( fdPort, &options ) != 0 )
        return makeErrorMsgReturn( ERR_ROOT, "could not retrieve terminal attributes" );
    slReturn brResp = getBaudRate((int) cfgetispeed( &options ));
    if( isErrorReturn( brResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( brResp ), "could not get baud rate" );
    result->baudRate =  getReturnInfoInt32( brResp );
    freeReturn( brResp );
    result->nsBit = 1000000000 / result->baudRate;
    int stopBits = (options.c_cflag & CSTOPB) ? 2 : 1;
    // CSIZE is 0..3 for CS5..CS8, so add 5 to get the actual number of data bits...
    int dataBits = (int) getBitField_slBits( options.c_cflag, CSIZE ) + 5;
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
    if( r != 0 ) {
        close( fd );
        return makeErrorFmtMsgReturn( ERR_ROOT, "can't retrieve terminal attributes for \"%s\": %s", deviceName,
                                     strerror(errno) );
    }

    close( fd );

    return makeOkReturn();
}


// Reads a single character from the given serial port, waiting at most the given number of milliseconds.  On an Ok
// response, returns the character read as a char in additional info.  Returns a warning if it timed out.
extern slReturn readSerialChar( int fdPort, long long msTimeout ) {

    unsigned char c;
    long long startTime = currentTimeMs();

    // Wait for data with poll() rather than spinning on read() with a short sleep.  The old loop called tcgetattr()
    // (via getSpeedInfo) on every call and woke every half character time (about 5 microseconds at 921600 baud),
    // which burned CPU and added latency.  poll() sleeps until a character arrives or the time runs out.
    while( true ) {

        long long remainingMs = msTimeout - (currentTimeMs() - startTime);
        if( remainingMs <= 0 ) break;  // we ran out of time...

        struct pollfd pfd = { .fd = fdPort, .events = POLLIN, .revents = 0 };
        int pr = poll( &pfd, 1, (int) (remainingMs > INT_MAX ? INT_MAX : remainingMs) );

        if( pr < 0 ) {
            if( errno == EINTR ) continue;  // interrupted by a signal; just wait again for the time that's left...
            return makeErrorFmtMsgReturn( ERR_ROOT, "poll() returned error: %s", strerror( errno ) );
        }
        if( pr == 0 ) break;  // timed out waiting for data...

        // something is ready (a character, or an error/hangup that read() will report), so try to read a character...
        ssize_t rx_length = read( fdPort, &c, 1 );

        // if we read a character, return it...
        if( rx_length == 1 )
            return makeOkInfoReturn( char2info(c) );

        // A zero-length read means the kernel has hung the port up: the device went away (a USB serial adapter
        // unplugged or reset, for instance).  Say that plainly, because further up this otherwise looks like a
        // baud rate or protocol problem, which it isn't.  poll() would report it again immediately, so don't spin
        // on it either...
        if( rx_length == 0 )
            return makeErrorMsgReturn( ERR_ROOT, "serial port hung up (device disconnected?)" );

        // a spurious wakeup or a signal is not an error; anything else is...
        if( (errno != EAGAIN) && (errno != EINTR) )
            return makeErrorFmtMsgReturn( ERR_ROOT, "read() returned error: %s", strerror( errno ) );

        // poll() also reports a hangup or a fatal port error, sometimes without any data to go with it: if that
        // is all there is, there is nothing left to wait for...
        if( pfd.revents & (POLLERR | POLLHUP | POLLNVAL) )
            return makeErrorMsgReturn( ERR_ROOT, "serial port hung up (device disconnected?)" );
    }

    // if we get here, then we timed out...
    return makeWarningReturn();
}


// Writes all of the given data to the given serial port.  The port is non-blocking, so write() can take only part of
// the data - or none of it at all, when the buffer is full - and neither is an error: the rest still has to go out,
// because a half-written message leaves the receiver waiting for an end that never comes.  Waits for the port to drain
// a little whenever it is full, and gives up only on a real error, or if nothing at all can be written for a second.
#define WRITE_WAIT_MS 1000
extern slReturn writeSerialData( int fdPort, const void* data, size_t length ) {

    if( fdPort < 0 ) return makeErrorMsgReturn( ERR_ROOT, "invalid serial port file descriptor" );

    const unsigned char* ptr = data;
    size_t remaining = length;
    long long startTime = currentTimeMs();

    while( remaining > 0 ) {

        ssize_t written = write( fdPort, ptr, remaining );

        // if the port took some of it, keep going with the rest.  Data is moving, so restart the clock: a slow but
        // working port must not time out while it is still making progress...
        if( written > 0 ) {
            ptr += written;
            remaining -= (size_t) written;
            startTime = currentTimeMs();
            continue;
        }

        // a zero-length write means the port took nothing after all; together with an interrupted or would-block
        // write, that is something to wait for.  Anything else is a real error...
        if( (written < 0) && (errno != EAGAIN) && (errno != EINTR) )
            return makeErrorFmtMsgReturn( ERR_ROOT, "write() returned error: %s", strerror( errno ) );

        // the port has no room right now, so wait for it to drain...
        long long remainingMs = WRITE_WAIT_MS - (currentTimeMs() - startTime);
        if( remainingMs <= 0 )
            return makeErrorMsgReturn( ERR_ROOT, "timed out waiting to write to serial port" );

        struct pollfd pfd = { .fd = fdPort, .events = POLLOUT, .revents = 0 };
        int pr = poll( &pfd, 1, (int) remainingMs );
        if( pr < 0 ) {
            if( errno == EINTR ) continue;  // interrupted by a signal; just wait again for the time that's left...
            return makeErrorFmtMsgReturn( ERR_ROOT, "poll() returned error: %s", strerror( errno ) );
        }
        if( pr == 0 ) return makeErrorMsgReturn( ERR_ROOT, "timed out waiting to write to serial port" );

        // a hangup or a fatal port error means the device has gone away, and no amount of waiting will bring it back...
        if( pfd.revents & (POLLERR | POLLHUP | POLLNVAL) )
            return makeErrorMsgReturn( ERR_ROOT, "serial port hung up (device disconnected?)" );
    }

    return makeOkReturn();
}


// Reports whether the given serial port has been hung up: the device has gone away (a USB serial adapter unplugged or
// reset, for instance), or the far end of a pty pair has closed.  A hung-up port never produces data again and cannot
// be reconfigured, so the only way back is to close it and open the device again.
extern bool portHungUp( int fdPort ) {

    if( fdPort < 0 ) return true;

    // poll() reports a hangup, a fatal port error, or a bad file descriptor straight away, and requests nothing.  A
    // signal can interrupt it, so retry rather than concluding from an interrupted call that the port is healthy...
    struct pollfd pfd = { .fd = fdPort, .events = 0, .revents = 0 };
    int pr;
    do {
        pr = poll( &pfd, 1, 0 );
    } while( (pr < 0) && (errno == EINTR) );

    if( (pr >= 0) && ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) ) return true;

    // A hung-up port refuses every ioctl with EIO, so ask it directly as well.  This is what distinguishes a healthy
    // port from one that poll() couldn't tell us about: a failed poll() must never be read as "the port is fine",
    // because that would silently turn off the caller's recovery...
    struct termios options;
    return tcgetattr( fdPort, &options ) != 0;
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
        return makeErrorMsgReturn( ERR_CAUSE( frResp ), "could not flush receiver prior to synchronization" );

    // read characters until either we appear to be synchronized or we run out of time...
    while( currentTimeMs() < start + maxTimeMs ) {

        slReturn rscResp = readSerialChar( fdPort, start + maxTimeMs - currentTimeMs() );
        if( isErrorReturn( rscResp ) )
            return makeErrorMsgReturn( ERR_CAUSE( rscResp ), "problem reading a character while synchronizing" );
        if( isWarningReturn( rscResp ) ) {
            freeReturn( rscResp );
            return makeOkInfoReturn( bool2info( false ) );
        }

        char c = getReturnInfoChar( rscResp );
        freeReturn( rscResp );
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

    int bauds[] = { 921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600, 4800, 2400,
                    1800, 1200, 600, 300, 200, 150, 134, 110, 75, 50   };

    // at low verbosity only the root message is printed, and it doesn't say which rates were tried, so say it
    // here - this is the only place that can...
    if( verbosity >= 1 ) printf( "Trying baud rates from 921600 to %d...\n", minBaud );

    for( int i = 0; (i < ARRAY_SIZE( bauds ) ) && (bauds[i] >= minBaud); i++ ) {

        if( verbosity >= 2 ) printf( "Trying %d baud...\n", bauds[i] );

        // set the baud rate for the current attempt...
        slReturn stobResp = setTermOptionsBaud( fdPort, bauds[i] );
        if( isErrorReturn( stobResp ) )
            return makeErrorFmtMsgReturn( ERR_CAUSE( stobResp ), "problem trying to set baud rate to %d", bauds[i] );

        // Now that the port is at the rate we are about to try, work out how long to give the synchronizer: 250
        // character times, or 4 seconds, whichever is greater.  This has to be done here, after the rate change,
        // because it is the rate being tried - not whatever rate the port was left at - that says how long the
        // characters take to arrive.  At 50 baud, 25 characters take five seconds, so a four-second budget would
        // never see them, and the slowest rates could never synchronize...
        speedInfo si;
        slReturn gsiResp = getSpeedInfo( fdPort, &si );
        if( isErrorReturn( gsiResp ) )
            return makeErrorMsgReturn( ERR_CAUSE( gsiResp ), "problem getting speed information" );
        freeReturn( gsiResp );
        long long maxMsLL = max_ll( 4000LL, (long long) si.nsChar * 250 / 1000000 );
        int maxMs = (maxMsLL > INT_MAX) ? INT_MAX : (int) maxMsLL;

        // try to synchronize on it...
        slReturn syncResp = synchronizer( fdPort, maxMs, verbosity );
        if( isErrorReturn( syncResp ) )
            return makeErrorMsgReturn( ERR_CAUSE( syncResp ), "problem while synchronizing" );
        bool synchronized = getReturnInfoBool( syncResp );
        freeReturn( syncResp );
        if( synchronized ) {
            if( verbosity >= 1 ) printf( "Synchronized on %d baud...\n", bauds[i] );
            return makeOkInfoReturn(int2info( bauds[i] ));
        }
    }
    return makeErrorFmtMsgReturn( ERR_ROOT, "could not synchronize at any baud rate from 921600 to %d", minBaud );
}
