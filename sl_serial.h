//
// Created by Tom Dilatush on 10/22/17.
//

#ifndef GPSCTL_SL_SERIAL_H
#define GPSCTL_SL_SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include "sl_return.h"

typedef slReturn baudRateSynchronizer( int fdPort, int maxTimeMs, int verbosity );

typedef struct speedInfo {
    int baudRate;  // baud rate (in bits/second)
    int nsBit;     // number of nanoseconds to receive or transmit a bit
    int nsChar;    // number of nanoseconds to receive or transmit a character
} speedInfo;

int getBaudRateCookie( int baudRate );
slReturn asciiBaudRateSynchronizer( int fdPort, int maxTimeMs, int verbosity );
slReturn autoBaudRate( int fdPort, int minBaud, baudRateSynchronizer synchronizer, int verbosity );
slReturn getSpeedInfo( int fdPort, speedInfo* );
slReturn verifySerialDevice( const char *deviceName );
slReturn checkSerialPortNotShared( const char *deviceName );
slReturn readSerialChar( int fdPort, long long msTimeout );
slReturn writeSerialData( int fdPort, const void* data, size_t length );
bool portHungUp( int fdPort );
slReturn getBaudRate( int cookie );
slReturn flushRx( int fdPort );
slReturn setTermOptions( int fdPort, int baud, int dataBits, int stopBits, bool parityEnable, bool odd );
slReturn setTermOptionsBaud( int fdPort, int baud );

// After a host baud change: discard input until the line has been quiet for quietMs (at most maxMs in all).
#define QUIET_LINE_MS     20
#define QUIET_LINE_MAX_MS 1000
#define AUTOBAUD_COOLDOWN_MS 2000  // idle time after a failed autobaud sweep, before a retry
slReturn waitForQuietLine( int fdPort, int quietMs, int maxMs );

#endif //GPSCTL_SL_SERIAL_H
