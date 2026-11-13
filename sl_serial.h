//
// Created by Tom Dilatush on 10/22/17.
//

#ifndef GPSCTL_SL_SERIAL_H
#define GPSCTL_SL_SERIAL_H

#include <stdbool.h>
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
slReturn readSerialChar( int fdPort, long long msTimeout );
bool portHungUp( int fdPort );
slReturn getBaudRate( int cookie );
slReturn flushRx( int fdPort );
slReturn setTermOptions( int fdPort, int baud, int dataBits, int stopBits, bool parityEnable, bool odd );
slReturn setTermOptionsBaud( int fdPort, int baud );

#endif //GPSCTL_SL_SERIAL_H
