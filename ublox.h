//
// Created by Tom Dilatush on 10/20/17.
//

#ifndef GPSCTL_UBLOX_H
#define GPSCTL_UBLOX_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/termios.h>
#include "sl_general.h"
#include "sl_bits.h"
#include "sl_buffer.h"
#include "sl_return.h"
#include "sl_serial.h"
#include "iniparser.h"

typedef enum ubxState    { Valid, NotValid, NotPresent } ubxState;

typedef struct ubxChecksum {
    byte ck_a;
    byte ck_b;
} ubxChecksum;

typedef struct ubxType {
    byte class;
    byte id;
} ubxType;

typedef struct {
    int          year;
    int          month;
    int          day;
    int          hour;
    int          minute;
    double       second;
    int          nanoseconds_correction;
    unsigned int time_accuracy_ns;
    bool         time_resolved;
    bool         date_valid;
    bool         time_valid;
    bool         fix_is_3d;
    bool         fix_valid;
    unsigned int number_of_satellites_used;
    double       longitude_deg;
    double       latitude_deg;
    int          height_above_ellipsoid_mm;
    int          height_above_sea_level_mm;
    unsigned int height_accuracy_mm;
    unsigned int horizontal_accuracy_mm;
    int          ground_speed_mm_s;
    unsigned int ground_speed_accuracy_mm_s;
    double       heading_deg;
    double       heading_accuracy_deg;

} ubxFix;

typedef struct {
    char *software;
    char *hardware;
    char **extensions;  // terminated by NULL entry...
    int number_of_extensions;
} ubxVersion;


typedef enum { GPS, SBAS, Galileo, BeiDou, IMES, QZSS, GLONASS } gnssID;
typedef enum { Portable, Stationary = 2, Pedestrian, Automotive, Sea, Air1G, Air2G, Air4G, Watch } dynModel;
typedef enum { Only2D = 1, Only3D, Auto2D3D } fixMode;
typedef enum { AutoUTC, USNO_UTC = 3, GLONASS_UTC = 6, BeiDou_UTC } utcType;
typedef enum { tgUTC, tgGPS, tgGLONASS, tgBeiDou, tgGalileo } timegridType;
typedef enum { fixUTC, fixGPS, fixGLONASS, fixBeiDou, fixGalileo } fixTimeRefType;
typedef enum { pmFull, pmBalanced, pmInterval, pmAggressive1Hz, pmAggressive2Hz, pmAggressive4Hz, pmInvalid=0xFF } powerModeType;
typedef enum { signalNone, signalSearching, signalAcquired, signalUnusable, signalCodeLocked, signalCodeCarrierLocked } signalQuality;
typedef enum { healthUnknown, healthOk, healthBad } satelliteHealth;
typedef enum { osNone, osEphemeris, osAlmanac, osAssistNowOffline, osAssistNowAutonomous, osOther } orbitSource;
typedef enum { GGA, GLL, GSA, GSV, RMC, VTG, GRS, GST, ZDA } nmeaMSG;

typedef struct {
    gnssID          gnssID;                 // GNSS that this satellite belongs to
    uint8_t         satelliteID;            // the GNSS-assigned satellite number
    uint8_t         cno;                    // the carrier-to-noise signal ratio in dBHz
    int8_t          elevation;              // elevation in degrees (0 is horizon, +/- 90 degrees
    int16_t         azimuth;                // azimuth in degrees (0-360, 0 is due north)
    double          pseudoRangeResidualM;   // pseudo range residual, in meters
    signalQuality   signalQuality;          // signal quality
    bool            used;                   // true if this satellite is being used for navigation
    satelliteHealth health;                 // current state of satellite health
    bool            diffCorr;               // true if differential correction data is available for this satellite
    bool            smoothed;               // true if a carrier-smoothed pseudorange is being used for this satellite
    orbitSource     orbitSource;            // the source for the orbital information for this satellite
    bool            haveEphemeris;          // true if the GPS has an ephemeris for this satellite
    bool            haveAlmanac;            // true if the GPS has an almanac for this satellite
    bool            haveAssistNowOff;       // true if the GPS has AssistNow offline data for this satellite
    bool            haveAssistNowAuto;      // true if the GPS has AssistNow autonomous data for this satellite
    bool            sbasCorrUsed;           // true if the GPS used SBAS corrections for this satellite
    bool            rtcmCorrUsed;           // true if the GPS used RTCM corrections for this satellite
    bool            prCorrUsed;             // true if the GPS used pseudorange corrections for this satellite
    bool            crCorrUsed;             // true if the GPS used carrier range corrections for this satellite
    bool            doCorrUsed;             // true if the GPS used range rate (Doppler) corrections for this satellite
} ubxSatellite;

typedef struct {
    int numberOfSatellites;
    ubxSatellite* satellites;
} ubxSatellites;

typedef struct {
    gnssID id;
    int minChnnls;
    int maxChnnls;
    bool enabled;
    int sigConfig;
} ubxGNSSConfig;

typedef struct {
    bool           antPwr;                  // true if antenna power is enabled
    bool           antShrtDet;              // true if antenna short circuit detection is enabled
    bool           antOpenDet;              // true if antenna open circuit detection is enabled
    bool           antPwrDwnOnShrt;         // true if power down antenna on short detected
    bool           antAutoRec;              // true if automatically recover from antenna short circuit
    int            trkChnnls;               // number of hardware tracking channels
    int            gnssRecs;                // number of GNSS records in gnss
    ubxGNSSConfig* gnss;                    // configurations for each GNSS
    dynModel       model;                   // navigation engine dynamic model
    fixMode        mode;                    // navigation engine fix mode
    double         fixedAltM;               // fixed altitude for 2D fix mode
    double         fixedAltVarM2;           // fixed altitude variance for 2D mode
    int            minElevDeg;              // minimum elevation for a GNSS satellite to be used in a fix
    double         pDoP;                    // position DoP mask
    double         tDoP;                    // time DoP mask
    uint16_t       pAccM;                   // position accuracy mask
    uint16_t       tAccM;                   // time accuracy mask
    uint8_t        staticHoldThreshCmS;     // static hold threshold
    uint8_t        dgnssTimeoutS;           // DGNSS timeout
    uint8_t        cnoThreshNumSVs;         // number of satellites required to have C/NO above cnoThresh for a fix to be attempted
    uint8_t        cnoThreshDbHz;           // C/NO threshold for deciding whether to attempt a fix
    uint16_t       staticHoldMaxDistM;      // static hold distance threshold (before quitting static hold)
    utcType        utcStandard;             // UTC standard used
    int16_t        antCableDelayNs;         // antenna cable delay in nanoseconds
    int16_t        rfGroupDelayNs;          // delay in RF amplifier (such as LNA on antenna, if present)
    uint32_t       freqPeriod;              // frequency (Hz) or period time (microseconds), depending on "isFreq"
    uint32_t       freqPeriodLock;          // frequency (Hz) or period time (microseconds) when locked, depending on "isFreq" and "lockedOtherSet"
    uint32_t       pulseLenRatio;           // pulse length (microseconds) or duty cycle (2^32), depending on "isLength"
    uint32_t       pulseLenRatioLock;       // pulse length (microseconds) or duty cycle (2^32) when locked, depending on "isFreq" and "lockedOtherSet"
    int32_t        userConfigDelay;         // user configurable time pulse delay
    bool           timePulse0Enabled;       // true if time pulse zero is enabled
    bool           lockGpsFreq;             // true to synchronize time pulse to GNSS as soon as GNSS time is valid
    bool           lockedOtherSet;          // true to use "Lock" variants of frequency when GNSS locked, non-"Lock" variants otherwise
    bool           isFreq;                  // true to interpret "freqPeriod" values as frequency rather than period
    bool           isLength;                // true to interpret "pulseLen" values as period rather than duty cycle
    bool           alignToTow;              // true to align leading edge of time pulse to top of second
    bool           polarity;                // true for rising edge at top of second, false for falling edge
    timegridType   gridUtcTnss;             // time grid for time pulse
    uint16_t       measRateMs;              // elapsed time between GNSS measurements, in milliseconds
    uint16_t       navRate;                 // number of measurements per navigation solution
    fixTimeRefType timeRef;                 // time system that navigation solutions are aligned to
    powerModeType  powerSetup;              // power setup
    uint16_t       powerIntervalSecs;       // power-on interval, in seconds, if in interval mode
    uint16_t       powerOnTimeSecs;         // power on time, in seconds, if in interval mode
    bool           nmeaEnabled;             // NMEA output enabled
    uint8_t        nmeaVersion;             // NMEA Version - 0x21, 0x23, 0x40, or 0x41
    bool           GGA;                     // NMEA output sentence types
    bool           GLL;
    bool           GSA;
    bool           GSV;
    bool           RMC;
    bool           VTG;
    bool           GRS;
    bool           GST;
    bool           ZDA;	
} ubxConfig;

typedef struct {
	byte		filter;
	uint8_t		nmeaVersion;			// set to 0x41 for Galileo
	uint8_t		numSV;
	byte		flags;
	byte		gnsToFilter1;
	byte		gnsToFilter2;
	byte		gnsToFilter3;
	byte		gnsToFilter4;
	uint8_t		svNumbering;
	uint8_t		mainTalkerID;
	uint8_t		gsvTalkerID;
	uint8_t		version;				// set to 1
	byte		bdsTalkerID1;
	byte		bdsTaklerID2;
	uint8_t		reserved1;
	uint8_t		reserved2;
	uint8_t		reserved3;
	uint8_t		reserved4;
	uint8_t		reserved5;
	uint8_t		reserved6;
} ubxNMEAConfig;

typedef struct {
	uint8_t		msgType;				// set to 0xFA
	uint8_t		version;
	uint8_t		port1;
	uint8_t		port2;
	uint8_t		port3;
	uint8_t		port4;
	uint8_t		port5;
	uint8_t		port6;
} ubxNMEAmsg;

char* getGnssName( gnssID id );
char* getDynamicModelName( dynModel model );
char* getFixModeName( fixMode mode );
char* getUTCTypeName( utcType utc );
char* getTimeGridTypeName( timegridType type );
char* getFixTimeRefName( fixTimeRefType type );
char* getPowerModeName( powerModeType type );
char* getSignalQuality( signalQuality qual );
char* getSatelliteHealth( satelliteHealth health );
char* getOrbitSource( orbitSource source );

slReturn ubxGetVersion( int fdPort, int verbosity, ubxVersion* version );
slReturn ubxGetSatellites( int fdPort, int verbosity, ubxSatellites* satellites );
slReturn ubxSaveConfig( int fdPort, int verbosity );
slReturn ubxGetFix( int fdPort, int verbosity, ubxFix* fix );
slReturn ubxReset( int fdPort, int verbosity );
int getConfigBool( const char *key, int notfound );
int getConfigInt( const char *key, long notfound, long min, long max );
slReturn ubxConfigNMEAVersion( int fdPort, int verbosity, uint8_t nmeaVersion );
slReturn ubxConfigSatellites( int fdPort, int verbosity );
slReturn ubxConfigTimePulse( int fdPort, int verbosity );
slReturn ubxConfigNavEngine( int fdPort, int verbosity );
slReturn ubxConfigForTiming( int fdPort, int verbosity );
slReturn ubxGetConfig( int fdPort, int verbosity, ubxConfig* config );
slReturn ubxChangeBaudRate( int fdPort, unsigned int newBaudRate, int verbosity );
slReturn ubxSetNMEAData( int fdPort, int verbosity, bool nmeaOn );
slReturn isNMEAmsgEnabled( int fdPort, int verbosity, nmeaMSG messageID );
slReturn ubxEnableNMEAMsg( int fdPort, int verbosity, nmeaMSG messageID, bool enable );
slReturn ubxSynchronizer( int fdPort, int maxTimeMs, int verbosity );

extern dictionary *gpsctlConf;

#endif //GPSCTL_UBLOX_H
