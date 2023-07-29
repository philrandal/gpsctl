/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * G P S C T L
 * (GPS control)
 *
 * A utility for querying, configuring, and using a U-Blox GPS on Linux.
 *
 * This program was tested ONLY on a Raspberry Pi 3B, running Jessie, with a Uputronics board that uses a
 * U-Blox M8 GPS module.
 *
 * Created: October 18, 2017
 *  Author: Tom Dilatush <tom@dilatush.com>
 *  Github:
 * License: MIT
 *
 * Copyright 2017 Tom Dilatush
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so.
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 * the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE A
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// TODO: make sure we free any allocated slBuffers!
// TODO: add needed printfs for verbose mode...
// TODO: troll all headers, inserting parameter names and return value comments

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#define _GNU_SOURCE

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
// #include <wiringPi.h>
#include <math.h>
#include "sl_return.h"
#include "sl_options.h"
#include "sl_general.h"
#include "sl_serial.h"
#include "ublox.h"
#include "cJSON.h"

typedef enum { fixQuery, versionQuery, configQuery, satelliteQuery } queryType;
typedef enum { syncNone, syncASCII, syncNMEA, syncUBX } syncType;
typedef struct { char* name; syncType type; } syncTypeRec;

static syncTypeRec syncTypeRecs[] = {
        { "ascii", syncASCII },
        { "nmea",  syncNMEA  },
        { "ubx",   syncUBX   }
};

struct clientData_slOptions {
    int verbosity;
    syncType syncMethod;
    const char *port;
    int baud;
    int minBaud;
    int newbaud;
    bool report;
    bool echo;
    int nmea;
    bool json;
    int echoSeconds;
    queryType queryType;
    bool error;
    int fdPort;
    bool ubxSynchronized;
};

dictionary *gpsctlConf;

// Set the sync method to that specified in the argument.  Returns Ok if there's no argument (and do nothing) or the
// argument was recognized; or an Error if there was an unrecognizable argument.
static slReturn setSyncMethod(const char* arg, clientData_slOptions* clientData) {

    // if no argument was supplied, don't do anything at all...
    if (arg == NULL) return makeOkReturn();

    // otherwise, try to match the argument...
    for (int i = 0; i < ARRAY_SIZE(syncTypeRecs); i++) {
        if (0 == strncmp(arg, syncTypeRecs[i].name, strlen(arg))) {
            clientData->syncMethod = syncTypeRecs[i].type;
            return makeOkReturn();
        }
    }

    // if we get here, we couldn't find the method, so do nothing but return a problem...
    return makeErrorFmtMsgReturn(ERR_ROOT, "\"%s\" is not a recognized synchronization method", arg);
}


// NMEA baud rate synchronizer. This looks for a complete, valid NMEA sentence and then declares victory.
// Returns Ok with true if synchronized, false if timed out.
static slReturn nmeaBaudRateSynchronizer(int fdPort, int maxTimeMs, int verbosity) {

    bool inLine = false;
    char last[3];
    int cksm = 0;
    long long start = currentTimeMs();

    // flush any junk we might have in the receiver buffer...
    slReturn frResp = flushRx(fdPort);
    if (isErrorReturn(frResp))
        makeErrorMsgReturn(ERR_CAUSE(frResp), "could not flush receiver prior to synchronization");

    // read characters until either we appear to be synchronized or we run out of time...
    while (currentTimeMs() < start + maxTimeMs) {

        slReturn rscResp = readSerialChar(fdPort, start + maxTimeMs - currentTimeMs());
        if (isErrorReturn(rscResp))
            return makeErrorMsgReturn(ERR_CAUSE(rscResp), "problem reading a character while synchronizing");
        if (isWarningReturn(rscResp))
            return makeOkInfoReturn(bool2info(false));

        char c = getReturnInfoChar(rscResp);
        if (inLine) {
            if ((c == '\r') || (c == '\n')) {
                if (last[0] == '*') {
                    int un = hex2int(last[1]);
                    if (un >= 0) {
                        int ln = hex2int(last[2]);
                        if (ln >= 0) {
                            int ck = (un << 4) | ln;

                            // correct checksum for the last three characters received...
                            cksm ^= last[0];
                            cksm ^= last[1];
                            cksm ^= last[2];

                            if (ck == cksm) {
                                return makeOkInfoReturn(bool2info(true));
                            }
                        }
                    }
                }
                inLine = false;
            }
            else {
                cksm ^= c;
                last[0] = last[1];
                last[1] = last[2];
                last[2] = c;
            }
        }
        else if (c == '$') {
            inLine = true;
            cksm = 0;
        }
    }
    return makeOkInfoReturn(bool2info(false));
}


// Attempt to synchronize, using the given method at the current host baud rate.  On an Ok response, returns true
// or false for synchronization success as a bool in additional info.
static slReturn syncSerial(const syncType type, clientData_slOptions* clientData) {

    // we're going to allow 250 character times, or 1.5 seconds, whichever is greater...
    speedInfo si;
    slReturn gsiResp = getSpeedInfo(clientData->fdPort, &si);
    if (isErrorReturn(gsiResp))
        return makeErrorMsgReturn(ERR_CAUSE(gsiResp), "problem getting speed information");
    int maxMs = (int) max_ll(1500, si.nsChar * 250 / 1000000);

    switch (type) {
        case syncNMEA:  return nmeaBaudRateSynchronizer(clientData->fdPort, maxMs, clientData->verbosity);
        case syncASCII: return asciiBaudRateSynchronizer(clientData->fdPort, maxMs, clientData->verbosity);
        case syncUBX:
            if (clientData->ubxSynchronized) return makeOkInfoReturn(bool2info(true));
            slReturn ubsxResp = ubxSynchronizer(clientData->fdPort, maxMs, clientData->verbosity);
            if (isErrorReturn(ubsxResp)) return ubsxResp;
            if (getReturnInfoBool(ubsxResp)) clientData->ubxSynchronized = true;
            return ubsxResp;
        default: return makeErrorFmtMsgReturn(ERR_ROOT, "unknown synchronization type: %d", type);
    }
}


// Outputs a fix query, in English or JSON.
static slReturn doFixQuery(const clientData_slOptions* clientData) {

    ubxFix fix = {0}; // zeroes all elements...
    slReturn result = ubxGetFix(clientData->fdPort, clientData->verbosity, &fix);
    double msl_height_feet = 0.00328084 * fix.height_above_sea_level_mm;
    double height_accuracy_feet = 0.00328084 * fix.height_accuracy_mm;
    double horizontal_accuracy_feet = 0.00328084 * fix.horizontal_accuracy_mm;
    double speed_mph = fix.ground_speed_mm_s * 0.00223694;
    double speed_accuracy_mph = fix.ground_speed_accuracy_mm_s * 0.00223694;
    if (isErrorReturn(result))
        return makeErrorMsgReturn(ERR_CAUSE(result), "Problem obtaining fix from GPS");

    if (clientData->json) {
        cJSON* root   = cJSON_CreateObject();
        cJSON* objFix = cJSON_CreateObject();
        cJSON* time   = cJSON_CreateObject();

        cJSON_AddItemToObject(root, "fix", objFix);
        cJSON_AddNumberToObject(objFix, "latitude_deg", fix.latitude_deg);
        cJSON_AddNumberToObject(objFix, "longitude_deg", fix.longitude_deg);
        cJSON_AddNumberToObject(objFix, "height_above_ellipsoid_mm", fix.height_above_ellipsoid_mm);
        cJSON_AddNumberToObject(objFix, "height_above_mean_sea_level_mm", fix.height_above_sea_level_mm);
        cJSON_AddNumberToObject(objFix, "horizontal_accuracy_mm", fix.horizontal_accuracy_mm);
        cJSON_AddNumberToObject(objFix, "height_accuracy_mm", fix.height_accuracy_mm);
        cJSON_AddNumberToObject(objFix, "ground_speed_mm_sec", fix.ground_speed_mm_s);
        cJSON_AddNumberToObject(objFix, "ground_speed_accuracy_mm_sec", fix.ground_speed_accuracy_mm_s);
        cJSON_AddNumberToObject(objFix, "heading_deg", fix.heading_deg);
        cJSON_AddNumberToObject(objFix, "heading_accuracy_deg", fix.heading_accuracy_deg);
        cJSON_AddBoolToObject(objFix, "valid", fix.fix_valid);
        cJSON_AddBoolToObject(objFix, "3d", fix.fix_is_3d);

        cJSON_AddNumberToObject(root,   "number_of_satellites_used", fix.number_of_satellites_used);

        cJSON_AddItemToObject(root, "time", time);
        cJSON_AddNumberToObject(time, "year", fix.year);
        cJSON_AddNumberToObject(time, "month", fix.month);
        cJSON_AddNumberToObject(time, "day", fix.day);
        cJSON_AddNumberToObject(time, "hour", fix.hour);
        cJSON_AddNumberToObject(time, "minute", fix.minute);
        cJSON_AddNumberToObject(time, "second", fix.second);
        cJSON_AddNumberToObject(time, "accuracy_ns", fix.time_accuracy_ns);
        cJSON_AddBoolToObject(time, "valid", fix.time_valid);
        cJSON_AddBoolToObject(time, "resolved", fix.time_resolved);

        char *jsonStr = cJSON_PrintUnformatted(root);
        printf("%s\n", jsonStr);

        cJSON_Delete(root);
    }
    else {
        if (fix.time_resolved && fix.time_valid && fix.date_valid)
            printf(" Time (UTC): %04d-%02d-%02d %02d:%02d:%02.0f (yyyy-mm-dd hh:mm:ss)\n",
                    fix.year, fix.month, fix.day, fix.hour, fix.minute, fix.second);
        else
            printf(" Time (UTC): not resolved or not valid\n");
        if (fix.fix_valid) {
            printf("   Latitude: %12.8f %c\n", fabs(fix.latitude_deg),  (fix.latitude_deg < 0)  ? 'S' : 'N');
            printf("  Longitude: %12.8f %c\n", fabs(fix.longitude_deg), (fix.longitude_deg < 0) ? 'W' : 'E');
            if (fix.fix_is_3d)
                printf("   Altitude: %.3f feet\n", msl_height_feet);
            else
                printf("   Altitude: unknown\n");
            printf("     Motion: %.3f mph at %.3f degrees heading\n", speed_mph, fix.heading_deg);
        }
        else
            printf("        Fix: invalid\n");
        printf(" Satellites: %d used for computing this fix\n", fix.number_of_satellites_used);
        printf("   Accuracy: time (%d ns), height (+/-%.3f feet), position (+/-%.3f feet), heading(+/-%.3f degrees), speed(+/-%.3f mph)\n",
                fix.time_accuracy_ns, height_accuracy_feet, horizontal_accuracy_feet, fix.heading_accuracy_deg, speed_accuracy_mph);
    }

    return makeOkReturn();
}


// Outputs a version query, in English or JSON.
static slReturn doVersionQuery(const clientData_slOptions* clientData) {

    ubxVersion version = {0};  // zeroes all elements...
    slReturn result = ubxGetVersion(clientData->fdPort, clientData->verbosity, &version);
    if (isErrorReturn(result))
        return makeErrorMsgReturn(ERR_CAUSE(result), "Problem obtaining version information from GPS");

    if (clientData->json) {

        cJSON *root = cJSON_CreateObject();
        cJSON_AddItemToObject(root, "software_version", cJSON_CreateString(version.software));
        cJSON_AddItemToObject(root, "hardware_version", cJSON_CreateString(version.hardware));
        cJSON_AddItemToObject(root, "extensions",
                              cJSON_CreateStringArray((const char**) version.extensions, version.number_of_extensions));

        char *jsonStr = cJSON_PrintUnformatted(root);
        printf("%s\n", jsonStr);

        cJSON_Delete(root);
    }
    else {
        printf("Software version: %s\n", version.software);
        free(version.software);
        printf("Hardware version: %s\n", version.hardware);
        free(version.hardware);
        char **ptr = version.extensions;
        while ((*ptr) != NULL) {
            printf("       Extension: %s\n", *ptr);
            free((*ptr));
            ptr++;
        }
    }

    return makeOkReturn();
}


// comparison function for satellite search...
int cmpSatellite(const void* a, const void* b) {

    ubxSatellite* c = (ubxSatellite*) a;
    ubxSatellite* d = (ubxSatellite*) b;

    if (c->used && !d->used) return -1;
    if (d->used && !c->used) return 1;
    return d->cno - c->cno;
}


// Outputs a satellite query, in English or JSON.
static slReturn doSatelliteQuery(const clientData_slOptions* clientData) {

    // get the satellite data from the GPS...
    ubxSatellites satellites = {0};  // zeroes all elements...
    slReturn ugsResp = ubxGetSatellites(clientData->fdPort, clientData->verbosity, &satellites);
    if (isErrorReturn(ugsResp))
        return makeErrorMsgReturn(ERR_CAUSE(ugsResp), "Problem obtaining satellites information from GPS");

    // sort the result by used or not, then in descending order of CNo...
    qsort(satellites.satellites, (unsigned) satellites.numberOfSatellites, sizeof(ubxSatellite), cmpSatellite);

    if (clientData->json) {

        cJSON* root = cJSON_CreateObject();
        cJSON* sats = cJSON_CreateArray();

        cJSON_AddNumberToObject(root, "number_of_satellites", satellites.numberOfSatellites);
        cJSON_AddItemToObject(root, "satellites", sats);

        for (int i = 0; i < satellites.numberOfSatellites; i++) {
            ubxSatellite* s = satellites.satellites + i;
            cJSON* sat = cJSON_CreateObject();
            cJSON_AddItemToArray(sats, sat);
            cJSON_AddStringToObject(sat, "gnssID", getGnssName(s->gnssID));
            cJSON_AddNumberToObject(sat, "satelliteID",                       s->satelliteID);
            cJSON_AddNumberToObject(sat, "CNo",                               s->cno);
            cJSON_AddNumberToObject(sat, "elevation",                         s->elevation);
            cJSON_AddNumberToObject(sat, "azimuth",                           s->azimuth);
            cJSON_AddNumberToObject(sat, "pseudoRangeResidual_meters",        s->pseudoRangeResidualM);
            cJSON_AddStringToObject(sat, "signalQuality", getSignalQuality(s->signalQuality));
            cJSON_AddBoolToObject(sat, "used",                                s->used);
            cJSON_AddStringToObject(sat, "satelliteHealth", getSatelliteHealth(s->health));
            cJSON_AddBoolToObject(sat, "diffCorr",                            s->diffCorr);
            cJSON_AddBoolToObject(sat, "smoothed",                            s->smoothed);
            cJSON_AddStringToObject(sat, "orbitSource", getOrbitSource(s->orbitSource));
            cJSON_AddBoolToObject(sat, "haveEphemeris",                       s->haveEphemeris);
            cJSON_AddBoolToObject(sat, "haveAlmanac",                         s->haveAlmanac);
            cJSON_AddBoolToObject(sat, "haveAssistNowOff",                    s->haveAssistNowOff);
            cJSON_AddBoolToObject(sat, "haveAssistNowAuto",                   s->haveAssistNowAuto);
            cJSON_AddBoolToObject(sat, "sbasCorrUsed",                        s->sbasCorrUsed);
            cJSON_AddBoolToObject(sat, "rtcmCorrUsed",                        s->rtcmCorrUsed);
            cJSON_AddBoolToObject(sat, "prCorrUsed",                          s->prCorrUsed);
            cJSON_AddBoolToObject(sat, "crCorrUsed",                          s->crCorrUsed);
            cJSON_AddBoolToObject(sat, "doCorrUsed",                          s->doCorrUsed);
        }

        char *jsonStr = cJSON_PrintUnformatted(root);
        printf("%s\n", jsonStr);

        cJSON_Delete(root);
    }
    else {

        // print out our column headers...
        printf("\n%-8s%3s %3s %3s %3s %5s %-20s%-8s%-15sFlags\n", "GNSS",     "ID", "CNo",  "El", "Azi",   "PRr", "Signal qual",         "Sat Hlt", "Orbit Src");
        printf("%-8s%3s %3s %3s %3s %5s %-20s%-8s%-15s=====\n",  "=======", "===", "===", "===", "===", "=====", "===================", "=======", "==============");

        // now print out all our satellites...
        for (int i = 0; i < satellites.numberOfSatellites; i++) {

            // build up our flags string...
            ubxSatellite* s = satellites.satellites + i;
            char* flags = NULL;
            if (s->used)          append(&flags, "u");
            if (s->diffCorr)      append(&flags, "d");
            if (s->smoothed)      append(&flags, "s");
            if (s->haveEphemeris) append(&flags, "e");
            if (s->haveAlmanac)   append(&flags, "a");
            if (s->sbasCorrUsed)  append(&flags, "S");
            if (s->rtcmCorrUsed)  append(&flags, "R");
            if (s->prCorrUsed)    append(&flags, "P");
            if (s->crCorrUsed)    append(&flags, "C");
            if (s->doCorrUsed)    append(&flags, "D");
            if (flags == NULL)    append(&flags, "");

            // print our satellite line...
            printf("%-8s%3d %3d %3d %3d %5.1f %-20s%-8s%-15s%s\n",
                    getGnssName(s->gnssID), s->satelliteID, s->cno, s->elevation, s->azimuth, s->pseudoRangeResidualM,
                    getSignalQuality(s->signalQuality), getSatelliteHealth(s->health), getOrbitSource(s->orbitSource), flags);

            free(flags);
        }
        printf("\nFlags:\n"
                "  u - used for navigation fix\n"
                "  d - differential correction is available\n"
                "  s - carrier-smoothed pseudorange used\n"
                "  e - ephemeris is available\n"
                "  a - almanac is available\n"
                "  S - SBAS corrections used\n"
                "  R - RTCM corrections used\n"
                "  P - pseudorange corrections used\n"
                "  C - carrier range corrections used\n"
                "  D - range rate (Doppler) corrections used\n"
);
    }

    // free up the memory we allocated...
    free(satellites.satellites);

    return makeOkReturn();
}


// Outputs a configuration query, in English or JSON.
static slReturn doConfigQuery(const clientData_slOptions* clientData) {

    ubxConfig config = {};  // zeroes all elements...
    slReturn result = ubxGetConfig(clientData->fdPort, clientData->verbosity, &config);
    if (isErrorReturn(result))
        return makeErrorMsgReturn(ERR_CAUSE(result), "Problem obtaining configuration information from GPS");

    if (clientData->json) {

        cJSON* root     = cJSON_CreateObject();
        cJSON* ant      = cJSON_CreateObject();
        cJSON* gnss     = cJSON_CreateObject();
        cJSON* gnssRecs = cJSON_CreateArray();
        cJSON* nav      = cJSON_CreateObject();
        cJSON* time     = cJSON_CreateObject();
        cJSON* rate     = cJSON_CreateObject();
        cJSON* pwr      = cJSON_CreateObject();
        cJSON* nmea     = cJSON_CreateObject();
        cJSON_AddItemToObject(root, "antenna",    ant);
        cJSON_AddItemToObject(root, "GNSS",       gnss);
        cJSON_AddItemToObject(gnss, "GNSS_records", gnssRecs);
        cJSON_AddItemToObject(root, "navigation", nav);
        cJSON_AddItemToObject(root, "time_pulse", time);
        cJSON_AddItemToObject(root, "fix_rate",   rate);
        cJSON_AddItemToObject(root, "power_mode", pwr);
        cJSON_AddItemToObject(root, "nmea", nmea);

        cJSON_AddBoolToObject(ant, "power_on", config.antPwr);
        cJSON_AddBoolToObject(ant, "short_detection", config.antShrtDet);
        cJSON_AddBoolToObject(ant, "open_detection", config.antOpenDet);
        cJSON_AddBoolToObject(ant, "power_down_on_short", config.antPwrDwnOnShrt);
        cJSON_AddBoolToObject(ant, "auto_recover_from_short", config.antAutoRec);

        cJSON_AddNumberToObject(gnss, "number_of_tracking_channels", config.trkChnnls);
        for (int i = 0; i < config.gnssRecs; i++) {
            cJSON* gnssRec = cJSON_CreateObject();
            cJSON_AddStringToObject(gnssRec, "name", getGnssName(config.gnss[i].id));
            cJSON_AddBoolToObject(gnssRec, "enabled", config.gnss[i].enabled);
            cJSON_AddNumberToObject(gnssRec, "min_channels", config.gnss[i].minChnnls);
            cJSON_AddNumberToObject(gnssRec, "max_channels", config.gnss[i].maxChnnls);
            cJSON_AddItemToArray(gnssRecs, gnssRec);
        }

        cJSON_AddStringToObject(nav, "dynamic_model", getDynamicModelName(config.model));
        cJSON_AddStringToObject(nav, "fix_mode", getFixModeName(config.mode));
        cJSON_AddNumberToObject(nav, "fixed_altitude_2D_meters", config.fixedAltM);
        cJSON_AddNumberToObject(nav, "fixed_altitude_2D_variance_m2", config.fixedAltVarM2);
        cJSON_AddNumberToObject(nav, "min_elevation", config.minElevDeg);
        cJSON_AddNumberToObject(nav, "position_dop_mask", config.pDoP);
        cJSON_AddNumberToObject(nav, "time_dop_mask", config.tDoP);
        cJSON_AddNumberToObject(nav, "position_accuracy_mask", config.pAccM);
        cJSON_AddNumberToObject(nav, "time_accuracy_mask", config.tAccM);
        cJSON_AddNumberToObject(nav, "static_hold_threshold_cms", config.staticHoldThreshCmS);
        cJSON_AddNumberToObject(nav, "dgnss_timeout_secs", config.dgnssTimeoutS);
        cJSON_AddNumberToObject(nav, "threshold_satellites_above_cno", config.cnoThreshNumSVs);
        cJSON_AddNumberToObject(nav, "cno_threshold_dbhz", config.cnoThreshDbHz);
        cJSON_AddNumberToObject(nav, "static_hold_max_distance_meters", config.staticHoldMaxDistM);
        cJSON_AddStringToObject(nav, "utc_standard", getUTCTypeName(config.utcStandard));

        cJSON_AddBoolToObject(time, "enabled", config.timePulse0Enabled);
        cJSON_AddBoolToObject(time, "is_freq", config.isFreq);
        cJSON_AddBoolToObject(time, "is_length", config.isLength);
        cJSON_AddBoolToObject(time, "lock_on_gps_freq", config.lockGpsFreq);
        cJSON_AddBoolToObject(time, "locked_other_set", config.lockedOtherSet);
        cJSON_AddBoolToObject(time, "align_to_top_of_second", config.alignToTow);
        cJSON_AddBoolToObject(time, "polarity_rising_edge", config.polarity);
        cJSON_AddStringToObject(time, "time_grid", getTimeGridTypeName(config.gridUtcTnss));
        cJSON_AddNumberToObject(time, "antenna_cable_delay_ns", config.antCableDelayNs);
        cJSON_AddNumberToObject(time, "rf_group_delay_ns", config.rfGroupDelayNs);
        cJSON_AddNumberToObject(time, "user_configured_delay_ns", config.userConfigDelay);
        cJSON_AddNumberToObject(time, "freq_period_lock", config.freqPeriodLock * (config.isFreq ? 1 : 0.000001));
        cJSON_AddNumberToObject(time, "freq_period", config.freqPeriod * (config.isFreq ? 1 : 0.000001));
        cJSON_AddNumberToObject(time, "pulse_length_ratio_lock", config.pulseLenRatioLock * (config.isLength ? 0.000001 : 0.000000023283064));
        cJSON_AddNumberToObject(time, "pulse_length_ratio", config.pulseLenRatio * (config.isLength ? 0.000001 : 0.000000023283064));

        cJSON_AddNumberToObject(rate, "measurement_rate_ms", config.measRateMs);
        cJSON_AddNumberToObject(rate, "measurements_per_fix", config.navRate);
        cJSON_AddStringToObject(rate, "time_reference", getFixTimeRefName(config.timeRef));

        cJSON_AddStringToObject(pwr, "power_setup", getPowerModeName(config.powerSetup));
        cJSON_AddNumberToObject(pwr, "period_secs_for_interval", config.powerIntervalSecs);
        cJSON_AddNumberToObject(pwr, "on_time_secs_for_interval", config.powerOnTimeSecs);

        cJSON_AddBoolToObject(nmea, "enabled", config.nmeaEnabled);
        cJSON_AddNumberToObject(nmea, "version", (config.nmeaVersion / 16) + (config.nmeaVersion % 16) * 0.1);
        cJSON_AddBoolToObject(nmea, "GGA", config.GGA);
        cJSON_AddBoolToObject(nmea, "GLL", config.GLL);
        cJSON_AddBoolToObject(nmea, "GSA", config.GSA);
        cJSON_AddBoolToObject(nmea, "GSV", config.GSV);
        cJSON_AddBoolToObject(nmea, "RMC", config.RMC);
        cJSON_AddBoolToObject(nmea, "VTG", config.VTG);
        cJSON_AddBoolToObject(nmea, "GRS", config.GRS);
        cJSON_AddBoolToObject(nmea, "GST", config.GST);
        cJSON_AddBoolToObject(nmea, "ZDA", config.ZDA);

        char *jsonStr = cJSON_PrintUnformatted(root);
        printf("%s\n", jsonStr);

        cJSON_Delete(root);
    }
    else {
        printf("U-Blox GPS configuration\n");

        printf("  Antenna:\n");
        printf("    Power enabled:                %s\n", yesNo(config.antPwr));
        printf("    Short detection:              %s\n", yesNo(config.antShrtDet));
        printf("    Open detection:               %s\n", yesNo(config.antOpenDet));
        printf("    Power down on short:          %s\n", yesNo(config.antPwrDwnOnShrt));
        printf("    Auto recovery from short:     %s\n", yesNo(config.antAutoRec));

        printf("  GNSS:\n");
        printf("    Tracking channels:            %d\n", config.trkChnnls);
        for (int i = 0; i < config.gnssRecs; i++) {
            printf("    Type: %s\n", getGnssName(config.gnss[i].id));
            printf("      Enabled:                    %s\n", yesNo(config.gnss[i].enabled));
            printf("      Minimum channels:           %d\n", config.gnss[i].minChnnls);
            printf("      Maximum channels:           %d\n", config.gnss[i].maxChnnls);
        }

        printf("  Navigation engine:\n");
        printf("    Dynamic model:                %s\n", getDynamicModelName(config.model));
        printf("    Fix mode:                     %s\n", getFixModeName(config.mode));
        printf("    Fixed altitude (2D):          %.2f meters\n",             config.fixedAltM);
        printf("    Fixed altitude variance (2D): %.4f meters^2\n",           config.fixedAltVarM2);
        printf("    Minimum elevation:            %d degrees\n",              config.minElevDeg);
        printf("    Position DoP mask:            %.1f\n",                    config.pDoP);
        printf("    Time DoP mask:                %.1f\n",                    config.tDoP);
        printf("    Position accuracy mask:       %d meters\n",               config.pAccM);
        printf("    Time accuracy mask:           %d meters\n",               config.tAccM);
        printf("    Static hold threshold:        %d cm/s\n",                 config.staticHoldThreshCmS);
        printf("    Dynamic GNSS timeout:         %d seconds\n",              config.dgnssTimeoutS);
        printf("    Threshold above C/No:         %d satellites\n",           config.cnoThreshNumSVs);
        printf("    C/No threshold:               %d dBHz\n",                 config.cnoThreshDbHz);
        printf("    Static hold max distance:     %d meters\n",               config.staticHoldMaxDistM);
        printf("    UTC standard:                 %s\n", getUTCTypeName(config.utcStandard));

        printf("  Time pulse:\n");
        printf("    Time pulse 0 enabled:         %s\n", yesNo(config.timePulse0Enabled));
        printf("    Use frequency vs. period:     %s\n", yesNo(config.isFreq));
        printf("    Use length vs. duty cycle:    %s\n", yesNo(config.isLength));
        printf("    Lock on GPS frequency:        %s\n", yesNo(config.lockGpsFreq));
        printf("    Lock use other set:           %s\n", yesNo(config.lockedOtherSet));
        printf("    Align to top of second:       %s\n", yesNo(config.alignToTow));
        printf("    Polarity rising edge:         %s\n", yesNo(config.polarity));
        printf("    Time grid:                    %s\n", getTimeGridTypeName(config.gridUtcTnss));
        printf("    Antenna cable delay:          %d nanoseconds\n",          config.antCableDelayNs);
        printf("    RF group delay:               %d nanoseconds\n",          config.rfGroupDelayNs);
        if (config.isFreq) {
            printf("    Locked pulse frequency:       %d Hz\n",               config.freqPeriodLock);
            printf("    Unlocked pulse frequency:     %d Hz\n",               config.freqPeriod);
        } else {
            printf("    Locked pulse period:          %d microseconds\n",     config.freqPeriodLock);
            printf("    Unlocked pulse period:        %d microseconds\n",     config.freqPeriod);
        }
        if (config.isLength) {
            printf("    Locked pulse length:          %d microseconds\n",     config.pulseLenRatioLock);
            printf("    Unlocked pulse length:        %d microseconds\n",     config.pulseLenRatio);
        } else {
            printf("    Locked pulse duty cycle:      %.2f%%\n",              config.pulseLenRatioLock * 0.000000023283064);
            printf("    Unlocked pulse duty cycle:    %.2f%%\n",              config.pulseLenRatio * 0.000000023283064);
        }
        printf("    User configurable delay:      %d nanoseconds\n",          config.userConfigDelay);

        printf("  Fix rate:\n");
        printf("    Measurement rate:             %d milliseconds\n",         config.measRateMs);
        printf("    Measurements per fix:         %d\n",                      config.navRate);
        printf("    Time reference:               %s\n", getFixTimeRefName(config.timeRef));

        printf("  Power mode:\n");
        printf("    Power setup:                  %s\n", getPowerModeName(config.powerSetup));
        printf("    Period (if interval):         %d seconds\n",              config.powerIntervalSecs);
        printf("    On time (if interval):        %d seconds\n",              config.powerOnTimeSecs);

        printf("  NMEA:\n");
        printf("    Enabled:                      %s\n", yesNo(config.nmeaEnabled));
        printf("    Version:                      %01x.%01x\n",               config.nmeaVersion / 16, config.nmeaVersion % 16);
        printf("    GGA:                          %s\n", onOff(config.GGA));
        printf("    GLL:                          %s\n", onOff(config.GLL));
        printf("    GSA:                          %s\n", onOff(config.GSA));
        printf("    GSV:                          %s\n", onOff(config.GSV));
        printf("    RMC:                          %s\n", onOff(config.RMC));
        printf("    VTG:                          %s\n", onOff(config.VTG));
        printf("    GRS:                          %s\n", onOff(config.GRS));
        printf("    GST:                          %s\n", onOff(config.GST));
        printf("    ZDA:                          %s\n", onOff(config.ZDA));

    }

    return makeOkReturn();
}


static slReturn parseMsgHelper(errorInfo_slReturn error, const optionDef_slOptions* def, char *pattern, ...) {

    char* resolved;

    // first resolve the pattern we were given...
    va_list args;
    va_start(args, pattern);
    vasprintf(&resolved, pattern, args);
    va_end(args);

    // return our bad news...
    slReturn result = makeErrorFmtMsgReturn(error, "option parsing problem: [%s] %s", getName_slOptions(def), resolved);
    free(resolved);
    return result;
}

// Test the argument to see if it represents a valid serial port.  On failure, return an error.
static slReturn parsePort(void* ptrArg, int intArg, const optionDef_slOptions *def, const char *arg, clientData_slOptions *clientData) {

    slReturn vsd = verifySerialDevice(arg);
    clientData->port = arg;
    if (isErrorReturn(vsd))
        return makeErrorFmtMsgReturn(ERR_CAUSE(vsd), "device \"%s\" is not a terminal", arg);

    // stuff it away to the given pointer...
    return makeOkReturn();
}


// Parse the given argument, which is presumed to be a string representing a positive integer that is one of the
// standard baud rates.  If the parsing fails for any reason, return an error
static slReturn parseBaud(void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData) {

    // make sure we can parse the entire argument into an integer...
    char *endPtr;
    int baud = (int) strtol(arg, &endPtr, 10);
    if (*endPtr != 0)
        return parseMsgHelper(ERR_ROOT, def, "the specified baud rate (\"%s\") is not an integer", arg);

    // make sure the result is a valid baud rate...
    if (getBaudRateCookie(baud) < 0)
        return parseMsgHelper(ERR_ROOT, def, "the specified baud rate (\"%s\") is not a valid one (4800, 9600, 19200, etc.)", arg);

    // we have a valid baud rate, so stuff it away and return in victory...
    *((int*)ptrArg) = baud;
    return makeOkReturn();
}


// Parse autobaud or sync, which have an optional argument of ascii, nmea, or ubx.
static slReturn parseSyncMethod(void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData) {

    // set up the sync method...
    slReturn ssmResp = setSyncMethod(arg, clientData);
    if (isErrorReturn(ssmResp))
        return parseMsgHelper(ERR_CAUSE(ssmResp), def, "the specified baud rate inference method (\"%s\") is unrecognizable; should be \"ascii\", \"nmea\", or \"ubx\"", arg);

    return makeOkReturn();
}


// Parse the given argument, which is presumed to be a string representing a yes or no value, which if parsed without
// error is stored to the argPtr. If the parsing fails for any reason, return an error.
static slReturn parseBool(void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData) {

    // see if we've got anything that like a yes or true (otherwise, we make it a false)...
    *((int*)ptrArg) = 0;
    if (arg) {
        *((int*)ptrArg) = (strchr("yYtT1", *arg) != NULL ? 1 : 0);
        // now look for "on" and treat it as true
        if (strlen(arg) > 1) {
            if ((tolower(arg[0]) == 'o') && (tolower(arg[1]) == 'n')) {
                *((int*)ptrArg) = 1;
            }
        }
    }
    // then return with no error...
    return makeOkReturn();
}


// Parse the given argument, which is presumed to be a string representing an integer, which if parsed without error is
// stored to the argPtr. If the parsing fails for any reason, return an error.
static slReturn parseInt(void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData) {

    // make sure we can parse the entire argument into an integer...
    int n = 0;
    if (arg != NULL) {
        char *endPtr;
        n = (int) strtol(arg, &endPtr, 10);
        if (*endPtr != 0)
            return parseMsgHelper(ERR_ROOT, def, "the specified %s (\"%s\") is not an integer", def->argName, arg);
    }

    // stuff it away to the given pointer...
    *((int*)ptrArg) = n;

    // then return with no error...
    return makeOkReturn();
}


// Set a boolean flag at the given ptrArg to the given intArg.
static slReturn parseFlag(void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData) {

    // do what we've been told to do...
    *((bool*)ptrArg) = (bool) intArg;

    // then return with no error...
    return makeOkReturn();
}

typedef struct { char* name; queryType queryType; } queryDef;
static queryDef queryDefs[] = {
        { "fix",        fixQuery       },
        { "version",    versionQuery   },
        { "satellites", satelliteQuery },
        { "config",     configQuery    }
};


// Parse the given query type and store the results.  Returns an error on any parsing problem.
static slReturn parseQuery(void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData) {

    // see what kind of query we've got here...
    for (int i = 0; i < ARRAY_SIZE(queryDefs); i++) {
        if (0 == strcmp(arg, queryDefs[i].name)) {
            clientData->queryType = queryDefs[i].queryType;
            return makeOkReturn();
        }
    }

    // if we get here then we have a bogus query type...
    return parseMsgHelper(ERR_ROOT, def, "the specified %s (\"%s\") is not a valid query type", def->argName, arg);
}


// Increment the int counter at the given ptrArg.
static slReturn parseCount(void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData) {

    // do what we've been told to do...
    (*((int*)ptrArg))++;

    // then return with no error...
    return makeOkReturn();
}

#undef V3
#undef V2
#undef V1
#undef V0
#undef CD


// Sync may only be specified if auto baud rate was NOT specified...
static slReturn  constrainSync(const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state) {

    if (hasShortOption_slOptions('a', state))
        return makeErrorMsgReturn(ERR_ROOT, "-s, --sync may only be specified if -a, --autobaud is NOT specified");

    return makeOkReturn();
}


// JSON may only be specified if a query was specified...
static slReturn  constrainJSON(const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state) {

    if (!hasShortOption_slOptions('Q', state))
        return makeErrorMsgReturn(ERR_ROOT, "-j, --json may only be specified if -Q, --query is also specified");

    return makeOkReturn();
}


// Baud may only be specified if auto baud rate was NOT specified...
static slReturn  constrainBaud(const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state) {

    if (hasShortOption_slOptions('a', state))
        return makeErrorMsgReturn(ERR_ROOT, "-b, --baud may only be specified if -a, --autobaud is NOT specified");

    return makeOkReturn();
}


// Minbaud may only be specified if auto baud rate was specified...
static slReturn  constrainMinBaud(const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state) {

    if (!hasShortOption_slOptions('a', state))
        return makeErrorMsgReturn(ERR_ROOT, "-M, --minbaud may only be specified if -a, --autobaud is also specified");

    return makeOkReturn();
}


// Verbosity may only be specified if quiet was NOT specified...
static slReturn  constrainVerbosity(const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state) {

    if (hasShortOption_slOptions('q', state))
        return makeErrorMsgReturn(ERR_ROOT, "-v, --verbosity d may only be specified if -q, --quiet is NOT specified");

    return makeOkReturn();
}

#define CD (config->clientData)
#define V0 (CD->verbosity >= 0)
#define V1 (CD->verbosity >= 1)
#define V2 (CD->verbosity >= 2)
#define V3 (CD->verbosity >= 3)


/* remove PPS signal test as wiringPi is now deprecates
// Test for the presence and quality of a 1 Hz signal on GPIO 18 (the PPS signal from the GPS), printing the result.
#define PPS_PIN 1
static slReturn actionTestPPS(const optionDef_slOptions* defs, const psloConfig* config) {

    wiringPiSetup();
    pinMode(PPS_PIN, INPUT);

    struct timespec tenmicro = { 0, 10000 };

    struct timespec lastTime = { 0, 0 };

    // measure for about 10 seconds...
    int lastState = 0;
    for (int i = 0; i < 100000; i++) {

        // wait 10 microseconds
        nanosleep(&tenmicro, NULL);

        // check our state...
        int state = digitalRead(PPS_PIN);

        // if the state has changed, time to measure and report...
        if (state != lastState) {

            // get the current time...
            struct timespec thisTime;
            int result = clock_gettime(CLOCK_MONOTONIC, &thisTime);
            if (result != 0)
                return makeErrorFmtMsgReturn(ERR_ROOT, "problem reading CPU clock: %s", strerror(errno));

            // if this isn't our first measurement...
            if ((lastTime.tv_sec != 0) || (lastTime.tv_nsec != 0)) {

                // figure out how long we were in this state...
                long long nanos = thisTime.tv_nsec - lastTime.tv_nsec;
                if (nanos < 0) nanos += 1000000000;
                double secs = 1.0 * nanos / 1000000000.0;

                // tell our results...
                char* signal = (lastState == 0) ? " low" : "high";
                printf("PPS was %s for %.3f seconds\n", signal, secs);
            }
            lastTime = thisTime;
            lastState = state;
        }
    }

    return makeOkReturn();
}
*/

// Autobaud action function, which infers the GPS baud rate by trying to synchronize at various baud rates.
static slReturn actionAutoBaud(const optionDef_slOptions* defs, const psloConfig* config) {

    // first we figure out what synchronization type we're going to use...
    baudRateSynchronizer* synchronizer;
    clientData_slOptions* cd = config->clientData;
    switch (cd->syncMethod) {
        case syncASCII:
            synchronizer = asciiBaudRateSynchronizer;
            break;
        case syncNMEA:
            synchronizer = nmeaBaudRateSynchronizer;
            break;
        case syncUBX:
            synchronizer = ubxSynchronizer;
            break;
        default: return makeErrorFmtMsgReturn(ERR_ROOT, "invalid synchronization type: %d", cd->syncMethod);
    }

    if (V2) printf("Automatically determining baud rate...\n");

    // now we set the correct options (the first baud rate, 8 data bits, 1 stop bit, no parity)
    slReturn result = setTermOptions(CD->fdPort, 460800, 8, 1, false, false);
    if (isErrorReturn(result)) {
        close(CD->fdPort);
        return makeErrorMsgReturn(ERR_CAUSE(result), "Error when setting terminal options");
    }

    // now we do the actual work...
    slReturn abResp = autoBaudRate(cd->fdPort, cd->minBaud, synchronizer, cd->verbosity);
    if (isErrorReturn(abResp))
        makeErrorFmtMsgReturn(ERR_CAUSE(abResp), "problem while automatically determining baud rate");

    // change the client data to reflect the new baud rate...
    cd->baud = cd->newbaud;

    return makeOkReturn();
}


// Baud action function, which sets the host's port to the specified baud rate.
static slReturn actionBaud(const optionDef_slOptions* defs, const psloConfig* config) {

    if (V2) printf("Setting baud rate to %d...\n", CD->baud);

    // now we set the correct options (the specified baud rate, 8 data bits, 1 stop bit, no parity)
    slReturn result = setTermOptions(CD->fdPort, CD->baud, 8, 1, false, false);
    if (isErrorReturn(result)) {
        close(CD->fdPort);
        return makeErrorMsgReturn(ERR_CAUSE(result), "Error when setting terminal options");
    }

    return makeOkReturn();
}


// New baud rate action function, which changes both the U-Blox GPS baud rate and the host baud rate.
static slReturn actionNewBaud(const optionDef_slOptions* defs, const psloConfig* config) {

    clientData_slOptions* cd = config->clientData;
    if (V2) printf("Changing baud rate to %d...\n", cd->newbaud);
    slReturn resp = ubxChangeBaudRate(cd->fdPort, (unsigned) cd->newbaud, cd->verbosity);
    if (isErrorReturn(resp))
        return makeErrorMsgReturn(ERR_CAUSE(resp), "failed to change baud rate");

    return makeOkReturn();
}


// Sync action function, which synchronizes serial port receiver with the data stream using the configured method.
static slReturn actionSync(const optionDef_slOptions* defs, const psloConfig* config) {

    slReturn ssResp = syncSerial(CD->syncMethod, config->clientData);
    if (isErrorReturn(ssResp))
        return makeErrorMsgReturn(ERR_CAUSE(ssResp), "error while synchronizing on serial data");

    bool success = getReturnInfoChar(ssResp);
    if (success)
        return makeOkReturn();
    else
        return makeErrorMsgReturn(ERR_ROOT, "failed to synchronize");
}


// NMEA action function, which turns NMEA data on or off on the U-Blox GPS.
static slReturn  actionNMEA(const optionDef_slOptions* defs, const psloConfig* config) {

    clientData_slOptions* clientData = config->clientData;
    // If nmea flag is not explicitly 0 or 1 return without touching it
    //   so nmea enabled option 'enabled=statusquo' wouldn't change setting
    if (clientData->nmea < 0 || clientData->nmea > 1)
        return makeOkReturn();
    slReturn usResp = syncSerial(syncUBX, clientData);
    if (isErrorReturn(usResp))
        return makeErrorMsgReturn(ERR_CAUSE(usResp), "could not synchronize UBX protocol");

    slReturn resp = ubxSetNMEAData(clientData->fdPort, clientData->verbosity, clientData->nmea);
    if (isErrorReturn(resp))
        return makeErrorFmtMsgReturn(ERR_CAUSE(resp), "failed to turn NMEA data %s", clientData->nmea ? "on" : "off");

    return makeOkReturn();
}


// Query action function, which queries the U-Blox GPS for the specified data.
static slReturn  actionQuery(const optionDef_slOptions* defs, const psloConfig* config) {

    clientData_slOptions* clientData = config->clientData;
    slReturn usResp = syncSerial(syncUBX, clientData);
    if (isErrorReturn(usResp))
        return makeErrorMsgReturn(ERR_CAUSE(usResp), "could not synchronize UBX protocol");

    slReturn resp;
    switch (clientData->queryType) {

        case fixQuery:       resp = doFixQuery(clientData); break;
        case versionQuery:   resp = doVersionQuery(clientData); break;
        case configQuery:    resp = doConfigQuery(clientData); break;
        case satelliteQuery: resp = doSatelliteQuery(clientData); break;

        default: return makeErrorFmtMsgReturn(ERR_ROOT, "invalid query type: %d", clientData->queryType);
    }
    if (isErrorReturn(resp))
        return makeErrorMsgReturn(ERR_CAUSE(resp), "problem executing GPS query");
    return makeOkReturn();
}


// Save configuration action function, which saves the current U-Blox GPS configuration to battery-backed RAM.
static slReturn  actionSaveConfig(const optionDef_slOptions* defs, const psloConfig* config) {

    // make sure we're synchronized...
    clientData_slOptions* clientData = config->clientData;
    slReturn usResp = syncSerial(syncUBX, clientData);
    if (isErrorReturn(usResp))
        return makeErrorMsgReturn(ERR_CAUSE(usResp), "could not synchronize UBX protocol");

    // save the GPS configuration...
    slReturn uscResp = ubxSaveConfig(config->clientData->fdPort, config->clientData->verbosity);
    if (isErrorReturn(uscResp))
        return makeErrorMsgReturn(ERR_CAUSE(uscResp), "problem saving GPS configuration");

    return makeOkReturn();
}


// Reset action function, which resets U-Blox GPS via a software-provoked hardware reset.
static slReturn  actionReset(const optionDef_slOptions* defs, const psloConfig* config) {

    // make sure we're synchronized...
    clientData_slOptions* clientData = config->clientData;
    slReturn usResp = syncSerial(syncUBX, clientData);
    if (isErrorReturn(usResp))
        return makeErrorMsgReturn(ERR_CAUSE(usResp), "could not synchronize UBX protocol");

    // reset the GPS...
    slReturn urResp = ubxReset(config->clientData->fdPort, config->clientData->verbosity);
    if (isErrorReturn(urResp))
        return makeErrorMsgReturn(ERR_CAUSE(urResp), "problem resetting the GPS");

    return makeOkReturn();
}


// Configure for timing action function, which configures the GPS for maximum timing accuracy...
static slReturn actionConfigForTiming(const optionDef_slOptions* defs, const psloConfig* config) {

    // make sure we're synchronized...
    clientData_slOptions* clientData = config->clientData;
    slReturn usResp = syncSerial(syncUBX, clientData);
    if (isErrorReturn(usResp))
        return makeErrorMsgReturn(ERR_CAUSE(usResp), "could not synchronize UBX protocol");

    // configure the GPS...
    slReturn ucftResp = ubxConfigForTiming(config->clientData->fdPort, config->clientData->verbosity);
    if (isErrorReturn(ucftResp))
        return makeErrorMsgReturn(ERR_CAUSE(ucftResp), "problem configuring the GPS for timing");

    return makeOkReturn();

}

// Configure Satellites only...
static slReturn actionConfigSatellites(const optionDef_slOptions* defs, const psloConfig* config) {

    // make sure we're synchronized...
    clientData_slOptions* clientData = config->clientData;
    slReturn usResp = syncSerial(syncUBX, clientData);
    if (isErrorReturn(usResp))
        return makeErrorMsgReturn(ERR_CAUSE(usResp), "could not synchronize UBX protocol");

    // configure the GPS...
    slReturn ucftResp = ubxConfigSatellites(config->clientData->fdPort, config->clientData->verbosity);
    if (isErrorReturn(ucftResp))
        return makeErrorMsgReturn(ERR_CAUSE(ucftResp), "problem configuring the GPS satellites");

    return makeOkReturn();

}

// Echo action function, which echoes NMEA data to stdout for a configurable period of time.
static slReturn  actionEcho(const optionDef_slOptions* defs, const psloConfig* config) {

    if (V2) printf("Starting echo...\n");

    // figure out our timing...
    long long startTime = currentTimeMs();
    long long maxEchoMS = 1000 * CD->echoSeconds;

    while ((CD->echoSeconds == 0) || ((currentTimeMs() - startTime) < maxEchoMS)) {
        slReturn rscResp = readSerialChar(CD->fdPort, 1000);
        if (isErrorReturn(rscResp))
            return makeErrorMsgReturn(ERR_CAUSE(rscResp), "couldn't read character while echoing");
        if (isWarningReturn(rscResp))
            continue;  // we timed out; just try again...
        int c = getReturnInfoChar(rscResp);
        if (c >= 0) printf("%c", c);
    }

    if (V2) printf("\nEnding echo...\n"); else printf("\n");

    return makeOkReturn();
}


// Quiet action function, which simply sets verbosity to zero.
static slReturn  actionQuiet(const optionDef_slOptions* defs, const psloConfig* config) {
    config->clientData->verbosity = 0;
    return makeOkReturn();
}

// Setup action function, which opens and configures the serial port.
static slReturn actionSetup(const optionDef_slOptions* defs, const psloConfig* config) {
    if (CD->port == NULL) {
        CD->port = iniparser_getstring(gpsctlConf, "gpsctl:port", "/dev/serial0");
        slReturn vsd = verifySerialDevice(CD->port);
        if (isErrorReturn(vsd)) {
            printf("Error: device \"%s\" is not a terminal\n", CD->port);
            exit(EXIT_FAILURE);
        }
    }

    // first we try opening the device for the port...
    int fdPort = open(CD->port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fdPort < 0) {
        printf("Failed to open serial port (\"%s\"): %s\n", CD->port,strerror(errno));
        exit(1);
    }
    if (V2) printf("Serial port (\"%s\") open...\n", CD->port);

    // stuff our file descriptor and return in victory...
    CD->fdPort = fdPort;
    if (V2) printf("Serial port open and configured...\n");

    // configure NMEA output here using setting derived from the conf file
    // it may be reconfigured by the -n or --nmea commandline parameters
    actionNMEA(defs, config);
    return makeOkReturn();
}


// Teardown action function, which closes the serial port.
static slReturn actionTeardown(const optionDef_slOptions* defs, const psloConfig* config) {

    close(CD->fdPort);
    return makeOkReturn();
}


static optionDef_slOptions* getOptionDefs(const clientData_slOptions* clientData) {

    optionDef_slOptions echoDef = {
            1, "echo", 'e', argOptional,                                        // max, long, short, arg
            parseInt, (void*) &clientData->echoSeconds, 0,                      // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionEcho,                                                         // action
            "seconds",                                                          // argument name
            "echo human-readable GPS data to stdout for the given time (0 or unspecified means forever)"
    };

    optionDef_slOptions portDef = {
            1, "port", 'p', argRequired,                                        // max, long, short, arg
            parsePort, (void *) &clientData->port, 0,                           // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            NULL,                                                               // action
            "device",                                                           // argument name
            "specify the port device (default is '/dev/serial0')"
    };

    optionDef_slOptions autobaudDef = {
            1, "autobaud", 'a', argOptional,                                    // max, long, short, arg
            parseSyncMethod, NULL, 0,                                           // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionAutoBaud,                                                     // action
            "method",                                                           // argument name
            "infer baud rate using ascii, nmea, ubx data (default is ubx)"
    };


    optionDef_slOptions verboseDef = {
            2, "verbose", 'v', argNone,                                         // max, long, short, arg
            parseCount, (void*) &clientData->verbosity, 0,                      // parser, ptrArg, intArg
            constrainVerbosity,                                                 // constrainer
            NULL,                                                               // action
            NULL,                                                               // argument name
            "get verbose messages"
    };


    optionDef_slOptions quietDef = {
            1, "quiet", 'q', argNone,                                           // max, long, short, arg
            NULL, NULL, 0,                                                      // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionQuiet,                                                        // action
            NULL,                                                               // argument name
            "get verbose messages"
    };


    optionDef_slOptions jsonDef = {
            3, "json", 'j', argNone,                                            // max, long, short, arg
            parseFlag, (void*) &clientData->json, 1,                            // parser, ptrArg, intArg
            constrainJSON,                                                      // constrainer
            NULL,                                                               // action
            NULL,                                                               // argument name
            "select JSON query output"
    };


    optionDef_slOptions baudDef = {
            1, "baud", 'b', argRequired,                                        // max, long, short, arg
            parseBaud, (void*) &clientData->baud, 0,                            // parser, ptrArg, intArg
            constrainBaud,                                                      // constrainer
            actionBaud,                                                         // action
            "baud rate",                                                        // argument name
            "specify different host baud rate; any standard rate is allowed"
    };


    optionDef_slOptions minBaudDef = {
            1, "minbaud", 'M', argRequired,                                     // max, long, short, arg
            parseBaud, (void*) &clientData->minBaud, 0,                         // parser, ptrArg, intArg
            constrainMinBaud,                                                   // constrainer
            NULL,                                                               // action
            "baud rate",                                                        // argument name
            "specify minimum baud rate for autobaud (default is '9600'); any standard rate is allowed"
    };


    optionDef_slOptions newbaudDef = {
            1, "newbaud", 'B', argRequired,                                     // max, long, short, arg
            parseBaud, (void*) &clientData->newbaud, 0,                         // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionNewBaud,                                                      // action
            "baud rate",                                                        // argument name
            "change the U-Blox and host baud rates to the specified standard rate"
    };

    optionDef_slOptions queryDef = {
            1, "query", 'Q', argRequired,                                       // max, long, short, arg
            parseQuery, NULL, 0,                                                // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionQuery,                                                        // action
            "query type",                                                       // argument name
            "query the GPS for info (see \"Query types\" below)"
    };

    optionDef_slOptions nmeaDef = {
            1, "nmea", 'n', argRequired,                                        // max, long, short, arg
            parseBool, (void*) &clientData->nmea, 0,                            // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionNMEA,                                                         // action
            "yes/no",                                                           // argument name
            "configure whether the GPS sends NMEA data to the serial port (see \"Yes or no values\" below)"
    };

    optionDef_slOptions syncDef = {
            1, "sync", 's', argOptional,                                        // max, long, short, arg
            parseSyncMethod, NULL, 0,                                           // parser, ptrArg, intArg
            constrainSync,                                                      // constrainer
            actionSync,                                                         // action
            "method",                                                           // argument name
            "synchronize using ascii, nmea, ubx data (default is ubx)"
    };

    optionDef_slOptions saveConfigDef = {
            1, "save_config", 0, argNone,                                       // max, long, short, arg
            NULL, NULL, 0,                                                      // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionSaveConfig,                                                   // action
            NULL,                                                               // argument name
            "synchronize using ascii, nmea, ubx data (default is ubx)"
    };

    optionDef_slOptions resetDef = {
            1, "reset", 0, argNone,                                             // max, long, short, arg
            NULL, NULL, 0,                                                      // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionReset,                                                        // action
            NULL,                                                               // argument name
            "reset the GPS"
    };

    /* remove as wiringPi now deprecated
    optionDef_slOptions testPpsDef = {
            1, "test", 0, argNone,                                              // max, long, short, arg
            NULL, NULL, 0,                                                      // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionTestPPS,                                                      // action
            NULL,                                                               // argument name
            "test the GPS' PPS signal"
    };
    */
    optionDef_slOptions configureForTimingDef = {
            1, "configure_for_timing", 0, argNone,                              // max, long, short, arg
            NULL, NULL, 0,                                                      // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionConfigForTiming,                                              // action
            NULL,                                                               // argument name
            "configure the GPS for maximum timing precision"
    };

   optionDef_slOptions configureForGalileoDef = {
            1, "galileo", 0, argNone,                                           // max, long, short, arg
            NULL, NULL, 0,                                                      // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionConfigSatellites,                                             // action
            NULL,                                                               // argument name
            "configure the GPS for Galileo Satellites"
    };

   optionDef_slOptions configureSatellitesDef = {
            1, "satellites", 0, argNone,                                        // max, long, short, arg
            NULL, NULL, 0,                                                      // parser, ptrArg, intArg
            NULL,                                                               // constrainer
            actionConfigSatellites,                                             // action
            NULL,                                                               // argument name
            "configure the GPS Satellites only"
    };

    optionDef_slOptions optionDefs[] = {

            // initialization elements...
            verboseDef,
            quietDef,
            portDef,
            jsonDef,
            autobaudDef,
            baudDef,
            minBaudDef,
//            testPpsDef,

            // all the above MUST come before those below, or port won't be initialized correctly...
            syncDef,
            newbaudDef,
            nmeaDef,
            queryDef,
            configureForGalileoDef,
            configureSatellitesDef,
            configureForTimingDef,
            saveConfigDef,
            resetDef,
            echoDef,
            { 0 }  // terminator; MUST be at the end of this list...
    };

    // allocate memory for our option definitions, and copy them in...
    int numDefs = ARRAY_SIZE(optionDefs);
    optionDef_slOptions* result = safeMalloc(sizeof(optionDefs));
    for (int i = 0; i < numDefs; i++)
        result[i] = optionDefs[i];

    return result;
}

static char* exampleText =
    "Additional information:\n"
    "\n"
    "  Query types:\n"
    "    fix         returns position, altitude, and time information\n"
    "    version     returns hardware and software version of the GPS\n"
    "    config      returns GPS configuration information\n"
    "    satellites  returns currently visible satellite information from the GPS\n"
    "\n"
    "  Yes or no values:\n"
    "    yes         indicated by an initial character of 'y', 'Y', 't', 'T', or '1'\n"
    "    no          indicated by anything else\n"
    "\n"
    "Examples:\n"
    "   gpsctl --port=/dev/serial1 --baud 9600 --echo=5\n"
    "      Selects the serial port \"/dev/serial1\", sets the host serial port baud rate to 9600, and\n"
    "      echoes any received characters (presumably NMEA data) to stdout.\n"
    "   gpsctl -p /dev/serial1 -b 9600 -e=5\n"
    "      Exactly the same as the preceding example, using short options.\n"
    "";


// The entry point for gpsctl...
int main(int argc, char *argv[]) {
    char *inifile = "/etc/gpsctl.conf";
    // initialize our client data structure, and set defaults...
    clientData_slOptions clientData = { 0 };
    clientData.baud = 0;            // triggers actionSetup to keep existing terminal options, including baud rate
    clientData.minBaud = 9600;
    clientData.ubxSynchronized = false;
    clientData.port = NULL;

    // check if /etc/gpsctl.conf exists, and load it if it does
    if (access(inifile, R_OK) == 0) {
        gpsctlConf = iniparser_load(inifile);
        if (gpsctlConf == NULL) {
            exit(EXIT_FAILURE);
        }
    }
    clientData.verbosity  = iniparser_getint(gpsctlConf, "gpsctl:verbosity", 1);
    clientData.syncMethod = iniparser_getint(gpsctlConf, "gpsctl:sync method", syncUBX);
    clientData.nmea =   iniparser_getboolean(gpsctlConf, "nmea:enabled", -1);
    // iniparser_dump_ini(gpsctlConf, stdout);
    psloConfig config = {
            getOptionDefs(&clientData),                         // our command-line option definitions...
            &clientData,                                        // our options processing data structure...
            NULL, NULL,                                         // before, after constraint-checking functions...
            actionSetup, actionTeardown,                        // before, after action functions.
            "gpsctl",                                           // name of gpsctl...
            "1.17",                                              // version of gpsctl...
            exampleText,                                        // usage examples...
            SL_OPTIONS_CONFIG_NORMAL                            // slOptions configuration options...
    };

    slReturn resp = process_slOptions(argc, (const char **) argv, &config);

    if (isErrorReturn(resp))
        switch (clientData.verbosity) {
            case 0:  printf("Errors occurred!\n"); break;
            case 1:  printReturn(resp, false, false); break;
            case 2:  printReturn(resp, true, false); break;
            case 3:  printReturn(resp, true, true); break;
            default: printReturn(resp, true, true); break;
        }

    freeReturn(resp);

    exit(EXIT_SUCCESS);
}
