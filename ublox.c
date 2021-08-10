//
// Created by Tom Dilatush on 10/20/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L

#include "ublox.h"

// for best time pulse:
//   - disable SBAS, IMES, and QZSS (UBX-CFG-GNSS)
//   - stationary mode (UBX-CFG-NAV5)
//   - calibrate antenna delay (UBX-CFG-TP5)
//   - set measurement rate (UBX-CFG-RATE) and time pulse frequency (UBX-CFG-TP5) to 1Hz
//   - set UTC variant to GPS (UBX-CFG-NAV5)


#define UBX_ACK      0x05
#define UBX_ACK_ACK  0x01
#define UBX_ACK_NAK  0x00
#define UBX_CFG      0x06
#define UBX_CFG_PRT  0x00
#define UBX_CFG_ANT  0x13
#define UBX_CFG_NAV5 0x24
#define UBX_CFG_GNSS 0x3E
#define UBX_CFG_TP5  0x31
#define UBX_CFG_PMS  0x86
#define UBX_CFG_RATE 0x08
#define UBX_CFG_RST  0x04
#define UBX_CFG_CFG  0x09
#define UBX_CFG_NMEA 0x17
#define UBX_CFG_MSG  0x01
#define UBX_MON      0x0A
#define UBX_MON_VER  0x04
#define UBX_NAV      0x01
#define UBX_NAV_PVT  0x07
#define UBX_NAV_SAT  0x35

#define NAV_PVT_MAX_MS      2000
#define NAV_SAT_MAX_MS      2000
#define CFG_NAV5_MAX_MS     2000
#define MON_VER_MAX_MS      2000
#define MON_VER_SYNC_MAX_MS 1200
#define CFG_GNSS_MAX_MS     1000
#define CFG_ANT_MAX_MS      1000
#define CFG_PRT_MAX_MS      1000
#define CFG_NMEA_MAX_MS     1000
#define CFG_TP5_MAX_MS      1000
#define CFG_RATE_MAX_MS     1000
#define CFG_PMS_MAX_MS      1000

static ubxType ut_ACK_NAK = { UBX_ACK, UBX_ACK_NAK };
static ubxType ut_CFG_PRT = { UBX_CFG, UBX_CFG_PRT };
static ubxType ut_NAV_PVT = { UBX_NAV, UBX_NAV_PVT };
static ubxType ut_CFG_CFG = { UBX_CFG, UBX_CFG_CFG };
static ubxType ut_CFG_MSG = { UBX_CFG, UBX_CFG_MSG };

typedef struct ubxMsg {
    ubxType type;
    slBuffer *body;
    ubxChecksum checksum;
} ubxMsg;


static void updateUbxChecksum(byte b, ubxChecksum *checksum) {
    checksum->ck_a += b;
    checksum->ck_b += checksum->ck_a;
}


// Calculate and return the Fletcher checksum for the given UBX message, without modifying the message at all.
static ubxChecksum calcFletcherChecksum(const ubxMsg msg) {

    ubxChecksum result = {0, 0};

    // first we get the class and id...
    updateUbxChecksum(msg.type.class, &result);
    updateUbxChecksum(msg.type.id, &result);

    // then the length, little-endian...
    updateUbxChecksum((byte) (size_slBuffer(msg.body)), &result);
    updateUbxChecksum((byte) (size_slBuffer(msg.body) >> 8), &result);

    // and finally the body itself...
    for (int i = 0; i < size_slBuffer(msg.body); i++) {
        updateUbxChecksum(buffer_slBuffer(msg.body)[i], &result);
    }

    return result;
}


ubxMsg createUbxMsg(ubxType type, slBuffer *body) {

    ubxMsg result;
    result.type     = type;
    result.body     = body;
    result.checksum = calcFletcherChecksum(result);
    return result;
}


// Returns if all the requested bytes were successfully read.
static slReturn readUbxBytes(int fdPort, byte *buffer, int *count, size_t max, int max_ms, long long startTime, long long charWaitMs) {

    *count = 0;

    while (((currentTimeMs() - startTime) < max_ms) && (*count < max)) {
        slReturn rscResp = readSerialChar(fdPort, charWaitMs);

        if (isErrorReturn(rscResp))
            makeErrorMsgReturn(ERR_CAUSE(rscResp), "couldn't read serial character");

        if (isWarningReturn(rscResp))
            makeErrorMsgReturn(ERR_ROOT, "timed out while trying to read UBX message");

        int c = getReturnInfoChar(rscResp);
        if (c >= 0) {
            buffer[*count] = (byte) c;
            (*count)++;
        }
    }
    return (*count >= max) ? makeOkReturn() : makeErrorMsgReturn(ERR_ROOT, "insufficient bytes read");
}


// Waits up to one second for a UBX message to be received.
// The body (an slBuffer) is only allocated if returns Ok.
#define UBX_SYNC1 (0xB5)
#define UBX_SYNC2 (0x62)
static slReturn rcvUbxMsg(int fdPort, ubxMsg* msg, int max_ms) {

    ubxMsg nomsg = { {0, 0}, NULL, {0, 0}};
    *msg = nomsg;

    long long startTime = currentTimeMs();

    while (((currentTimeMs() - startTime) < max_ms)) {

        // first we synchronize on the 0xB5, 0x62 byte pair that starts every UBX message...
        int count = 0;
        while (((currentTimeMs() - startTime) < max_ms) && (count < 2)) {
            slReturn rscResp = readSerialChar(fdPort, startTime + max_ms - currentTimeMs());

            if (isErrorReturn(rscResp))
                return makeErrorMsgReturn(ERR_CAUSE(rscResp), "couldn't read serial character");

            if (isWarningReturn(rscResp))
                break;  // if we timed out...

            int c= getReturnInfoChar(rscResp);
            if (c >= 0) {
                if ((count == 0) && (c == UBX_SYNC1)) {
                    count++;
                    continue;
                }
                if ((count == 1) && (c == UBX_SYNC2)) {
                    count++;
                    continue;
                }
                count = 0;
            }
        }
        if (count < 2) continue;

        // we're synchronized, so it's time to get the message type...
        byte buff[2];
        slReturn rubResp = readUbxBytes(fdPort, buff, &count, 2, max_ms, startTime, startTime + max_ms - currentTimeMs());
        if (isErrorReturn(rubResp))
            return makeErrorMsgReturn(ERR_CAUSE(rubResp), "couldn't read serial characters while receiving UBX message type");
        if (count < 2) continue;
        ubxType type = {buff[0], buff[1]};
        msg->type = type;

        // get the body length, little-endian...
        rubResp = readUbxBytes(fdPort, buff, &count, 2, max_ms, startTime, startTime + max_ms - currentTimeMs());
        if (isErrorReturn(rubResp))
            return makeErrorMsgReturn(ERR_CAUSE(rubResp), "couldn't read serial characters while receiving UBX message body length");
        if (count < 2) continue;
        size_t length = buff[0] | (buff[1] << 8);

        // now get the body...
        msg->body = create_slBuffer(length, LittleEndian);
        rubResp = readUbxBytes(fdPort, buffer_slBuffer(msg->body), &count, length, max_ms, startTime, startTime + max_ms - currentTimeMs());
        if (isErrorReturn(rubResp))
            return makeErrorFmtMsgReturn(ERR_CAUSE(rubResp),
                                          "couldn't read serial characters while receiving UBX message body (0x%02x/0x%02x, %d bytes)",
                                          type.class, type.id, length);
        msg->checksum = calcFletcherChecksum(*msg);  // this calculates what the checksum SHOULD be...
        if (count >= length) {

            // finally, get the checksum...
            rubResp = readUbxBytes(fdPort, buff, &count, 2, max_ms, startTime, startTime + max_ms - currentTimeMs());
            if (isErrorReturn(rubResp))
                return makeErrorMsgReturn(ERR_CAUSE(rubResp), "couldn't read serial characters while receiving UBX message checksum");
            if (count >= 2) {
                if ((buff[0] == msg->checksum.ck_a) && (buff[1] == msg->checksum.ck_b))
                    return makeOkReturn();
                else {
                    free(msg->body);
                    return makeErrorMsgReturn(ERR_ROOT, "UBX received message with checksum error");
                }
            }
        }
        free(msg->body);
    }

    return makeErrorMsgReturn(ERR_ROOT, "timed out while waiting to receive UBX message");
}


// Waits for an ACK/NAK message.
#define ACK_WAIT_MS 1500
static slReturn ubxAckWait(int fdPort) {

    ubxMsg ackMsg;
    slReturn result = rcvUbxMsg(fdPort, &ackMsg, ACK_WAIT_MS);
    if (isErrorReturn(result)) return makeErrorMsgReturn(ERR_CAUSE(result), "Problem receiving ACK message");

    free(ackMsg.body);

    if (ackMsg.type.class != UBX_ACK)
        return makeErrorFmtMsgReturn(ERR_ROOT, "Unexpected message received while waiting for ACK: %02x/%02x", ackMsg.type.class, ackMsg.type.id);
    else
        if (ackMsg.type.id == UBX_ACK_ACK)
            return makeOkReturn();
        else
            return makeErrorMsgReturn(ERR_ROOT, "Received a NAK");
}


// Transmits the given message to the UBX GPS, waiting for completion.
static slReturn sendUbxMsg(int fdPort, ubxMsg msg) {

    // first we send out our sync bytes, message type, and body length...
    byte buff[6] = { UBX_SYNC1,
                     UBX_SYNC2,
                     msg.type.class,
                     msg.type.id,
                     (byte) size_slBuffer(msg.body),
                     (byte)(size_slBuffer(msg.body) >> 8)
    };

    if (sizeof(buff) != write(fdPort, buff, sizeof(buff)))
        return makeErrorFmtMsgReturn(ERR_ROOT, "error writing UBX message header: %s", strerror(errno));

    // then we send out the body itself...
    if (size_slBuffer(msg.body) != write(fdPort, buffer_slBuffer(msg.body), size_slBuffer(msg.body)))
        return makeErrorFmtMsgReturn(ERR_ROOT, "error writing UBX message body: %s", strerror(errno));

    // and finally, out goes the checksum...
    byte buffChk[2] = {msg.checksum.ck_a, msg.checksum.ck_b};
    if (sizeof(buffChk) != write(fdPort, buffChk, sizeof(buffChk)))
        return makeErrorFmtMsgReturn(ERR_ROOT, "error writing UBX message checksum: %s", strerror(errno));

    // now we wait for everything to actually be sent...
    if (tcdrain(fdPort))
        return makeErrorFmtMsgReturn(ERR_ROOT, "error waiting for tcdrain: %s", strerror(errno));

    return makeOkReturn();
}


// Send the given ubxMsg, then wait for an ACK and return the result.
static slReturn sendUbxAckedMsg(int fdPort, ubxMsg msg) {
    slReturn result = sendUbxMsg(fdPort, msg);
    if (isErrorReturn(result)) return result;
    return ubxAckWait(fdPort);
}


// Returns true if the two given ubxType arguments are equal.
static bool cmpUbxType(ubxType a, ubxType b) {
    return (a.class == b.class) && (a.id == b.id);
}


// Polls the GPX with the given message, then waits for the response and possibly an ACK/NAK.  The response message
// body is only allocated on an ok response.
static slReturn pollUbx(int fdPort, ubxMsg pollMsg, int max_ms, ubxMsg* answer) {

    // send the poll message...
    slReturn txResult = sendUbxMsg(fdPort, pollMsg);
    free(pollMsg.body);
    if (isErrorReturn(txResult)) return makeErrorMsgReturn(ERR_CAUSE(txResult), "Problem sending poll message");

    // now get the poll response message...
    slReturn resp = rcvUbxMsg(fdPort, answer, max_ms);
    if (isErrorReturn(resp)) return makeErrorMsgReturn(ERR_CAUSE(resp), "Problem receiving poll response");

    // make sure it's the response type we expected...
    if (cmpUbxType(pollMsg.type, answer->type)) {

        // certain poll messages (notably configuration) ALSO send an ACK-ACK; for those, get it...
        if (pollMsg.type.class == UBX_CFG) {

            // get our ack, if our class of messages so requires...
            slReturn ackResp = ubxAckWait(fdPort);
            if (isErrorReturn(ackResp))
                return makeErrorMsgReturn(ERR_CAUSE(ackResp), "Problem receiving expected ACK for poll");
        }
        return makeOkReturn();
    }
    else {

        // we might have received a NAK (in the case of an invalid poll message being sent),
        // or maybe we got complete garbage, so we distinguish between them...
        slReturn errResp;
        if (cmpUbxType(answer->type, ut_ACK_NAK))
            errResp = makeErrorMsgReturn(ERR_ROOT, "received NAK in response to UBX poll");
        else
            errResp = makeErrorFmtMsgReturn(ERR_ROOT, "received unexpected message type (0x%02x-0x%02x) in response to UBX poll",
                                            answer->type.class, answer->type.id);
        free(answer->body);
        return errResp;
    }
}


// Corrects the raw time as reported by the GPS.
static void correctTime(ubxFix *fix) {

    // no matter what, we're gonna add the nanoseconds correction to the seconds...
    fix->second += ((int64_t) fix->nanoseconds_correction) * 1e-9;

    // if the result is positive, we're done...
    if (fix->second >= 0.0) return;

    // otherwise, the time was rounded up and now we have to undo that...
    fix->second += 60;  // fix the seconds wrap-around - note this is wrong for leap-seconds, but how to fix that?

    // now fix the minutes...
    fix->minute--;
    if (fix->minute >= 0) return;
    fix->minute += 60;

    // then the hours...
    fix->hour--;
    if (fix->hour >= 0) return;
    fix->hour += 24;

    // we rounded to the next day - sheesh, fix it...
    fix->day--;
    if (fix->day >= 1) return;

    // oh oh - we rounded up from the preceding month!
    // now this starts to get a little complicated - we have to know the days in a month, which means leap years...
    fix->month--;
    if (fix->month >= 1) {
        fix->day = daysInMonth((unsigned) fix->year, (unsigned) fix->month);
        return;
    }

    // sheesh - we even rounded up the year!
    fix->year--;
    fix->month = 12;
    fix->day = daysInMonth((unsigned) fix->year, (unsigned) fix->month);
}


// Resets the U-Blox GPS (hardware reset).
extern slReturn ubxReset(int fdPort, int verbosity) {

    // construct our reset message (which won't be acknowledged in any way)...
    slBuffer* body = create_slBuffer(4, LittleEndian);
    put_uint16_slBuffer(body, 0xffff, 0);  // we're doing a cold start...
    put_uint8_slBuffer(body, 2, 0);  // immediate restart...
    ubxType type = { UBX_CFG, UBX_CFG_RST };
    ubxMsg msg = createUbxMsg(type, body);
    slReturn sumResp = sendUbxMsg(fdPort, msg);
    return sumResp;
}

// Query NMEA Message config
extern slReturn isNMEAmsgEnabled(int fdPort, int verbosity, nmeaMSG messageID) {
    // poll for the current configuration...
    ubxMsg poll = createUbxMsg(ut_CFG_MSG, init_slBuffer(LittleEndian, 0xf0, messageID));
    ubxMsg current;
    slReturn resp = pollUbx(fdPort, poll, CFG_NMEA_MAX_MS, &current);
    if (isErrorReturn(resp)) return resp;

    bool state =  get_uint8_slBuffer(current.body, 3);
    return makeOkInfoReturn(bool2info(state));
}

// Enable / disable NMEA Message types
extern slReturn ubxEnableNMEAMsg(int fdPort, int verbosity, nmeaMSG messageID, bool enable) {
    // construct the configuration message...
    slBuffer* body = create_slBuffer(8, LittleEndian);

    put_uint8_slBuffer(body,  0, 0xf0);             //  Message type NMEA
    put_uint8_slBuffer(body,  1, messageID);
    put_uint8_slBuffer(body,  2, 0x00);             //  other ports
    put_uint8_slBuffer(body,  3, (enable) ? 1 : 0); //  Serial port
    put_uint8_slBuffer(body,  4, 0x00);             //  other ports
    put_uint8_slBuffer(body,  5, 0x00);             //  other ports
    put_uint8_slBuffer(body,  6, 0x00);             //  other ports
    put_uint8_slBuffer(body,  7, 0x00);             //  other ports
    ubxMsg nmeaMsg = createUbxMsg(ut_CFG_MSG, body);

    // now send it, and wait for our ack...
    slReturn nmeaResp = sendUbxAckedMsg(fdPort, nmeaMsg);
    if (isErrorReturn(nmeaResp))
        return makeErrorMsgReturn(ERR_CAUSE(nmeaResp), "failed to enable /disable NMEA MSG");

    free(body);

    return makeOkReturn();
}


extern slReturn ubxConfigNMEAVersion(int fdPort, int verbosity, uint8_t nmeaVersion) {
    // configure the NMEA version...
    // get the NMEA configuration...
    ubxType nmeaType = { UBX_CFG, UBX_CFG_NMEA };
    slBuffer* body = create_slBuffer(0, LittleEndian);
    ubxMsg msg = createUbxMsg(nmeaType, body);
    ubxMsg nmeaMsg;
    slReturn nmeaResp = pollUbx(fdPort, msg, CFG_NMEA_MAX_MS, &nmeaMsg);
    if (isErrorReturn(nmeaResp))
        return makeErrorMsgReturn(ERR_CAUSE(nmeaResp), "problem getting NMEA Version information from GPS");

    // make the changes we need to make only if we need to...
    slBuffer* b = nmeaMsg.body;
    if (get_uint8_slBuffer(b,  1) != nmeaVersion) {
        put_uint8_slBuffer(b,  1,      nmeaVersion);
        // now send it back to the GPS...
        ubxMsg newnmeaMsg = createUbxMsg(nmeaMsg.type, b);
        nmeaResp = sendUbxAckedMsg(fdPort, newnmeaMsg);
        if (isErrorReturn(nmeaResp))
            return makeErrorMsgReturn(ERR_CAUSE(nmeaResp), "problem sending NMEA Version configuration to GPS");
    }
    free(body);
    free(nmeaMsg.body);

    return makeOkReturn();
}


// Configure satellites
extern slReturn ubxConfigSatellites(int fdPort, int verbosity) {

    // configure the GNSS for GPS, GLONASS, BeiDou, and Galileo, with no SBAS.
    // first read the current configuration...
    int minch, maxch, enabled, beidou_enabled, galileo_enabled, glonass_enabled, gps_enabled, nmeaver;
    double dnmeaver;

    gps_enabled = iniparser_getboolean(gpsctlConf, "gps:enabled", true);
    galileo_enabled = iniparser_getboolean(gpsctlConf, "galileo:enabled", true);

    // only one of BeiDou and Glonass can be enabled at the same time if GPS or Galileo are enabled
    beidou_enabled = iniparser_getboolean(gpsctlConf, "beidou:enabled", false);
    glonass_enabled = iniparser_getboolean(gpsctlConf, "glonass:enabled", true);
    if (beidou_enabled && glonass_enabled && (gps_enabled || galileo_enabled)) {
      if (iniparser_getboolean(gpsctlConf, "gpsctl:prefer beidou to glonass if both enabled", false))
        glonass_enabled = false;
      else
        beidou_enabled = false;
    }
    ubxType gnssType = { UBX_CFG, UBX_CFG_GNSS };
    slBuffer* body = create_slBuffer(0, LittleEndian);
    ubxMsg msg = createUbxMsg(gnssType, body);
    ubxMsg gnssMsg;
    slReturn gnssResp = pollUbx(fdPort, msg, CFG_GNSS_MAX_MS, &gnssMsg);
    if (isErrorReturn(gnssResp))
        return makeErrorMsgReturn(ERR_CAUSE(gnssResp), "problem getting GNSS information from GPS");

    // make the changes we need to make...
    uint8_t gnssRecs = get_uint8_slBuffer(gnssMsg.body, 3);  // number of configuration blocks...
    slBuffer* b = gnssMsg.body;
    for (int i = 0; i < gnssRecs; i++) {

        size_t o = 4 + 8 * (size_t)i;
        gnssID id = (gnssID) get_uint8_slBuffer(b,  o + 0);
        uint32_t flags;
        switch (id) {

            case GPS:

                // enable it...
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, gps_enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = iniparser_getint(gpsctlConf, "gps:minimum channels", 8);
                maxch = iniparser_getint(gpsctlConf, "gps:maximum channels", 16);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 8...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 16...

                break;

            case SBAS:

                // disable everything else...
                enabled = iniparser_getboolean(gpsctlConf, "sbas:enabled", false);
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = iniparser_getint(gpsctlConf, "sbas:minimum channels", 1);
                maxch = iniparser_getint(gpsctlConf, "sbas:maximum channels", 3);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 8...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 16...

                break;

            case Galileo:

                // enable it...
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, galileo_enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = iniparser_getint(gpsctlConf, "galileo:minimum channels", 4);
                maxch = iniparser_getint(gpsctlConf, "galileo:maximum channels", 8);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 4...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 8...

                break;

            case BeiDou:

                // disable everything else...
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, beidou_enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = iniparser_getint(gpsctlConf, "beidou:minimum channels", 8);
                maxch = iniparser_getint(gpsctlConf, "beidou:maximum channels", 16);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 8...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 16...

                break;

            case IMES:

                // disable everything else...
                enabled = iniparser_getboolean(gpsctlConf, "imes:enabled", false);
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = iniparser_getint(gpsctlConf, "imes:minimum channels", 0);
                maxch = iniparser_getint(gpsctlConf, "imes:maximum channels", 8);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 0...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 8...

                break;

            case QZSS:

                // disable everything else...
                enabled = iniparser_getboolean(gpsctlConf, "qzss:enabled", false);
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = iniparser_getint(gpsctlConf, "qzss:minimum channels", 0);
                maxch = iniparser_getint(gpsctlConf, "qzss:maximum channels", 3);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 0...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 3...

                break;

            case GLONASS:

                // enable it...
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, glonass_enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = iniparser_getint(gpsctlConf, "glonass:minimum channels", 8);
                maxch = iniparser_getint(gpsctlConf, "glonass:maximum channels", 14);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 8...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 14...

                break;

        }
    }

    // now send it back to the GPS...
    ubxMsg newGnssMsg = createUbxMsg(gnssMsg.type, b);  // this recomputes the checksum...
    slReturn suamResp = sendUbxAckedMsg(fdPort, newGnssMsg);
    if (isErrorReturn(suamResp))
        return makeErrorMsgReturn(ERR_CAUSE(suamResp), "problem sending GNSS configuration to GPS");
    free(body);
    free(gnssMsg.body);

    // We don't care if NMEA version is 4.0, 40, 4.1, or 41, etc, we do the right thing anyway
    dnmeaver = iniparser_getdouble(gpsctlConf, "nmea:version", 41) + 0.005;
    if (dnmeaver < 20.0) {
        dnmeaver = dnmeaver * 10.0;
    }
    nmeaver = (int) dnmeaver;
    // force nmea version 41 or higher if BeiDou or Galileo are enabled
    if ((beidou_enabled || galileo_enabled) && nmeaver < 41) {
        nmeaver = 41;
    }
    // convert to from integer 40, 41, etc to hex 0x40, 0x41
    nmeaver = (nmeaver / 10) * 16 + (nmeaver % 10);
    ubxConfigNMEAVersion(fdPort, verbosity, nmeaver);

    return makeOkReturn();
}


// Configure Time Pulse
extern slReturn ubxConfigTimePulse(int fdPort, int verbosity) {
    // configure the time pulse...
    // get the time pulse configuration...
    ubxType tp5Type = { UBX_CFG, UBX_CFG_TP5 };
    slBuffer* body = create_slBuffer(1, LittleEndian);
    *buffer_slBuffer(body) = 0;  // we only look at time pulse zero...
    ubxMsg msg = createUbxMsg(tp5Type, body);
    ubxMsg tp5Msg;
    slReturn tp5Resp = pollUbx(fdPort, msg, CFG_TP5_MAX_MS, &tp5Msg);
    if (isErrorReturn(tp5Resp))
        return makeErrorMsgReturn(ERR_CAUSE(tp5Resp), "problem getting time pulse information from GPS");

    // make the changes we need to make...
    slBuffer* b = tp5Msg.body;
    put_uint16_slBuffer(b,  4, iniparser_getint(gpsctlConf, "time pulse:antenna cable delay", 56));         // set the antenna cable delay to 56 ns...
    put_uint16_slBuffer(b,  6, iniparser_getint(gpsctlConf, "time pulse:rf group delay", 20));              // set the RF group delay to 20 ns...
    put_uint32_slBuffer(b,  8, iniparser_getint(gpsctlConf, "time pulse:unlocked pulse period", 1000000));  // set the unlocked period to 1 second (1,000,000 microseconds)...
    put_uint32_slBuffer(b, 16, iniparser_getint(gpsctlConf, "time pulse:unlocked pulse length", 0));        // set the unlocked pulse length to 0 seconds...
    put_uint32_slBuffer(b, 12, iniparser_getint(gpsctlConf, "time pulse:locked pulse period", 1000000));    // set the locked period to 1 second (1,000,000 microseconds)...
    put_uint32_slBuffer(b, 20, iniparser_getint(gpsctlConf, "time pulse:locked pulse length", 500000));     // set the locked pulse length to 0.5 seconds (500,000 microseconds)...
    put_uint32_slBuffer(b, 24, iniparser_getint(gpsctlConf, "time pulse:user configurable delay", 0));      // set the user-configurable delay to zero...
    put_uint32_slBuffer(b, 28,    0x77);  // set the flags...

    // now send it back to the GPS...
    ubxMsg newTp5Msg = createUbxMsg(tp5Msg.type, b);
    slReturn suamResp = sendUbxAckedMsg(fdPort, newTp5Msg);
    if (isErrorReturn(suamResp))
        return makeErrorMsgReturn(ERR_CAUSE(suamResp), "problem sending time pulse configuration to GPS");
    free(body);
    free(tp5Msg.body);

    return makeOkReturn();
}

extern slReturn ubxConfigNavEngine(int fdPort, int verbosity) {
    // get the navigation engine configuration...
    ubxType nav5Type = { UBX_CFG, UBX_CFG_NAV5 };
    slBuffer* body = create_slBuffer(0, LittleEndian);
    ubxMsg msg = createUbxMsg(nav5Type, body);
    ubxMsg nav5Msg;
    slReturn nav5Resp = pollUbx(fdPort, msg, CFG_NAV5_MAX_MS, &nav5Msg);
    if (isErrorReturn(nav5Resp))
        return makeErrorMsgReturn(ERR_CAUSE(nav5Resp), "problem getting navigation engine information from GPS");

    // make our changes...
    slBuffer* b = nav5Msg.body;
    put_uint16_slBuffer(b,  0, 0x0577);  // configure all settings...
    put_uint8_slBuffer(b,  2, iniparser_getint(gpsctlConf, "navigation engine:dynamic model", Stationary));                  // use stationary mode...
    put_uint8_slBuffer(b,  3, iniparser_getint(gpsctlConf, "navigation engine:fix mode", Only3D));                         // use any mode...
    put_uint32_slBuffer(b,  4, (int) (iniparser_getdouble(gpsctlConf, "navigation engine:fixed altitude (2d)", 0) * 100.0)); // fixed altitude for 2D (not used)...
    put_uint32_slBuffer(b,  8, (int) (iniparser_getdouble(gpsctlConf, "navigation engine:fixed altitude variance (2d)", 0) * 10000.0));  // fixed altitude variance for 2D (1 m^2)...
    put_uint8_slBuffer(b, 12, iniparser_getint(gpsctlConf, "navigation engine:minimum elevation", 20));                       // minimum elevation is 5 degrees...
    put_uint16_slBuffer(b, 14, (int) (iniparser_getdouble(gpsctlConf, "navigation engine:position dop mask", 10) * 10.0));   // position DoP mask is 10.0...
    put_uint16_slBuffer(b, 16, (int) (iniparser_getdouble(gpsctlConf, "navigation engine:time dop mask", 10) * 10.0));       // time DoP mask is 10.0...
    put_uint16_slBuffer(b, 18, iniparser_getint(gpsctlConf, "navigation engine:position accuracy mask", 40));               // position accuracy mask is 100 meters...
    put_uint16_slBuffer(b, 20, iniparser_getint(gpsctlConf, "navigation engine:time accuracy mask", 40));                   // time accuracy mask is 300 m...
    put_uint8_slBuffer(b, 22, iniparser_getint(gpsctlConf, "navigation engine:static hold threshold", 0));                   // static hold threshold is 0 cm/s...
    put_uint8_slBuffer(b, 23, iniparser_getint(gpsctlConf, "navigation engine:dynamic gnss timeout", 60));                   // dynamic GNSS timeout is 60 seconds (not used)...
    put_uint8_slBuffer(b, 24, iniparser_getint(gpsctlConf, "navigation engine:threshold above c/no", 8));                    // number of satellites that must be above CNo threshold...
    put_uint8_slBuffer(b, 25, iniparser_getint(gpsctlConf, "navigation engine:c/no threshold", 20));                          // CNo threshold...
    put_uint16_slBuffer(b, 28, iniparser_getint(gpsctlConf, "navigation engine:static hold max distance", 0));               // static hold distance threshold...
    put_uint8_slBuffer(b, 30, iniparser_getint(gpsctlConf, "navigation engine:utc standard", USNO_UTC));                     // use USNO UTC...

    // send it back to the GPS...
    ubxMsg newNav5Msg = createUbxMsg(nav5Msg.type, b);
    slReturn suamResp = sendUbxAckedMsg(fdPort, newNav5Msg);
    if (isErrorReturn(suamResp))
        return makeErrorMsgReturn(ERR_CAUSE(suamResp), "problem sending navigation engine configuration to GPS");
    free(body);
    free(nav5Msg.body);

    return makeOkReturn();
}

// Configures the GPS for maximum timing accuracy...
extern slReturn ubxConfigForTiming(int fdPort, int verbosity) {

    // configure the GNSS for GPS, GLONASS, BeiDou, and Galileo, with no SBAS.
    ubxConfigSatellites(fdPort, verbosity);

    // configure the time pulse...
    ubxConfigTimePulse(fdPort, verbosity);

    // configure the navigation engine...
    ubxConfigNavEngine(fdPort, verbosity);

    // Suppress NMEA output except for ZDA messages
    ubxEnableNMEAMsg(fdPort, verbosity, GGA, iniparser_getboolean(gpsctlConf, "nmea:gga", false));
    ubxEnableNMEAMsg(fdPort, verbosity, GLL, iniparser_getboolean(gpsctlConf, "nmea:gll", false));
    ubxEnableNMEAMsg(fdPort, verbosity, GSA, iniparser_getboolean(gpsctlConf, "nmea:gsa", false));
    ubxEnableNMEAMsg(fdPort, verbosity, GSV, iniparser_getboolean(gpsctlConf, "nmea:gsv", false));
    ubxEnableNMEAMsg(fdPort, verbosity, RMC, iniparser_getboolean(gpsctlConf, "nmea:rmc", true));
    ubxEnableNMEAMsg(fdPort, verbosity, VTG, iniparser_getboolean(gpsctlConf, "nmea:vtg", false));
    ubxEnableNMEAMsg(fdPort, verbosity, GRS, iniparser_getboolean(gpsctlConf, "nmea:grs", false));
    ubxEnableNMEAMsg(fdPort, verbosity, GST, iniparser_getboolean(gpsctlConf, "nmea:gst", false));
    ubxEnableNMEAMsg(fdPort, verbosity, ZDA, iniparser_getboolean(gpsctlConf, "nmea:zda", true));

    return makeOkReturn();
}


// Fills the fix structure at the given pointer with information about a time and position fix obtained from the
// GPS system.
#define NAV_PVT_BODY_SIZE 92
extern slReturn ubxGetFix(int fdPort, int verbosity, ubxFix* fix) {

    // get a fix from the GPS...
    ubxMsg fixMsg;
    slReturn pollResp = pollUbx(fdPort, createUbxMsg(ut_NAV_PVT, create_slBuffer(0, LittleEndian)), NAV_PVT_MAX_MS, &fixMsg);
    if (isErrorReturn(pollResp)) return pollResp;
    if (size_slBuffer(fixMsg.body) < NAV_PVT_BODY_SIZE)
        return makeErrorFmtMsgReturn(ERR_ROOT, "unexpected NAV_PVT message body size: %d bytes", size_slBuffer(fixMsg.body));

    // convenience variable...
    slBuffer *body = fixMsg.body;

    fix->year                              = get_uint16_slBuffer(body, 4);
    fix->month                             = get_uint8_slBuffer(body, 6);
    fix->day                               = get_uint8_slBuffer(body, 7);
    fix->hour                              = get_uint8_slBuffer(body, 8);
    fix->minute                            = get_uint8_slBuffer(body, 9);
    fix->second                            = get_uint8_slBuffer(body, 10);
    fix->nanoseconds_correction            = get_int32_slBuffer(body, 16);
    fix->time_accuracy_ns                  = get_uint32_slBuffer(body, 12);
    fix->date_valid                        = isBitSet_slBits(get_uint8_slBuffer(body, 11), 0);
    fix->time_valid                        = isBitSet_slBits(get_uint8_slBuffer(body, 11), 1);
    fix->time_resolved                     = isBitSet_slBits(get_uint8_slBuffer(body, 11), 2);
    fix->fix_is_3d                         = (get_uint8_slBuffer(body, 20) == 3);
    fix->fix_valid                         = isBitSet_slBits(get_uint8_slBuffer(body, 21), 0);
    fix->number_of_satellites_used         = get_uint8_slBuffer(body, 23);
    fix->latitude_deg                      = get_int32_slBuffer(body, 28) * 1e-7;
    fix->longitude_deg                     = get_int32_slBuffer(body, 24) * 1e-7;
    fix->height_above_ellipsoid_mm         = get_int32_slBuffer(body, 32);
    fix->height_above_sea_level_mm         = get_int32_slBuffer(body, 36);
    fix->height_accuracy_mm                = get_uint32_slBuffer(body, 44);
    fix->horizontal_accuracy_mm            = get_uint32_slBuffer(body, 40);
    fix->ground_speed_mm_s                 = get_int32_slBuffer(body, 60);
    fix->ground_speed_accuracy_mm_s        = get_uint32_slBuffer(body, 68);
    fix->heading_deg                       = get_int32_slBuffer(body, 64) * 1e-5;
    fix->heading_accuracy_deg              = get_uint32_slBuffer(body, 72) * 1e-5;
    #undef body
    correctTime(fix);

    return makeOkReturn();
}


// Fills the ubxVersion structure at the given pointer with information about the GPS's version.
extern slReturn ubxGetVersion(int fdPort, int verbosity, ubxVersion* version) {

    ubxType type = { UBX_MON, UBX_MON_VER };
    slBuffer* body = create_slBuffer(0, LittleEndian);
    ubxMsg msg = createUbxMsg(type, body);
    ubxMsg versionMsg;
    slReturn result = pollUbx(fdPort, msg, MON_VER_MAX_MS, &versionMsg);
    if (isErrorReturn(result)) return result;

    // decode the message and print the results...
    char* buf = (char *) buffer_slBuffer(versionMsg.body);

    version->software = getAllocatedStringCopy(buf);
    version->hardware = getAllocatedStringCopy(buf + 30);

    // calculate the number of extension entries we have...
    int ne = ((int) size_slBuffer(versionMsg.body) - 40) / 30;
    version->number_of_extensions = ne;
    if (ne == 0)
        version->extensions = NULL;
    else {
        version->extensions = safeMalloc((size_t) (sizeof(buf) * ne));
        char** ptr = version->extensions;
        for (int i = 40; i < (int) size_slBuffer(versionMsg.body); i += 30) {
            *ptr = getAllocatedStringCopy(buf + i);
            ptr++;
        }
        *ptr = NULL;
    }

    return makeOkReturn();
}


extern char* getSignalQuality(signalQuality qual) {
    switch (qual) {
        case signalNone: return "None";
        case signalAcquired: return "Acquired";
        case signalCodeCarrierLocked: return "Code/carrier locked";
        case signalCodeLocked: return "Code locked";
        case signalSearching: return "Searching";
        case signalUnusable: return "Unusable";
        default: return "Unknown";
    }
}


extern char* getSatelliteHealth(satelliteHealth health) {
    switch (health) {
        case healthUnknown: return "Unknown";
        case healthOk: return "Ok";
        case healthBad: return "Bad";
        default: return "Unknown";
    }
}


extern char* getOrbitSource(orbitSource source) {
    switch (source) {
        case osNone: return "None";
        case osAlmanac: return "Almanac";
        case osAssistNowAutonomous: return "AssistNow auto";
        case osAssistNowOffline: return "AssistNow off";
        case osEphemeris: return "Ephemeris";
        case osOther: return "Other";
        default: return "Unknown";
    }
}


// Fills the ubxSatellites structure at the given pointer with information about the GPS satellites.
extern slReturn ubxGetSatellites(int fdPort, int verbosity, ubxSatellites* satellites) {

    ubxType type = { UBX_NAV, UBX_NAV_SAT };
    slBuffer* body = create_slBuffer(0, LittleEndian);
    ubxMsg msg = createUbxMsg(type, body);
    ubxMsg satMsg;
    slReturn nsResp = pollUbx(fdPort, msg, MON_VER_MAX_MS, &satMsg);
    if (isErrorReturn(nsResp)) return nsResp;

    // now decode our message, starting with the number of satellites...
    void* b = satMsg.body;
    ubxSatellites* ss = satellites;
    ss->numberOfSatellites = get_uint8_slBuffer(b, 5);

    // make a place to store our results...
    ss->satellites = safeMalloc(ss->numberOfSatellites * sizeof(ubxSatellite));

    // now examine each of our satellites...
    for (int i = 0; i < ss->numberOfSatellites; i++) {
        ubxSatellite* s = ss->satellites + i;
        size_t o = (unsigned) (8 + 12 * i);

        s->gnssID               = get_uint8_slBuffer(b, o + 0);
        s->satelliteID          = get_uint8_slBuffer(b, o + 1);
        s->cno                  = get_uint8_slBuffer(b, o + 2);
        s->elevation            = get_int8_slBuffer(b, o + 3);
        s->azimuth              = get_int16_slBuffer(b, o + 4);
        s->pseudoRangeResidualM = get_int8_slBuffer(b, o + 6) * 0.1;

        uint32_t flags       = get_uint32_slBuffer(b, o + 8);
        s->signalQuality     = (signalQuality) min_i((int) getBitField_slBits(flags, 0x07), 5);
        s->used              = isBitSet_slBits(flags, 3);
        s->health            = (satelliteHealth) getBitField_slBits(flags, 0x30);
        s->diffCorr          = isBitSet_slBits(flags, 6);
        s->smoothed          = isBitSet_slBits(flags, 7);
        s->orbitSource       = (orbitSource) min_i((int) getBitField_slBits(flags, 0x700), 5);
        s->haveEphemeris     = isBitSet_slBits(flags, 11);
        s->haveAlmanac       = isBitSet_slBits(flags, 12);
        s->haveAssistNowOff  = isBitSet_slBits(flags, 13);
        s->haveAssistNowAuto = isBitSet_slBits(flags, 14);
        s->sbasCorrUsed      = isBitSet_slBits(flags, 16);
        s->rtcmCorrUsed      = isBitSet_slBits(flags, 17);
        s->prCorrUsed        = isBitSet_slBits(flags, 20);
        s->crCorrUsed        = isBitSet_slBits(flags, 21);
        s->doCorrUsed        = isBitSet_slBits(flags, 22);
    }

    return makeOkReturn();
}


extern char* getGnssName(gnssID id) {
    switch (id) {
        case GPS:     return "GPS";
        case SBAS:    return "SBAS";
        case Galileo: return "Galileo";
        case BeiDou:  return "BeiDou";
        case IMES:    return "IMES";
        case QZSS:    return "QZSS";
        case GLONASS: return "GLONASS";
        default:      return "unknown";
    }
}


extern char* getDynamicModelName(dynModel model) {
    switch (model) {
        case Portable:   return "Portable";
        case Stationary: return "Stationary";
        case Pedestrian: return "Pedestrian";
        case Automotive: return "Automotive";
        case Sea:        return "Sea";
        case Air1G:      return "Air (less than 1G acceleration)";
        case Air2G:      return "Air (less than 2G acceleration)";
        case Air4G:      return "Air (less than 4G acceleration)";
        case Watch:      return "Watch";
        default:         return "unknown";
    }
}


extern char* getFixModeName(fixMode mode) {
    switch (mode) {
        case Only2D:   return "2D only";
        case Only3D:   return "3D only";
        case Auto2D3D: return "Auto 2D/3D";
        default:       return "unknown";
    }
}


extern char* getUTCTypeName(utcType utc) {
    switch (utc) {
        case AutoUTC:     return "Auto";
        case USNO_UTC:    return "USNO";
        case GLONASS_UTC: return "GLONASS";
        case BeiDou_UTC:  return "BeiDou";
        default:          return "unknown";
    }
}


extern char* getFixTimeRefName(fixTimeRefType type) {
    switch (type) {
        case fixUTC:     return "UTC";
        case fixGPS:     return "GPS";
        case fixGLONASS: return "GLONASS";
        case fixBeiDou:  return "BeiDou";
        case fixGalileo: return "Galileo";
        default:         return "unknown";
    }
}


extern char* getPowerModeName(powerModeType type) {
    switch (type) {
        case pmFull:          return "Full";
        case pmBalanced:      return "Balanced";
        case pmInterval:      return "Interval";
        case pmAggressive1Hz: return "Aggressive (1 Hz)";
        case pmAggressive2Hz: return "Aggressive (2 Hz)";
        case pmAggressive4Hz: return "Aggressive (3 Hz)";
        case pmInvalid:       return "Invalid";
        default:              return "unknown";
    }
}


// Fills the ubxVersion structure at the given pointer with information about the GPS's version.  Returns the
// appropriate reportResponse value.
extern char* getTimeGridTypeName(timegridType type) {
    switch (type) {
        case tgUTC: return "UTC";
        case tgGPS: return "GPS";
        case tgGLONASS: return "GLONASS";
        case tgBeiDou: return "BeiDou";
        case tgGalileo: return "Galileo";
        default: return "unknown";
    }
}

// Returns the current state of NMEA data on the GPS's UART.  If the return value is ok, the result is stored
// as a bool in the information field.
static slReturn isNmeaOn(int fdPort) {

    // poll for the current configuration...
    ubxMsg poll = createUbxMsg(ut_CFG_PRT, init_slBuffer(LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */));
    ubxMsg current;
    slReturn resp = pollUbx(fdPort, poll, CFG_PRT_MAX_MS, &current);
    if (isErrorReturn(resp)) return resp;

    bool state =  isBitSet_slBits(get_uint16_slBuffer(current.body, 14), 1);
    return makeOkInfoReturn(bool2info(state));
}

// Fills the ubxConfig structure at the given pointer with information about the GPS's configuration.
extern slReturn ubxGetConfig(int fdPort, int verbosity, ubxConfig* config) {

    // get the antenna configuration...
    ubxType antType = { UBX_CFG, UBX_CFG_ANT };
    slBuffer* body = create_slBuffer(0, LittleEndian);
    ubxMsg msg = createUbxMsg(antType, body);
    ubxMsg antMsg;
    slReturn antResp = pollUbx(fdPort, msg, CFG_ANT_MAX_MS, &antMsg);
    if (isErrorReturn(antResp))
        return makeErrorMsgReturn(ERR_CAUSE(antResp), "problem getting antenna information from GPS");
    uint16_t antMask = get_uint16_slBuffer(antMsg.body, 0);
    free(body);
    free(antMsg.body);
    config->antPwr          = isBitSet_slBits(antMask, 0);
    config->antShrtDet      = isBitSet_slBits(antMask, 1);
    config->antOpenDet      = isBitSet_slBits(antMask, 2);
    config->antPwrDwnOnShrt = isBitSet_slBits(antMask, 3);
    config->antAutoRec      = isBitSet_slBits(antMask, 4);

    // get the GNSS configuration...
    ubxType gnssType = { UBX_CFG, UBX_CFG_GNSS };
    body = create_slBuffer(0, LittleEndian);
    msg = createUbxMsg(gnssType, body);
    ubxMsg gnssMsg;
    slReturn gnssResp = pollUbx(fdPort, msg, CFG_GNSS_MAX_MS, &gnssMsg);
    if (isErrorReturn(gnssResp))
        return makeErrorMsgReturn(ERR_CAUSE(gnssResp), "problem getting GNSS information from GPS");
    config->trkChnnls = get_uint8_slBuffer(gnssMsg.body, 1);
    config->gnssRecs = get_uint8_slBuffer(gnssMsg.body, 3);  // number of configuration blocks...
    config->gnss = safeMalloc(sizeof(ubxGNSSConfig) * config->gnssRecs);
    for (int i = 0; i < config->gnssRecs; i++) {
        config->gnss[i].id        = (gnssID) get_uint8_slBuffer(gnssMsg.body,  4 + 8 * (size_t)i);
        config->gnss[i].minChnnls =          get_uint8_slBuffer(gnssMsg.body,  5 + 8 * (size_t)i);
        config->gnss[i].maxChnnls =          get_uint8_slBuffer(gnssMsg.body,  6 + 8 * (size_t)i);
        config->gnss[i].enabled   = (bool)  (get_uint8_slBuffer(gnssMsg.body,  8 + 8 * (size_t)i) & 0x01);
        config->gnss[i].sigConfig =          get_uint8_slBuffer(gnssMsg.body, 10 + 8 * (size_t)i);
    }
    free(body);
    free(gnssMsg.body);

    // get the navigation engine configuration...
    ubxType nav5Type = { UBX_CFG, UBX_CFG_NAV5 };
    body = create_slBuffer(0, LittleEndian);
    msg = createUbxMsg(nav5Type, body);
    ubxMsg nav5Msg;
    slReturn nav5Resp = pollUbx(fdPort, msg, CFG_NAV5_MAX_MS, &nav5Msg);
    if (isErrorReturn(nav5Resp))
        return makeErrorMsgReturn(ERR_CAUSE(nav5Resp), "problem getting navigation engine information from GPS");
    config->model               = (dynModel) get_uint8_slBuffer(nav5Msg.body,  2);
    config->mode                = (fixMode)  get_uint8_slBuffer(nav5Msg.body,  3);
    config->fixedAltM           = 0.01 *     get_int32_slBuffer(nav5Msg.body,  4);
    config->fixedAltVarM2       = 0.0001 *   get_int32_slBuffer(nav5Msg.body,  8);
    config->minElevDeg          =            get_int8_slBuffer(nav5Msg.body, 12);
    config->pDoP                = 0.1 *      get_uint16_slBuffer(nav5Msg.body, 14);
    config->tDoP                = 0.1 *      get_uint16_slBuffer(nav5Msg.body, 16);
    config->pAccM               =            get_uint16_slBuffer(nav5Msg.body, 18);
    config->tAccM               =            get_uint16_slBuffer(nav5Msg.body, 20);
    config->staticHoldThreshCmS =            get_uint8_slBuffer(nav5Msg.body, 22);
    config->dgnssTimeoutS       =            get_uint8_slBuffer(nav5Msg.body, 23);
    config->cnoThreshNumSVs     =            get_uint8_slBuffer(nav5Msg.body, 24);
    config->cnoThreshDbHz       =            get_uint8_slBuffer(nav5Msg.body, 25);
    config->staticHoldMaxDistM  =            get_uint16_slBuffer(nav5Msg.body, 28);
    config->utcStandard         = (utcType)  get_uint8_slBuffer(nav5Msg.body, 30);
    free(body);
    free(nav5Msg.body);

    // get the time pulse configuration...
    ubxType tp5Type = { UBX_CFG, UBX_CFG_TP5 };
    body = create_slBuffer(1, LittleEndian);
    *buffer_slBuffer(body) = 0;  // we only look at time pulse zero...
    msg = createUbxMsg(tp5Type, body);
    ubxMsg tp5Msg;
    slReturn tp5Resp = pollUbx(fdPort, msg, CFG_TP5_MAX_MS, &tp5Msg);
    if (isErrorReturn(tp5Resp))
        return makeErrorMsgReturn(ERR_CAUSE(tp5Resp), "problem getting time pulse information from GPS");
    config->antCableDelayNs   = get_int16_slBuffer(tp5Msg.body,  4);
    config->rfGroupDelayNs    = get_int16_slBuffer(tp5Msg.body,  6);
    config->freqPeriod        = get_uint32_slBuffer(tp5Msg.body,  8);
    config->freqPeriodLock    = get_uint32_slBuffer(tp5Msg.body, 12);
    config->pulseLenRatio     = get_uint32_slBuffer(tp5Msg.body, 16);
    config->pulseLenRatioLock = get_uint32_slBuffer(tp5Msg.body, 20);
    config->userConfigDelay   = get_int32_slBuffer(tp5Msg.body, 24);
    uint32_t flags            = get_uint32_slBuffer(tp5Msg.body, 28);
    config->timePulse0Enabled = isBitSet_slBits(flags, 0);
    config->lockGpsFreq      = isBitSet_slBits(flags, 1);
    config->lockedOtherSet    = isBitSet_slBits(flags, 2);
    config->isFreq            = isBitSet_slBits(flags, 3);
    config->isLength          = isBitSet_slBits(flags, 4);
    config->alignToTow        = isBitSet_slBits(flags, 5);
    config->polarity          = isBitSet_slBits(flags, 6);
    config->gridUtcTnss       = (timegridType) getBitField_slBits(flags, 0x000000780);
    free(body);
    free(tp5Msg.body);

    // get the fix rate configuration...
    ubxType rateType = { UBX_CFG, UBX_CFG_RATE };
    body = create_slBuffer(0, LittleEndian);
    msg = createUbxMsg(rateType, body);
    ubxMsg rateMsg;
    slReturn rateResp = pollUbx(fdPort, msg, CFG_RATE_MAX_MS, &rateMsg);
    if (isErrorReturn(rateResp))
        return makeErrorMsgReturn(ERR_CAUSE(rateResp), "problem getting fix rate information from GPS");
    config->measRateMs =                  get_uint16_slBuffer(rateMsg.body, 0);
    config->navRate    =                  get_uint16_slBuffer(rateMsg.body, 2);
    config->timeRef    = (fixTimeRefType) get_uint16_slBuffer(rateMsg.body, 4);
    free(body);
    free(rateMsg.body);

    // get the power mode configuration...
    ubxType pmsType = { UBX_CFG, UBX_CFG_PMS };
    body = create_slBuffer(0, LittleEndian);
    msg = createUbxMsg(pmsType, body);
    ubxMsg pmsMsg;
    slReturn pmsResp = pollUbx(fdPort, msg, CFG_PMS_MAX_MS, &pmsMsg);
    if (isErrorReturn(pmsResp))
        return makeErrorMsgReturn(ERR_CAUSE(pmsResp), "problem getting power mode information from GPS");
    config->powerSetup        = (powerModeType) get_uint8_slBuffer(pmsMsg.body, 1);
    config->powerIntervalSecs =                 get_uint16_slBuffer(pmsMsg.body, 2);
    config->powerOnTimeSecs   =                 get_uint16_slBuffer(pmsMsg.body, 4);
    free(body);
    free(pmsMsg.body);

    // get the NMEA configuration...
    // is NMEA enabled?
    slReturn cn = isNmeaOn(fdPort);
    if (isErrorReturn(cn)) return cn;
    config->nmeaEnabled = getReturnInfoBool(cn);

    // get NMEA Version
    ubxType nmeaType = { UBX_CFG, UBX_CFG_NMEA };
    body = create_slBuffer(0, LittleEndian);
    msg = createUbxMsg(nmeaType, body);
    ubxMsg nmeaMsg;
    slReturn nmeaResp = pollUbx(fdPort, msg, CFG_NMEA_MAX_MS, &nmeaMsg);
    if (isErrorReturn(nmeaResp))
        return makeErrorMsgReturn(ERR_CAUSE(nmeaResp), "problem getting NMEA Version information from GPS");
    config->nmeaVersion        = get_uint8_slBuffer(nmeaMsg.body, 1);
    free(body);
    free(nmeaMsg.body);

    // Get NMEA sentence config
    cn = isNMEAmsgEnabled(fdPort, verbosity, GGA);
    if (isErrorReturn(cn)) return cn;
    config->GGA = getReturnInfoBool(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GLL);
    if (isErrorReturn(cn)) return cn;
    config->GLL = getReturnInfoBool(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GSA);
    if (isErrorReturn(cn)) return cn;
    config->GSA = getReturnInfoBool(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GSV);
    if (isErrorReturn(cn)) return cn;
    config->GSV = getReturnInfoBool(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, RMC);
    if (isErrorReturn(cn)) return cn;
    config->RMC = getReturnInfoBool(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, VTG);
    if (isErrorReturn(cn)) return cn;
    config->VTG = getReturnInfoBool(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GRS);
    if (isErrorReturn(cn)) return cn;
    config->GRS = getReturnInfoBool(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GST);
    if (isErrorReturn(cn)) return cn;
    config->GST = getReturnInfoBool(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, ZDA);
    if (isErrorReturn(cn)) return cn;
    config->ZDA = getReturnInfoBool(cn);

    return makeOkReturn();
}


// Returns Ok with additional information true if synchronized, false if timed out.  Note that this function
// should be called prior to any series of UBX operations to ensure the UART receiver is synced to the data stream.
extern slReturn ubxSynchronizer(int fdPort, int maxTimeMs, int verbosity) {

    long long start = currentTimeMs();

    // flush any junk we might have in the receiver buffer...
    slReturn frResp = flushRx(fdPort);
    if (isErrorReturn(frResp))
        makeErrorMsgReturn(ERR_CAUSE(frResp), "could not flush receiver prior to synchronization");

    // read characters until either we appear to be synchronized or we run out of time...
    while (currentTimeMs() < start + maxTimeMs) {

        // send the request message and wait for the response...
        ubxType type = {UBX_MON, UBX_MON_VER};
        slBuffer *body = create_slBuffer(0, LittleEndian);
        ubxMsg msg = createUbxMsg(type, body);
        ubxMsg ans;
        slReturn result = pollUbx(fdPort, msg, MON_VER_SYNC_MAX_MS, &ans);
        if (isOkReturn(result))
            return makeOkInfoReturn(bool2info(true));
    }
    return makeOkInfoReturn(bool2info(false));
}


extern slReturn ubxSaveConfig(int fdPort, int verbosity) {

    // construct the save configuration message...
    slBuffer* saveMsgBody = create_slBuffer(13, LittleEndian);
    put_uint32_slBuffer(saveMsgBody,  0, 0x0000);  // clear mask all off...
    put_uint32_slBuffer(saveMsgBody,  4, 0x041B);  // save mask: ioPort, msgConf, navConf, rxmConf, antConf all on...
    put_uint32_slBuffer(saveMsgBody,  8, 0x0000);  // load mask all off...
    put_uint8_slBuffer(saveMsgBody, 12, 0x01);     // battery-backed RAM...
    ubxMsg saveMsg = createUbxMsg(ut_CFG_CFG, saveMsgBody);

    // now send it, and wait for our ack...
    slReturn submResp = sendUbxAckedMsg(fdPort, saveMsg);
    if (isErrorReturn(submResp))
        return makeErrorMsgReturn(ERR_CAUSE(submResp), "failed to save GPS configuration");

    if (verbosity >= 1) printf("Saved GPS configuration\n");
    return makeOkReturn();
}


// Configures whether the GPS transmits NMEA periodically on the UART.  Returns success or failure.
extern slReturn ubxSetNMEAData(int fdPort, int verbosity, bool nmeaOn) {

    char* onoff = nmeaOn ? "on" : "off";

    // first we poll for the current configuration...
    if (verbosity >= 3) printf("Querying to see if GPS UART already has NMEA data turned %s...\n", onoff);
    slReturn cn = isNmeaOn(fdPort);
    if (isErrorReturn(cn)) return cn;

    // extract our state information...
    bool currentState = getReturnInfoBool(cn);

    // if we're already in the state we want, just leave...
    if (currentState == nmeaOn) {
        if (verbosity >= 3) printf("GPS UART already has NMEA data turned %s...\n", onoff);
        return makeOkReturn();
    }

    // change the configuration to the way we want it, starting with polling it again...
    if (verbosity >= 3) printf("Polling GPS UART configuration...\n");
    ubxMsg poll = createUbxMsg(ut_CFG_PRT, init_slBuffer(LittleEndian, 1));  // UART is index 1; defines won't work here...
    ubxMsg current;
    slReturn resp = pollUbx(fdPort, poll, CFG_PRT_MAX_MS, &current);
    if (isErrorReturn(resp)) return resp;

    // now change the bit we need to diddle...
    slBuffer *bdy = current.body;
    put_uint16_slBuffer(bdy, 14, (uint16_t) setBit_slBits(get_uint16_slBuffer(bdy, 14), 1, nmeaOn));

    // send out the message to actually make the change...
    if (verbosity >= 3) printf("Sending new configuration to GPS with NMEA data turned %s for the UART...\n", onoff);
    slReturn ackResp = sendUbxAckedMsg(fdPort, createUbxMsg(ut_CFG_PRT, bdy));
    if (isOkReturn(ackResp)) {
        if (verbosity >= 1) printf("Successfully set new configuration to turn NMEA data %s for GPS UART...\n", onoff);
        return makeOkReturn();
    }
    else {
        if (verbosity >= 1) printf("Failed to change GPS UART configuration to turn NMEA data %s...\n", onoff);
        return makeErrorFmtMsgReturn(ERR_CAUSE(ackResp), "failed to change GPS UART configuration to turn NMEA data %s: %s", onoff, getReturnMsg(ackResp));
    }
}


extern slReturn ubxChangeBaudRate(int fdPort, unsigned int newBaudRate, int verbosity) {

    // first we poll for the current configuration...
    ubxMsg current;
    ubxMsg pollMsg = createUbxMsg(ut_CFG_PRT, init_slBuffer(LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */));
    slReturn pollAns = pollUbx(fdPort, pollMsg, CFG_PRT_MAX_MS, &current);
    if (isErrorReturn(pollAns))
        return makeErrorMsgReturn(ERR_CAUSE(pollAns), "Problem polling for current configuration before changing baud rate");

    // update the baud rate in the configuration...
    put_uint32_slBuffer(current.body, 8, newBaudRate);

    // send out the message to change the baud rate...
    slReturn sendResp = sendUbxMsg(fdPort, createUbxMsg(ut_CFG_PRT, current.body));
    if (isErrorReturn(sendResp))
        return makeErrorMsgReturn(ERR_CAUSE(sendResp), "Problem sending message to change baud rate");

    // change our host's baud rate to the new one...
    slReturn stoResp = setTermOptionsBaud(fdPort, newBaudRate);
    if (isErrorReturn(stoResp))
        return makeErrorMsgReturn(ERR_CAUSE(stoResp), "Problem changing the host baud rate");

    // now get our ACK...
    slReturn ackResp = ubxAckWait(fdPort);

    // sometimes we don't get the ACK, likely hosed by the baud rate change...
    // so in that case we'll query the port and see what we actually have...
    if (isErrorReturn(ackResp)) {

        // poll the current configuration again...
        if (verbosity > 1) printf("ACK missing, repolling for status...\n");
        ubxMsg repollResp;
        ubxMsg repollMsg = createUbxMsg(ut_CFG_PRT, init_slBuffer(LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */));
        slReturn pollResp = pollUbx(fdPort, repollMsg, CFG_PRT_MAX_MS, &repollResp);
        if (isErrorReturn(pollResp))
            return makeErrorMsgReturn(ERR_CAUSE(pollResp), "Problem on second attempt to verify changed baud rate");
    }

    if (verbosity >= 1) printf("Successfully changed baud rate to %d...\n", newBaudRate);

    return makeOkReturn();
}
