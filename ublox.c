//
// Created by Tom Dilatush on 10/20/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <limits.h>
#include <math.h>
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
#define UBX_SYNC_BACKLOG_CHARS 1024  // allowance for queued output ahead of a reply, in characters at the line rate
#define CFG_GNSS_MAX_MS     1000
#define GNSS_SETTLE_MS      500   // u-blox: after the ACK of a UBX-CFG-GNSS, wait this long before the next command
#define GNSS_RESTART_WAIT_MS 500  // after asking for a restart, leave the receiver this long to go down before polling it
#define GNSS_RESTART_MAX_MS 10000
#define FACTORY_BAUD 9600       // the receiver's default UART rate // ...and up to this long, in all, for it to answer again
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

// Forward declaration: the body-size check is defined below the message decoders that use it...
static slReturn validateBodySize(ubxMsg* msg, size_t minSize, const char* name);


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
    for (size_t i = 0; i < size_slBuffer(msg.body); i++) {
        updateUbxChecksum(buffer_slBuffer(msg.body)[i], &result);
    }

    return result;
}


static ubxMsg createUbxMsg(ubxType type, slBuffer *body) {

    ubxMsg result;
    result.type     = type;
    result.body     = body;
    result.checksum = calcFletcherChecksum(result);
    return result;
}


// Bytes from a frame that failed its checksum.  A bad frame doesn't mean its bytes are junk: a corrupt or bogus length
// (typical when the receive buffer starts mid-message) can swallow the start of the real message that follows, so those
// bytes are fed back through the synchronizer, minus the sync pair that led to the bad frame, before reading any more
// from the port.
typedef struct ubxReplay {
    byte   *data;
    size_t  len;
    size_t  pos;
} ubxReplay;


// Reads one byte for the UBX parser: from the replay buffer if it has any left, otherwise from the serial port.
// Returns what readSerialChar() does: Ok with the byte as info, a warning on timeout, or an error.
static slReturn readUbxByte(int fdPort, ubxReplay *replay, long long msTimeout) {

    if (replay->pos < replay->len)
        return makeOkInfoReturn(char2info(replay->data[replay->pos++]));
    return readSerialChar(fdPort, msTimeout);
}


// Returns if all the requested bytes were successfully read.
static slReturn readUbxBytes(int fdPort, ubxReplay *replay, byte *buffer, int *count, size_t max, int max_ms, long long startTime, long long charWaitMs) {

    *count = 0;

    // (bytes being replayed after a bad frame don't wait on the port, so they're not bound by the time limit)...
    while ((((currentTimeMs() - startTime) < max_ms) || (replay->pos < replay->len)) && ((size_t) *count < max)) {
        slReturn rscResp = readUbxByte(fdPort, replay, charWaitMs);

        if (isErrorReturn(rscResp))
            return makeErrorMsgReturn(ERR_CAUSE(rscResp), "couldn't read serial character");

        if (isWarningReturn(rscResp)) {
            freeReturn(rscResp);
            return makeErrorMsgReturn(ERR_ROOT, "timed out while trying to read UBX message");
        }

        int c = getReturnInfoChar(rscResp);
        freeReturn(rscResp);
        if (c >= 0) {
            buffer[*count] = (byte) c;
            (*count)++;
        }
    }
    return ((size_t) *count >= max) ? makeOkReturn() : makeErrorMsgReturn(ERR_ROOT, "insufficient bytes read");
}


// Called when the frame just read (type, length, body and checksum) turned out to be bad: queues everything after its
// first byte-pair of sync for another look, ahead of anything still unread on the port.  Anything not yet consumed from
// an earlier replay is kept, after the new bytes.
static void queueReplay(ubxReplay *replay, const byte *hdr, slBuffer *body, const byte *chk) {

    size_t bodyLen = size_slBuffer(body);
    size_t leftover = replay->len - replay->pos;
    size_t total = 4 + bodyLen + 2 + leftover;
    byte *data = safeMalloc(total);

    memcpy(data, hdr, 4);
    memcpy(data + 4, buffer_slBuffer(body), bodyLen);
    memcpy(data + 4 + bodyLen, chk, 2);
    if (leftover > 0) memcpy(data + 4 + bodyLen + 2, replay->data + replay->pos, leftover);

    free(replay->data);
    replay->data = data;
    replay->len  = total;
    replay->pos  = 0;
}


// Queues the four header bytes (class, id, length) that followed a B5 62 pair we've decided wasn't a frame start, for
// another look ahead of anything still unread on the port.  Anything not yet consumed from an earlier replay is kept,
// after the new bytes.
static void queueReplayHeader(ubxReplay *replay, const byte *hdr) {

    size_t leftover = replay->len - replay->pos;
    byte *data = safeMalloc(4 + leftover);

    memcpy(data, hdr, 4);
    if (leftover > 0) memcpy(data + 4, replay->data + replay->pos, leftover);

    free(replay->data);
    replay->data = data;
    replay->len  = 4 + leftover;
    replay->pos  = 0;
}


// The longest message body we'll believe.  The biggest the receiver sends us (NAV-SAT with every channel, CFG-GNSS,
// MON-VER with its extensions) is well under 4 KB, so a longer length means the B5 62 before it was noise, and waiting
// for that many bytes could swallow a genuine reply that arrives meanwhile.
#define UBX_MAX_BODY 4096


// Waits up to max_ms for a UBX message to be received.  The body (an slBuffer) is only allocated if returns Ok.
#define UBX_SYNC1 (0xB5)
#define UBX_SYNC2 (0x62)
static slReturn rcvUbxMsgReplay(int fdPort, ubxMsg* msg, int max_ms, ubxReplay *replay) {

    ubxMsg nomsg = { {0, 0}, NULL, {0, 0}};
    *msg = nomsg;

    long long startTime = currentTimeMs();

    // keep going while there's time, or while bytes from a bad frame are still waiting to be re-examined...
    while (((currentTimeMs() - startTime) < max_ms) || (replay->pos < replay->len)) {

        // first we synchronize on the 0xB5, 0x62 byte pair that starts every UBX message...
        int count = 0;
        while ((((currentTimeMs() - startTime) < max_ms) || (replay->pos < replay->len)) && (count < 2)) {
            slReturn rscResp = readUbxByte(fdPort, replay, startTime + max_ms - currentTimeMs());

            if (isErrorReturn(rscResp))
                return makeErrorMsgReturn(ERR_CAUSE(rscResp), "couldn't read serial character");

            if (isWarningReturn(rscResp)) {
                freeReturn(rscResp);
                break;  // if we timed out...
            }

            int c= getReturnInfoChar(rscResp);
            freeReturn(rscResp);
            if (c >= 0) {
                if ((count == 0) && (c == UBX_SYNC1)) {
                    count++;
                    continue;
                }
                if ((count == 1) && (c == UBX_SYNC2)) {
                    count++;
                    continue;
                }
                // a byte that breaks the pair may itself be the first half of the next pair (B5 B5 62...), so keep it...
                count = (c == UBX_SYNC1) ? 1 : 0;
            }
        }
        if (count < 2) continue;

        // we're synchronized, so it's time to get the message type and the body length (kept, with the checksum
        // below, so that a bad frame can be replayed)...
        byte hdr[4];
        slReturn rubResp = readUbxBytes(fdPort, replay, hdr, &count, 4, max_ms, startTime, startTime + max_ms - currentTimeMs());
        if (isErrorReturn(rubResp))
            return makeErrorMsgReturn(ERR_CAUSE(rubResp), "couldn't read serial characters while receiving UBX message header");
        freeReturn(rubResp);
        if (count < 4) continue;
        ubxType type = {hdr[0], hdr[1]};
        msg->type = type;
        size_t length = hdr[2] | (hdr[3] << 8);  // little-endian...

        // a length no real message has means this wasn't a frame start: look through the header bytes again instead
        // of waiting for up to 64 KB of "body" that would swallow whatever genuine message comes next...
        if (length > UBX_MAX_BODY) {
            queueReplayHeader(replay, hdr);
            continue;
        }

        // now get the body...
        msg->body = create_slBuffer(length, LittleEndian);
        rubResp = readUbxBytes(fdPort, replay, buffer_slBuffer(msg->body), &count, length, max_ms, startTime, startTime + max_ms - currentTimeMs());
        if (isErrorReturn(rubResp)) {
            free(msg->body);
            msg->body = NULL;
            return makeErrorFmtMsgReturn(ERR_CAUSE(rubResp),
                                          "couldn't read serial characters while receiving UBX message body (0x%02x/0x%02x, %zu bytes)",
                                          type.class, type.id, length);
        }
        freeReturn(rubResp);
        msg->checksum = calcFletcherChecksum(*msg);  // this calculates what the checksum SHOULD be...
        if ((size_t) count >= length) {

            // finally, get the checksum...
            byte buff[2];
            rubResp = readUbxBytes(fdPort, replay, buff, &count, 2, max_ms, startTime, startTime + max_ms - currentTimeMs());
            if (isErrorReturn(rubResp)) {
                free(msg->body);
                msg->body = NULL;
                return makeErrorMsgReturn(ERR_CAUSE(rubResp), "couldn't read serial characters while receiving UBX message checksum");
            }
            freeReturn(rubResp);
            if (count >= 2) {
                if ((buff[0] == msg->checksum.ck_a) && (buff[1] == msg->checksum.ck_b))
                    return makeOkReturn();

                // A bad checksum usually means we synchronized on a B5 62 that wasn't really a frame start (the
                // receive buffer began mid-message, or a payload happened to contain the pair), so what we took for
                // this frame may have swallowed the start of a genuine one.  Look through its bytes again, and keep
                // hunting rather than giving up on the first bad frame...
                queueReplay(replay, hdr, msg->body, buff);
            }
        }
        free(msg->body);
        msg->body = NULL;
    }

    return makeErrorMsgReturn(ERR_ROOT, "timed out while waiting to receive UBX message");
}


// Waits up to max_ms for a UBX message to be received.  A frame with a bad checksum is skipped and the search for a
// good one goes on, so the error only shows up (as a timeout) if no good message arrives in time.  The body (an
// slBuffer) is only allocated if returns Ok.
static slReturn rcvUbxMsg(int fdPort, ubxMsg* msg, int max_ms) {

    ubxReplay replay = { NULL, 0, 0 };
    slReturn result = rcvUbxMsgReplay(fdPort, msg, max_ms, &replay);
    free(replay.data);
    return result;
}


// Extra time to allow, on top of a fixed wait for a reply, for output the receiver has already queued ahead of it: the
// time to receive UBX_SYNC_BACKLOG_CHARS characters at the port's current rate.  Negligible at 115200 baud and above
// (under 90 ms), but about a second at 9600 baud, where the line carries only 960 characters a second and a reply can
// sit behind a second's worth of NMEA.  Returns 0 if the rate can't be read.
static long long lineBacklogMs(int fdPort) {

    speedInfo si;
    slReturn gsiResp = getSpeedInfo(fdPort, &si);
    long long ms = isOkReturn(gsiResp) ? (long long) si.nsChar * UBX_SYNC_BACKLOG_CHARS / 1000000 : 0;
    freeReturn(gsiResp);
    return ms;
}


// Waits for the ACK/NAK that refers to the given message type, for at most max_ms milliseconds.  An ACK or NAK for
// some other message (one left over from an earlier command, or from a poll) is not ours to interpret and is
// ignored, so that a stale ACK can never be mistaken for the answer to the command we just sent.  Anything else is
// most likely a late reply to an earlier poll, so it's remembered and ignored too, until our waiting time runs out.
#define ACK_WAIT_MS 1500
#define POLL_ACK_DRAIN_MS 250
static slReturn ubxAckWait(int fdPort, ubxType sentType, int max_ms) {

    long long startTime = currentTimeMs();
    ubxType lastOther = { 0, 0 };
    while (true) {

        // work out what's left of our waiting time, so that ignoring stray messages can't extend it...
        long long remaining = max_ms - (currentTimeMs() - startTime);
        if (remaining <= 0) {
            if ((lastOther.class != 0) || (lastOther.id != 0))
                return makeErrorFmtMsgReturn(ERR_ROOT, "timed out waiting for ACK, last message received was 0x%02x-0x%02x",
                                             lastOther.class, lastOther.id);
            return makeErrorMsgReturn(ERR_ROOT, "timed out while waiting for ACK message");
        }

        // get the next message...
        ubxMsg ackMsg;
        slReturn result = rcvUbxMsg(fdPort, &ackMsg, (int) remaining);
        if (isErrorReturn(result)) return makeErrorMsgReturn(ERR_CAUSE(result), "Problem receiving ACK message");
        freeReturn(result);

        // anything that isn't an ACK/NAK is not for us, so remember it and keep waiting...
        if (ackMsg.type.class != UBX_ACK) {
            lastOther = ackMsg.type;
            free(ackMsg.body);
            continue;
        }

        // an ACK/NAK names the message it refers to, so drop any that isn't about ours (a stale one)...
        uint8_t ackClass = get_uint8_slBuffer(ackMsg.body, 0);
        uint8_t ackId    = get_uint8_slBuffer(ackMsg.body, 1);
        free(ackMsg.body);
        if ((ackClass != sentType.class) || (ackId != sentType.id))
            continue;

        // this one is ours...
        if (ackMsg.type.id == UBX_ACK_ACK)
            return makeOkReturn();

        return makeErrorMsgReturn(ERR_ROOT, "Received a NAK");
    }
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

    // the writes below may take only part of what they are given, or none of it, and the rest still has to go out:
    // a half-sent message would leave the receiver waiting for an end that never comes...
    slReturn hdrResp = writeSerialData(fdPort, buff, sizeof(buff));
    if (isErrorReturn(hdrResp))
        return makeErrorMsgReturn(ERR_CAUSE(hdrResp), "error writing UBX message header");
    freeReturn(hdrResp);

    // then we send out the body itself...
    slReturn bodyResp = writeSerialData(fdPort, buffer_slBuffer(msg.body), size_slBuffer(msg.body));
    if (isErrorReturn(bodyResp))
        return makeErrorMsgReturn(ERR_CAUSE(bodyResp), "error writing UBX message body");
    freeReturn(bodyResp);

    // and finally, out goes the checksum...
    byte buffChk[2] = {msg.checksum.ck_a, msg.checksum.ck_b};
    slReturn chkResp = writeSerialData(fdPort, buffChk, sizeof(buffChk));
    if (isErrorReturn(chkResp))
        return makeErrorMsgReturn(ERR_CAUSE(chkResp), "error writing UBX message checksum");
    freeReturn(chkResp);

    // now we wait for everything to actually be sent...
    if (tcdrain(fdPort))
        return makeErrorFmtMsgReturn(ERR_ROOT, "error waiting for tcdrain: %s", strerror(errno));

    return makeOkReturn();
}


// Send the given ubxMsg, then wait for an ACK and return the result.
static slReturn sendUbxAckedMsg(int fdPort, ubxMsg msg) {
    slReturn result = sendUbxMsg(fdPort, msg);
    if (isErrorReturn(result)) return result;
    return ubxAckWait(fdPort, msg.type, (int) (ACK_WAIT_MS + lineBacklogMs(fdPort)));  // (queued output can delay the ACK at slow rates)
}


// Returns true if the two given ubxType arguments are equal.
static bool cmpUbxType(ubxType a, ubxType b) {
    return (a.class == b.class) && (a.id == b.id);
}


// Polls the GPX with the given message, then waits for the response and possibly an ACK/NAK.  NOTE: pollUbx takes
// ownership of pollMsg.body and frees it; callers must NOT free or reuse it afterwards.  The response message
// body is only allocated on an ok response.
static slReturn pollUbx(int fdPort, ubxMsg pollMsg, int max_ms, ubxMsg* answer) {

    // discard anything already sitting in the receiver before we ask our question: late replies to earlier
    // polls (the synchronizer may send several before one succeeds) and NMEA remnants can be queued up, and
    // we must not mistake one of them for the answer to this poll...
    slReturn frResp = flushRx(fdPort);
    freeReturn(frResp);

    // send the poll message...
    slReturn txResult = sendUbxMsg(fdPort, pollMsg);
    free(pollMsg.body);
    if (isErrorReturn(txResult)) return makeErrorMsgReturn(ERR_CAUSE(txResult), "Problem sending poll message");
    freeReturn(txResult);

    // A poll may be answered by more than one message, and the receiver is free to send them in any order.  It
    // may also still have an ACK-ACK, or a reply to an earlier poll, queued up or in flight.  So keep reading
    // until we see the response we actually asked for, noting any ACK-ACK along the way...
    long long startTime = currentTimeMs();
    max_ms += (int) lineBacklogMs(fdPort);  // (queued output can delay the reply at slow rates)
    bool gotAck = false;
    ubxType unexpectedType = {0, 0};
    bool sawUnexpected = false;
    while (true) {

        // work out what's left of our overall polling time, so that skipping stray messages can't extend it...
        long long remaining = max_ms - (currentTimeMs() - startTime);
        if (remaining <= 0) {
            if (sawUnexpected)
                return makeErrorFmtMsgReturn(ERR_ROOT, "received unexpected message type (0x%02x-0x%02x) in response to UBX poll",
                                             unexpectedType.class, unexpectedType.id);
            return makeErrorMsgReturn(ERR_ROOT, "timed out while waiting for UBX poll response");
        }

        // now get the next message...
        slReturn resp = rcvUbxMsg(fdPort, answer, (int) remaining);
        if (isErrorReturn(resp)) return makeErrorMsgReturn(ERR_CAUSE(resp), "Problem receiving poll response");
        freeReturn(resp);

        // if this is the response we asked for, we're done...
        if (cmpUbxType(pollMsg.type, answer->type))
            break;

        // we might have received a NAK (in the case of an invalid poll message being sent)...
        if (cmpUbxType(answer->type, ut_ACK_NAK)) {
            free(answer->body);
            answer->body = NULL;
            return makeErrorMsgReturn(ERR_ROOT, "received NAK in response to UBX poll");
        }

        // or an ACK-ACK that arrived ahead of the response itself; remember it and keep waiting...
        if ((answer->type.class == UBX_ACK) && (answer->type.id == UBX_ACK_ACK)) {
            gotAck = true;
            free(answer->body);
            answer->body = NULL;
            continue;
        }

        // anything else is most likely a late reply to an earlier poll, so note it and keep waiting for our
        // own response rather than giving up on it...
        unexpectedType = answer->type;
        sawUnexpected = true;
        free(answer->body);
        answer->body = NULL;
    }

    // certain poll messages (notably configuration) ALSO send an ACK-ACK, but the response itself already tells us
    // the poll succeeded, so the ACK is best effort.  If it does show up, consume it so that it can't later be
    // mistaken for the ACK of a set command; if it doesn't, so be it...
    if ((pollMsg.type.class == UBX_CFG) && !gotAck) {
        ubxMsg ackMsg;
        slReturn ackResp = rcvUbxMsg(fdPort, &ackMsg, POLL_ACK_DRAIN_MS + (int) lineBacklogMs(fdPort));
        if (isOkReturn(ackResp)) free(ackMsg.body);
        freeReturn(ackResp);
    }

    return makeOkReturn();
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

    // we rounded to the next day - sheesh, fix it (but without a valid date there is no day to step back from)...
    if (!fix->date_valid) return;
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
    put_uint16_slBuffer(body, 0, 0xffff);  // navBbrMask: cold start
    put_uint8_slBuffer(body, 2, 0);        // resetMode:  immediate restart
    put_uint8_slBuffer(body, 3, 0);        // reserved
    ubxType type = { UBX_CFG, UBX_CFG_RST };
    ubxMsg msg = createUbxMsg(type, body);
    slReturn sumResp = sendUbxMsg(fdPort, msg);
    free(body);
    return sumResp;
}

// Query NMEA Message config
extern slReturn isNMEAmsgEnabled(int fdPort, int verbosity, nmeaMSG messageID) {
    // poll for the current configuration...
    ubxMsg poll = createUbxMsg(ut_CFG_MSG, init_slBuffer(LittleEndian, 0xf0, messageID));
    ubxMsg current;
    slReturn resp = pollUbx(fdPort, poll, CFG_NMEA_MAX_MS, &current);
    if (isErrorReturn(resp)) return resp;
    freeReturn(resp);

    // The body comes from the device, so never trust it: the rate for UART1 is the fourth byte, and a reply too short
    // to hold that byte would otherwise read as "disabled", reporting a configuration the receiver never sent...
    slReturn sizeResp = validateBodySize(&current, 4, "CFG-MSG");
    if (isErrorReturn(sizeResp)) return sizeResp;
    freeReturn(sizeResp);

    bool state =  get_uint8_slBuffer(current.body, 3);
    free(current.body);
    return makeOkInfoReturn(bool2info(state));
}

// Enable / disable NMEA Message types on the UART.  The message's current output rates are polled first, and only the
// UART1 rate is changed: a CFG-MSG sets the rate for every port, so sending zeros for the others (as this once did)
// would silently switch the message off on I2C, USB, and SPI.
extern slReturn ubxEnableNMEAMsg(int fdPort, int verbosity, nmeaMSG messageID, bool enable) {

    // poll for the message's current rates...
    ubxMsg poll = createUbxMsg(ut_CFG_MSG, init_slBuffer(LittleEndian, 0xf0, messageID));
    ubxMsg current;
    slReturn pollResp = pollUbx(fdPort, poll, CFG_NMEA_MAX_MS, &current);
    if (isErrorReturn(pollResp))
        return makeErrorMsgReturn(ERR_CAUSE(pollResp), "failed to get current NMEA MSG configuration");
    freeReturn(pollResp);

    // the body comes from the device and goes straight back to it, so it must be complete: msgClass, msgID, then one
    // rate for each of six ports (I2C, UART1, UART2, USB, SPI, reserved)...
    slReturn sizeResp = validateBodySize(&current, 8, "CFG-MSG");
    if (isErrorReturn(sizeResp)) return sizeResp;
    freeReturn(sizeResp);

    // if the serial port is already the way we want it, there's nothing to do...
    slBuffer* body = current.body;
    if ((get_uint8_slBuffer(body, 3) != 0) == enable) {
        free(body);
        return makeOkReturn();
    }

    // change only the rate for the serial port (UART1), leaving every other port as the receiver has it...
    put_uint8_slBuffer(body, 3, (enable) ? 1 : 0);
    ubxMsg nmeaMsg = createUbxMsg(ut_CFG_MSG, body);

    // now send it, and wait for our ack...
    slReturn nmeaResp = sendUbxAckedMsg(fdPort, nmeaMsg);
    free(body);
    if (isErrorReturn(nmeaResp))
        return makeErrorMsgReturn(ERR_CAUSE(nmeaResp), "failed to enable /disable NMEA MSG");
    freeReturn(nmeaResp);

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
    freeReturn(nmeaResp);

    // the body comes from the device and goes straight back to it, so it must be complete: the NMEA version is byte 1,
    // and a short body would otherwise be sent back unchanged (the put below is silently skipped)...
    slReturn nmeaSize = validateBodySize(&nmeaMsg, 4, "CFG-NMEA");
    if (isErrorReturn(nmeaSize)) return nmeaSize;
    freeReturn(nmeaSize);

    // make the changes we need to make only if we need to...
    slBuffer* b = nmeaMsg.body;
    if (get_uint8_slBuffer(b,  1) != nmeaVersion) {
        put_uint8_slBuffer(b,  1,      nmeaVersion);
        // now send it back to the GPS...
        ubxMsg newnmeaMsg = createUbxMsg(nmeaMsg.type, b);
        nmeaResp = sendUbxAckedMsg(fdPort, newnmeaMsg);
        if (isErrorReturn(nmeaResp)) {
            free(nmeaMsg.body);
            return makeErrorMsgReturn(ERR_CAUSE(nmeaResp), "problem sending NMEA Version configuration to GPS");
        }
        freeReturn(nmeaResp);
    }
    free(nmeaMsg.body);

    return makeOkReturn();
}


// Reads a yes/no value from the configuration.  iniparser_getboolean() decides on the first character(s) only and quietly
// returns the default for any other text, so "enabled = disabled" would leave a default-on setting ON.  A recognised
// value gives the same answer whatever default is passed, so asking with both defaults tells us whether the text was
// understood; if it wasn't, say so (on stderr) rather than silently doing something the user didn't ask for.  The
// result is the same as iniparser_getboolean()'s.
extern int getConfigBool(const char *key, int notfound) {
    int result = iniparser_getboolean(gpsctlConf, key, notfound);
    if (iniparser_find_entry(gpsctlConf, key) &&
        (iniparser_getboolean(gpsctlConf, key, 0) != iniparser_getboolean(gpsctlConf, key, 1))) {
        fprintf(stderr, "Warning: %s = \"%s\" is not a recognised yes/no value (use yes/no, true/false, on/off or 1/0); "
                        "using the default\n", key, iniparser_getstring(gpsctlConf, key, ""));
    }
    return result;
}


// Reads an integer setting from the configuration and checks that it is a number that fits the field it is going into.
// iniparser_getint() returns an int, and putting that into a byte or a 16-bit field used to truncate it silently:
// "maximum channels = 300" was sent as 44, and a negative unsigned value became a huge one.  It also reads text that
// isn't a number as 0 ("abc") or as its numeric prefix ("12abc" is 12), so a typo quietly became a setting.  Text that
// isn't entirely a number, or a value outside min..max, is reported (on stderr) and the default is used instead.
// Anything after the number (a unit, say) must be a comment: write "56 ; nanoseconds", not "56 nanoseconds".
// Numbers are read as iniparser reads them (strtol, base 0: decimal, 0x hex, leading-0 octal), as a long: the int
// that iniparser_getint() returns wraps a long value, so 2^32 + 10 would have arrived as a perfectly good 10.
extern int getConfigInt(const char *key, long notfound, long min, long max) {
    const char *text = iniparser_getstring(gpsctlConf, key, NULL);
    if (text == NULL) return (int) notfound;                         // not in the configuration: the default

    char *end;
    long value = strtol(text, &end, 0);
    if ((end == text) || (*end != 0)) {
        fprintf(stderr, "Warning: %s = \"%s\" is not a whole number; using the default (%ld)\n", key, text, notfound);
        return (int) notfound;
    }
    if ((value < min) || (value > max)) {
        fprintf(stderr, "Warning: %s = %ld is outside %ld..%ld; using the default (%ld)\n", key, value, min, max, notfound);
        return (int) notfound;
    }
    return (int) value;
}


// Reads a decimal setting from the configuration, as strictly as getConfigInt() reads an integer.  iniparser_getdouble()
// uses atof(), which reads "abc" as 0 and "2.5m" as 2.5, so a typo quietly became a setting.  Text that isn't entirely a
// finite number is reported (on stderr) and the default is used instead.  Units or other remarks belong in a comment
// (after ';' or '#').
extern double getConfigDouble(const char *key, double notfound) {
    const char *text = iniparser_getstring(gpsctlConf, key, NULL);
    if (text == NULL) return notfound;                               // not in the configuration: the default

    char *end;
    double value = strtod(text, &end);
    if ((end == text) || (*end != 0) || !isfinite(value)) {
        fprintf(stderr, "Warning: %s = \"%s\" is not a number; using the default (%g)\n", key, text, notfound);
        return notfound;
    }
    return value;
}


// A channel count goes into a single byte of the CFG-GNSS message.
static int getConfigChannels(const char *key, int notfound) {
    return getConfigInt(key, notfound, 0, 255);
}


// Configure satellites
// Sleep for a number of milliseconds.
static void sleepMs(int ms) {
    struct timespec ts = { ms / 1000, (long) (ms % 1000) * 1000000L };
    while (nanosleep(&ts, &ts) != 0 && errno == EINTR)
        ;
}

// Asks the receiver for a hardware reset after a controlled shutdown (CFG-RST resetMode 0x04) that keeps the ephemeris
// (navBbrMask 0, a hot start), then waits for it to come back and answer.  A hardware reset reloads the saved
// configuration and discards the current one.
static slReturn ubxHardwareRestartAndWait(int fdPort, int verbosity) {

    slBuffer* body = create_slBuffer(4, LittleEndian);
    put_uint16_slBuffer(body, 0, 0x0000);  // navBbrMask: hot start (nothing cleared)
    put_uint8_slBuffer(body, 2, 0x04);     // resetMode:  hardware reset (watchdog) after a controlled shutdown
    put_uint8_slBuffer(body, 3, 0);        // reserved
    ubxType type = { UBX_CFG, UBX_CFG_RST };
    ubxMsg msg = createUbxMsg(type, body);
    slReturn sendResp = sendUbxMsg(fdPort, msg);  // (never acknowledged)
    free(body);
    if (isErrorReturn(sendResp))
        return makeErrorMsgReturn(ERR_CAUSE(sendResp), "problem restarting the GPS");
    freeReturn(sendResp);

    // let it go down before we start asking whether it is there...
    sleepMs(GNSS_RESTART_WAIT_MS);
    slReturn syncResp = ubxSynchronizer(fdPort, GNSS_RESTART_MAX_MS, verbosity);
    if (isErrorReturn(syncResp))
        return makeErrorMsgReturn(ERR_CAUSE(syncResp), "problem waiting for the GPS to restart");
    bool answered = getReturnInfoBool(syncResp);
    freeReturn(syncResp);
    if (!answered)
        return makeErrorMsgReturn(ERR_ROOT, "the GPS did not answer after it was restarted");
    if (verbosity >= 2) printf("GPS restarted...\n");
    return makeOkReturn();
}

// u-blox M8 (SPG 3.01, SPG 3.51, EXT CORE 3.01): "If Galileo is enabled, UBX-CFG-GNSS must be followed by UBX-CFG-RST
// with resetMode set to Hardware reset."  A hardware reset reloads the saved configuration and discards the current
// one, so the new GNSS configuration (and everything else we have changed so far) is saved first.
static slReturn ubxRestartAfterGnssChange(int fdPort, int verbosity) {

    if (verbosity >= 1)
        printf("Saving the GPS configuration and restarting the GPS, as u-blox requires after changing GNSS with Galileo enabled...\n");
    slReturn saveResp = ubxSaveConfig(fdPort, verbosity);
    if (isErrorReturn(saveResp))
        return makeErrorMsgReturn(ERR_CAUSE(saveResp), "problem saving the GPS configuration before restarting the GPS");
    freeReturn(saveResp);

    return ubxHardwareRestartAndWait(fdPort, verbosity);
}

// Sets the receiver back to its factory defaults: CFG-CFG with every section cleared (clearMask 0xFFFF, which deletes
// the saved configuration) and loaded (loadMask 0xFFFF, which makes the defaults current), in battery-backed RAM,
// flash, EEPROM and SPI flash (deviceMask 0x17; devices the receiver doesn't have are ignored).  The defaults put the
// UART back at 9600 baud, so the host follows it there; the receiver is then restarted (as with any GNSS change, since
// the defaults include a GNSS configuration) from the now-empty saved configuration.  On success the port is at 9600.
extern slReturn ubxFactoryReset(int fdPort, int verbosity) {

    if (verbosity >= 1) printf("Restoring the GPS to its factory defaults (its serial port will be at %d baud)...\n", FACTORY_BAUD);

    slBuffer* body = create_slBuffer(13, LittleEndian);
    put_uint32_slBuffer(body,  0, 0xFFFF);  // clear mask: all sections
    put_uint32_slBuffer(body,  4, 0x0000);  // save mask: nothing
    put_uint32_slBuffer(body,  8, 0xFFFF);  // load mask: all sections
    put_uint8_slBuffer(body, 12, 0x17);     // BBR, flash, EEPROM, SPI flash
    ubxMsg msg = createUbxMsg(ut_CFG_CFG, body);
    slReturn sendResp = sendUbxMsg(fdPort, msg);
    free(body);
    if (isErrorReturn(sendResp))
        return makeErrorMsgReturn(ERR_CAUSE(sendResp), "problem sending the factory reset to the GPS");
    freeReturn(sendResp);

    // The receiver may ACK at the old rate, at the new one, or not at all, so don't depend on it: give it time to apply
    // the defaults, move the host to their rate (which also throws away anything unread), and see whether it answers...
    sleepMs(GNSS_RESTART_WAIT_MS);
    slReturn stoResp = setTermOptionsBaud(fdPort, FACTORY_BAUD);
    if (isErrorReturn(stoResp))
        return makeErrorMsgReturn(ERR_CAUSE(stoResp), "problem changing the host baud rate after the factory reset");
    freeReturn(stoResp);

    slReturn syncResp = ubxSynchronizer(fdPort, GNSS_RESTART_MAX_MS, verbosity);
    if (isErrorReturn(syncResp))
        return makeErrorMsgReturn(ERR_CAUSE(syncResp), "problem waiting for the GPS after the factory reset");
    bool answered = getReturnInfoBool(syncResp);
    freeReturn(syncResp);
    if (!answered)
        return makeErrorMsgReturn(ERR_ROOT, "the GPS did not answer at 9600 baud after the factory reset (try the -a option)");
    if (verbosity >= 2) printf("GPS answers at %d baud...\n", FACTORY_BAUD);

    slReturn restartResp = ubxHardwareRestartAndWait(fdPort, verbosity);
    if (isErrorReturn(restartResp))
        return makeErrorMsgReturn(ERR_CAUSE(restartResp), "problem restarting the GPS after the factory reset");
    freeReturn(restartResp);
    return makeOkReturn();
}

extern slReturn ubxConfigSatellites(int fdPort, int verbosity) {

    // configure the GNSS for GPS, GLONASS, BeiDou, and Galileo, with no SBAS.
    // first read the current configuration...
    int minch, maxch, enabled, beidou_enabled, galileo_enabled, glonass_enabled, gps_enabled, nmeaver;
    double dnmeaver;

    gps_enabled = getConfigBool("gps:enabled", true);
    galileo_enabled = getConfigBool("galileo:enabled", true);

    // only one of BeiDou and Glonass can be enabled at the same time if GPS or Galileo are enabled
    beidou_enabled = getConfigBool("beidou:enabled", false);
    glonass_enabled = getConfigBool("glonass:enabled", true);
    if (beidou_enabled && glonass_enabled && (gps_enabled || galileo_enabled)) {
      if (getConfigBool("gpsctl:prefer beidou to glonass if both enabled", false))
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
    freeReturn(gnssResp);

    // make the changes we need to make...
    slBuffer* b = gnssMsg.body;

    // the body comes from the device, so never trust its record count further than the body can actually hold
    // (4 header bytes, then 8 per block), and reject a body too short to hold even one block...
    size_t bodySize = size_slBuffer(b);
    if (bodySize < 12) {
        free(gnssMsg.body);
        return makeErrorFmtMsgReturn(ERR_ROOT, "unexpected CFG-GNSS message body size: %zu bytes", bodySize);
    }

    // keep what the receiver has now, to see whether we change anything...
    byte* before = safeMalloc(bodySize);
    if (before == NULL) {
        free(gnssMsg.body);
        return makeErrorMsgReturn(ERR_ROOT, "out of memory");
    }
    memcpy(before, buffer_slBuffer(b), bodySize);
    uint8_t gnssRecs = get_uint8_slBuffer(b, 3);  // number of configuration blocks...
    size_t maxRecs = (bodySize - 4) / 8;  // (not uint8_t: a long body would wrap this, e.g. 256 blocks to 0)
    if (gnssRecs > maxRecs) gnssRecs = maxRecs;
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
                minch = getConfigChannels("gps:minimum channels", 8);
                maxch = getConfigChannels("gps:maximum channels", 16);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 8...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 16...

                break;

            case SBAS:

                // disable everything else...
                enabled = getConfigBool("sbas:enabled", false);
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = getConfigChannels("sbas:minimum channels", 1);
                maxch = getConfigChannels("sbas:maximum channels", 3);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 8...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 16...

                break;

            case Galileo:

                // enable it...
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, galileo_enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = getConfigChannels("galileo:minimum channels", 4);
                maxch = getConfigChannels("galileo:maximum channels", 8);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 4...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 8...

                break;

            case BeiDou:

                // disable everything else...
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, beidou_enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = getConfigChannels("beidou:minimum channels", 8);
                maxch = getConfigChannels("beidou:maximum channels", 16);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 8...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 16...

                break;

            case IMES:

                // disable everything else...
                enabled = getConfigBool("imes:enabled", false);
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = getConfigChannels("imes:minimum channels", 0);
                maxch = getConfigChannels("imes:maximum channels", 8);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 0...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 8...

                break;

            case QZSS:

                // disable everything else...
                enabled = getConfigBool("qzss:enabled", false);
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = getConfigChannels("qzss:minimum channels", 0);
                maxch = getConfigChannels("qzss:maximum channels", 3);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 0...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 3...

                break;

            case GLONASS:

                // enable it...
                flags = (uint32_t) setBit_slBits(get_uint32_slBuffer(b, o + 4), 0, glonass_enabled);
                put_uint32_slBuffer(b, o + 4, flags);

                // set our min and max channels...
                minch = getConfigChannels("glonass:minimum channels", 8);
                maxch = getConfigChannels("glonass:maximum channels", 14);
                put_uint8_slBuffer(b, o + 1,  minch);  // min of 8...
                put_uint8_slBuffer(b, o + 2,  maxch);  // max of 14...

                break;

        }
    }

    // if nothing changed there is nothing to send: every UBX-CFG-GNSS makes the receiver restart its GNSS engine
    // ("Resetting GNSS"), whether or not anything changed...
    bool gnssChanged = memcmp(before, buffer_slBuffer(b), bodySize) != 0;
    free(before);
    if (!gnssChanged) {
        if (verbosity >= 2) printf("GNSS configuration is already as requested...\n");
        free(gnssMsg.body);
    } else {

        // now send it back to the GPS...
        ubxMsg newGnssMsg = createUbxMsg(gnssMsg.type, b);  // this recomputes the checksum...
        slReturn suamResp = sendUbxAckedMsg(fdPort, newGnssMsg);
        if (isErrorReturn(suamResp)) {
            free(gnssMsg.body);
            return makeErrorMsgReturn(ERR_CAUSE(suamResp), "problem sending GNSS configuration to GPS");
        }
        freeReturn(suamResp);
        free(gnssMsg.body);

        // u-blox: wait for the ACK and then 0.5 seconds before sending the next command; and if Galileo is enabled,
        // follow it with a hardware reset...
        sleepMs(GNSS_SETTLE_MS);
        if (galileo_enabled) {
            slReturn restartResp = ubxRestartAfterGnssChange(fdPort, verbosity);
            if (isErrorReturn(restartResp))
                return makeErrorMsgReturn(ERR_CAUSE(restartResp), "problem restarting the GPS after changing its GNSS configuration");
            freeReturn(restartResp);
        }
    }

    // We don't care if NMEA version is 4.0, 40, 4.1, or 41, etc, we do the right thing anyway
    dnmeaver = getConfigDouble("nmea:version", 41) + 0.005;
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
    slReturn nmeaVerResp = ubxConfigNMEAVersion(fdPort, verbosity, nmeaver);
    if (isErrorReturn(nmeaVerResp)) {
        if (verbosity >= 1)
            printf("Warning: problem configuring NMEA version: %s\n", getReturnMsg(nmeaVerResp));
        freeReturn(nmeaVerResp);
    }

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
    freeReturn(tp5Resp);
    slReturn tp5Size = validateBodySize(&tp5Msg, 32, "CFG-TP5");
    if (isErrorReturn(tp5Size)) return tp5Size;
    freeReturn(tp5Size);

    // make the changes we need to make...
    slBuffer* b = tp5Msg.body;
    put_uint16_slBuffer(b,  4, getConfigInt("time pulse:antenna cable delay", 56, -32768, 32767));         // set the antenna cable delay to 56 ns...
    put_uint16_slBuffer(b,  6, getConfigInt("time pulse:rf group delay", 20, -32768, 32767));              // set the RF group delay to 20 ns...
    put_uint32_slBuffer(b,  8, getConfigInt("time pulse:unlocked pulse period", 1000000, 0, 2147483647));  // set the unlocked period to 1 second (1,000,000 microseconds)...
    put_uint32_slBuffer(b, 16, getConfigInt("time pulse:unlocked pulse length", 0, 0, 2147483647));        // set the unlocked pulse length to 0 seconds...
    put_uint32_slBuffer(b, 12, getConfigInt("time pulse:locked pulse period", 1000000, 0, 2147483647));    // set the locked period to 1 second (1,000,000 microseconds)...
    put_uint32_slBuffer(b, 20, getConfigInt("time pulse:locked pulse length", 500000, 0, 2147483647));     // set the locked pulse length to 0.5 seconds (500,000 microseconds)...
    put_uint32_slBuffer(b, 24, getConfigInt("time pulse:user configurable delay", 0, INT32_MIN, 2147483647));      // set the user-configurable delay to zero...
    put_uint32_slBuffer(b, 28,    0x77);  // set the flags...

    // now send it back to the GPS...
    ubxMsg newTp5Msg = createUbxMsg(tp5Msg.type, b);
    slReturn suamResp = sendUbxAckedMsg(fdPort, newTp5Msg);
    if (isErrorReturn(suamResp)) {
        free(tp5Msg.body);
        return makeErrorMsgReturn(ERR_CAUSE(suamResp), "problem sending time pulse configuration to GPS");
    }
    freeReturn(suamResp);
    free(tp5Msg.body);

    return makeOkReturn();
}

// Scales a configuration value to the integer the receiver wants, rounding to nearest.  Plain (int) truncation is wrong
// here: 1.15 * 100.0 is 114.99999999999999, which would be sent as 114.  (Done by hand to avoid needing libm.)
static int scaleRound(double value, double scale) {
    double v = value * scale;
    return (int) ((v < 0.0) ? (v - 0.5) : (v + 0.5));
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
    freeReturn(nav5Resp);
    slReturn nav5Size = validateBodySize(&nav5Msg, 36, "CFG-NAV5");
    if (isErrorReturn(nav5Size)) return nav5Size;
    freeReturn(nav5Size);

    // make our changes...
    slBuffer* b = nav5Msg.body;
    put_uint16_slBuffer(b,  0, 0x05F7);  // configure all settings (including DGPS/DGNSS)...
    put_uint8_slBuffer(b,  2, getConfigInt("navigation engine:dynamic model", Stationary, 0, 255));                  // use stationary mode...
    put_uint8_slBuffer(b,  3, getConfigInt("navigation engine:fix mode", Only3D, 0, 255));                         // use any mode...
    put_uint32_slBuffer(b,  4, (uint32_t) scaleRound(getConfigDouble("navigation engine:fixed altitude (2d)", 0), 100.0));// fixed altitude for 2D (not used)...
    put_uint32_slBuffer(b,  8, (uint32_t) scaleRound(getConfigDouble("navigation engine:fixed altitude variance (2d)", 0), 10000.0));  // fixed altitude variance for 2D (1 m^2)...
    put_uint8_slBuffer(b, 12, getConfigInt("navigation engine:minimum elevation", 20, -128, 127));                       // minimum elevation is 5 degrees...
    put_uint16_slBuffer(b, 14, (uint16_t) scaleRound(getConfigDouble("navigation engine:position dop mask", 10), 10.0));   // position DoP mask is 10.0...
    put_uint16_slBuffer(b, 16, (uint16_t) scaleRound(getConfigDouble("navigation engine:time dop mask", 10), 10.0));       // time DoP mask is 10.0...
    put_uint16_slBuffer(b, 18, getConfigInt("navigation engine:position accuracy mask", 40, 0, 65535));               // position accuracy mask is 100 meters...
    put_uint16_slBuffer(b, 20, getConfigInt("navigation engine:time accuracy mask", 40, 0, 65535));                   // time accuracy mask is 300 m...
    put_uint8_slBuffer(b, 22, getConfigInt("navigation engine:static hold threshold", 0, 0, 255));                   // static hold threshold is 0 cm/s...
    put_uint8_slBuffer(b, 23, getConfigInt("navigation engine:dynamic gnss timeout", 60, 0, 255));                   // dynamic GNSS timeout is 60 seconds (not used)...
    put_uint8_slBuffer(b, 24, getConfigInt("navigation engine:threshold above c/no", 8, 0, 255));                    // number of satellites that must be above CNo threshold...
    put_uint8_slBuffer(b, 25, getConfigInt("navigation engine:c/no threshold", 20, 0, 255));                          // CNo threshold...
    put_uint16_slBuffer(b, 28, getConfigInt("navigation engine:static hold max distance", 0, 0, 65535));               // static hold distance threshold...
    put_uint8_slBuffer(b, 30, getConfigInt("navigation engine:utc standard", USNO_UTC, 0, 255));                     // use USNO UTC...

    // send it back to the GPS...
    ubxMsg newNav5Msg = createUbxMsg(nav5Msg.type, b);
    slReturn suamResp = sendUbxAckedMsg(fdPort, newNav5Msg);
    if (isErrorReturn(suamResp)) {
        free(nav5Msg.body);
        return makeErrorMsgReturn(ERR_CAUSE(suamResp), "problem sending navigation engine configuration to GPS");
    }
    freeReturn(suamResp);
    free(nav5Msg.body);

    return makeOkReturn();
}

// Configures the GPS for maximum timing accuracy...
extern slReturn ubxConfigForTiming(int fdPort, int verbosity) {

    slReturn resp;

    // configure the GNSS for GPS, GLONASS, BeiDou, and Galileo, with no SBAS.
    resp = ubxConfigSatellites(fdPort, verbosity);
    if (isErrorReturn(resp))
        return makeErrorMsgReturn(ERR_CAUSE(resp), "problem configuring GNSS satellites for timing");

    // configure the time pulse...
    resp = ubxConfigTimePulse(fdPort, verbosity);
    if (isErrorReturn(resp))
        return makeErrorMsgReturn(ERR_CAUSE(resp), "problem configuring time pulse for timing");

    // configure the navigation engine...
    resp = ubxConfigNavEngine(fdPort, verbosity);
    if (isErrorReturn(resp))
        return makeErrorMsgReturn(ERR_CAUSE(resp), "problem configuring navigation engine for timing");

    // Suppress NMEA output except for the messages enabled in gpsctl.conf...
    struct {
        nmeaMSG id;
        const char *key;
        bool def;
    } nmeaMsgs[] = {
        { GGA, "nmea:gga", false },
        { GLL, "nmea:gll", false },
        { GSA, "nmea:gsa", false },
        { GSV, "nmea:gsv", false },
        { RMC, "nmea:rmc", true  },
        { VTG, "nmea:vtg", false },
        { GRS, "nmea:grs", false },
        { GST, "nmea:gst", false },
        { ZDA, "nmea:zda", true  },
    };
    for (size_t i = 0; i < sizeof(nmeaMsgs) / sizeof(nmeaMsgs[0]); i++) {
        resp = ubxEnableNMEAMsg(fdPort, verbosity, nmeaMsgs[i].id,
                                getConfigBool(nmeaMsgs[i].key, nmeaMsgs[i].def));
        if (isErrorReturn(resp)) {
            if (verbosity >= 1)
                printf("Warning: problem configuring NMEA message output for timing: %s\n", getReturnMsg(resp));
            freeReturn(resp);
        }
    }

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
    freeReturn(pollResp);
    if (size_slBuffer(fixMsg.body) < NAV_PVT_BODY_SIZE) {
        size_t actualSize = size_slBuffer(fixMsg.body);
        free(fixMsg.body);
        return makeErrorFmtMsgReturn(ERR_ROOT, "unexpected NAV_PVT message body size: %zu bytes", actualSize);
    }

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
    free(fixMsg.body);

    // the correction only means something once the receiver has a valid time: before that the time fields are zero,
    // and a negative correction would turn them into nonsense (a second of -0.3 becomes 59.7 of minute -1, and so on)...
    if (fix->time_valid) correctTime(fix);

    return makeOkReturn();
}


// Returns a newly allocated, always NUL-terminated copy of the string that starts at the given offset within a
// message body of the given size.  The string is at most maxLen characters long and never extends past the end of the
// body, even if the device didn't NUL-terminate it (or sent a body that is too short to contain it at all, in which
// case the result is an empty string).
static char* copyBodyString(const char* body, size_t bodySize, size_t offset, size_t maxLen) {

    size_t avail = (offset < bodySize) ? (bodySize - offset) : 0;
    size_t limit = (avail < maxLen) ? avail : maxLen;
    size_t len = (limit > 0) ? strnlen(body + offset, limit) : 0;
    char* result = safeMalloc(len + 1);
    if (len > 0) memcpy(result, body + offset, len);
    result[len] = 0;
    return result;
}


// Fills the ubxVersion structure at the given pointer with information about the GPS's version.
extern slReturn ubxGetVersion(int fdPort, int verbosity, ubxVersion* version) {

    ubxType type = { UBX_MON, UBX_MON_VER };
    slBuffer* body = create_slBuffer(0, LittleEndian);
    ubxMsg msg = createUbxMsg(type, body);
    ubxMsg versionMsg;
    slReturn result = pollUbx(fdPort, msg, MON_VER_MAX_MS, &versionMsg);
    if (isErrorReturn(result)) return result;
    freeReturn(result);

    // decode the message and print the results...
    char* buf = (char *) buffer_slBuffer(versionMsg.body);

    // the body comes from the device, so never trust it: software version is 30 bytes, hardware version 10 bytes,
    // and each extension 30 bytes, none of them guaranteed to be NUL-terminated or even present...
    size_t bodyLen = size_slBuffer(versionMsg.body);
    version->software = copyBodyString(buf, bodyLen, 0, 30);
    version->hardware = copyBodyString(buf, bodyLen, 30, 10);

    // calculate the number of extension entries we have...
    // (round up so the count matches the number of loop iterations below, and never go negative)...
    int bodySize = (int) bodyLen;
    int ne = (bodySize > 40) ? ((bodySize - 40) + 29) / 30 : 0;
    version->number_of_extensions = ne;

    // always allocate room for ne pointers PLUS the NULL terminator, even when ne is zero, so that callers that
    // walk the array until NULL never dereference a NULL array pointer...
    version->extensions = safeMalloc((size_t) (ne + 1) * sizeof(char*));
    char** ptr = version->extensions;
    for (int i = 0; i < ne; i++) {
        *ptr = copyBodyString(buf, bodyLen, 40 + 30 * (size_t) i, 30);
        ptr++;
    }
    *ptr = NULL;

    free(versionMsg.body);
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
    slReturn nsResp = pollUbx(fdPort, msg, NAV_SAT_MAX_MS, &satMsg);
    if (isErrorReturn(nsResp)) return nsResp;
    freeReturn(nsResp);

    // now decode our message, starting with the number of satellites...  The body comes from the device, so never
    // trust its satellite count further than the body can actually hold (8 header bytes, then 12 per satellite)...
    void* b = satMsg.body;
    ubxSatellites* ss = satellites;
    size_t bodySize = size_slBuffer(satMsg.body);
    if (bodySize < 8) {
        free(satMsg.body);
        return makeErrorFmtMsgReturn(ERR_ROOT, "unexpected NAV-SAT message body size: %zu bytes", bodySize);
    }
    ss->numberOfSatellites = get_uint8_slBuffer(b, 5);
    int maxSatellites = (int) ((bodySize - 8) / 12);
    if (ss->numberOfSatellites > maxSatellites)
        ss->numberOfSatellites = maxSatellites;

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
        s->pseudoRangeResidualM = get_int16_slBuffer(b, o + 6) * 0.1;

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

    free(satMsg.body);
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
        case pmAggressive4Hz: return "Aggressive (4 Hz)";
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
    freeReturn(resp);

    // The body comes from the device, so never trust it: the UART flags are bytes 14 and 15, and a reply too short to
    // hold them would otherwise read as "NMEA off", reporting a configuration the receiver never sent...
    slReturn sizeResp = validateBodySize(&current, 16, "CFG-PRT");
    if (isErrorReturn(sizeResp)) return sizeResp;
    freeReturn(sizeResp);

    bool state =  isBitSet_slBits(get_uint16_slBuffer(current.body, 14), 1);
    free(current.body);
    return makeOkInfoReturn(bool2info(state));
}

// Returns an Ok if the given message body is at least the given size, so that a short or corrupt reply from the
// device is reported rather than silently decoded into a zeroed configuration.  On failure the body is freed,
// since there is nothing left for the caller to do with it.
static slReturn validateBodySize(ubxMsg* msg, size_t minSize, const char* name) {

    if (size_slBuffer(msg->body) >= minSize) return makeOkReturn();

    size_t actualSize = size_slBuffer(msg->body);
    free(msg->body);
    msg->body = NULL;
    return makeErrorFmtMsgReturn(ERR_ROOT, "unexpected %s message body size: %zu bytes", name, actualSize);
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
    freeReturn(antResp);
    slReturn antSize = validateBodySize(&antMsg, 4, "CFG-ANT");
    if (isErrorReturn(antSize)) return antSize;
    freeReturn(antSize);
    uint16_t antMask = get_uint16_slBuffer(antMsg.body, 0);
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
    freeReturn(gnssResp);
    slReturn gnssSize = validateBodySize(&gnssMsg, 12, "CFG-GNSS");
    if (isErrorReturn(gnssSize)) return gnssSize;
    freeReturn(gnssSize);
    config->trkChnnls = get_uint8_slBuffer(gnssMsg.body, 1);
    config->gnssRecs = get_uint8_slBuffer(gnssMsg.body, 3);  // number of configuration blocks...

    // the body comes from the device, so never trust its record count further than the body can actually hold
    // (4 header bytes, then 8 per block), and a body that claims no blocks at all can't describe a configuration...
    int maxGnssRecs = (int) ((size_slBuffer(gnssMsg.body) - 4) / 8);
    if (config->gnssRecs > maxGnssRecs) config->gnssRecs = maxGnssRecs;
    if (config->gnssRecs <= 0) {
        free(gnssMsg.body);
        return makeErrorMsgReturn(ERR_ROOT, "CFG-GNSS message reports no configuration blocks");
    }
    config->gnss = safeMalloc(sizeof(ubxGNSSConfig) * config->gnssRecs);
    for (int i = 0; i < config->gnssRecs; i++) {
        config->gnss[i].id        = (gnssID) get_uint8_slBuffer(gnssMsg.body,  4 + 8 * (size_t)i);
        config->gnss[i].minChnnls =          get_uint8_slBuffer(gnssMsg.body,  5 + 8 * (size_t)i);
        config->gnss[i].maxChnnls =          get_uint8_slBuffer(gnssMsg.body,  6 + 8 * (size_t)i);
        config->gnss[i].enabled   = (bool)  (get_uint8_slBuffer(gnssMsg.body,  8 + 8 * (size_t)i) & 0x01);
        config->gnss[i].sigConfig =          get_uint8_slBuffer(gnssMsg.body, 10 + 8 * (size_t)i);
    }
    free(gnssMsg.body);

    // get the navigation engine configuration...
    ubxType nav5Type = { UBX_CFG, UBX_CFG_NAV5 };
    body = create_slBuffer(0, LittleEndian);
    msg = createUbxMsg(nav5Type, body);
    ubxMsg nav5Msg;
    slReturn nav5Resp = pollUbx(fdPort, msg, CFG_NAV5_MAX_MS, &nav5Msg);
    if (isErrorReturn(nav5Resp))
        return makeErrorMsgReturn(ERR_CAUSE(nav5Resp), "problem getting navigation engine information from GPS");
    freeReturn(nav5Resp);
    slReturn nav5Size = validateBodySize(&nav5Msg, 36, "CFG-NAV5");
    if (isErrorReturn(nav5Size)) return nav5Size;
    freeReturn(nav5Size);
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
    freeReturn(tp5Resp);
    slReturn tp5Size = validateBodySize(&tp5Msg, 32, "CFG-TP5");
    if (isErrorReturn(tp5Size)) return tp5Size;
    freeReturn(tp5Size);
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
    free(tp5Msg.body);

    // get the fix rate configuration...
    ubxType rateType = { UBX_CFG, UBX_CFG_RATE };
    body = create_slBuffer(0, LittleEndian);
    msg = createUbxMsg(rateType, body);
    ubxMsg rateMsg;
    slReturn rateResp = pollUbx(fdPort, msg, CFG_RATE_MAX_MS, &rateMsg);
    if (isErrorReturn(rateResp))
        return makeErrorMsgReturn(ERR_CAUSE(rateResp), "problem getting fix rate information from GPS");
    freeReturn(rateResp);
    slReturn rateSize = validateBodySize(&rateMsg, 6, "CFG-RATE");
    if (isErrorReturn(rateSize)) return rateSize;
    freeReturn(rateSize);
    config->measRateMs =                  get_uint16_slBuffer(rateMsg.body, 0);
    config->navRate    =                  get_uint16_slBuffer(rateMsg.body, 2);
    config->timeRef    = (fixTimeRefType) get_uint16_slBuffer(rateMsg.body, 4);
    free(rateMsg.body);

    // get the power mode configuration...
    ubxType pmsType = { UBX_CFG, UBX_CFG_PMS };
    body = create_slBuffer(0, LittleEndian);
    msg = createUbxMsg(pmsType, body);
    ubxMsg pmsMsg;
    slReturn pmsResp = pollUbx(fdPort, msg, CFG_PMS_MAX_MS, &pmsMsg);
    if (isErrorReturn(pmsResp))
        return makeErrorMsgReturn(ERR_CAUSE(pmsResp), "problem getting power mode information from GPS");
    freeReturn(pmsResp);
    slReturn pmsSize = validateBodySize(&pmsMsg, 8, "CFG-PMS");
    if (isErrorReturn(pmsSize)) return pmsSize;
    freeReturn(pmsSize);
    config->powerSetup        = (powerModeType) get_uint8_slBuffer(pmsMsg.body, 1);
    config->powerIntervalSecs =                 get_uint16_slBuffer(pmsMsg.body, 2);
    config->powerOnTimeSecs   =                 get_uint16_slBuffer(pmsMsg.body, 4);
    free(pmsMsg.body);

    // get the NMEA configuration...
    // is NMEA enabled?
    slReturn cn = isNmeaOn(fdPort);
    if (isErrorReturn(cn)) return cn;
    config->nmeaEnabled = getReturnInfoBool(cn);
    freeReturn(cn);

    // get NMEA Version
    ubxType nmeaType = { UBX_CFG, UBX_CFG_NMEA };
    body = create_slBuffer(0, LittleEndian);
    msg = createUbxMsg(nmeaType, body);
    ubxMsg nmeaMsg;
    slReturn nmeaResp = pollUbx(fdPort, msg, CFG_NMEA_MAX_MS, &nmeaMsg);
    if (isErrorReturn(nmeaResp))
        return makeErrorMsgReturn(ERR_CAUSE(nmeaResp), "problem getting NMEA Version information from GPS");
    freeReturn(nmeaResp);
    slReturn nmeaSize = validateBodySize(&nmeaMsg, 12, "CFG-NMEA");
    if (isErrorReturn(nmeaSize)) return nmeaSize;
    freeReturn(nmeaSize);
    config->nmeaVersion        = get_uint8_slBuffer(nmeaMsg.body, 1);
    free(nmeaMsg.body);

    // Get NMEA sentence config
    cn = isNMEAmsgEnabled(fdPort, verbosity, GGA);
    if (isErrorReturn(cn)) return cn;
    config->GGA = getReturnInfoBool(cn);
    freeReturn(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GLL);
    if (isErrorReturn(cn)) return cn;
    config->GLL = getReturnInfoBool(cn);
    freeReturn(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GSA);
    if (isErrorReturn(cn)) return cn;
    config->GSA = getReturnInfoBool(cn);
    freeReturn(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GSV);
    if (isErrorReturn(cn)) return cn;
    config->GSV = getReturnInfoBool(cn);
    freeReturn(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, RMC);
    if (isErrorReturn(cn)) return cn;
    config->RMC = getReturnInfoBool(cn);
    freeReturn(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, VTG);
    if (isErrorReturn(cn)) return cn;
    config->VTG = getReturnInfoBool(cn);
    freeReturn(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GRS);
    if (isErrorReturn(cn)) return cn;
    config->GRS = getReturnInfoBool(cn);
    freeReturn(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, GST);
    if (isErrorReturn(cn)) return cn;
    config->GST = getReturnInfoBool(cn);
    freeReturn(cn);
    cn = isNMEAmsgEnabled(fdPort, verbosity, ZDA);
    if (isErrorReturn(cn)) return cn;
    config->ZDA = getReturnInfoBool(cn);
    freeReturn(cn);

    return makeOkReturn();
}


// Returns Ok with additional information true if synchronized, false if timed out.  Note that this function
// should be called prior to any series of UBX operations to ensure the UART receiver is synced to the data stream.
extern slReturn ubxSynchronizer(int fdPort, int maxTimeMs, int verbosity) {

    long long start = currentTimeMs();

    // flush any junk we might have in the receiver buffer...
    slReturn frResp = flushRx(fdPort);
    if (isErrorReturn(frResp))
        return makeErrorMsgReturn(ERR_CAUSE(frResp), "could not flush receiver prior to synchronization");

    // pollUbx() allows for output queued ahead of the reply (see lineBacklogMs()), so a poll can take that much longer
    // than MON_VER_SYNC_MAX_MS; make sure the overall limit allows for at least two of them...
    long long pollMs = MON_VER_SYNC_MAX_MS + lineBacklogMs(fdPort);
    if (pollMs > INT_MAX / 2) pollMs = INT_MAX / 2;
    if (maxTimeMs < 2 * pollMs) maxTimeMs = (int) (2 * pollMs);

    // read characters until either we appear to be synchronized or we run out of time...
    while (currentTimeMs() < start + maxTimeMs) {

        // send the request message and wait for the response...
        ubxType type = {UBX_MON, UBX_MON_VER};
        slBuffer *body = create_slBuffer(0, LittleEndian);
        ubxMsg msg = createUbxMsg(type, body);
        ubxMsg ans;
        slReturn result = pollUbx(fdPort, msg, MON_VER_SYNC_MAX_MS, &ans);
        if (isOkReturn(result)) {
            free(ans.body);
            return makeOkInfoReturn(bool2info(true));
        }
        freeReturn(result);
    }
    return makeOkInfoReturn(bool2info(false));
}


extern slReturn ubxSaveConfig(int fdPort, int verbosity) {

    // construct the save configuration message...
    slBuffer* saveMsgBody = create_slBuffer(13, LittleEndian);
    put_uint32_slBuffer(saveMsgBody,  0, 0x0000);  // clear mask all off...
    put_uint32_slBuffer(saveMsgBody,  4, 0x041B);  // save mask: ioPort, msgConf, navConf, rxmConf, antConf all on...
    put_uint32_slBuffer(saveMsgBody,  8, 0x0000);  // load mask all off...
    put_uint8_slBuffer(saveMsgBody, 12, 0x17);     // BBR|flash|EEPROM|SPI flash
    ubxMsg saveMsg = createUbxMsg(ut_CFG_CFG, saveMsgBody);

    // now send it, and wait for our ack...
    slReturn submResp = sendUbxAckedMsg(fdPort, saveMsg);
    free(saveMsgBody);
    if (isErrorReturn(submResp))
        return makeErrorMsgReturn(ERR_CAUSE(submResp), "failed to save GPS configuration");
    freeReturn(submResp);

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
    freeReturn(cn);

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
    freeReturn(resp);

    // The body comes from the device and goes straight back to it, so it must be complete: the UART flags are bytes
    // 14 and 15, and a short body would otherwise be sent back as a truncated (and so rejected) configuration...
    slReturn sizeResp = validateBodySize(&current, 20, "CFG-PRT");
    if (isErrorReturn(sizeResp)) return sizeResp;
    freeReturn(sizeResp);

    // now change the bit we need to diddle...
    slBuffer *bdy = current.body;
    put_uint16_slBuffer(bdy, 14, (uint16_t) setBit_slBits(get_uint16_slBuffer(bdy, 14), 1, nmeaOn));

    // send out the message to actually make the change...
    if (verbosity >= 3) printf("Sending new configuration to GPS with NMEA data turned %s for the UART...\n", onoff);
    slReturn ackResp = sendUbxAckedMsg(fdPort, createUbxMsg(ut_CFG_PRT, bdy));
    free(bdy);
    if (isOkReturn(ackResp)) {
        freeReturn(ackResp);
        if (verbosity >= 1) printf("Successfully set new configuration to turn NMEA data %s for GPS UART...\n", onoff);
        return makeOkReturn();
    }
    else {
        if (verbosity >= 1) printf("Failed to change GPS UART configuration to turn NMEA data %s...\n", onoff);
        return makeErrorFmtMsgReturn(ERR_CAUSE(ackResp), "failed to change GPS UART configuration to turn NMEA data %s: %s", onoff, getReturnMsg(ackResp));
    }
}


// Polls the receiver's UART configuration and returns the baud rate it reports.  The host port must already be at
// the rate the receiver is actually using, or there will be no valid answer.
static slReturn pollUartBaudRate(int fdPort, unsigned int* baudRate) {

    ubxMsg pollMsg = createUbxMsg(ut_CFG_PRT, init_slBuffer(LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */));
    ubxMsg current;
    slReturn pollAns = pollUbx(fdPort, pollMsg, CFG_PRT_MAX_MS, &current);
    if (isErrorReturn(pollAns)) return pollAns;
    freeReturn(pollAns);

    slReturn sizeResp = validateBodySize(&current, 20, "CFG-PRT");
    if (isErrorReturn(sizeResp)) return sizeResp;
    freeReturn(sizeResp);

    *baudRate = get_uint32_slBuffer(current.body, 8);
    free(current.body);
    return makeOkReturn();
}


// Asks the receiver which baud rate it reports for its UART (see pollUartBaudRate).  Used to confirm an autobaud
// result: if something else is answering on the port, the rate we found may not be the receiver's.
slReturn ubxGetUartBaudRate(int fdPort, unsigned int* baudRate) {
    return pollUartBaudRate(fdPort, baudRate);
}


// Sends the receiver its UART configuration with the new baud rate, then moves the host to that rate.  The ACK is
// only swallowed, never trusted: the receiver sends it at the old rate, so switching the host first can lose it,
// and the ACK for an earlier CFG-PRT poll names the same message type, so it cannot be told apart from this one.
static slReturn sendUartBaudRate(int fdPort, unsigned int newBaudRate) {

    // poll for the current configuration, so that we change the baud rate and leave everything else alone...
    ubxMsg current;
    ubxMsg pollMsg = createUbxMsg(ut_CFG_PRT, init_slBuffer(LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */));
    slReturn pollAns = pollUbx(fdPort, pollMsg, CFG_PRT_MAX_MS, &current);
    if (isErrorReturn(pollAns))
        return makeErrorMsgReturn(ERR_CAUSE(pollAns), "Problem polling for current configuration before changing baud rate");
    freeReturn(pollAns);

    // this body goes straight back to the receiver, so it must be complete: the baud rate is bytes 8 to 11, and a short
    // body would otherwise be sent back unchanged, and reported as a failed change...
    slReturn sizeResp = validateBodySize(&current, 20, "CFG-PRT");
    if (isErrorReturn(sizeResp)) return sizeResp;
    freeReturn(sizeResp);

    // update the baud rate in the configuration and send it back to the receiver...
    put_uint32_slBuffer(current.body, 8, newBaudRate);
    slReturn sendResp = sendUbxMsg(fdPort, createUbxMsg(ut_CFG_PRT, current.body));
    free(current.body);
    if (isErrorReturn(sendResp))
        return makeErrorMsgReturn(ERR_CAUSE(sendResp), "Problem sending message to change baud rate");
    freeReturn(sendResp);

    // change our host's baud rate to the new one, then give the ACK a moment to show up (best effort).  It proves
    // nothing, and it is easily lost: the receiver may send it before the change, in which case the flush in
    // setTermOptionsBaud() throws it away, or at the old rate, in which case the host reads garbage.  So wait a short
    // time rather than the full ACK wait - the change itself is confirmed by asking the receiver what rate it is
    // really using, which is what the caller does next...
    slReturn stoResp = setTermOptionsBaud(fdPort, (int) newBaudRate);
    if (isErrorReturn(stoResp))
        return makeErrorMsgReturn(ERR_CAUSE(stoResp), "Problem changing the host baud rate");
    freeReturn(stoResp);

    slReturn ackResp = ubxAckWait(fdPort, ut_CFG_PRT, POLL_ACK_DRAIN_MS);
    freeReturn(ackResp);

    return makeOkReturn();
}


// Changes both the receiver's UART baud rate and the host's, and proves that it worked.  An ACK proves nothing on
// its own: the receiver may acknowledge a command it has not applied, and that ACK is easily lost when the host
// switches rate.  So the new rate is confirmed by asking the receiver what it is actually using, and a missing
// answer is asked for again before it is believed, because the receiver can be briefly deaf at the new rate.  Only
// if it still won't confirm is the host put back where the receiver is and the change sent once more.  If that fails
// too, the receiver is asked which rate it actually answers on, and the host is left there, so that the port is still
// usable.
extern slReturn ubxChangeBaudRate(int fdPort, unsigned int newBaudRate, int verbosity) {

    // find out where we are now, both to report it and so that the host can be put back there if the change fails...
    unsigned int oldBaudRate = 0;
    slReturn oldResp = pollUartBaudRate(fdPort, &oldBaudRate);
    if (isErrorReturn(oldResp))
        return makeErrorMsgReturn(ERR_CAUSE(oldResp), "Problem polling for current configuration before changing baud rate");
    freeReturn(oldResp);

    if (verbosity >= 3) printf("GPS UART is currently at %u baud...\n", oldBaudRate);

    // if it is already at the rate we want, there is nothing to send.  Rewriting the same configuration would only
    // cost a write and an ACK wait - and if that write went unanswered, it would report a change that was never
    // needed as a failure...
    if (oldBaudRate == newBaudRate) {
        if (verbosity >= 1) printf("GPS UART is already at %u baud...\n", newBaudRate);
        return makeOkReturn();
    }

    for (int attempt = 0; attempt < 2; attempt++) {

        // send the new configuration and move the host to the new rate...
        slReturn changeResp = sendUartBaudRate(fdPort, newBaudRate);
        if (isErrorReturn(changeResp))
            return makeErrorMsgReturn(ERR_CAUSE(changeResp), "Problem changing the baud rate");
        freeReturn(changeResp);

        // Ask the receiver what rate it is actually using now, and believe only that.  The first exchange at the new
        // rate can be lost, so ask twice before concluding that the change didn't take...
        bool verified = false;
        for (int ask = 0; (ask < 2) && !verified; ask++) {

            unsigned int actualBaudRate = 0;
            slReturn verifyResp = pollUartBaudRate(fdPort, &actualBaudRate);
            verified = isOkReturn(verifyResp) && (actualBaudRate == newBaudRate);
            freeReturn(verifyResp);

            if (!verified && (ask == 0) && (verbosity >= 2))
                printf("No confirmation at %u baud, asking again...\n", newBaudRate);
        }

        if (verified) {
            if (verbosity >= 1) printf("Successfully changed baud rate to %u...\n", newBaudRate);
            return makeOkReturn();
        }

        // The change didn't take, or the replies were lost after it did.  Put the host back at the old rate for the
        // retry; whether that is really where the receiver is gets settled below, once the retry has had its chance...
        if (verbosity >= 2)
            printf("GPS did not answer at %u baud, %s...\n", newBaudRate, (attempt == 0) ? "retrying" : "giving up");
        slReturn backResp = setTermOptionsBaud(fdPort, (int) oldBaudRate);
        if (isErrorReturn(backResp))
            return makeErrorMsgReturn(ERR_CAUSE(backResp), "Problem restoring the host baud rate after a failed change");
        freeReturn(backResp);
    }

    // Neither attempt could confirm the new rate, but that does not mean the receiver is still at the old one: it may
    // have taken the change and lost the replies.  The host is at the old rate now (each failed attempt put it back),
    // so ask there first, and if that gets no answer, ask at the new rate: that keeps the host at a rate the receiver
    // still answers on whenever there is one...
    slReturn backResp = setTermOptionsBaud(fdPort, (int) oldBaudRate);
    if (isErrorReturn(backResp))
        return makeErrorMsgReturn(ERR_CAUSE(backResp), "Problem restoring the host baud rate after a failed change");
    freeReturn(backResp);

    unsigned int probedBaudRate = 0;
    slReturn askResp = pollUartBaudRate(fdPort, &probedBaudRate);
    bool answered = isOkReturn(askResp);
    freeReturn(askResp);

    if (!answered) {
        slReturn newResp = setTermOptionsBaud(fdPort, (int) newBaudRate);
        if (isErrorReturn(newResp))
            return makeErrorMsgReturn(ERR_CAUSE(newResp), "Problem changing the host baud rate after a failed change");
        freeReturn(newResp);

        askResp = pollUartBaudRate(fdPort, &probedBaudRate);
        answered = isOkReturn(askResp);
        freeReturn(askResp);
    }

    // if the receiver reports the rate we asked for, the change did take, and only its confirmations were lost...
    if (answered && (probedBaudRate == newBaudRate)) {
        if (verbosity >= 1) printf("Successfully changed baud rate to %u...\n", newBaudRate);
        return makeOkReturn();
    }

    if (answered)
        return makeErrorFmtMsgReturn(ERR_ROOT, "GPS did not change to %u baud (it is at %u)", newBaudRate, probedBaudRate);
    return makeErrorFmtMsgReturn(ERR_ROOT, "GPS did not change to %u baud (it answers at neither %u nor %u)",
                                 newBaudRate, oldBaudRate, newBaudRate);
}
