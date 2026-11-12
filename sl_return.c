//
// Created by Tom Dilatush on 11/3/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#define _GNU_SOURCE

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include "sl_return.h"
#include "sl_general.h"

/*
 * Each slReturn value stores return information in one of three different ways:
 *
 * 1.  For the specific case of an Ok return with no message and no additional info, a NULL in the value pointer.
 *
 * 2.  For an Ok or Warning return with either a message or additional information, a pointer to a structure with
 *     a definition field and either a pointer to the message or the client-defined additional information.
 *
 * 3.  For all other cases, a pointer to a structure that has a pointer to a message, client-defined additional
 *     information, and a block of entries related to errors: a pointer to a file name string, a line number,
 *     and a slReturn of the error's cause.
 */


// Constants used to define type and memory management flags...
#define SLRETURN_OK         0x0001
#define SLRETURN_WARNING    0x0002
#define SLRETURN_ERROR      0x0003
#define SLRETURN_TYPE_MASK  0x0003
#define SLRETURN_LONG_FORM  0x0000
#define SLRETURN_FIELD_NONE 0x0004
#define SLRETURN_FIELD_MSG  0x0008
#define SLRETURN_FIELD_INFO 0x000C
#define SLRETURN_FIELD_MASK 0x000C
#define SLRETURN_MSG_ALLOC  0x0010

typedef enum {
    Ok, Warning, Error
} returnType;

typedef errorInfo_slReturn errInfo;

typedef struct {
    uint32_t def;
    void* ptr;
} shortRecord;

typedef struct {
    uint32_t def;
    const char* msg;
    void* info;
    errInfo error;
} longRecord;

static errInfo NOERR = { NULL, NULL, 0 };

// The basic method for creating a return value, called by the exposed convenience functions for making particular kinds
// of return values that don't include formatted messages.
static slReturn makeReturnInt( returnType type, errInfo error, void* info, const char* msg, bool msgAlloc ) {

    // if we have an Ok with no message and no additional information, just return a null...
    if((type == Ok) && (msg == NULL) && (info == NULL)) return NULL;

    // if we have either an Ok or a Warning, and either a message or additional info (or neither, but not both!), use a short record...
    if(((type == Ok) || (type == Warning)) && ((msg == NULL) || (info == NULL))) {

        // build and return our short record...
        shortRecord* result = safeMalloc( sizeof( shortRecord ));
        result->def = (type == Ok) ? SLRETURN_OK : SLRETURN_WARNING;
        if( msg != NULL) {
            result->ptr = (void*) msg;
            result->def |= SLRETURN_FIELD_MSG;
            if( msgAlloc ) result->def |= SLRETURN_MSG_ALLOC;
        } else if( info != NULL) {
            result->ptr = info;
            result->def |= SLRETURN_FIELD_INFO;
        } else {
            result->ptr = NULL;
            result->def |= SLRETURN_FIELD_NONE;
        }

        return result;
    }

    // otherwise, we use the long form...
    longRecord* result = safeMalloc( sizeof( longRecord ));
    result->def = (type == Ok) ? SLRETURN_OK : ((type == Warning) ? SLRETURN_WARNING : SLRETURN_ERROR);
    result->def |= SLRETURN_LONG_FORM;
    if( msgAlloc ) result->def |= SLRETURN_MSG_ALLOC;
    result->info = info;
    result->msg = msg;
    result->error = error;

    return result;
}


// The basic method for creating a return value, called by the exposed convenience functions for making particular kinds
// of return values that don't include formatted messages.
static slReturn makeReturn( returnType type, errInfo error, void* info, const char* msg ) {
    return makeReturnInt( type, error, info, msg, false );
}


// The basic method for creating a return value, called by the exposed convenience functions for making
// particular kinds of return values that include formatted messages.
static slReturn makeFmtReturn( returnType type, errInfo error, void* info, const char* msg, va_list args ) {

    // first we format the string we were given, into allocated memory...
    char* resolved;
    vasprintf( &resolved, msg, args );
    va_end( args );
    return makeReturnInt( type, error, info, resolved, true );
}


// Returns an error information structure with the given values.
extern errorInfo_slReturn createErrorInfo( char* fileName, char* functionName, uint32_t lineNumber, slReturn cause ) {
    errorInfo_slReturn result;
    result.cause = cause;
    result.lineNumber = lineNumber;
    result.fileName = fileName;
    result.functionName = functionName;
    return result;
}


// Returns true if the given slReturn type is Ok.
extern bool isOkReturn( slReturn value ) {
    if( value == NULL ) return true;
    return (((shortRecord*) value)->def & SLRETURN_TYPE_MASK) == SLRETURN_OK;
}


// Returns true if the given slReturn type is an Error.
extern bool isErrorReturn( slReturn value ) {
    if( value == NULL ) return false;
    return (((shortRecord*) value)->def & SLRETURN_TYPE_MASK) == SLRETURN_ERROR;
}


// Returns true if the given slReturn type is a Warning.
extern bool isWarningReturn( slReturn value ) {
    if( value == NULL ) return false;
    return (((shortRecord*) value)->def & SLRETURN_TYPE_MASK) == SLRETURN_WARNING;
}


// Return the message in the given slReturn, or NULL if there is none.
extern const char* getReturnMsg( slReturn value ) {
    if( value == NULL ) return NULL;
    if( (((shortRecord*) value)->def & SLRETURN_FIELD_MASK) == SLRETURN_FIELD_MSG )
        return (const char* ) (((shortRecord*) value)->ptr);
    if( (((shortRecord*) value)->def & SLRETURN_FIELD_MASK) == SLRETURN_LONG_FORM )
        return ((longRecord*) value)->msg;
    return NULL;
}


// Return the additional information in the given slReturn, or NULL if there is none.
extern void* getReturnInfo( slReturn value ) {
    if( value == NULL ) return NULL;
    if( (((shortRecord*) value)->def & SLRETURN_FIELD_MASK) == SLRETURN_FIELD_INFO )
        return ((shortRecord*) value)->ptr;
    if( (((shortRecord*) value)->def & SLRETURN_FIELD_MASK) == SLRETURN_LONG_FORM )
        return ((longRecord*) value)->info;
    return NULL;
}


// Print the message in the given slReturn, following the chain of errors if there is one.
extern void printReturn( slReturn value, bool includeCaused, bool includeDebug ) {

    // if we have a simple Ok, just leave...
    if( value == NULL ) return;

    // a little analysis...
    const char* msg = getReturnMsg( value );
    bool longRec = (((shortRecord*) value)->def & SLRETURN_FIELD_MASK) == SLRETURN_LONG_FORM;
    errInfo err = NOERR;  // only long-form records have an error block; short-form records are smaller
    if( longRec ) err = ((longRecord*) value)->error;
    slReturn next = err.cause;
    bool root = (next == NULL);

    // if we're not at the root cause, recurse until we get there...
    if( !root ) printReturn( next, includeCaused, includeDebug );

    // if we're not supposed to be printing this line, skedaddle...
    if( !root && !includeCaused ) return;

    // finally, it's time to print our actual message...
    char* prefix = root ? (isErrorReturn( value ) ? "Error: " : "") : "   Caused: ";
    if( includeDebug && isErrorReturn( value ) )
        printf( "%s%s  (%s:%s:%d)\n", prefix, msg, err.fileName, err.functionName, err.lineNumber );
    else
        printf( "%s%s\n", prefix, msg );
}


// Free any memory allocated by slReturn in the given value.  Note that caller-supplied information (messages or
// additional information) are NOT freed by this call - freeing those is the responsibility of the caller.
extern void freeReturn( slReturn value ) {

    // if we have a simple Ok, there's nothing to free...
    if( value == NULL ) return;

    // set up for possible looping through chained errors...
    slReturn current = value;
    do{
        // if we allocated space for a message in this return, free it...
        if( (((shortRecord*) current)->def & SLRETURN_MSG_ALLOC) == SLRETURN_MSG_ALLOC )
            free( (void*) getReturnMsg( current ) );

        // save a pointer to our current record so we can free it in a moment...
        void* ptr = current;

        // go to the next return value in our chain, if we have one...
        if( (((shortRecord*) current)->def & SLRETURN_FIELD_MASK) == SLRETURN_LONG_FORM )
            current = ((longRecord*) current)->error.cause;
        else
            current = NULL;

        // free the return we just handled...
        free( ptr );

    } while( current != NULL );
}


// Make an Ok return value with no message and no info.
extern slReturn makeOkReturn() {
    return makeReturn( Ok, NOERR, NULL, NULL );
}


// Make an Error return value with no message and no info.
extern slReturn makeErrorReturn( errInfo error ) {
    return makeReturn( Error, error, NULL, NULL );
}


// Make a Warning return value with no message and no info.
extern slReturn makeWarningReturn() {
    return makeReturn( Warning, NOERR, NULL, NULL );
}


// Make an Ok return value with a message and no info.
extern slReturn makeOkMsgReturn( const char* msg ) {
    return makeReturn( Ok, NOERR, NULL, msg );
}


// Make an Error return value with a message and no info.
extern slReturn makeErrorMsgReturn( errInfo error, const char* msg ) {
    return makeReturn( Error, error, NULL, msg );
}


// Make a Warning return value with a message and no info.
extern slReturn makeWarningMsgReturn( const char* msg ) {
    return makeReturn( Warning, NOERR, NULL, msg );
}


// Make an Ok return value with a formatted message and no info.
extern slReturn makeOkFmtMsgReturn( const char* msg, ... ) {
    va_list args;
    va_start( args, msg );
    slReturn result = makeFmtReturn( Ok, NOERR, NULL, msg, args );
    va_end( args );
    return result;
}


// Make an Error return value with a formatted message and no info.
extern slReturn makeErrorFmtMsgReturn( errInfo error, const char* msg, ... ) {
    va_list args;
    va_start( args, msg );
    slReturn result = makeFmtReturn( Error, error, NULL, msg, args );
    va_end( args );
    return result;
}


// Make a Warning return value with a formatted message and no info.
extern slReturn makeWarningFmtMsgReturn( const char* msg, ... ) {
    va_list args;
    va_start( args, msg );
    slReturn result = makeFmtReturn( Warning, NOERR, NULL, msg, args );
    va_end( args );
    return result;
}


// Make an Ok return value with no message and additional info.
extern slReturn makeOkInfoReturn( void* info ) {
    return makeReturn( Ok, NOERR, info, NULL );
}


// Make an Error return value with no message and additional info.
extern slReturn makeErrorInfoReturn( errInfo error, void* info ) {
    return makeReturn( Error, error, info, NULL );
}


// Make a Warning return value with no message and additional info.
extern slReturn makeWarningInfoReturn( void* info ) {
    return makeReturn( Warning, NOERR, info, NULL );
}


// Make an Ok return value with a message and additional info.
extern slReturn makeOkInfoMsgReturn( void* info, const char* msg ) {
    return makeReturn( Ok, NOERR, info, msg );
}


// Make an Error return value with a message and additional info.
extern slReturn makeErrorInfoMsgReturn( errInfo error, void* info, const char* msg ) {
    return makeReturn( Error, error, info, msg );
}


// Make a Warning return value with a message and additional info.
extern slReturn makeWarningInfoMsgReturn( void* info, const char* msg ) {
    return makeReturn( Warning, NOERR, info, msg );
}


// Make an Ok return value with a formatted message and additional info.
extern slReturn makeOkInfoFmtMsgReturn( void* info, const char* msg, ... ) {
    va_list args;
    va_start( args, msg );
    slReturn result = makeFmtReturn( Ok, NOERR, info, msg, args );
    va_end( args );
    return result;
}


// Make an Error return value with a formatted message and additional info.
extern slReturn makeErrorInfoFmtMsgReturn( errInfo error, void* info, const char* msg, ... ) {
    va_list args;
    va_start( args, msg );
    slReturn result = makeFmtReturn( Error, error, info, msg, args );
    va_end( args );
    return result;
}


// Make a Warning return value with a formatted message and additional info.
extern slReturn makeWarningInfoFmtMsgReturn( void* info, const char* msg, ... ) {
    va_list args;
    va_start( args, msg );
    slReturn result = makeFmtReturn( Warning, NOERR, info, msg, args );
    va_end( args );
    return result;
}
