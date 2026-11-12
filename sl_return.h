//
// Created by Tom Dilatush on 11/3/17.
//

#ifndef GPSCTL_SL_RESULT_H
#define GPSCTL_SL_RESULT_H

#include <stdint.h>
#include <stdbool.h>

// opaque type for function return values...
typedef void* slReturn;         // managed by slReturn internally; users shouldn't do anything with it directly...

// contains error information other than client-generated error message...
typedef struct {
    slReturn cause;
    char* fileName;
    char* functionName;
    uint32_t lineNumber;
} errorInfo_slReturn;


errorInfo_slReturn createErrorInfo( char*, char*, uint32_t, slReturn );
bool isOkReturn( slReturn );
bool isErrorReturn( slReturn );
bool isWarningReturn( slReturn );
const char* getReturnMsg( slReturn );
void* getReturnInfo( slReturn );
void printReturn( slReturn, bool, bool );
void freeReturn( slReturn  );
slReturn makeOkReturn();
slReturn makeErrorReturn( errorInfo_slReturn );
slReturn makeWarningReturn();
slReturn makeOkMsgReturn( const char* );
slReturn makeErrorMsgReturn( errorInfo_slReturn, const char* );
slReturn makeWarningMsgReturn( const char* );
slReturn makeOkFmtMsgReturn( const char*, ... );
slReturn makeErrorFmtMsgReturn( errorInfo_slReturn, const char*, ... );
slReturn makeWarningFmtMsgReturn( const char*, ... );
slReturn makeOkInfoReturn( void* );
slReturn makeErrorInfoReturn( errorInfo_slReturn, void* );
slReturn makeWarningInfoReturn( void* );
slReturn makeOkInfoMsgReturn( void*, const char* );
slReturn makeErrorInfoMsgReturn( errorInfo_slReturn, void*, const char* );
slReturn makeWarningInfoMsgReturn( void*, const char* );
slReturn makeOkInfoFmtMsgReturn( void*, const char*, ... );
slReturn makeErrorInfoFmtMsgReturn( errorInfo_slReturn, void*, const char*, ... );
slReturn makeWarningInfoFmtMsgReturn( void*, const char*, ... );

#define ERR_CAUSE(x) (createErrorInfo( __FILE__, (char*)__func__, __LINE__, (x) ) )
#define ERR_ROOT (ERR_CAUSE(NULL))

#define getReturnInfoBool(x) ((bool)(uintptr_t)getReturnInfo(x))
#define getReturnInfoInt32(x) ((int32_t)(uintptr_t)getReturnInfo(x))
#define getReturnInfoUint32(x) ((uint32_t)(uintptr_t)getReturnInfo(x))
// Note that this is deliberately an unsigned char: plain char is signed on some platforms (e.g. x86) and unsigned on
// others (e.g. ARM), and a signed char would turn bytes 0x80..0xFF (such as the UBX sync byte 0xB5) negative, which
// callers that use a negative value to mean "no character" would then misinterpret...
#define getReturnInfoChar(x) ((unsigned char)(uintptr_t)getReturnInfo(x))

#define char2info(x) ((void*)(uintptr_t)(x))
#define int2info(x) ((void*)(uintptr_t)(x))
#define bool2info(x) ((void*)(uintptr_t)(x))

#endif //GPSCTL_SL_RESULT_H
