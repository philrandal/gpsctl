//
// Created by Tom Dilatush on 10/26/17.
//

#ifndef GPSCTL_SL_OPTIONS_H
#define GPSCTL_SL_OPTIONS_H

#include <stdio.h>
#include "sl_return.h"
#include "sl_buffer.h"

typedef enum { argRequired, argOptional,    argNone          } argMode_slOptions;

typedef struct clientData_slOptions clientData_slOptions;  // a client-defined structure that typically contains the results of options parsing
typedef struct state_slOptions state_slOptions;  // an slOptions-defined data structure containing the state of slOptions while it's processing

// the extra information in the return value from a process_slOptions() call
typedef struct {
    int argc;       // the number of arguments remaining after processing options
    char** argv;    // a pointer to the remaining arguments (NULL if argc is zero)
} args_slOptions;

typedef struct optionDef_slOptions optionDef_slOptions;

typedef struct psloConfig psloConfig;

typedef slReturn  parse_slOptions(  void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* );  // option parsing function def
typedef slReturn  cnstr_slOptions(  const optionDef_slOptions*, const psloConfig*, const state_slOptions* );  // constraint checking function def
typedef slReturn action_slOptions( const optionDef_slOptions*, const psloConfig* );  // action function def

struct optionDef_slOptions {
    unsigned int            maxCount;   // how many times this option may appear in a command line
    char*                   longOpt;    // optional; the long form of the option
    char                    shortOpt;   // optional; the short form of the option
    argMode_slOptions       arg;        // controls whether the option has an argument, required or optional
    parse_slOptions*        parse;      // option parsing function, or NULL if not needed
    void*                   parsePtr;   // optional pointer passed as an argument to the parse function
    int                     parseInt;   // optional integer passed as an argument to the parse function
    cnstr_slOptions*        cnstr;      // constraint checking function, or NULL if not needed
    action_slOptions*       action;     // action function, or NULL if not needed
    char*                   argName;    // short name of argument (REQUIRED if there could be an argument )
    char*                   helpMsg;    // help message for the option (REQUIRED)

};


#define SL_OPTIONS_CONFIG_DEBUG       (0x0001)   // enables debug tracing
#define SL_OPTIONS_CONFIG_STRICT      (0x0002)   // enables strict mode (no permutation)
#define SL_OPTIONS_CONFIG_ADD_USAGE   (0x0004)   // enables automatically adding usage option definition
#define SL_OPTIONS_CONFIG_ADD_HELP    (0x0008)   // enables automatically adding help option definition
#define SL_OPTIONS_CONFIG_ADD_VERSION (0x0010)   // enables automatically adding verson option definition
#define SL_OPTIONS_CONFIG_NORMAL      (SL_OPTIONS_CONFIG_ADD_USAGE | SL_OPTIONS_CONFIG_ADD_HELP | SL_OPTIONS_CONFIG_ADD_VERSION)

// the configuration for a process_slOptions() call
struct psloConfig {
    optionDef_slOptions* optionDefs;    // pointer to client-defined option definitions
    clientData_slOptions* clientData;   // pointer to client-defined data structure
    cnstr_slOptions* beforeConstraint;  // optional constraint-checking function to call BEFORE calling any per-option constraint functions
    cnstr_slOptions* afterConstraint;   // optional constraint-checking function to call AFTER calling all per-option constraint functions
    action_slOptions* beforeAction;     // optional action function to call BEFORE calling any per-option action functions
    action_slOptions* afterAction;      // optional action function to call AFTER calling all other per-option action functions
    char* name;                         // a client-defined program name
    char* version;                      // a client-defined string representing the version of the program
    char* addedInfo;                    // a client-defined string giving additional information about using the program
    int options;                        // options for the behavior of slOptions
    action_slOptions* recoverAction;    // optional action function to call when one fails; returns Ok to retry it
};

bool hasShortOption_slOptions( char, const state_slOptions* );
bool hasLongOption_slOptions( const char*, const state_slOptions* );
char* getName_slOptions( const optionDef_slOptions* );

slReturn process_slOptions( int argc, const char *argv[], const psloConfig* config );

#endif //GPSCTL_SL_OPTIONS_H
