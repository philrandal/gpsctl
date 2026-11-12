//
// Created by Tom Dilatush on 10/26/17.
//


// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#define _GNU_SOURCE

#include "sl_return.h"
#include "sl_options.h"


#define MAX_SL_OPTIONS 100   // the maximum number of options that slOptions can parse...

typedef struct {
    const optionDef_slOptions *def;
    char *arg;
} procOptionRecord;


struct state_slOptions {
    const optionDef_slOptions *optDefs;             // the option definitions, possibly enhanced
    int remainingArgs;                              // the number of arguments we haven't yet processed
    int currentArgIndex;                            // the index of the argument we're currently working on
    procOptionRecord optionRecords[MAX_SL_OPTIONS]; // our list of parsed option records
    int numOptionRecords;                           // the number of parsed option records in optionRecords
    char **ourArgv;                                 // our local and modifiable copy of the original argv
    bool debug;                                     // true if debug tracing is on
};


// Returns true if there is a parsed option definition with the given short (single character) name.
extern bool hasShortOption_slOptions( const char shortName, const state_slOptions *state ) {

    if( shortName == 0 ) return false;
    for( int i = 0; i < state->numOptionRecords; i++ ) {
        if((state->optionRecords[i].def->shortOpt != 0) && (shortName == state->optionRecords[i].def->shortOpt))
            return true;
    }
    return false;
}


// Returns true if there is a parsed option definition with the given long name.
extern bool hasLongOption_slOptions( const char *longName, const state_slOptions *state ) {

    if( longName == NULL) return false;
    for( int i = 0; i < state->numOptionRecords; i++ ) {
        if((state->optionRecords[i].def->longOpt != NULL) &&
           (0 == strcmp( longName, state->optionRecords[i].def->longOpt )))
            return true;
    }
    return false;
}


// Returns a string identifying the option in the given optionDef_slOptions record.  The string may be of the form
// "-l", "--long", or "-l, --long" depending on whether the short, long, or both option names are defined.  The string
// returned has been allocated and should be deleted when no longer needed.
extern char *getName_slOptions( const optionDef_slOptions *def ) {

    // figure out how much memory we need for the result...
    size_t size = 0;
    bool shortDefined = (def->shortOpt != 0);
    bool longDefined = (def->longOpt != NULL);
    if( shortDefined ) size += 2;
    if( longDefined ) size += 2 + strlen( def->longOpt );
    if( shortDefined && longDefined ) size += 2;
    size++;  // don't forget the terminating zero...

    // now allocate the memory and build the result...
    char *result = safeMalloc( size );
    char *ptr = result;
    if( shortDefined ) {
        *ptr++ = '-';
        *ptr++ = def->shortOpt;
        *ptr = 0;  // just in case there's no long name defined
    }
    if( shortDefined && longDefined ) {
        *ptr++ = ',';
        *ptr++ = ' ';
    }
    if( longDefined ) {
        *ptr++ = '-';
        *ptr++ = '-';
        strcpy( ptr, def->longOpt );
    }
    return result;
}


// Searches through the client-defined options for one with a short option that matches the given character.  If such
// an option is found, a pointer to it is returned - otherwise, NULL is returned.
static const optionDef_slOptions *findByCharOption( char opt, const optionDef_slOptions *optionDefs ) {
    const optionDef_slOptions *curDef = optionDefs;
    while( curDef->maxCount != 0 ) {
        if( curDef->shortOpt == opt )
            return curDef;
        curDef++;
    }
    return NULL;
}


// Searches through the client-defined options for one with a long option that matches the given string.  If such an
// option is found, a pointer to it is returned - otherwise, NULL is returned.  Note that the string match ONLY matches
// the first 'n' characters of the opt argument, where 'n' is the length of the long option in the definition.  This
// means that opt arguments that include an optional or required option argument will still be matched.
static const optionDef_slOptions *findByStringOption( char *opt, const optionDef_slOptions *optionDefs ) {
    const optionDef_slOptions *curDef = optionDefs;
    while( curDef->maxCount != 0 ) {
        if( !strempty( curDef->longOpt )) {
            if( 0 == strncmp( opt, curDef->longOpt, strlen( curDef->longOpt )))
                return curDef;
        }
        curDef++;
    }
    return NULL;
}


// Returns an error response with the given pattern and arguments resolved into a string.  Note that the errMsg value
// in the response was allocated, and should be freed when no longer needed.
static slReturn errorMessageHelper( errorInfo_slReturn error, bool debug, const char *class, const char *pattern, ... ) {

    // first we resolve the pattern we were given...
    va_list args;
    va_start( args, pattern );
    char *resolved;
    vasprintf( &resolved, pattern, args );
    va_end( args );

    // then we insert that into our pattern...
    slReturn result = makeErrorFmtMsgReturn( error, "command line options processing error during %s phase: %s.", class, resolved );
    if( debug ) printf( "Options processing error during %s phase: %s.\n", class, resolved );
    free( resolved );

    // return our bad news...
    return result;
}


// Prints a warning message.
static void warningMessageHelper( slReturn response, const char *class ) {
    printf( "Options %s warning: %s.\n", class, getReturnMsg( response ));
}


// If there is sufficient room, and if the maximum number of the given definition wouldn't be exceeed,
// adds the given definition to the list of option definition records parsed.
static slReturn addOptionRecord( state_slOptions *state, const optionDef_slOptions *def ) {

    // make sure we have room to record this one...
    if( state->numOptionRecords >= MAX_SL_OPTIONS )
        return errorMessageHelper(ERR_ROOT, state->debug, "parsing", "exceeded maximum number of options (%d)", MAX_SL_OPTIONS );

    // make sure this definition hasn't already been recorded the max number of times...
    int hits = 0;
    for( int i = 0; i < state->numOptionRecords; i++ )
        if( def == (state->optionRecords[i]).def )
            hits++;
    if( hits >= def->maxCount )
        return errorMessageHelper(ERR_ROOT, state->debug, "parsing", "too many occurrences of option \"%s\": max is %d",
                                   getName_slOptions( def ), def->maxCount );

    // ok, all is well - record it...
    (state->optionRecords[state->numOptionRecords]).def = def;
    (state->optionRecords[state->numOptionRecords]).arg = NULL;
    state->numOptionRecords++;

    return makeOkReturn();
}


// Prints the given arguments.
static void printArgs( int argc, char **args ) {
    for( int i = 0; i < argc; i++ ) {
        printf( "%s ", *(args + i));
    }
}


// Validate the given configuration, returning an error if any problems are found.
static slReturn validateConfig( const psloConfig *config ) {

    bool debug = (0 != (config->options & SL_OPTIONS_CONFIG_DEBUG));

    // make sure we've got an options definition array...
    if( config->optionDefs == NULL)
        return errorMessageHelper(ERR_ROOT, debug, "validation", "no option definitions configured" );

    // iterate over all the supplied definitions, making sure that all required arguments are present...
    int n = 0;
    for( optionDef_slOptions *curDef = config->optionDefs; curDef->maxCount != 0; curDef++, n++ ) {
        if((curDef->longOpt == NULL) && (curDef->shortOpt == 0))
            return errorMessageHelper(ERR_ROOT, debug, "validation",
                                       "option definition %d has neither a short form or a long form specified", n );
        if((curDef->shortOpt != 0) && !isprint( curDef->shortOpt ))
            return errorMessageHelper(ERR_ROOT, debug, "validation",
                                       "option definition '%s' has an invalid short name with character code: %d",
                                       getName_slOptions( curDef ), curDef->shortOpt );
        if((curDef->longOpt != NULL) && (NULL != strchr( curDef->longOpt, '=' )))
            return errorMessageHelper(ERR_ROOT, debug, "validation",
                                       "option definition '%s' contains an equal sign (\"=\"): '%s'",
                                       getName_slOptions( curDef ), curDef->longOpt );
        if((curDef->longOpt != NULL) && !issgraph( curDef->longOpt ))
            return errorMessageHelper(ERR_ROOT, debug, "validation", "option definition '%s' has an invalid long name: '%s'",
                                       getName_slOptions( curDef ), curDef->longOpt );
        if((curDef->maxCount < 0) || (curDef->maxCount > MAX_SL_OPTIONS))
            return errorMessageHelper(ERR_ROOT, debug, "validation", "option definition '%s' has a maxCount out of range %d",
                                       getName_slOptions( curDef ), curDef->maxCount );
        if( strempty( curDef->helpMsg ))
            return errorMessageHelper(ERR_ROOT, debug, "validation", "option definition '%s' has no help message specified",
                                       getName_slOptions( curDef ));
        if((curDef->arg != argNone) && strempty( curDef->argName ))
            return errorMessageHelper(ERR_ROOT, debug, "validation",
                                       "option definition '%s' can have an argument, but has no argument name specified",
                                       getName_slOptions( curDef ));
        if((curDef->shortOpt == ' ') && (curDef->arg != argNone))
            return errorMessageHelper(ERR_ROOT, debug, "validation", "single-hyphen option cannot have an argument" );
        if((curDef->shortOpt == ' ') && !strempty( curDef->longOpt ))
            return errorMessageHelper(ERR_ROOT, debug, "validation", "single-hyphen option cannot have a long name" );

        // iterate over the supplied definitions again to make sure we don't have a duplicate short or long name...
        for( optionDef_slOptions *chkDef = config->optionDefs; chkDef->maxCount != 0; chkDef++ ) {
            if( chkDef != curDef ) {
                if((curDef->shortOpt != 0) && (curDef->shortOpt == chkDef->shortOpt))
                    return errorMessageHelper(ERR_ROOT, debug, "validation",
                                               "two option definitions have the same short name: %c",
                                               curDef->shortOpt );
                if((curDef->longOpt != NULL) && (chkDef->longOpt != NULL) &&
                   (strcmp( curDef->longOpt, chkDef->longOpt ) == 0))
                    return errorMessageHelper(ERR_ROOT, debug, "validation",
                                               "two option definitions have the same long name: %s", curDef->longOpt );
            }
        }
    }

    return makeOkReturn();
}


typedef struct {
    const optionDef_slOptions *curDef;
    int curRec;
} optRecIteratorState;


// Initializes the iterator state.
static void optRecIteratorInit( optRecIteratorState *state, const optionDef_slOptions *optionDefs ) {
    state->curDef = optionDefs;
    state->curRec = 0;
}


// Returns the next procOptionRecord in the iteration, or NULL if there are no more.  The order of this iterator is
// slightly tricky: it first iterates over the client-supplied option definitions, in order.  Within each of those
// definitions, it then iterates over the option records in the processor state, which are in the order specified
// by the program's user on the command line.  Each of those records whose definition matches the current client
// option definition is then returned.  This means that procOptionRecords are returned in the same order that the
// definitions were supplied in, and there will be multiple records with the same option definition returned IF the
// user specified more than one AND multiple instances of that particular option are allowed.
static const procOptionRecord *optRecIterator( optRecIteratorState *itState, const state_slOptions *state ) {

    // iterate over all the option definitions looking for those with parsing functions, then for each of those
    // iterate over the parsed option records those with the same definition, then calling the parsing function for each
    for( ; itState->curDef->maxCount != 0; itState->curDef++ ) {
        for( ; itState->curRec < state->numOptionRecords; itState->curRec++ ) {
            const procOptionRecord *rec = &state->optionRecords[itState->curRec];
            if( itState->curDef == rec->def ) {
                itState->curRec++;
                return rec;
            }
        }
        itState->curRec = 0;
    }
    return NULL;
}


// Print a help message.
static slReturn helpAction( const optionDef_slOptions *defs, const psloConfig *config ) {

    // print a nice header...
    printf( "Options for %s:\n\n", config->name );

    // iterate over all the option definitions to find out how wide the widest name/argument is..
    int maxWidth = 0;
    for( const optionDef_slOptions *curDef = defs; curDef->maxCount != 0; curDef++ ) {
        int thisWidth = 0;
        if( curDef->shortOpt != 0 ) thisWidth += 2;  // allow for '-' plus the short name...
        if( !strempty( curDef->longOpt ))
            thisWidth += 2 + strlen( curDef->longOpt );  // allow for '--' plus the long name...
        if((curDef->shortOpt != 0) && (!strempty( curDef->longOpt )))
            thisWidth += 2;  // allow for the ', ' between short and long name...
        if( curDef->arg != argNone )
            thisWidth += 3 + strlen( curDef->argName );  // allow for space and surrounding [] or <>...
        if( thisWidth > maxWidth ) maxWidth = thisWidth;
    }

    // iterate over all the option definitions again, this time to actually print the help...
    bool gotReqArg = false;
    bool gotOptArg = false;
    for( const optionDef_slOptions *curDef = defs; curDef->maxCount != 0; curDef++ ) {

        // first we make our format string...
        char *format;
        asprintf( &format, "  %%-%ds %%s%%s\n", maxWidth + 2 );

        // now we make the name/argument string pieces...
        char *name = getName_slOptions( curDef );
        if( name[1] == '-' ) {
            char* name1 = concat( "    ", name );
            free( name );
            name = name1;
        }
        char *namearg;
        if( curDef->arg == argNone )
            namearg = concat( name, "" );
        else {
            char *lb = (curDef->arg == argRequired) ? " <" : "[=";
            char *rb = (curDef->arg == argRequired) ? ">" : "]";
            char *x1 = concat( lb, curDef->argName );
            char *x2 = concat( x1, rb );
            namearg = concat( name, x2 );
            free( x1 );
            free( x2 );
        }
        if( curDef->arg == argRequired ) gotReqArg = true;
        if( curDef->arg == argOptional ) gotOptArg = true;

        // now we determine the suffix (for options that can be repeated n times)...
        char *repeat;
        if( curDef->maxCount == 1 )
            repeat = "";
        else {
            asprintf( &repeat, " (may repeat up to %d times)", curDef->maxCount );
        }

        // finally, we actually do it...
        printf( format, namearg, curDef->helpMsg, repeat );
        free( namearg );
        if( strlen( repeat ) > 0 ) free( repeat );
    }

    // now we put our trailing arguments explanation out...
    if( gotOptArg || gotReqArg ) printf( "\n" );
    if( gotReqArg ) printf( "  Arguments in <angle brackets> are required; they MUST be supplied.\n" );
    if( gotOptArg ) printf( "  Arguments in [square brackets] are optional; they may be omitted.\n" );

    // if we were given additional info text, print it...
    if( config->addedInfo != NULL) {
        printf( "\n" );
        printf( config->addedInfo );
    }

    return makeOkReturn();
}


// Print a version message.
static slReturn versionAction( const optionDef_slOptions *defs, const psloConfig *config ) {
    printf( "%s version %s\n", config->name, config->version );
    return makeOkReturn();
}


// Append string to an existing string.
static void accum_append( char **existing, char *append ) {
    if( *existing == NULL) {
        *existing = safeMalloc( 1 + strlen( append ));
        strcpy( *existing, append );
        return;
    }
    size_t existingLen = strlen( *existing );
    char *result = safeMalloc( 1 + existingLen + strlen( append ));
    strcpy( result, *existing );
    strcpy( result + existingLen, append );
    free( *existing );
    *existing = result;
}


// Append character to existing string.
static void accum_append_char( char **existing, char append ) {
    char onechar[2] = {append, 0};
    accum_append( existing, onechar );
}


// Print a usage message.
static slReturn usageAction( const optionDef_slOptions *defs, const psloConfig *config ) {

    // where we're going to accumulate information...
    char *shortNoArg = NULL;
    char *shortArg = NULL;
    char *longNoArg = NULL;
    char *longArg = NULL;

    // iterate over all our option definitions, collecting usage information...
    for( const optionDef_slOptions *curDef = defs; curDef->maxCount != 0; curDef++ ) {

        // if we have a short name defined, include it...
        if( curDef->shortOpt != 0 ) {
            if( curDef->arg == argNone ) {
                if( shortNoArg == NULL) accum_append( &shortNoArg, "-" );
                accum_append_char( &shortNoArg, curDef->shortOpt );
            } else {
                if( shortArg != NULL) accum_append( &shortArg, " " );
                accum_append_char( &shortArg, '-' );
                accum_append_char( &shortArg, curDef->shortOpt );
                if( curDef->arg == argRequired ) {
                    accum_append( &shortArg, " <" );
                    accum_append( &shortArg, curDef->argName );
                    accum_append_char( &shortArg, '>' );
                } else {
                    accum_append( &shortArg, "[=" );
                    accum_append( &shortArg, curDef->argName );
                    accum_append_char( &shortArg, ']' );
                }
            }
        }

        // if we have a long name defined, include it...
        if( !strempty( curDef->longOpt )) {
            if( curDef->arg == argNone ) {
                if( !strempty( longNoArg )) accum_append( &longNoArg, " " );
                accum_append( &longNoArg, "--" );
                accum_append( &longNoArg, curDef->longOpt );
            } else {
                if( longArg != NULL) accum_append( &longArg, " " );
                accum_append( &longArg, "--" );
                accum_append( &longArg, curDef->longOpt );
                if( curDef->arg == argRequired ) {
                    accum_append( &longArg, " <" );
                    accum_append( &longArg, curDef->argName );
                    accum_append( &longArg, ">" );
                } else {
                    accum_append( &longArg, "[=" );
                    accum_append( &longArg, curDef->argName );
                    accum_append( &longArg, "]" );
                }
            }
        }
    }

    // now print our pretty lines...
    printf( "Usage:\n" );
    printf( "  With short options: %s %s %s\n", config->name, shortNoArg, shortArg );
    printf( "  With long options: %s %s %s\n", config->name, longNoArg, longArg );

    // free up any memory we used...
    if( shortArg != NULL) free( shortArg );
    if( shortNoArg != NULL) free( shortNoArg );
    if( longArg != NULL) free( longArg );
    if( longNoArg != NULL) free( longNoArg );

    return makeOkReturn();
}


// default option definitions, potentially automatically included...
static optionDef_slOptions helpOptDef = {
        1, "help", '?', argNone,
        NULL, NULL, 0,
        NULL,
        helpAction,
        NULL,
        "display this help message"
};

static optionDef_slOptions versionOptDef = {
        1, "version", 'V', argNone,
        NULL, NULL, 0,
        NULL,
        versionAction,
        NULL,
        "display the version of this program"
};

static optionDef_slOptions usageOptDef = {
        1, "usage", 'U', argNone,
        NULL, NULL, 0,
        NULL,
        usageAction,
        NULL,
        "display a short description of this program's usage"
};


static void checkOptionalDefEntry( optionDef_slOptions **optDefs, int *count, int mask, optionDef_slOptions *def,
                                   const psloConfig *config ) {

    // if it isn't enabled, do nothing...
    if((config->options & mask) == 0 ) return;

    // see if the client defined anything conflicting,and if so, suppress our def...
    if((def->shortOpt != 0) && findByCharOption( def->shortOpt, config->optionDefs ))
        def->shortOpt = 0;
    if((def->longOpt != NULL) && findByStringOption( def->longOpt, config->optionDefs ))
        def->longOpt = NULL;

    // if we have any names still defined, add this option def...
    if((def->longOpt != NULL) || (def->shortOpt != 0)) {
        (*optDefs)[*count] = *def;
        (*count)++;
    }
}


static const optionDef_slOptions *ensureOptDefs( const psloConfig *config ) {

    // how many client-defined option definitions are there?
    int numOptDefs;
    for( numOptDefs = 0; config->optionDefs[numOptDefs].maxCount != 0; numOptDefs++ );

    // allocate enough memory to hold all the client-defined options, plus up to three more, and our terminator...
    optionDef_slOptions *result = safeMalloc( sizeof( optionDef_slOptions ) * (4 + numOptDefs));
    numOptDefs = 0;  // start building our result at the beginning...

    // if we should, add our automatic entries...
    checkOptionalDefEntry( &result, &numOptDefs, SL_OPTIONS_CONFIG_ADD_VERSION, &versionOptDef, config );
    checkOptionalDefEntry( &result, &numOptDefs, SL_OPTIONS_CONFIG_ADD_USAGE, &usageOptDef, config );
    checkOptionalDefEntry( &result, &numOptDefs, SL_OPTIONS_CONFIG_ADD_HELP, &helpOptDef, config );

    // now copy the client-defined option definitions...
    for( int i = 0; config->optionDefs[i].maxCount != 0; i++, numOptDefs++ )
        result[numOptDefs] = config->optionDefs[i];

    // finally, stuff a terminator and return...
    result[numOptDefs].maxCount = 0;

    return result;
}


#define CUR_ARG       (*(state->ourArgv + state->currentArgIndex))
#define LAST_OPT_REC  (state->optionRecords[state->numOptionRecords - 1])
#define REM_ARGS      (state->remainingArgs)
#define CUR_ARG_INDEX (state->currentArgIndex)
#define DEBUG         (state->debug)

static slReturn parsePhase( int argc, const psloConfig *config, state_slOptions *state ) {

    // in the options processing phase, we need to read each argument once (unless we get stopped by something)...
    while( REM_ARGS > 0 ) {

        if( DEBUG) printf( "Parsing argument %d (\"%s\")\n", CUR_ARG_INDEX, CUR_ARG);

        // if we're expecting an argument for a previously processed option, handle it...
        if((state->numOptionRecords > 0) && (LAST_OPT_REC.def->arg == argRequired) && (LAST_OPT_REC.arg == NULL)) {
            if( DEBUG)
                printf( "Assigning argument \"%s\" to option \"%s\"\n", CUR_ARG, getName_slOptions(LAST_OPT_REC.def ));
            LAST_OPT_REC.arg = CUR_ARG;
            CUR_ARG_INDEX++;
            REM_ARGS--;
        }

            // if this argument is a standalone "--?", and it's the first argument, then we're turning on debug tracing...
        else if((CUR_ARG_INDEX == 1) && (0 == strcmp( "--?", CUR_ARG))) {
            DEBUG = true;
            printf( "Debug tracing turned on by --? option\n" );
            REM_ARGS--;
            CUR_ARG_INDEX++;
        }

            // if this argument is a standalone "--", stop our option processing here...
        else if( 0 == strcmp( "--", CUR_ARG)) {
            REM_ARGS = 0;
            CUR_ARG_INDEX++;
        }

            // if this argument starts with a '-', handle it...
        else if( *CUR_ARG == '-' ) {

            // how much argument do we have??
            size_t curArgLen = strlen(CUR_ARG);

            // if the second character is ALSO a "-", the we've got a long option, so handle that...
            if((curArgLen > 2) && (*(CUR_ARG + 1) == '-')) {

                char *curOpt = CUR_ARG + 2;  // point to the start of the long option...

                // try to find this option in our option definitions...
                const optionDef_slOptions *def = findByStringOption( curOpt, state->optDefs );

                // if we found it, process it...
                if( def != NULL) {

                    if( DEBUG) printf( "Found long option: %s\n", curOpt );

                    // first we add the option record...
                    slReturn response = addOptionRecord( state, def );
                    if( isErrorReturn( response ) ) return response;

                    // if we have a suffix (which might be an option argument), handle that...
                    if( strlen( curOpt ) > strlen( def->longOpt )) {
                        curOpt += strlen( def->longOpt );  // move to the beginning of the suffix...

                        // if we have an equals sign, then the rest of the suffix is the argument...
                        if( *curOpt == '=' ) {

                            // if the option is allowed to have an argument, assign it...
                            if( def->arg != argNone ) {
                                LAST_OPT_REC.arg = ++curOpt;
                                if( DEBUG)
                                    printf( "Assigning argument \"%s\" to \"%s\" option\n", curOpt,
                                            LAST_OPT_REC.def->longOpt );
                            } else
                                return errorMessageHelper(ERR_ROOT, DEBUG, "parsing",
                                                          "unexpected argument (\"%s\") for long option (\"%s\")",
                                                          ++curOpt, def->longOpt );
                        }

                            // otherwise, we've got something that makes no sense...
                        else
                            return errorMessageHelper(ERR_ROOT, DEBUG, "parsing",
                                                      "invalid option argument (possibly missing the \"=\"): \"%s\"",
                                                      curOpt );
                    }
                } else
                    return errorMessageHelper(ERR_ROOT, DEBUG, "parsing", "unrecognized long option (\"%s\") in argument %d",
                                              curOpt, CUR_ARG_INDEX);
            }

                // otherwise, we have one or more one-character options - process them one-by-one...
            else {

                // if we've got the special case of "-"...
                if( curArgLen == 1 ) {

                    // try to find this option in our option definitions...
                    const optionDef_slOptions *def = findByCharOption( ' ', state->optDefs );

                    // if we found it, then process it...
                    if( def != NULL) {
                        if( DEBUG) printf( "Found single-hyphen option: '-'\n" );
                        slReturn response = addOptionRecord( state, def );
                        if( isErrorReturn( response ) ) return response;
                    } else
                        return errorMessageHelper(ERR_ROOT, DEBUG, "parsing",
                                                  "unrecognized single-hyphen option (\"-\") in argument %d",
                                                  CUR_ARG_INDEX);
                } else {
                    char *curOpt = CUR_ARG + 1;
                    while( *curOpt != 0 ) {

                        // try to find this option in our option definitions...
                        const optionDef_slOptions *def = findByCharOption( *curOpt, state->optDefs );

                        // if we found it, then process it...
                        if( def != NULL) {
                            if( DEBUG) printf( "Found single-character option: '%c'\n", def->shortOpt );
                            slReturn response = addOptionRecord( state, def );
                            if( isErrorReturn( response ) ) return response;

                            // if we have optional or required arguments, handle that...
                            bool haveMore = (*(curOpt + 1) != 0);  // true if the terminator isn't the next character...
                            if( (def->arg != argNone) && haveMore ) {
                                curOpt++;  // get past our option letter...
                                if( *curOpt == '=' ) curOpt++;  // move past the equals sign, if there is one...
                                LAST_OPT_REC.arg = curOpt;
                                if( DEBUG)
                                    printf( "Assigning argument \"%s\" to \"%c\" option\n", curOpt,
                                            LAST_OPT_REC.def->shortOpt );
                                curOpt += strlen( curOpt );  // move to the null terminator...
                            } else
                                curOpt++;  // move to the next character in the argument...
                        }

                            // otherwise, someone fed us an option we don't know about...
                        else
                            return errorMessageHelper(ERR_ROOT, DEBUG, "parsing",
                                                      "unrecognized single character option (\"%c\") in argument %d",
                                                      *curOpt, CUR_ARG_INDEX);
                    }
                }
            }
            CUR_ARG_INDEX++;
            REM_ARGS--;
        }

            // otherwise, we have no idea what this is, so handle THAT...
        else {

            // if we're in strict mode, we're done...
            if( config->options & SL_OPTIONS_CONFIG_STRICT) {
                if( DEBUG) printf( "Unidentified argument in strict mode; halting option processing\n" );
                REM_ARGS = 0;
            }

                // otherwise, it's time to permute...
            else {

                if( DEBUG) printf( "Command line argument not recognized as either an option or an option argument\n" );

                // we only need to do this if there are at least two arguments remaining...
                if((1 + argc - CUR_ARG_INDEX) >= 2 ) {
                    if( DEBUG) {
                        printf( "Before permuting: " );
                        printArgs( argc, state->ourArgv );
                        printf( "\n" );
                    }
                    char *thisArg = *(state->ourArgv + CUR_ARG_INDEX);
                    for( int i = CUR_ARG_INDEX + 1; i < argc; i++ ) {
                        *(state->ourArgv + i - 1) = *(state->ourArgv + i);
                    }
                    *(state->ourArgv + argc - 1) = thisArg;
                    if( DEBUG) {
                        printf( " After permuting: " );
                        printArgs( argc, state->ourArgv );
                        printf( "\n" );
                    }
                }
                REM_ARGS--;
            }
        }
    }

    // make sure the last option has an argument, if it requires one...
    if( (state->numOptionRecords > 0) && (LAST_OPT_REC.def->arg == argRequired) && (LAST_OPT_REC.arg == NULL) )
        return errorMessageHelper(ERR_ROOT, DEBUG, "parsing", "option [%s] missing required argument",
                                  getName_slOptions(LAST_OPT_REC.def ));

    // return with an error-free response...
    // Note that ourArgv is a local array in process_slOptions() that disappears when that function returns, so we
    // must NOT hand back a pointer into it.  Instead we return a freshly allocated array holding the remaining
    // argument pointers (which themselves point to the caller's original argv strings).  The caller owns both the
    // args_slOptions and its argv array (if any), and may free() them.
    int remaining = argc - CUR_ARG_INDEX;
    if( remaining < 0 ) remaining = 0;
    args_slOptions *args = safeMalloc( sizeof( args_slOptions ));
    args->argc = remaining;
    args->argv = NULL;
    if( remaining > 0 ) {
        args->argv = safeMalloc( sizeof( char* ) * (size_t) remaining );
        for( int i = 0; i < remaining; i++ )
            args->argv[i] = *(state->ourArgv + CUR_ARG_INDEX + i);
    }
    return makeOkInfoReturn( args );
}

#undef DEBUG
#undef REM_ARGS
#undef CUR_ARG_INDEX
#undef CUR_ARG
#undef LAST_OPT_REC


#define CUR_ARG (*(state.ourArgv + state.currentArgIndex))   //(*(state.ourArgv) + state.currentArgIndex)
#define LAST_OPT_REC (state.optionRecords[state.numOptionRecords - 1])
#define DEBUG (state.debug)
#define EH( result, class )                                                                     \
    if ( isErrorReturn( result )) {                                                             \
        free( (void*) state.optDefs );                                                          \
        free( args->argv );                                                                     \
        free( args );                                                                           \
        return errorMessageHelper( ERR_CAUSE( result ), DEBUG, class, "fatal error" );          \
    } else if( isWarningReturn( result )) {                                                     \
        warningMessageHelper( result, class );                                                  \
    }

extern slReturn process_slOptions( int argc, const char *argv[], const psloConfig *config ) {

    // validate the client's supplied configuration, to the extent we can...
    if( config->options & SL_OPTIONS_CONFIG_DEBUG) printf( "Validating client-supplied configuration\n" );
    slReturn response = validateConfig( config );
    if( isErrorReturn( response )) return response;

    // get copy of our option definitions, possibly enhanced...
    const optionDef_slOptions *optDefs = ensureOptDefs( config );

    // make a local (and mutable) copy of the original argv...
    char *argv_copy[argc];
    for( int i = 0; i < argc; i++ ) argv_copy[i] = (char *) argv[i];

    // initialize our state...
    bool haveDebug = (0 != (config->options & SL_OPTIONS_CONFIG_DEBUG));
    state_slOptions state;              // the current state of options processing...
    state.optDefs = optDefs;            // our (possibly enhanced) option definitions...
    state.remainingArgs = argc - 1;     // the number of arguments we haven't yet processed...
    state.currentArgIndex = 1;          // the index of the argument we're currently working on...
    state.numOptionRecords = 0;         // of course we haven't parsed any yet...
    state.ourArgv = (char **) &argv_copy; // a copy of argv that we can modify...
    state.debug = haveDebug;            // if the caller asked for debug, he's got it...

    // parse our options, and bail if we got an error...
    if( DEBUG) printf( "Starting command line option parsing phase\n" );
    response = parsePhase( argc, config, &state );
    if( isErrorReturn( response )) {
        free((void *) state.optDefs );
        return response;
    }

    // save the results of the parse phase for return when we're finished (if successful)...
    args_slOptions *args = getReturnInfo( response );
    freeReturn( response );  // frees the return record itself; the args it carried (which we now own) are not freed

    // when we get here, response.argc is the count of unprocessed arguments pointed to by response.argv...
    if( DEBUG) printf( "Starting individual option parsing phase\n" );
    optRecIteratorState itState;
    optRecIteratorInit( &itState, state.optDefs );
    const procOptionRecord *rec;
    while((rec = optRecIterator( &itState, &state )) != NULL) {

        // if we have a parse function, call it...
        if( rec->def->parse ) {
            slReturn result = rec->def->parse( rec->def->parsePtr, rec->def->parseInt, rec->def, rec->arg,
                                               config->clientData );
            EH( result, "parsing" )
        }
    }

    if( DEBUG) printf( "Starting individual option constraint phase\n" );
    if( config->beforeConstraint ) {
        slReturn cr = config->beforeConstraint( state.optDefs, config, &state );
        EH( cr, "constraint" )
    }
    optRecIteratorInit( &itState, state.optDefs );
    while((rec = optRecIterator( &itState, &state )) != NULL) {

        // if we have a constraint-checking function, call it...
        if( rec->def->cnstr ) {
            slReturn result = rec->def->cnstr( state.optDefs, config, &state );
            EH( result, "constraint" )
        }
    }
    if( config->afterConstraint ) {
        slReturn cr = config->afterConstraint( state.optDefs, config, &state );
        EH( cr, "constraint" )
    }

    if( DEBUG) printf( "Starting individual option action phase\n" );
    if( config->beforeAction ) {
        slReturn ar = config->beforeAction( state.optDefs, config );
        EH( ar, "action" )
    }
    optRecIteratorInit( &itState, state.optDefs );
    while((rec = optRecIterator( &itState, &state )) != NULL) {

        // if we have an action function, call it...
        if( rec->def->action ) {
            slReturn result = rec->def->action( state.optDefs, config );
            EH( result, "action" )
        }
    }
    if( config->afterAction ) {
        slReturn ar = config->afterAction( state.optDefs, config );
        EH( ar, "action" )
    }

    free((void *) state.optDefs );
    return makeOkInfoReturn( args );
}

#undef DEBUG
#undef LAST_OPT_REC
#undef CUR_ARG
