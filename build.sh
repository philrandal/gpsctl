#!/bin/bash
cd ~/gpsctl
gcc -std=c11 -Wall -rdynamic -g0 -fno-omit-frame-pointer -funwind-tables -o gpsctl gpsctl.c sl_general.c sl_serial.c sl_return.c sl_buffer.c sl_bits.c sl_options.c cJSON.c ublox.c iniparser.c dictionary.c
#
# -rdynamic                                                            // allows stack dumps to include symbols
# -g -fno-omit-frame-pointer -funwind-tables -mapcs -mno-sched-prolog  // magic that allows gdbserver to work
# -o gpsctl                                                            // specifies output file name
