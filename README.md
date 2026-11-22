<h1 align="center"><b>gpsctl 1.41</b></h1>

## What is gpsctl?
*gpsctl* is a utility program written for Raspberry Pi computers using a U-Blox GPS board.  The
author's rig is a
[Raspberry Pi 3 Model B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) with a
[Uputronix GPS board](https://store.uputronics.com/index.php?route=product/product&product*id=81),
which uses a [U-Blox MAX-M8Q](https://www.u-blox.com/en/product/max-m8-series) GPS module.  Some
of *gpsctl*'s functions are generic for any GPS that streams NMEA data to a Linux serial port, and
these should work on any Linux system.  Most of *gpsctl*'s functions, however, are quite specific
to the U-Blox products, and use their proprietary UBX protocol (again, on a serial port).

## Why does the world need gpsctl?
Well, probably the world doesn't actually need *gpsctl* - but if you happen to be attempting the same
thing the author was, you might find it useful.  Configuring and controlling a U-Blox GPS connected to a
Linux host is a frustrating exercise.  The author ran into this when implementing a
[stratum 1 NTP server](https://www.endruntechnologies.com/stratum1.htm) using the aforementioned
hardware.  He needed to make some configuration changes to better support this.  Those configuration
changes required the use of the UBX protocol, and support for this on Linux is ... minimal.  Hence
*gpsctl*!  

On the other hand, anyone trying to "talk" to a U-Blox GPS from a C program might find the
code in *gpsctl* to be a useful building block.  The code in *gpsctl* includes functions for sending and
receiving UBX protocol messages, for interpreting them, modifying them, etc.  This could easily be extended to
serve other purposes; after all, not everyone lusts for a stratum 1 NTP server, and there might actually be other
useful purposes for a GPS (though I can't think of one right now).

## What, exactly, does *gpsctl* do?
Some *gpsctl* functions work for any GPS streaming NMEA data over a serial port:
* Echoes NMEA data to stdout, optionally filtered by NMEA message type.
* Infers the GPS' baud rate from the stream of NMEA data.

Other *gpsctl* functions are specific to U-Blox GPS hardware:
* Configures the baud rate that the U-Blox GPS uses for communication.
* Configures whether the U-Blox GPS transmits NMEA data.
* Infers the GPS' baud rate using the UBX protocol only.
* Configures the use of the European Galileo satellites (which is off by default in U-Blox 3.01 firmware)
and enables NMEA version 4.1 output for Galileo
* Queries the U-Blox GPS for position, time, GPS version, GPS configuration, and GPS satellite information.
  The format of query results are selectable: either plain English or JSON.
* Tests the pulse-per-second output of the U-Blox GPS (fundamental for NTP).
* Configures the U-Blox GPS for maximum timing pulse accuracy (useful for building a stratum 1 NTP server
based on the GPS' notion of time).
* Disables NMEA RMC, VTG, GSA, GSV, GLL, GGA messages and enables ZDA (for NTP driver 20 mode 8)
* Saves the U-Blox GPS configuration to on-module battery-backed memory.
* Resets the U-Blox GPS.

## Dependencies
The only dependencies *gpsctl* has (other than the standard C library) is on the JSON library
[cJSON](https://github.com/DaveGamble/cJSON), and [iniparser](https://github.com/philrandal/iniparser). The source for iniparser and cJSON (which have MIT licenses) are incorporated in this project.  

## Why is *gpsctl*'s code so awful?
Mainly because this is the first C program the author has written in over 30 years, but also because the author
has serious deficiencies in aptitude, intelligence, and knowledge (not to mention choices of hobbies).
You can also blame Claude and DeepSeek for making things better or worse, if you want.

## How is *gpsctl* licensed?
*gpsctl* is licensed with the quite permissive MIT license:
> Created: October 18, 2017
> Author: Tom Dilatush <tom@dilatush.com>  
> Github:  
> License: MIT
>
> Copyright 2017 Tom Dilatush
>
> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
> documentation files (the "Software"), to deal in the Software without restriction, including without limitation
> the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
> to permit persons to whom the Software is furnished to do so.
>
> The above copyright notice and this permission notice shall be included in all copies or substantial portions of
> the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
> THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE A
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
> TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.

## Usage

    Options for gpsctl:

    -V, --version               display the version of this program
    -U, --usage                 display a short description of this program's usage
    -?, --help                  display this help message
    -v, --verbose               get verbose messages (may repeat up to 2 times)
    -q, --quiet                 suppress normal output
    -p, --port <device>         specify the port device (default is '/dev/serial0')
    -j, --json                  select JSON query output (may repeat up to 3 times)
        --shared_port           allow use of a serial port that another process has open
    -a, --autobaud[=method]     infer baud rate using ascii, nmea, ubx data (default is ubx)
    -b, --baud <baud rate>      specify different host baud rate; any standard rate is allowed
    -M, --minbaud <baud rate>   specify minimum baud rate for autobaud (default is '9600'); any standard rate is allowed
    -s, --sync[=method]         synchronize using ascii, nmea, ubx data (default is ubx)
    -B, --newbaud <baud rate>   change the U-Blox and host baud rates to the specified standard rate
    -n, --nmea <yes/no>         configure whether the GPS sends NMEA data to the serial port (see "Yes or no values" below)
    -Q, --query <query type>    query the GPS for info (see "Query types" below)
        --galileo               configure the GPS for Galileo Satellites
        --satellites            configure the GPS Satellites only
        --configure_for_timing  configure the GPS for maximum timing precision
        --save_config           save the GPS configuration to battery-backed RAM
        --reset                 reset the GPS
    -e, --echo[=seconds]        echo human-readable GPS data to stdout for the given time (0 or unspecified means forever)

    Arguments in <angle brackets> are required; they MUST be supplied.
    Arguments in [square brackets] are optional; they may be omitted.

    Additional information:

    Query types:
      fix         returns position, altitude, and time information
      version     returns hardware and software version of the GPS
      config      returns GPS configuration information
      satellites  returns currently visible satellite information from the GPS

    Yes or no values:
      yes         indicated by an initial character of 'y', 'Y', 't', 'T', or '1', or by 'on'
      no          indicated by anything else

    Examples:
      gpsctl --port=/dev/serial1 --baud 9600 --echo=5
        Selects the serial port "/dev/serial1", sets the host serial port baud rate to 9600, and
        echoes any received characters (presumably NMEA data) to stdout.
      gpsctl -p /dev/serial1 -b 9600 -e=5
        Exactly the same as the preceding example, using short options.

## Examples
To autodetect the current baud rate, set the pi and u-blox module to 115200 baud, configure stationary mode, Galileo satellites, and NMEA ZDA output only, and to see what it's doing

     gpsctl -a -B 115200 --configure_for_timing -vv

To autodetect the current baud rate and configure satellites only

     gpsctl -a --satellites

To view the current u-blox config

     gpsctl -a -Q config

 ## New in version 0.7 and later

 At long last, the file /etc/gpsctl.conf is parsed for configuration options.

 The [gpsctl] section is used to determine which serial port gpsctl uses.

 The other sections are honoured by --galileo and --configure_for_timing, and override the defaults if present.

 Without the gpsctl.conf file the program's behaviour will be the same as with version 0.6.

 Numbers must be written as numbers and nothing else: a value that isn't entirely a number, or is outside the range its
 field allows, is reported on stderr and the default is used.  Anything else on the line (units, remarks) goes in a
 comment, which starts with ';' or '#' and runs to the end of the line, e.g. `Antenna cable delay = 56 ; nanoseconds`.
 (A value can therefore not contain ';' or '#'.)

 Note: Either BeiDou or Glonass can be configured, but not both at the same time as GPS or Galileo.  If neither GPS nor Galileo are enabled, you can enable both BeiDou and Glonass.  If you try with GPS or Galileo enabled, the [gpsctl] option "prefer beidou to glonass if both enabled" (default: no) will override one of them.


 An example which configures for Galileo satellites:

    #
    # example gpsctl.conf which enables Galileo
    #   C/No threshold reduced to 20 from the original gpsctl's 40
    #   This gets a fix on 14 satellites in my home testing with an indoor GPS antenna 
    #
    [gpsctl]
    port = /dev/ttyAMA0
    # sync method: ASCII = 1, NMEA = 2, UBX = 3
    sync method = 3
    verbosity = 0
    # Only one of Beidou and GLONASS can be used at the same time
    # so rather than error out trying to enable both
    # choose which one to use
    prefer beidou to glonass if both enabled = no

    [NMEA]
    # enabled = true / false
    enabled = true
    version = 4.1
    GGA = off
    GLL = off
    GSA = off
    GSV = off
    RMC = on
    VTG = off
    GRS = off
    GST = off
    ZDA = on

    [GPS]
    enabled = yes
    minimum channels=8
    maximum channels=16

    [SBAS]
    enabled = no
    minimum channels=1
    maximum channels=3

    [Galileo]
    enabled = yes
    minimum channels=4
    maximum channels=8

    [Beidou]
    enabled = no
    minimum channels=8
    maximum channels=16

    [IMES]
    enabled = no
    minimum channels=0
    maximum channels=8

    [QZSS]
    enabled = no
    minimum channels=0
    maximum channels=3

    [GLONASS]
    enabled = yes
    minimum channels=8
    maximum channels=14

    [Navigation Engine]
    # Dynamic model: Portable = 0, Stationary = 2, Pedestrian = 3, Automotive = 4, Sea = 5, Air1G = 6, Air2G = 7, Air4G = 8, Watch = 9
    Dynamic model =                2
    # Fix mode: 2D only = 1, 3D only = 2, auto 2D/3D = 3
    Fix mode =                     2
    Fixed altitude (2D) =          0.00 ; meters
    Fixed altitude variance (2D) = 0.0000 ; meters^2
    Minimum elevation =            20 ; degrees
    Position DoP mask =            10.0
    Time DoP mask =                10.0
    Position accuracy mask =       40 ; meters
    Time accuracy mask =           40 ; meters
    Static hold threshold =        0 ; cm/s
    Dynamic GNSS timeout =         60 ; seconds
    Threshold above C/No =         8 ; satellites
    C/No threshold =               20 ; dBHz
    Static hold max distance =     0 ; meters
    # UTC Standard: AutoUTC = 0, USNO_UTC = 3, GLONASS_UTC = 6, BeiDou_UTC = 7
    UTC standard =                 3

    [Time Pulse]
    # the nanoseconds / microseconds after the numbers are just reminders, they don't mean anything to the config parser
    Antenna cable delay = 56 ; nanoseconds
    RF group delay = 20 ; ns
    Unlocked pulse period = 1000000 ; microseconds
    Unlocked pulse length = 0
    Locked pulse period = 1000000 ; microseconds
    Locked pulse length = 500000 ; microseconds
    User configurable delay = 0

## Why did you fork it, Phil?

It all started when I decided to build a [Raspberry Pi Stratum 1 NTP server](http://www.philrandal.co.uk/blog/archives/2019/04/entry_213.html).

I found Tom Dilatush's original gpsctl when looking for something to configure my Raspberry Pi plus u-blox GPS to receive Galileo
satellite output.  And then I tinkered a bit, tweaking settings, recompiling, tweaking again, and so on.

Hence the idea to use a configuration file to drive it.

Enjoy.

## In Memoriam

Tom Dilatush (aka slightlyloony on here, slightlyloony1 on Twitter), sadly passed away on February 14th, 2024, from Amyotrophic lateral sclerosis (ALS), also known as motor neurone disease (MND).

If you find this program useful, please consider donating to your local ALS / MND charity.
