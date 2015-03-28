# semi-autonomous streaming wind station 

this repo contains the code and schematics for building a cheap and simple,
continuously streaming wind station with a PIC 16F88, a davis 7911 wind sensor 
and an icomsat/sim900 GSM/GPRS "modem".

it's semi-autonomous as it needs a server to collect and display the data
(GPRS is expensive hereabouts and the remote station needs to be simple 
and robust, and consume a minimal amount of energy as it needs to 
work off-grid).

directional readings are taken once per second, speed readings once
per 5 seconds. results are sent to the server every 5 seconds, in a single tiny UDP packet.

there's also the code for the receiving side, a small piece of perl that
listens for inbound readings (from any number of stations) and saves them 
in an sqlite database. it can also display and export readings, 
and two example plot files for Gnuplot are provided.

## building the station firmware

get the Microchip MPLAB/XC8 compiler (or adjust the code to work with SDCC,
which i've used in an earlier version of this project),
adjust the Makefile.example with your comms parameters (APN, target server 
and port) and make. flash to your 16F88, done.

## building the station hardware

check out the schematics and board layout files in the remote_station 
directory.

## further details

the detailed description of this project is available on my site 
at <http://snafu.priv.at/interests/tinkering/wind-mk3.html>.

## copyright, license

all this stuff is (c) 2013-2015 Alexander Zangerl <az@snafu.priv.at>
and licensed under the GNU General Public License.

share and enjoy `:-)`



