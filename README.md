bmw-ibus
========

BMW IBus Daemon reads BMW IBus data through serial port. 
- It detects BMW board monitor(at least BM53) unit and steering wheel button 
presses from IBus data, maps them to key events and injects them to system 
event queue via uinput.

- It also can be configured to inject key events only in certain state like
TAPE or AUX which can be useful if you want to hijack for example TAPE
mode for other use.

- It can be configured to control video input pin(reverse cam) via CTS,RTS

IBUS device,message and data codes are from various sources but mostly found 
from following web sites:
http://autos.groups.yahoo.com/group/HackTheIBus/
http://ibus.stuge.se/IBus_Messages

Compile:
gcc -o bmw-ibus-daemon -Wall bmw-ibus.c

Usage: 
./bmw-ibus-daemon <options>-d serial device name (Mandatory)
-h hijack mode. FM/TAPE/AUX
-v video input switch. CTS/RTS/GPIO
-t tracelevel mask. TRACE_FUNCTION=1<<0, TRACE_IBUS=1<<2 etc..
-f trace file

example: ./bmw-ibus-daemon -d /dev/ttyUSB0 -h AUX -v CTS -t 15 -f ~/tracefile.log 


I have tested this with old Resler IBUS adapter but it should work also with
new USB adapter. See more info about Resler IBUS adapter from 
http://www.reslers.de/IBUS/index.html
