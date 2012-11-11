/**
 *   BMW IBus Daemon reads BMW IBus data through serial port. It detects
 *   BMW board monitor(at least BM53) unit and steering wheel button presses
 *   from IBus data, maps them to key events and injects them to system event
 *   queue via uinput.
 *
 *   It also can be configured to inject key events only in certain state like
 *   TAPE or AUX which can be useful if you want to hijack for example TAPE
 *   mode for other use.
 *
 *   It can be configured to control video input pin(reverse cam) via CTS,RTS
 *
 *   IBUS device,message and data codes are mostly found from following web sites:
 *   http://autos.groups.yahoo.com/group/HackTheIBus/
 *   http://ibus.stuge.se/IBus_Messages
 *
 *   Copyright (C) 2012 Kari Suvanto karis79@gmail.com
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <error.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <linux/uinput.h>


/**
 * TODO
 * 1.1-6 buttons change volume in aux, do nothing in aux, can be used in tape
 * 2.EStateCDChanger handle this also
 * 3.Language support for other than english?
 *
 */


/******************************************************************************
 * IBUS constants
 *****************************************************************************/
/* IBUS message has the following bytes defined:
 * 1. Sender
 * 2. Message length after this byte.
 * 3. Receiver
 * 4. Message
 * 5. data (0-252 bytes)
 * 6. checksum
 */
const int EPosSender = 0;
const int EPosLength = 1;
const int EPosReceiver = 2;
const int EPosMessage = 3;
const int EPosDataStart = 4;

const unsigned char ESenderAndLengthLength = 2;
/* minimum message does not have data at all so sender,length,receiver,message and checksum = 5 bytes*/
const unsigned char EMinimumMessageLength = 5;
/* As length can be 0xFF that makes the maximum possible message length to be 0xFF + 2 = 0x101 or 257 bytes */
const unsigned int EMaximumMessageLength = 257;

/*devices*/
const unsigned char GM = 0x00; /*Body module*/
const unsigned char SHD = 0x08; /*Sunroof Control*/
const unsigned char CDC = 0x18; /*CD Changer*/
const unsigned char FUH = 0x28; /*Radio controlled clock*/
const unsigned char CCM = 0x30; /*Check control module*/
const unsigned char GT = 0x3B; /*Graphics driver (in navigation system)*/
const unsigned char DIA = 0x3F; /*Diagnostic*/
const unsigned char FBZV = 0x40; /*Remote control central locking*/
const unsigned char GTF = 0x43; /*Graphics driver for rear screen (in navigation system)*/
const unsigned char EWS = 0x44; /*Immobiliser*/
const unsigned char CID = 0x46; /*Central information display (flip-up LCD screen)*/
const unsigned char MFL = 0x50; /*Multi function steering wheel*/
const unsigned char MM0 = 0x51; /*Mirror memory*/
const unsigned char IHK = 0x5B; /*Integrated heating and air conditioning*/
const unsigned char PDC = 0x60; /*Park distance control*/
const unsigned char ONL = 0x67; /*unknown*/
const unsigned char RAD = 0x68; /*Radio*/
const unsigned char DSP = 0x6A; /*Digital signal processing audio amplifier*/
const unsigned char SM0 = 0x72; /*Seat memory*/
const unsigned char SDRS = 0x73; /*Sirius Radio*/
const unsigned char CDCD = 0x76; /*CD changer, DIN size.*/
const unsigned char NAVE = 0x7F; /*Navigation (Europe)*/
const unsigned char IKE = 0x80; /*Instrument cluster electronics*/
const unsigned char MM1 = 0x9B; /*Mirror memory*/
const unsigned char MM2 = 0x9C; /*Mirror memory*/
const unsigned char FMID = 0xA0; /*Rear multi-info-display*/
const unsigned char ABM = 0xA4; /*Air bag module*/
const unsigned char KAM = 0xA8; /*unknown*/
const unsigned char ASP = 0xAC; /*unknown*/
const unsigned char SES = 0xB0; /*Speed recognition system*/
const unsigned char NAVJ = 0xBB; /*Navigation (Japan)*/
const unsigned char GLO = 0xBF; /*Global, broadcast address*/
const unsigned char MID = 0xC0; /*Multi-info display*/
const unsigned char TEL = 0xC8; /*Telephone*/
const unsigned char TCU = 0xCA; /*unknown (BMW Assist?)*/
const unsigned char LCM = 0xD0; /*Light control module*/
const unsigned char GTHL = 0xDA; /*unknown*/
const unsigned char IRIS = 0xE0; /*Integrated radio information system*/
const unsigned char ANZV = 0xE7; /*Front display*/
const unsigned char RLS = 0xE8; /*Rain/Light Sensor*/
const unsigned char TV = 0xED; /*Television*/
const unsigned char BMBT = 0xF0; /*On-board monitor operating part*/
const unsigned char CSU = 0xF5; /*unknown*/
const unsigned char LOC = 0xFF; /*Local*/

const char *IBUSDevices[] = {
    "Body module",
    "0x01",
    "0x02",
    "0x03",
    "0x04",
    "0x05",
    "0x06",
    "0x07",
    "Sunroof Control",
    "0x09",
    "0x0A",
    "0x0B",
    "0x0C",
    "0x0D",
    "0x0E",
    "0x0F",
    "0x10",
    "0x11",
    "0x12",
    "0x13",
    "0x14",
    "0x15",
    "0x16",
    "0x17",
    "CD Changer",
    "0x19",
    "0x1A",
    "0x1B",
    "0x1C",
    "0x1D",
    "0x1E",
    "0x1F",
    "0x20",
    "0x21",
    "0x22",
    "0x23",
    "0x24",
    "0x25",
    "0x26",
    "0x27",
    "Radio controlled clock",
    "0x29",
    "0x2A",
    "0x2B",
    "0x2C",
    "0x2D",
    "0x2E",
    "0x2F",
    "Check control module",
    "0x31",
    "0x32",
    "0x33",
    "0x34",
    "0x35",
    "0x36",
    "0x37",
    "0x38",
    "0x39",
    "0x3A",
    "Graphics driver",
    "0x3C",
    "0x3D",
    "0x3E",
    "Diagnostic",
    "Remote control central locking",
    "0x41",
    "0x42",
    "Graphics driver for rear screen",
    "Immobiliser",
    "0x45",
    "Central information display",
    "0x47",
    "0x48",
    "0x49",
    "0x4A",
    "0x4B",
    "0x4C",
    "0x4D",
    "0x4E",
    "0x4F",
    "Multi function steering wheel",
    "Mirror memory",
    "0x52",
    "0x53",
    "0x54",
    "0x55",
    "0x56",
    "0x57",
    "0x58",
    "0x59",
    "0x5A",
    "Integrated heating and air conditioning",
    "0x5C",
    "0x5D",
    "0x5E",
    "0x5F",
    "Park distance control",
    "0x61",
    "0x62",
    "0x63",
    "0x64",
    "0x65",
    "0x66",
    "0x67",
    "Radio",
    "0x69",
    "Digital signal processing audio amplifier",
    "0x6B",
    "0x6C",
    "0x6D",
    "0x6E",
    "0x6F",
    "0x70",
    "0x71",
    "Seat memory",
    "Sirius Radio",
    "0x74",
    "0x75",
    "CD changer, DIN size",
    "0x77",
    "0x78",
    "0x79",
    "0x7A",
    "0x7B",
    "0x7C",
    "0x7D",
    "0x7E",
    "Navigation",
    "Instrument cluster electronics",
    "0x81",
    "0x82",
    "0x83",
    "0x84",
    "0x85",
    "0x86",
    "0x87",
    "0x88",
    "0x89",
    "0x8A",
    "0x8B",
    "0x8C",
    "0x8D",
    "0x8E",
    "0x8F",
    "0x90",
    "0x91",
    "0x92",
    "0x93",
    "0x94",
    "0x95",
    "0x96",
    "0x97",
    "0x98",
    "0x99",
    "0x9A",
    "Mirror memory",
    "Mirror memory",
    "0x9D",
    "0x9E",
    "0x9F",
    "Rear multi-info-display",
    "0xA1",
    "0xA2",
    "0xA3",
    "Air bag module",
    "0xA5",
    "0xA6",
    "0xA7",
    "0xA8",
    "0xA9",
    "0xAA",
    "0xAB",
    "0xAC",
    "0xAD",
    "0xAE",
    "0xAF",
    "Speed recognition system",
    "0xB1",
    "0xB2",
    "0xB3",
    "0xB4",
    "0xB5",
    "0xB6",
    "0xB7",
    "0xB8",
    "0xB9",
    "0xBA",
    "Navigation",
    "0xBC",
    "0xBD",
    "0xBE",
    "Global, broadcast address",
    "Multi-info display",
    "0xC1",
    "0xC2",
    "0xC3",
    "0xC4",
    "0xC5",
    "0xC6",
    "0xC7",
    "Telephone",
    "0xC9",
    "0xCA",
    "0xCB",
    "0xCC",
    "0xCD",
    "0xCE",
    "0xCF",
    "Light control module",
    "0xD1",
    "0xD2",
    "0xD3",
    "0xD4",
    "0xD5",
    "0xD6",
    "0xD7",
    "0xD8",
    "0xD9",
    "0xDA",
    "0xDB",
    "0xDC",
    "0xDD",
    "0xDE",
    "0xDF",
    "Integrated radio information system",
    "0xE1",
    "0xE2",
    "0xE3",
    "0xE4",
    "0xE5",
    "0xE6",
    "Front display",
    "Rain/Light Sensor",
    "0xE9",
    "0xEA",
    "0xEB",
    "0xEC",
    "Television",
    "0xEE",
    "0xEF",
    "On-board monitor operating part",
    "0xF1",
    "0xF2",
    "0xF3",
    "0xF4",
    "0xF5",
    "0xF6",
    "0xF7",
    "0xF8",
    "0xF9",
    "0xFA",
    "0xFB",
    "0xFC",
    "0xFD",
    "0xFE",
    "Local"
    };



/*Messages*/
const unsigned char DSREQ = 0x01;/*"Device status request"*/
const unsigned char DSRED = 0x02;/*"Device status ready"*/
const unsigned char BSREQ = 0x03;/*"Bus status request"*/
const unsigned char BS = 0x04;/*"Bus status"*/
const unsigned char DRM = 0x06;/*"DIAG read memory"*/
const unsigned char DWM = 0x07;/*"DIAG write memory"*/
const unsigned char DRCD = 0x08;/*"DIAG read coding data"*/
const unsigned char DWCD = 0x09;/*"DIAG write coding data"*/
const unsigned char VC = 0x0C;/*"Vehicle control"*/

const unsigned char ISREQ = 0x10;/*"Ignition status request"*/
const unsigned char IS = 0x11;/*"Ignition status"*/
const unsigned char ISSREQ = 0x12;/*"IKE sensor status request"*/
const unsigned char ISS = 0x13;/*"IKE sensor status"*/
const unsigned char CCSREQ = 0x14;/*"Country coding status request"*/
const unsigned char CCS = 0x15;/*"Country coding status"*/
const unsigned char OREQ = 0x16;/*"Odometer request"*/
const unsigned char O = 0x17;/*"Odometer"*/
const unsigned char SR = 0x18;/*"Speed/RPM"*/
const unsigned char T = 0x19;/*"Temperature"*/
const unsigned char ITDG = 0x1A;/*"IKE text display/Gong"*/
const unsigned char ITS = 0x1B;/*"IKE text status"*/
const unsigned char G = 0x1C;/*"Gong"*/
const unsigned char TREQ = 0x1D;/*"Temperature request"*/
const unsigned char UTAD = 0x1F;/*"UTC time and date"*/

const unsigned char MT = 0x21; /*Radio Short cuts*/
const unsigned char TDC = 0x22; /*Text display confirmation*/
const unsigned char UMID = 0x23;/*"Display Text"*/
const unsigned char UANZV = 0x24;/*"Update ANZV"*/
const unsigned char OBCSU = 0x2A;/*"On-Board Computer State Update"*/
const unsigned char TI = 0x2b; /*Telephone indicators*/

const unsigned char MFLB = 0x32;/*"MFL buttons"*/
const unsigned char DSPEB = 0x34;/*"DSP Equalizer Button"*/
const unsigned char CDSREQ = 0x38;/*"CD status request"*/
const unsigned char CDS = 0x39;/*"CD status"*/
const unsigned char MFLB2 = 0x3B;/*"MFL buttons"*/
const unsigned char SDRSSREQ = 0x3D;/*"SDRS status request"*/
const unsigned char SDRSS = 0x3E;/*"SDRS status"*/

const unsigned char SOBCD = 0x40;/*"Set On-Board Computer Data"*/
const unsigned char OBCDR = 0x41;/*"On-Board Computer Data Request"*/
const unsigned char LCDC = 0x46;/* LCD Clear*/
const unsigned char BMBTB0 = 0x47;/*"BMBT buttons"*/
const unsigned char BMBTB1 = 0x48;/*"BMBT buttons"*/
const unsigned char KNOB = 0x49;/*"KNOB button"*/ /*this is for right knob turn, pressing know is BMBTB1 and ButtonMenuKnob*/
const unsigned char CC = 0x4a;/*Cassette control*/
const unsigned char CS = 0x4b;/*"Cassette Status"*/
const unsigned char RGBC = 0x4F;/*"RGB Control"*/

const unsigned char VDREQ = 0x53;/*"Vehicle data request"*/
const unsigned char VDS = 0x54;/*"Vehicle data status"*/
const unsigned char LSREQ = 0x5A;/*"Lamp status request"*/
const unsigned char LS = 0x5B;/*"Lamp Status"*/
const unsigned char ICLS = 0x5C;/*"Instrument cluster lighting status"*/

const unsigned char RSSREQ = 0x71;/*"Rain sensor status request"*/
const unsigned char RKB = 0x72;/*"Remote Key buttons"*/
const unsigned char EWSKS = 0x74;/*"EWS key status"*/
const unsigned char DWSREQ = 0x79;/*"Doors/windows status request"*/
const unsigned char DWS = 0x7A;/*"Doors/windows status"*/
const unsigned char SHDS = 0x7C;/*"SHD status"*/

const unsigned char RCL = 0xD4; /*RDS channel list*/

const unsigned char DD = 0xA0;/*"DIAG data"*/
const unsigned char CPAT = 0xA2;/*"Current position and time*/
const unsigned char CL = 0xA4;/*Current location, always 23 bytes, data has 2 byte order number and then ascii: 00 01 4F 55 4C 55 00 == 1st packet, OULU\0*/
const unsigned char ST = 0xa5; /*Screen text*/
const unsigned char TMCSREQ = 0xA7;/*"TMC status request"*/
const unsigned char NC = 0xAA;/*"Navigation Control"*/

const char *IBUSMessages[] = {
    "0x00",
    "Device status request",
    "Device status ready",
    "Bus status request",
    "Bus status",
    "0x05",
    "DIAG read memory",
    "DIAG write memory",
    "DIAG read coding data",
    "DIAG write coding data",
    "0x0A",
    "0x0B",
    "Vehicle control",
    "0x0D",
    "0x0E",
    "0x0F",
    "Ignition status request",
    "Ignition status",
    "IKE sensor status request",
    "IKE sensor status",
    "Country coding status request",
    "Country coding status",
    "Odometer request",
    "Odometer",
    "Speed/RPM",
    "Temperature",
    "IKE text display/Gong",
    "IKE text status",
    "Gong",
    "Temperature request",
    "0x1E",
    "UTC time and date",
    "0x20",
    "Radio Short cuts",
    "Text display confirmation",
    "Display Text",
    "Update ANZV",
    "0x25",
    "0x26",
    "0x27",
    "0x28",
    "0x29",
    "On-Board Computer State Update",
    "Telephone indicators",
    "0x2C",
    "0x2D",
    "0x2E",
    "0x2F",
    "0x30",
    "0x31",
    "MFL buttons",
    "0x33",
    "DSP Equalizer Button",
    "0x35",
    "0x36",
    "0x37",
    "CD status request",
    "CD status",
    "0x3A",
    "MFL buttons 2",
    "0x3C",
    "SDRS status request",
    "SDRS status",
    "0x3F",
    "Set On-Board Computer Data",
    "On-Board Computer Data Request",
    "0x42",
    "0x43",
    "0x44",
    "0x45",
    "LCD Clear",
    "BMBT buttons",
    "BMBT buttons",
    "KNOB button",
    "Cassette control",
    "Cassette status",
    "0x4C",
    "0x4D",
    "0x4E",
    "RGB Control",
    "0x50",
    "0x51",
    "0x52",
    "Vehicle data request",
    "Vehicle data status",
    "0x55",
    "0x56",
    "0x57",
    "0x58",
    "0x59",
    "Lamp status request",
    "Lamp status",
    "Instrument cluster lighting status",
    "0x5D",
    "0x5E",
    "0x5F",
    "0x60",
    "0x61",
    "0x62",
    "0x63",
    "0x64",
    "0x65",
    "0x66",
    "0x67",
    "0x68",
    "0x69",
    "0x6A",
    "0x6B",
    "0x6C",
    "0x6D",
    "0x6E",
    "0x6F",
    "0x70",
    "Rain sensor status request",
    "Remote Key buttons",
    "0x73",
    "EWS key status",
    "0x75",
    "0x76",
    "0x77",
    "0x78",
    "Doors/windows status request",
    "Doors/windows status",
    "0x7B",
    "SHD status",
    "0x7D",
    "0x7E",
    "0x7F",
    "0x80",
    "0x81",
    "0x82",
    "0x83",
    "0x84",
    "0x85",
    "0x86",
    "0x87",
    "0x88",
    "0x89",
    "0x8A",
    "0x8B",
    "0x8C",
    "0x8D",
    "0x8E",
    "0x8F",
    "0x90",
    "0x91",
    "0x92",
    "0x93",
    "0x94",
    "0x95",
    "0x96",
    "0x97",
    "0x98",
    "0x99",
    "0x9A",
    "0x9B",
    "0x9C",
    "0x9D",
    "0x9E",
    "0x9F",
    "DIAG data",
    "0xA1",
    "Current position and time",
    "0xA3",
    "Current location",
    "Screen text",
    "0xA6",
    "TMC status request",
    "0xA8",
    "0xA9",
    "Navigation Control",
    "0xAB",
    "0xAC",
    "0xAD",
    "0xAE",
    "0xAF",
    "0xB0",
    "0xB1",
    "0xB2",
    "0xB3",
    "0xB4",
    "0xB5",
    "0xB6",
    "0xB7",
    "0xB8",
    "0xB9",
    "0xBA",
    "0xBB",
    "0xBC",
    "0xBD",
    "0xBE",
    "0xBF",
    "0xC0",
    "0xC1",
    "0xC2",
    "0xC3",
    "0xC4",
    "0xC5",
    "0xC6",
    "0xC7",
    "0xC8",
    "0xC9",
    "0xCA",
    "0xCB",
    "0xCC",
    "0xCD",
    "0xCE",
    "0xCF",
    "0xD0",
    "0xD1",
    "0xD2",
    "0xD3",
    "RDS channel list",
    "0xD5",
    "0xD6",
    "0xD7",
    "0xD8",
    "0xD9",
    "0xDA",
    "0xDB",
    "0xDC",
    "0xDD",
    "0xDE",
    "0xDF",
    "0xE0",
    "0xE1",
    "0xE2",
    "0xE3",
    "0xE4",
    "0xE5",
    "0xE6",
    "0xE7",
    "0xE8",
    "0xE9",
    "0xEA",
    "0xEB",
    "0xEC",
    "0xED",
    "0xEE",
    "0xEF",
    "0xF0",
    "0xF1",
    "0xF2",
    "0xF3",
    "0xF4",
    "0xF5",
    "0xF6",
    "0xF7",
    "0xF8",
    "0xF9",
    "0xFA",
    "0xFB",
    "0xFC",
    "0xFD",
    "0xFE",
    "0xFF"
    };

/*Data*/

/*these are added to the BMBT key event. for example pressing long for Button2 is Button2+ButtonLongPress=0x41*/
const unsigned char ButtonPress = 0x00;
const unsigned char ButtonLongPress = 0x40;
const unsigned char ButtonRelease = 0x80;

/*data for button codes from BMBT to RAD in BMBTB1 message*/
const unsigned char ButtonArrowRight = 0x00;
const unsigned char Button2 = 0x01;
const unsigned char Button4 = 0x02;
const unsigned char Button6 = 0x03;
const unsigned char ButtonTone = 0x04;
const unsigned char ButtonMenuKnob = 0x05; /*sent to GT*/
const unsigned char ButtonRadioPower = 0x06;
const unsigned char ButtonClock = 0x07;  /*sent to LOC*/
const unsigned char ButtonTelephone = 0x08;  /*sent to LOC*/
const unsigned char ButtonArrowLeft = 0x10;
const unsigned char Button1 = 0x11;
const unsigned char Button3 = 0x12;
const unsigned char Button5 = 0x13;
const unsigned char ButtonReversePlay = 0x14;/*small arrows next to clock button*/
const unsigned char ButtonAM = 0x21;
const unsigned char ButtonRDS = 0x22;
const unsigned char ButtonMode = 0x23;
const unsigned char ButtonEject = 0x24;
const unsigned char ButtonSwitch = 0x30;/*icon next to Mode button*/
const unsigned char ButtonFM = 0x31;
const unsigned char ButtonTP = 0x32;
const unsigned char ButtonDolby = 0x33;
const unsigned char ButtonMenu = 0x34; /*sent to LOC*/


/*data for button codes from BMBT to GT in KNOB message*/
const unsigned char ButtonMenuKnobClockwiseMask = 0x80; /*0x81 once, 0x82 twice etc*/
const unsigned char ButtonMenuKnobCounterClockwiseMask = 0x00;

/*data for button codes from BMBT to LOCAL in BMBTB0 message*/
const unsigned char ButtonSelectInTapeMode = 0x0f; /*second byte of data*/
const unsigned char ButtonUnknownInTapeMode = 0x38; /*second byte of data*/

/*to radio, lenght 4, message MFLB, high 4 bits of data is how many steps,low 4 bit is direction-> 1=up, 0=down*/
const unsigned char MFLButtonVolumeUp = 0x01;
const unsigned char MFLButtonVolumeDown = 0x00;

/*MFLB2*/
const unsigned char MFL2ButtonPress = 0x00;
const unsigned char MFL2ButtonRelease = 0x20;

const unsigned char MFL2ButtonChannelUp = 0x01;
const unsigned char MFL2ButtonChannelDown = 0x08;

const unsigned char MFL2AnswerButton = 0x80;


/*not real data codes, there are meant be used with headunit_buttons*/
const unsigned char MenuKnobClockwiseMask = 0x35;
const unsigned char MenuKnobCounterClockwiseMask = 0x36;
const unsigned char SelectInTapeMode = 0x37;
const unsigned char MFL2ChannelUp = 0x38;
const unsigned char MFL2ChannelDown = 0x39;

struct ibus_buttons {
    const char * name;
    const uint16_t key_code;
};

/**
 * used for the buttons that changes the BM state.
 * these are not send via uinput
 */
#define RESERVED_BUTTON 0xFFFF
/**
 * This is the key mapping from BMW IBUS to Linux key codes
 *
 * Do not map buttons that changes the state like power, fm, mode
 */
const struct ibus_buttons headunit_buttons[] = {
    {  "ButtonArrowRight",  KEY_UP  },   		/*0x00*/
    {  "Button2",           KEY_BACKSPACE  },   /*0x01*/
    {  "Button4",           KEY_4  },       	/*0x02*/
    {  "Button6",           KEY_6  },       	/*0x03*/
    {  "ButtonTone",        RESERVED_BUTTON  },    	/*0x04*/ /*TONE can be used in tape mode but not in AUX mode*/
    {  "ButtonMenuKnob",    KEY_ENTER  },   	/*0x05*/ /*knob push*/
    {  "ButtonRadioPower",  RESERVED_BUTTON  }, 	/*0x06*/ /*power button not passed forward*/
    {  "ButtonClock",       KEY_SETUP  },   	/*0x07*/
    {  "ButtonTelephone",   KEY_SETUP  },   	/*0x08*/
    {  "0x08",              KEY_UNKNOWN  }, 	/*0x09*/
    {  "0x09",              KEY_UNKNOWN  }, 	/*0x0A*/
    {  "0x0A",              KEY_UNKNOWN  }, 	/*0x0B*/
    {  "0x0B",              KEY_UNKNOWN  }, 	/*0x0C*/
    {  "0x0C",              KEY_UNKNOWN  }, 	/*0x0D*/
    {  "0x0D",              KEY_UNKNOWN  }, 	/*0x0E*/
    {  "0x0F",              KEY_UNKNOWN  }, 	/*0x0F*/
    {  "ButtonArrowLeft",   KEY_DOWN  },   		/*0x10*/
    {  "Button1",           KEY_MENU  },       	/*0x11*/
    {  "Button3",           KEY_SPACE  },       /*0x12*/
    {  "Button5",           KEY_5  },       	/*0x13*/
    {  "ButtonReversePlay", KEY_SETUP  },   	/*0x14*/
    {  "0x15",              KEY_UNKNOWN  }, 	/*0x15*/
    {  "0x16",              KEY_UNKNOWN  }, 	/*0x16*/
    {  "0x17",              KEY_UNKNOWN  }, 	/*0x17*/
    {  "0x18",              KEY_UNKNOWN  }, 	/*0x18*/
    {  "0x19",              KEY_UNKNOWN  }, 	/*0x19*/
    {  "0x1A",              KEY_UNKNOWN  }, 	/*0x1A*/
    {  "0x1B",              KEY_UNKNOWN  }, 	/*0x1B*/
    {  "0x1C",              KEY_UNKNOWN  }, 	/*0x1C*/
    {  "0x1D",              KEY_UNKNOWN  }, 	/*0x1D*/
    {  "0x1E",              KEY_UNKNOWN  }, 	/*0x1E*/
    {  "0x1F",              KEY_UNKNOWN  }, 	/*0x1F*/
    {  "0x20",              KEY_UNKNOWN  }, 	/*0x20*/
    {  "ButtonAM",          RESERVED_BUTTON  }, 	/*0x21*/
    {  "ButtonRDS",         RESERVED_BUTTON  }, 	/*0x22*/
    {  "ButtonMode",        RESERVED_BUTTON  }, 	/*0x23*/
    {  "ButtonEject",       RESERVED_BUTTON  }, 	/*0x24*/
    {  "0x25",              KEY_UNKNOWN  }, 	/*0x25*/
    {  "0x26",              KEY_UNKNOWN  }, 	/*0x26*/
    {  "0x27",              KEY_UNKNOWN  }, 	/*0x27*/
    {  "0x28",              KEY_UNKNOWN  }, 	/*0x28*/
    {  "0x29",              KEY_UNKNOWN  }, 	/*0x29*/
    {  "0x2A",              KEY_UNKNOWN  }, 	/*0x2A*/
    {  "0x2B",              KEY_UNKNOWN  }, 	/*0x2B*/
    {  "0x2C",              KEY_UNKNOWN  }, 	/*0x2C*/
    {  "0x2D",              KEY_UNKNOWN  }, 	/*0x2D*/
    {  "0x2E",              KEY_UNKNOWN  }, 	/*0x2E*/
    {  "0x2F",              KEY_UNKNOWN  }, 	/*0x2F*/
    {  "ButtonSwitch",      RESERVED_BUTTON  }, 	/*0x30*/
    {  "ButtonFM",          RESERVED_BUTTON  }, 	/*0x31*/
    {  "ButtonTP",          RESERVED_BUTTON  }, 	/*0x32*/
    {  "ButtonDolby",       KEY_UNKNOWN  }, 	/*0x33*/
    {  "ButtonMenu",        RESERVED_BUTTON  }, 	/*0x34*/
    {  "ButtonMenuKnobClockwiseMask",           KEY_RIGHT  },    /*0x35*/
    {  "ButtonMenuKnobCounterClockwiseMask",    KEY_LEFT  },    /*0x36*/
    {  "ButtonSelectInTapeMode",                KEY_ESC },    /*0x37*/
    {  "MFL2ButtonChannelUp",                   KEY_UP  },    /*0x38*/
    {  "MFL2ButtonChannelDown",                 KEY_DOWN  }    /*0x39*/
};

enum EIbusState
    {
    EStateUnknown = 0,
    EStatePowerOff,
    EStateMenu, /*audio lines stay open to old state, only display shows menu*/
    EStateFM,
    EStateTAPE,
    EStateAUX,
    EStateCDChanger
    };

enum EVideoInputSwitch
    {
    ESwitchCTS = 0,
    ESwitchRTS,
    ESwitchGPIO,
    ESwitchUnknown
    };


/******************************************************************************
 * static variables
 *****************************************************************************/
static volatile int exit_request = 0;

static int uinput_device_fd;
static unsigned char send_key_events = 0;

static int ibus_device_fd;
static unsigned char ibus_data[257*8]; /*EMaximumMessageLength*/
static unsigned int ibus_data_max_length = 257*8;
static unsigned int ibus_data_index;

static enum EIbusState ibus_state = EStateUnknown;
static enum EIbusState IbusHijackState = EStateUnknown;
static enum EVideoInputSwitch VideoInputSwitch = ESwitchUnknown;

static unsigned int trace_level = 0;

static FILE* stdout_fp = 0;

/******************************************************************************
 * trace macros
 *****************************************************************************/
#define TRACE_FUNCTION 1<<0
#define TRACE_IBUS     1<<1
#define TRACE_INPUT    1<<2
#define TRACE_STATE    1<<3
#define TRACE_ALL      (TRACE_FUNCTION|TRACE_IBUS|TRACE_INPUT|TRACE_STATE)

#define CHECK_TRACELEVEL(level) (level&trace_level)

#define TRACE_WARGS(debug_level, format, ...) \
({ \
    if(CHECK_TRACELEVEL(debug_level)) { \
        struct timeval now; \
        gettimeofday(&now, 0); \
        printf("%ld.%06ld: " format ,(long)now.tv_sec, (long)now.tv_usec, __VA_ARGS__); \
    } \
})

#define TRACE(debug_level, format) \
({ \
    if(CHECK_TRACELEVEL(debug_level)) { \
        struct timeval now; \
        gettimeofday(&now, 0); \
        printf("%ld.%06ld: " format ,(long)now.tv_sec, (long)now.tv_usec); \
    } \
})

#define TRACE_ERROR(format) \
({ \
	struct timeval now; \
	gettimeofday(&now, 0); \
	printf("%ld.%06ld: %s:%d ERROR=%d=%s: " format "\n",(long)now.tv_sec, (long)now.tv_usec, __FILE__,__LINE__, -errno, strerror(errno)); \
	if(stdout_fp) fflush(stdout_fp); \
})

#define TRACE_HEX(debug_level,message, data, length) \
({ \
    unsigned int idx = 0; \
    if(CHECK_TRACELEVEL(debug_level)) { \
        TRACE(debug_level, message); \
        do{ \
            printf("%02x",data[idx]); \
            idx++; \
        } \
        while(idx < length); \
        printf("\n"); \
    } \
})

#define TRACE_ENTRY(debug_level) TRACE_WARGS(debug_level, "++ %s\n",__func__);
#define TRACE_ENTRY_WARGS(debug_level,format, ...) TRACE_WARGS(debug_level, "++ %s " format,__func__,__VA_ARGS__);
#define TRACE_EXIT(debug_level) TRACE_WARGS(debug_level, "-- %s\n",__func__);
#define TRACE_EXIT_WARGS(debug_level,format, ...) TRACE_WARGS(debug_level, "-- %s " format,__func__,__VA_ARGS__);

/******************************************************************************
 * signal functions
 *****************************************************************************/
static void signal_handler(int sig)
{
    TRACE_ENTRY_WARGS(TRACE_FUNCTION, "%s got %d\n",__func__,sig);
	exit_request = 1;
	TRACE_EXIT(TRACE_FUNCTION);
}

/******************************************************************************
 * uinput functions
 *****************************************************************************/
static int uinput_create()
{
	struct uinput_user_dev dev;
	int fd;
	unsigned int i;
	TRACE_ENTRY((TRACE_INPUT|TRACE_FUNCTION));

	fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if (fd < 0) {
		fd = open("/dev/input/uinput", O_WRONLY | O_NONBLOCK);
		if (fd < 0) {
			fd = open("/dev/misc/uinput", O_WRONLY | O_NONBLOCK);
			if (fd < 0) {
				TRACE_ERROR("Can't open input device");
				goto err;
			}
		}
	}

	memset(&dev, 0, sizeof(dev));
	snprintf(dev.name, UINPUT_MAX_NAME_SIZE, "BMW IBUS");

	dev.id.bustype = BUS_RS232;
	dev.id.vendor  = 0x0000;
	dev.id.product = 0x0000;
	dev.id.version = 0x0100;

	if (write(fd, &dev, sizeof(dev)) < 0) {
		TRACE_ERROR("Can't write device information");
		goto close;
	}

	if(ioctl(fd, UI_SET_EVBIT, EV_KEY) < 0){
		TRACE_ERROR("Can't set event bit");
		goto close;
	}

	for(i=0; i < sizeof(headunit_buttons)/sizeof(struct ibus_buttons); i++){
        if(headunit_buttons[i].key_code!=KEY_UNKNOWN) {
            if(ioctl(fd, UI_SET_KEYBIT, headunit_buttons[i].key_code) < 0){
            	TRACE_ERROR("Can't set key bit");
                goto close;
            }
        }
	}

	if (ioctl(fd, UI_DEV_CREATE, NULL) < 0) {
		TRACE_ERROR("Can't create uinput device");
		goto close;
	}

    TRACE_EXIT((TRACE_INPUT|TRACE_FUNCTION));
	return fd;

close:
	close(fd);
err:
    TRACE_EXIT_WARGS((TRACE_INPUT|TRACE_FUNCTION)," error %d\n",-errno);
    return -errno;
}

static void uinput_close()
{
    TRACE_ENTRY((TRACE_INPUT|TRACE_FUNCTION));
    if (uinput_device_fd >= 0) {
		ioctl(uinput_device_fd, UI_DEV_DESTROY);
		close(uinput_device_fd);
		uinput_device_fd = -1;
	}
	TRACE_EXIT((TRACE_INPUT|TRACE_FUNCTION));
}

static int send_key_event(uint16_t key, uint16_t value)
{
    struct input_event key_event;

    TRACE_ENTRY_WARGS((TRACE_INPUT|TRACE_FUNCTION), "key event %d, value %d\n",key,value);

    /*send key press event*/
    memset(&key_event, 0, sizeof(key_event));
	key_event.type	= EV_KEY;
	key_event.code	= key;
	key_event.value	= value;
	if(write(uinput_device_fd, &key_event, sizeof(key_event)) < 0){
		TRACE_ERROR("Can't write key event");
		goto err;
	}

    /*send sync event*/
    memset(&key_event, 0, sizeof(key_event));
	key_event.type	= EV_SYN;
	key_event.code	= SYN_REPORT;
	key_event.value	= 0;
	if(write(uinput_device_fd, &key_event, sizeof(key_event)) < 0){
		TRACE_ERROR("Can't write syn event");
		goto err;
	}

	TRACE_EXIT((TRACE_INPUT|TRACE_FUNCTION));
	return 0;
err:
    TRACE_EXIT_WARGS((TRACE_INPUT|TRACE_FUNCTION),"error %d",-errno);
    return -errno;
}

static void handle_ibus_button(unsigned char button, unsigned char released,unsigned char longPress)
{
    TRACE_ENTRY_WARGS((TRACE_INPUT|TRACE_FUNCTION), "button %d, released %d, longPress %d\n",button,released,longPress);

    if(send_key_events){
    	/**
    	 * todo: long press/repeat not supported at the moment
    	 */

		if( headunit_buttons[button].key_code!=KEY_UNKNOWN ||
		    headunit_buttons[button].key_code!=RESERVED_BUTTON){
			if(send_key_event(headunit_buttons[button].key_code,released?0:1) < 0){
				TRACE_ERROR("Can't set key event");
			}
		}
    }

    TRACE_EXIT((TRACE_INPUT|TRACE_FUNCTION));
}


/******************************************************************************
 * IBUS functions
 *****************************************************************************/
/*
 * Set/Disable line.
 */
static int set_line(int line,int enable)
{
    int status;
    TRACE_ENTRY_WARGS(TRACE_STATE, "line=%x,enable=%d\n",line,enable);

    if(line < TIOCM_LE || line > TIOCM_DSR){
    	printf("invalid line %d",line);
    	errno = EINVAL;
    	goto err;
    }

    if(ioctl(ibus_device_fd, TIOCMGET, &status) < 0){
    	TRACE_ERROR("Can't get TIOCM");
        goto err;
    }

    TRACE_WARGS(TRACE_STATE, "old state %x\n",status);
    /* check if already enabled/disabled before altering */
    if(enable){
    	if((status&line)==line){
    		goto exit;
    	}
        status |= line;
    }
    else {
    	if(!((status&line)==line)){
			goto exit;
		}
        status &= ~line;
    }

    if(ioctl(ibus_device_fd, TIOCMSET, &status) < 0){
    	TRACE_ERROR("Can't set TIOCM");
    	goto err;
    }

exit:
    TRACE_EXIT_WARGS(TRACE_STATE, "new state %x\n",status);
    return 0;
err:
	TRACE_EXIT_WARGS(TRACE_STATE, "error %d",-errno);
	return -errno;
}

static void enable_video_input(int enable)
{
	TRACE_ENTRY_WARGS(TRACE_STATE, "enable %d\n",enable);

	switch(VideoInputSwitch)
		{
		case ESwitchCTS:
			{
			set_line(TIOCM_CTS,enable);
			break;
			}
		case ESwitchRTS:
			{
			set_line(TIOCM_RTS,enable);
			break;
			}
		case ESwitchGPIO:
			{
			//TODO
			break;
			}
		case ESwitchUnknown:
		default:
			{
			//TODO
			break;
			}
		}

	TRACE_EXIT(TRACE_STATE);
}
/*
 * Change the IBUS state machine state. This controls when video output is
 * enabled and when buttons events are injected to the system queue
 */
static void ibus_change_state(enum EIbusState aNewState)
    {
    TRACE_ENTRY_WARGS(TRACE_STATE, "new state %d\n",aNewState);

    if(ibus_state==aNewState){
    	TRACE_WARGS(TRACE_STATE, "state already %d -> do nothing\n",aNewState);
    	goto exit;
    }

    ibus_state = aNewState;

    if(ibus_state==IbusHijackState && IbusHijackState!=EStateUnknown){
    	send_key_events = 1;
    	enable_video_input(1);
    }
    else{
    	send_key_events = 0;
    	enable_video_input(0);
    }

    if(ibus_state==EStateAUX)
        TRACE(TRACE_STATE,"IBUS STATE changed to AUX\n");
    else if(ibus_state==EStateFM)
        TRACE(TRACE_STATE,"IBUS STATE changed to FM\n");
    else if(ibus_state==EStateMenu)
        TRACE(TRACE_STATE,"IBUS STATE changed to MENU\n");
    else if(ibus_state==EStatePowerOff)
        TRACE(TRACE_STATE,"IBUS STATE changed to POWEROFF\n");
    else if(ibus_state==EStateTAPE)
        TRACE(TRACE_STATE,"IBUS STATE changed to TAPE\n");
    else if(ibus_state==EStateUnknown)
        TRACE(TRACE_STATE,"IBUS STATE changed to UNKNOWN\n");

exit:
    TRACE_EXIT(TRACE_STATE);
    }


static unsigned char calc_ibus_checksum(unsigned int checksum_index)
    {
    unsigned char i,checksum;
    for(i = 0,checksum=0; i < checksum_index; i++)
        {
        checksum = checksum ^ ibus_data[i];
        }
    return checksum;
    }

static inline unsigned char get_message_length()
{
    return (ibus_data[EPosLength]+ESenderAndLengthLength);
}

static inline unsigned char get_data_length()
    {
    unsigned char len = get_message_length();
    if(len <= EMinimumMessageLength)
        len = 0; /*set it to 0 just in case*/
    else
        len -= EMinimumMessageLength;
    return len;
    }

static inline unsigned char get_sender()
    {
    return ibus_data[EPosSender];
    }
static inline unsigned char get_receiver()
    {
    return ibus_data[EPosReceiver];
    }
static inline unsigned char get_message()
    {
    return ibus_data[EPosMessage];
    }

/* returns the nth data byte */
static inline unsigned char get_data_byte(unsigned int idx)
{
    return ibus_data[EPosDataStart+idx];
}

static inline unsigned int data_contains(const char* tag)
{
    return strstr((char*)&ibus_data[EPosDataStart],tag)?1:0;
}

static void print_ibus_message()
    {
    int addData = 1;
    unsigned int idx = 0;
    unsigned char data = 0;
    unsigned char dataLen = get_data_length();
    unsigned int curr_mes_len = get_message_length();

    TRACE(TRACE_IBUS,"");

    /*print message in hex. print data without spaces and send,len,res,mes and cs with spaces*/
    /*F0 04 53 23 ABBADABBAAAA C8*/

    do{
        if(idx < 4 || idx == curr_mes_len-1)
            printf(" %02x",ibus_data[idx]);
        else
            printf("%02x",ibus_data[idx]);
        idx++;
    }
    while(idx < curr_mes_len);

    printf(" = %s",IBUSDevices[get_sender()]);
    printf(" SENT ");

    if(get_message()==BMBTB1 && get_data_length()==1)
        {
    	data = get_data_byte(0);
        int longPress = 0;
		int release = 0;

		if(data & ButtonLongPress)
			{
			data &= ~ButtonLongPress;
			longPress = 1;
			}
		else if(data & ButtonRelease)
			{
			data &= ~ButtonRelease;
			release = 1;
			}

		printf("button %s",headunit_buttons[data].name);

		if(release)
			printf(" released");
		else if(longPress)
			printf(" pressed long");
		else
			printf(" pressed");
        addData = 0;
        }
    else if(get_message()==KNOB && get_data_length()==1)
        {
    	data = get_data_byte(0);
        if(data & ButtonMenuKnobClockwiseMask)
			{
			printf("Menu knob turned clockwise ");
			data &= ~ButtonMenuKnobClockwiseMask;
			}
		else
			{
			printf("Menu knob turned counter clockwise ");
			}

		printf("%d time(s)",data);
        addData = 0;
        }
    else
        printf("%s",IBUSMessages[get_message()]);

    printf(" TO ");
    printf("%s",IBUSDevices[get_receiver()]);

    idx = 0;
    if(addData && dataLen > 0){
        printf(" DATA:");
        if(get_sender()==RAD && get_receiver()==BMBT && (get_message()==CC || get_message()==CS) ) {
            do{
                printf(" 0x%02x",ibus_data[EPosDataStart+idx]);
                idx++;
            } while(idx < dataLen);
        }
        else{
            do{
                if(ibus_data[EPosDataStart+idx] < 0x20 || ibus_data[EPosDataStart+idx] > 0x7F)
                    printf("0x%02x ",ibus_data[EPosDataStart+idx]);
                else
                    printf("%c",ibus_data[EPosDataStart+idx]);
                idx++;
            } while(idx < dataLen);
        }
    }
    printf("\n");
    }

static void handle_headunit_state()
{
    TRACE_ENTRY(TRACE_FUNCTION);

    if(get_sender()==RAD && get_receiver()==GT) {
        if(get_message()==UMID) {
            if(get_data_byte(0)==0x62 ) { /*layout RadioDisplay*/
                if(data_contains("AUX")) {
                    ibus_change_state(EStateAUX);
                } else if(data_contains("TAPE")) { /*TODO: TAPE state could be checked from mode button also so that display could be switched before TAPE is shown in screen*/
                    ibus_change_state(EStateTAPE);
                }
            }
        }else if(get_message()==ST){
            if(get_data_byte(0)==0x62 ) { /*layout RadioDisplay*/
                if(data_contains("RDS") || data_contains("FM") || data_contains("REG") || data_contains("MWA")) {
                    ibus_change_state(EStateFM);
                }
            }
        }else if(get_message()==LCDC) {
            if(get_data_length()==1) {
                switch(get_data_byte(0)) { /*menu brought foreground, state stays,*/
                    case 0x01: /*No Display Required*/
                    case 0x02: /*Radio Display Off*/
                        {
                        ibus_change_state(EStateMenu);
                        break;
                        }
                    default:
                        {
                        /*TRACE_WARGS(TRACE_IBUS,"LCD Clear, data: %02x\n",get_data_byte(0));*/
                        break;
                        }
                    }
            }
        }
    }

    TRACE_EXIT(TRACE_FUNCTION);
    return;
}

/*
 * Processes the IBus message. If message is invalid, its silently discarded and next message is read
 */
static void process_ibus_message()
{
    unsigned int checksum_index;
    unsigned char databyte;
    unsigned int cur_mes_len = 0;
    TRACE_ENTRY(TRACE_FUNCTION);

    do{
    	cur_mes_len = get_message_length();

		/* 1. Validate the IBUS message */
		if(ibus_data_index < EMinimumMessageLength ||
		   ibus_data_index < cur_mes_len) {
			TRACE_WARGS(TRACE_IBUS,"Invalid message length!! %d\n",ibus_data_index);
			goto err;
		}

		/* 2. Validate the IBUS message checksum */
		/*checksum is located at the last byte of message*/
		checksum_index = cur_mes_len - 1;
		if(calc_ibus_checksum(checksum_index) != ibus_data[checksum_index]){
			TRACE_WARGS(TRACE_IBUS,"Invalid checksum!! %x\n",ibus_data[checksum_index]);
			goto err;
		}

		/* 3. print valid message if trace enabled*/
		if(trace_level&TRACE_IBUS)
			print_ibus_message();

		/* 4. Handle the buttons messages */
		if(get_sender()==BMBT) {
			if(get_message()==BMBTB1) {
				databyte = get_data_byte(0);
				unsigned char longPress = 0;
				unsigned char released = 0;
				if(databyte & ButtonLongPress) {
					databyte &= ~ButtonLongPress;
					longPress = 1;
				}
				else if(databyte & ButtonRelease) {
					databyte &= ~ButtonRelease;
					released = 1;
				}

				if(databyte==ButtonRadioPower){
					ibus_change_state(EStatePowerOff);
				}

				handle_ibus_button(databyte,released,longPress);
			}
			else if(get_message()==BMBTB0) {
				/*button command for select is in second byte of data*/
				databyte = get_data_byte(1);

				unsigned char longPress = 0;
				unsigned char released = 0;
				if(databyte & ButtonLongPress) {
					databyte &= ~ButtonLongPress;
					longPress = 1;
				}
				else if(databyte & ButtonRelease) {
					databyte &= ~ButtonRelease;
					released = 1;
				}

				if(databyte == ButtonSelectInTapeMode) {
					handle_ibus_button(SelectInTapeMode,released,longPress);
				}
				else{
					printf("0x%02x, longPress %d, released %d\n",databyte,longPress,released);
				}
			}
			else if(get_message()==KNOB) {
				databyte = get_data_byte(0);
				int clockwise = 0;
				if(databyte & ButtonMenuKnobClockwiseMask) {
					databyte &= ~ButtonMenuKnobClockwiseMask;
					clockwise = 1;
				}

				/*databyte tells how many times need to send this command*/
				while(databyte) {
					send_key_event(headunit_buttons[clockwise?MenuKnobClockwiseMask:MenuKnobCounterClockwiseMask].key_code,1);
					send_key_event(headunit_buttons[clockwise?MenuKnobClockwiseMask:MenuKnobCounterClockwiseMask].key_code,0);
					databyte--;
				}
			}
			else if(get_message()==MFLB) {
				databyte = get_data_byte(0);
				/*TODO: to volume function*/
				/*volume*/
				unsigned char steps = (databyte&0xF0)>>4;
				if(databyte&0xF){ /*volume up*/
					printf("volue up %d steps\n",steps);
				} else { /*volume down*/
					printf("volue down %d steps\n",steps);
				}
			}
		}
		else if(get_sender()==MFL && get_receiver()==RAD) {
			databyte = get_data_byte(0);
			if(get_message()==MFLB){
				/*TODO: to volume function*/
				/*volume*/
				unsigned char steps = (databyte&0xF0)>>4;
				if(databyte&0xF){ /*volume up*/
					printf("volue up %d steps\n",steps);
				} else { /*volume down*/
					printf("volue down %d steps\n",steps);
				}
			}
			else if(get_message()==MFLB2){
				/*channel*/
				unsigned char released = 0;
				if(databyte & MFL2ButtonRelease) {
					databyte &= ~MFL2ButtonRelease;
					released = 1;
				}
				if(databyte & MFL2ButtonChannelUp){
					handle_ibus_button(MFL2ChannelUp,released,0);
				} else if(databyte & MFL2ButtonChannelDown){
					handle_ibus_button(MFL2ChannelDown,released,0);
				}

				/*TODO: handle answer buttons and other mfl buttons*/
			}

		}

		/* 5. Handle the state messages */
		/*handle state only if hijack state is given*/
		if(IbusHijackState != EStateUnknown)
			handle_headunit_state();

		/*move data in ibus_data so that first message at the beginning is cleared*/
		if(ibus_data_index >= cur_mes_len){
			/* Discard the first message in the buffer by
			 * 1. Moving data from the end to the start
			 * 2. Adjust ibus_Data_index
			 * 3. Clear end of the buffer */
			memmove(&ibus_data[0],&ibus_data[cur_mes_len],(ibus_data_index-cur_mes_len));
			ibus_data_index = ibus_data_index - cur_mes_len;
			memset(&ibus_data[ibus_data_index],0,cur_mes_len);
		}
    }while(ibus_data_index);

    /* 6. Exit */
    TRACE_EXIT(TRACE_FUNCTION);
    memset(ibus_data, 0, sizeof(ibus_data));
    ibus_data_index = 0;
    return;
err:
    TRACE_EXIT_WARGS(TRACE_FUNCTION, "Invalid message %d\n", -EINVAL);
    memset(ibus_data, 0, sizeof(ibus_data));
    ibus_data_index = 0;
}

#ifdef __TEST__
static void testibusmessage(char* buf){
    memset (ibus_data, 0, sizeof(ibus_data));
    ibus_data_index = strlen(buf)/2;
    int unsigned i;
    for (i = 0; i < ibus_data_index; i++)
        sscanf(&buf[i * 2], "%2hhx", &ibus_data[i]);

    process_ibus_message();
}
#endif

static void print_help(char* name)
{
	fprintf(stderr, "Usage: %s <options>",name);
	fprintf(stderr, "-d serial device name (Mandatory)\n");
	fprintf(stderr, "-h hijack mode. FM/TAPE/AUX\n");
	fprintf(stderr, "-v video input switch. CTS/RTS/GPIO\n");
	fprintf(stderr, "-t tracelevel mask. TRACE_FUNCTION=1<<0, TRACE_IBUS=1<<2, TRACE_INPUT=1<<4 and TRACE_STATE=1<<8\n");
	fprintf(stderr, "-f trace file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "example: %s -d /dev/ttyUSB0 -h AUX -v CTS -t 15 -f ~/tracefile.log \n",name);
	fprintf(stderr, "\n");
}

/******************************************************************************
 * Main loop
 *****************************************************************************/
int main (int argc, char *argv[])
{
	int opt/*,fd*/;
	sigset_t mask;
	sigset_t orig_mask;
	struct sigaction act;
    struct timespec char_timeout,shutdown_timeout;

    struct termios newtio,oldtio;
    char name[128],hijackState[10],videoinputswitch[10];
    bzero(&name, sizeof(name));

    /* Handle command line arguments */
    while ((opt = getopt(argc, argv, "d:t:f:h:v:")) != -1) {
        switch (opt) {
        case 'd':
            strncpy(name,optarg,sizeof(name));
            break;
        case 'h':
            strncpy(hijackState,optarg,sizeof(hijackState));
            fprintf(stderr, "hijack state %s\n",optarg);
            if(strlen(hijackState) > 0){
                if(strcmp(hijackState,"TAPE")==0)
                    IbusHijackState = EStateTAPE;
                else if(strcmp(hijackState,"AUX")==0)
                    IbusHijackState = EStateAUX;
                else
                    IbusHijackState = EStateUnknown;
            }
            break;
        case 'v':
        	strncpy(videoinputswitch,optarg,sizeof(videoinputswitch));
			fprintf(stderr, "video input switch %s\n",optarg);
			if(strlen(videoinputswitch) > 0){
				if(strcmp(videoinputswitch,"CTS")==0)
					VideoInputSwitch = ESwitchCTS;
				else if(strcmp(videoinputswitch,"RTS")==0)
					VideoInputSwitch = ESwitchRTS;
				else if(strcmp(videoinputswitch,"GPIO")==0)
					VideoInputSwitch = ESwitchGPIO;
				else
					VideoInputSwitch = ESwitchUnknown;
			}
        	break;
        case 't':
            trace_level = atoi(optarg);
            break;
        case 'f':
        	stdout_fp = freopen(optarg, "a+", stdout);
        	if(stdout_fp < 0){
        		TRACE_ERROR("Can't open trace file");
        		//continue, not fatal
        	}
            break;
        default: /* '?' */
        	print_help(argv[0]);
            goto exit;
        }
    }

    TRACE_WARGS(TRACE_FUNCTION, "%s\n",__func__);

    if(strlen(name)<=0){
    	print_help(argv[0]);
    	errno = EINVAL;
    	TRACE_ERROR("No serial device provided\n");
    	goto exit;
    }

    /* Open uinput device */
    uinput_device_fd = uinput_create();
    if(uinput_device_fd < 0){
    	TRACE_ERROR("Can't create uinput device");
        goto exit;
    }

    /* set signal handlers and block them temporarily to avoid
     * race conditions before pselect */
    sigemptyset (&mask);

	memset (&act, 0, sizeof(act));
	act.sa_handler = signal_handler;
	if (sigaction(SIGTERM, &act, 0)) {
		TRACE_ERROR ("sigaction SIGTERM");
		goto uinput_close;
	}
	sigaddset (&mask, SIGTERM);

	memset (&act, 0, sizeof(act));
	act.sa_handler = signal_handler;
	if (sigaction(SIGINT, &act, 0)) {
		TRACE_ERROR ("sigaction SIGINT");
		goto uinput_close;
	}
	sigaddset (&mask, SIGINT);

	if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0) {
		TRACE_ERROR ("sigprocmask SIG_BLOCK");
		goto uinput_close;
	}

    /* Open IBUS serial line */
	ibus_device_fd = open(name, O_RDONLY |  /*we want only read IBUS*/
                                O_NOCTTY |  /*no controlling*/
                                O_NONBLOCK);
	if (ibus_device_fd < 0) {
		TRACE_ERROR("Can't open ibus device");
        goto uinput_close;
	}

	/* save current serial port settings */
    if (tcgetattr(ibus_device_fd, &oldtio) < 0) {
    	TRACE_ERROR("Can't get current port settings");
		goto uinput_close;
	}

    /*set line*/
	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
    newtio.c_cflag =    B9600 | /*9600 baud*/
                        CS8 | /*8 bits.*/
                        PARENB | /*Parity enable.*/
                        CLOCAL | /*Ignore modem status lines.*/
                        CREAD; /*Enable receiver.*/
    newtio.c_iflag = IGNPAR | IGNBRK; /*Ignore characters with parity errors., Ignore break condition.*/
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN]=1; /*read one byte at the time. TODO: try vmin =max and VTIME=0.2*/
    newtio.c_cc[VTIME]=0;
    if(tcflush(ibus_device_fd, TCIFLUSH) < 0){
    	TRACE_ERROR("tcflush");
    	goto uinput_close;
    }
    if(tcsetattr(ibus_device_fd,TCSANOW,&newtio) < 0){
    	TRACE_ERROR("tcsetattr");
    	goto uinput_close;
    }

    /* Set timeout value within input loop */
    /* 9600baud = 9600 bits per second*/
    /* 1 start bit, 8 data bits,1 stop bit, even parity = 11 bit = 1 char*/
    /* 11 bits x 1sec/9600 = 1,15ms/char*/
    char_timeout.tv_nsec = 2*1150000L;
    char_timeout.tv_sec  = 0;  /* seconds*/

    /* Shutdown timeout */
    shutdown_timeout.tv_nsec = 0;
    shutdown_timeout.tv_sec = 60*10;

    /* set ibus state to unknown => video input disabled, key events disabled */
    ibus_change_state(EStateUnknown);

	memset (ibus_data, 0, sizeof(ibus_data));
    ibus_data_index = 0;

	while (!exit_request) {
        fd_set fds;
        int res;

		FD_ZERO (&fds);
		FD_SET (ibus_device_fd, &fds);

        if(ibus_data_index) /*if transfer ongoing, then use timeout to know when ibus message is ready*/
            res = pselect (ibus_device_fd + 1, &fds, NULL, NULL, &char_timeout, &orig_mask);
        else
            res = pselect (ibus_device_fd + 1, &fds, NULL, NULL, &shutdown_timeout, &orig_mask);

		if (res < 0 && errno != EINTR) {
            TRACE_WARGS(1, "pselect returned %d\n",res);
			break;
		}

		else if (exit_request) {
            TRACE(1, "User requested EXIT\n");
			break;
		}
		else if (res == 0) {
			if(ibus_data_index){
				/*timeout occured => ibus message ready*/
				process_ibus_message();
				continue;
			}else{
				/*TODO: shutdown the system*/
				TRACE(TRACE_ALL,"10min without messages on the bus => shutdown");
				break;
			}
        }

		if (FD_ISSET(ibus_device_fd, &fds)) {
            unsigned char data[5];
            res = read(ibus_device_fd,data,1);
            if(res == 1){
            	ibus_data[ibus_data_index++] = data[0];
            	if(ibus_data_index==ibus_data_max_length){
            		TRACE_ERROR("BUFFER FULL!! ");
            		//Flush the buffer and continue
            		//TODO: maybe the buffer could be processed before flushing
            		memset(ibus_data, 0, sizeof(ibus_data));
            		ibus_data_index = 0;
            		continue;
            	}
            }
            else {
            	TRACE_WARGS(1, "WARNING!!! read returned %d\n",res);
            }
		}
	}

	if(tcsetattr(ibus_device_fd,TCSANOW,&oldtio) < 0){
		/*Ignore error as we are exiting*/
	}
	if(close(ibus_device_fd) < 0){
		/*Ignore error as we are exiting*/
	}
uinput_close:
    uinput_close();
exit:
	if(stdout_fp) fflush(stdout_fp);
	return 0;
}
