// Dutchtronix
//
//  Copyright @ 2010 Jan P.M. de Rie
//
//  All Rights Reserved
//
//  This file is part of the Dutchtronix Oscilloscope Clock Distribution.
//  Use, modification, and re-distribution is permitted subject to the
//  terms in the file named "LICENSE.TXT", which contains the full text
//  of the legal notices and should always accompany this Distribution.
//
//  This software is provided "AS IS" with NO WARRANTY OF ANY KIND.
//
//  This notice (including the copyright and warranty disclaimer)
//  must be included in all copies or derivations of this software.
//
//
#if defined( __ASSEMBLER__ )

#define	GXOffset r7
#define	GYOffset r8
#define	__zero_reg_ r1
//
//-- map the IO register back into the IO address space
//
#define PB_DDR		(_SFR_IO_ADDR(PORTB) - 1)
#define PC_DDR		(_SFR_IO_ADDR(PORTC) - 1)
#define PD_DDR		(_SFR_IO_ADDR(PORTD) - 1)
#define PB_OUT		_SFR_IO_ADDR(PORTB)
#define PC_OUT		_SFR_IO_ADDR(PORTC)
#define PD_OUT		_SFR_IO_ADDR(PORTD)
#define PB_IN		(_SFR_IO_ADDR(PORTB) - 2)
#define PC_IN		(_SFR_IO_ADDR(PORTC) - 2)
#define PD_IN		(_SFR_IO_ADDR(PORTD) - 2)

#else
// Registers r2..r6, r9..r10 are used in the render functions.
#if 0
    register unsigned char _r2_ asm("r2"); 
    register unsigned char _r3_ asm("r3"); 
    register unsigned char _r4_ asm("r4"); 
    register unsigned char _r5_ asm("r5"); 
    register unsigned char _r6_ asm("r6"); 
	//
	// GXOffset and GYOffset can easily be ram variables
	// but this results in more code and slower access
	//
    register unsigned char GXOffset asm("r7"); 
    register unsigned char GYOffset asm("r8"); 
	//
	// Warning: using r10 as a register variable with Winavr is not safe.
	// For certain math operations (e.g. long long multiply) r10 is used as
	// an argument register, but not saved because it is a register variable.
	//
	// In general, register variables are not safe in ISRs when using
	// library functions. We only reserve these registers so we don't
	// have to save them, ever. r7/r8, also unsafe in ISRs, are not used there.
	//
    register unsigned char _r9_ asm("r9"); 
    register unsigned char _r10_ asm("r10"); 
#endif
#endif

#define MajorVersion 4
#define MinorVersion 0

#define DEBUG 0
#define EXTDEBUG 0
#define NETFREQUENCY 0
#define CLOCKLEDENABLED !NETFREQUENCY
#define SRAMTABLES	1			// necessary for C version of MiniDDS
#define ASCIILINK 0
#define Mega328p 1
#define ds1307 0				// Sparkfun Hardware
#define pcf8563 1				// Dutchtronix Hardware
//
// 8 bit parallel data (7..0) is mapped to PC3..PC0 PB3..PB0
//	DACA/DACB select is mapped to PD3 (low is A)
//	WR (active low) is mapped to PD4
//	LED (active high) or NET input is mapped to PD6
//	LDAC is mapped to PD5
//	Z control is mapped to PD7
//	Tactile Switch 1 is mapped to PB4
//	Tactile Switch 2 is mapped to PB5
//
#if NETFREQUENCY
//
// Sync with the net frequency
// Cannot use the LED
//
#define	NET_DDR PD_DDR
#define	NET_PORT PD_OUT
#define	NET_PIN PD_IN
#define	NET_BIT 6
#define	NETDDR DDRD
#define	NETPort PORTD
#define	NETPin PIND
#define	NETBit 6
#else
#define	LED_DDR PD_DDR
#define	LED_PORT PD_OUT
#define	LED_PIN PD_IN
#define	LED_BIT 6
#define	LEDDDR DDRD
#define	LEDPort PORTD
#define	LEDPin PIND
#define	LEDBit 6

#endif	// NETFREQUENCY

#define DACCTL PD_OUT
#define DACCTLDDR PD_DDR
#define DACSELECT	PD_OUT,3
#define DACSELBit 3
#define DACWRBit 4
#define DACLDACBit 5
#define PPSPort PORTD
#define PPSBit	2
#define	PPS_IN PD_OUT,PPSBit

#define DACHIDATA PC_OUT
#define DACLODATA PB_OUT
#define	DACHIDDR  PC_DDR
#define	DACLODDR  PB_DDR
#define DACZX PD_OUT,7
#define DACCTLPort PORTD
#define DACZXBit	7
#define	DACZXDDR  PD_DDR,7
//
// use PB4 and PB5 as input lines
//
#define	SW_DDR PB_DDR
#define	SW_PORT PB_OUT
#define	SW_PIN PB_IN
#define	SW1_BIT 4
#define	SW2_BIT 5
#define SWDDR	DDRB
#if 0
#define SWPort	PORTB
#define SWPin 	(PB0&1)//PINB
#endif
#define SW1Bit	4
#define SW2Bit	5
//
// RTC Internal Registers
//
#if ds1307
#define	RTCCNT		7
#define RTCYEAR		6
#define RTCMONTH	5
#define RTCDAY		4
#define RTCWEEKDAY	3
#define RTCHOUR		2
#define RTCMIN		1
#define RTCSEC		0
#elif pcf8563
#define RTCYEAR		8
#define RTCMONTH	7
#define RTCWEEKDAY	6
#define RTCDAY		5
#define RTCHOUR		4
#define RTCMIN		3
#define RTCSEC		2
#define	RTCCS2		1
#define RTCCLK		0x0D
#define RTCTCR		0x0E
#define	RTCCNT		0x0F
#endif
//
// Scope Clock Constants
//
#define	SRAM_START	0x100

#ifndef NULL
#define	NULL  0
#endif
#define	TRUE	1
#define	FALSE	0
#define	BS		8
#define	TAB		9
#define	CR		13
#define	LF		10
#define	TICKS_HZ 			60
#define	TIMEOUTTICKS 	  	62		//>1  second timeout
#define	TERMTIMEOUTTICKS 	66		//>1.1  second delay
#define	FASTTIMEOUTTICKS 	2		// force clock to run fast
#define	FLASHPERSEC  		4
#define	FLASHTICKS			TICKS_HZ/FLASHPERSEC
#define	BURNINCOUNTDOWN 	5		// move grid every 5 minutes to prevent burn-in
#define	INP_TIMEOUT  		30		// must be <60
#define	TIMER0TIMEOUT	  	125		// a little over 1 second
#define	TIMER1TIMEOUT1SEC 	60		// 1 seconds

#define FIRST_CHAR 32
#define	CHAR_WIDTH 6
#define CHAR_HEIGHT 8

#define SCANTABLEN   8
#define SCANSTRTABLEN    5
#define SCANSSTRTABLEN    2
#define MENUSCANTABLEN    31
#define	HELPSCANTABLEN    12
#define	HELPSCANEXTTABLEN 2
#define	NUMDISPBUFLEN 	12
#define MaxFlashTblEntries 6
#define FlashTblEntrySize 10
#define	GPSINBUFLENGTH 	75
//
// Timer computations
//
// 20 Mhz:		60 Hz: -1302 = 0xfaea
//
#define	Timer1CountUpL  lo8(1302)	// timer1 set for 60 Hz
#define	Timer1CountUpH  hi8(1302)
#define	Timer1CountUp  1302
#define	Timer0CountUp  127			// timer0 set for 6.5536 mSec
#define	Timer2CountUp  97			// timer2 set for 4.99 mSec

#define	INT0CNTDOWNVAL	4096
#if pcf8563|ds1307
#define	ONEHZCODE		0
#define	F4096HZCODE		127
#define	MENUONEHZCODE	0
#define	MENUF4096HZCODE	1
#endif
//
// GPIOR0 bits
//
#define		fRcvdChar	0
#define		fDebouncing	1
#define		fUpdSecond	2
#define		fUPS1		3
#define		fTimeoutS1	4
#define		fUPS2		5
#define		fTimeoutS2	6
#define		fFLASH		7
//
// flags in DemoEnabled
//
#define		fDemoGoing	0
#define		fDemoPPS	1
#define		fDemoBurnIn	2

#define	DEFCHARWIDTH		12
#define MENUSPACINGWIDTH	6
#define	HALFSPACECHARWIDTH	4
#define	COMMACHARWIDTH		6
#define	COLONCHARWIDTH		8
#define	DASHCHARWIDTH		10
#define	MINNUMERICH			6
#define	MINNUMERICV			8
#define	NUMERICVPOS			(81 - MINNUMERICV)
#define	TERMNUMERICVPOS		0
//
// ScanTbl entries
//
#define stDialMarks		0
#define stDialDigits	1
#define stSecHand 		2
#define stSecHand2		3
#define stMinHand 		4
#define stMinHand2		5
#define stHrHand 		6
#define stHrHand2		7

#define stStrFlash		(SCANTABLEN)
#define stPushtoStart	(stStrFlash)
#define stMsgG			(stStrFlash+1)
#define stMsgB			(stStrFlash+2)
#define stMsgP			(stStrFlash+3)
#define stMsgSig		(stStrFlash+4)

#define stStrSRam		(SCANTABLEN+SCANSTRTABLEN)
#define stUsrName		(stStrSRam)
#define	stNumBuf		(stUsrName+1)

#define SecHandsDataSize 5
#define MinHandsDataSize 9
#define HrHandsDataSize 9
#define DialMarksDataSize 5
#define DialDotsDataSize 9

#define LOGOHPOS			25
#define LOGOVPOS			235
#define MENUHPOS			3
#define MENUH2POS			134
#define MENU0VPOS			195
#define MENU1VPOS			170
#define MENU2VPOS			145
#define MENU3VPOS			120
#define MENU4VPOS			95
#define MENU5VPOS			70
#define MENU6VPOS			45
#define MENU7VPOS			20

#define	DEMOHPOS			105
#define	DEMOVPOS			45

#define	STATUSGHPOSANALOG	121	
#define	STATUSGHPOSBIN		121
#define	STATUSGHPOSDIGITAL	121
#define	STATUSBHPOSANALOG	107	
#define	STATUSBHPOSBIN		107
#define	STATUSBHPOSDIGITAL	107
#define	STATUSPHPOSANALOG	135	
#define	STATUSPHPOSBIN		135
#define	STATUSPHPOSDIGITAL	135
#define	STATUSVPOSANALOG	180
#define	STATUSVPOSBIN		1
#define	STATUSVPOSDIGITAL	190

#define USRNAMEACLOCKVPOS 145
#define USRNAMEBCLOCKVPOS 25
#define USRNAMEDCLOCKVPOS 165
#define TEXTHPOS	60		// initially

#define	P2SVPOS		DEMOVPOS
#define	P2SHPOS		43

#define ESCAPE		0x0b	// make sure this ESCAPE value is never used in the vector tables
#define	SPACE		32
#define	UNDERSCORE	95
#define	HALFSPACE	128

#define BINVPOS0	105
#define BINVPOS1	140
#define BINVPOS2	175
#define BINVPOS3	210

#define LedHHHPOS	32
#define LedHLHPOS	55
#define LedMHHPOS	110
#define LedMLHPOS	133
#define LedSHHPOS	184
#define LedSLHPOS	207

#define DigHHHPOS	23
#define DigHLHPOS	53
#define DigMHHPOS	94
#define DigMLHPOS	124
#define DigSHHPOS	164
#define DigSLHPOS	194
#define DigSepHPOS	83

#define	CMODDATE	0b00010000
#define CMODDAY		(0|CMODDATE)
#define CMODMONTH	(1|CMODDATE)
#define CMODPREYEAR	(2|CMODDATE)
#define CMODYEAR	(3|CMODDATE)
#define	CMODDATESET (4|CMODDATE)
#define CMODPREDAY	(5|CMODDATE)		// values are bit tested for CMODDATE
#define CMODIDLE	40
#define CMODPREHRNR	41
#define CMODHRNR	42
#define CMODMINNR	43
#define CMODSECNR	44
#define CMODPREHRHAND 45
#define CMODHRHAND	46
#define	CMODMINHAND 47
#define	CMODSECHAND 48
#define	CMODPRESET	49
#define	CMODSET		50
#define CMODCANCEL	51

#define	CMODPREHRBIN	52
#define	CMODHRBIN		53
#define	CMODMINBIN		54
#define	CMODSECBIN		55
#define CMODPREALARMHR	56
#define CMODALARMHR		57
#define CMODALARMMIN	58
#define CMODALARMSEC	59
#define CMODALARMSET	60
//
// CANNOT use CMOD values > 0x3f (63) because NOPUSH & DBLHAND status bits
//
#define NOPUSH			0x80
#define DBLHAND			0x40

#define RUNCLOCKREVERSE _BV(0)
#define RUNCLOCKFAST    _BV(1)
#if DEBUG
#define FUNNORM         32
#else
#define FUNNORM         0
#endif
#define FUNREV          (FUNNORM | RUNCLOCKREVERSE)
#define FUNFASTF        (FUNNORM | RUNCLOCKFAST)
#define FUNFASTR        (FUNNORM | RUNCLOCKFAST | RUNCLOCKREVERSE)
#define FUNLAST			FUNFASTR

#define	NOGPS		60
#define	GPSOFSMIN12	61
#define	GPSOFSZERO	73
#define	GPSOFSPLUS12 85

#define BURNINOFFVAL	5
#define MAXBURNINVAL	9

#define NUMFIELDFLASH	0x8000				//assumes no data address has this bit set
#define	HourInString	1
#define	MinInString		2
#define	SecInString		3
#define DayInString		HourInString
#define	MonthInString	4
#define YearInString	5
#define NoneInString	6

#define DSTFORWARDMONTH 3
#define	DSTUSBACKMONTH 11
#define	DSTEUBACKMONTH 10
#define DSTBACKHOUR 2
#define DSTUSFORWARDHR 2
#define DSTEUFORWARDHR 1

#define AUTOREPEATVAL 15

#define GPSStatusField 2
#define GPSTimeField 1
#define GPSDateField 9
//
// Terminal related constants
//
#define	MAXLINESIZE	20
#define	LINEVSIZE 18
#define	MAXSCREENLINES	14
#define DEFSCREENLINES 13
#define	TOPLINEVPOS	(((MAXSCREENLINES - 1)*LINEVSIZE)+3)
#define	TERMSCANSSTRTABLEN 14
#define	MAXBUNCHEDBYTES	128
#define TERMESCAPE 0xff
#define MAXVECTENTRIES 128
//
// Function Generator
//
//
// value choses so they are "Call Saved" for WinAVR
// use r16,r17 (also "Call Saved") for DAC Access
// r30/r31 (Z) is still "Call Used"
//
#define ACC0	r5
#define ACC1	r6
#define ADDER0	r2
#define ADDER1	r3
#define ADDER2	r4
