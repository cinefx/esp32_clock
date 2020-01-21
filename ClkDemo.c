// Dutchtronix AVR Oscilloscope Clock
//
//  Copyright © 2010 Johannes P.M. de Rie
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
// Firmware for Dutchtronix AVR Oscilloscope Clock
//
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "./ClkConfig.h"
#include "./ClkData.h"
#include "./ClkDebug.h"
#include "./ClkISR.h"
#include "./ClkFlash.h"
#include "./ClkSupport.h"

extern char	TMenuDemo[] PROGMEM;

char TYourName		[] PROGMEM =	"Your Name here";
char TBinClock		[] PROGMEM =	"Binary Clock";
char TDigClock		[] PROGMEM =	"Digital Clock";
char TFunClock		[] PROGMEM =	"Play Mode";
char TLedClock		[] PROGMEM =	"LED Time";
char TWarnClock		[] PROGMEM =	"Warnings";
char TScreenMenu	[] PROGMEM = 	"On Screen Menu";
char TThanksClock	[] PROGMEM =	"Thank You";
char TBiHexClock	[] PROGMEM =	"Bi-Hex Clock";
char TDayoftheWeek	[] PROGMEM =	"Day of the Week";
char TCalScreen		[] PROGMEM =	"Calibration Screen";
char TTermScreen	[] PROGMEM =	"Terminal";
char TScoreLine		[] PROGMEM =	"Score:  23300";
char TVGScreen		[] PROGMEM =	"Vector Graphics";
char THelpScreen	[] PROGMEM =	"Help Screen";
char TBurnClock		[] PROGMEM =	"BurnIn Prevention";
char TNotShown		[] PROGMEM =	"Also Supported";
char TGPSClock		[] PROGMEM =	"GPS NMEA Parsing";
char TDaylighClock	[] PROGMEM =	"Daylight Saving Time";
char TAlarm			[] PROGMEM =	"Alarm";
char T37Combinations[] PROGMEM = 	"37 Combinations";
char UsrDemoHelpText[] PROGMEM =
		"This demo mode shows some of the capabilities of the\r\n"\
		"AVR Clock. The demo preserves the state of your clock\r\n";


extern void DisablePlayModes(void);
extern void UpdateAlarm(byte newVal);
extern void UpdateChronoDisplay(byte newVal);
extern void UpdateNumericDisplay(byte newVal);
extern void UpdateUsrNameMode(byte newMode);
extern void UpdateDialDisplay(byte newVal);
extern void UpdateLedOptions(byte newVal);
extern void UpdateBurnInPrevention(byte newVal);
extern void UpdPlayMode(byte newVal);
extern void ClockWorks(void);
extern void CMenuSetTable(void);
extern void GPSStartFlashing(byte cnt, BOOL fSolid);
extern void BatteryStartFlashing(byte cnt);
extern void PPSStartFlashing(byte cnt);
extern void	SetDisplay(void);
extern void	UsrPrintNewLine(void);
extern void	SetDialDisplay(void);
extern void DiscardGPSInput(void);
extern void	ClockRefresh(void);
extern void	MenuRefresh(void);
extern void CalibrationRefresh(void);
extern void TermRefresh(void);
extern void DisplayHelpscreen(void);
extern void StartMenuStateFlash(MScanTblEntry_t *pTbl);
extern void ProcessCmd1(byte cmd, byte arg);
extern void ProcessCmd2(byte cmd, byte arg1, byte arg2);
extern void ProcessCmd3(byte cmd, byte *pArgs);
extern void DisplayableChar(char ch);
extern byte args[];

#define DisableNumbers		UpdateNumericDisplay(NUMOFF)
#define Enable12HRNumbers	UpdateNumericDisplay(NUM12HR)
#define Enable24HRNumbers	UpdateNumericDisplay(NUM24HR)
#define EnableHexNumbers	UpdateNumericDisplay(NUMHEX)
#define EnableDateNumbers	UpdateNumericDisplay(NUMDATE)

#define DisableUsrName		UpdateUsrNameMode(USRNAMEOFF)

#define Enable12HRDial	UpdateDialDisplay(DIAL12HR)
#define Enable24HRDial	UpdateDialDisplay(DIAL24HR)
#define EnableMinDial	UpdateDialDisplay(DIALMIN)
#define EnableBinDial	UpdateDialDisplay(DIALBIN)
#define EnableRomanDial	UpdateDialDisplay(DIALROMAN)
#define EnableDigitalDial UpdateDialDisplay(DIALDIG)

#define DisableLedBlinking	UpdateLedOptions(LEDDISABLED)	//no EEProm update
#define EnableLedBlinking	UpdateLedOptions(LEDENABLED)	//no EEProm update
#define	LedMorseTime		UpdateLedOptions(LEDMORSE)		//no EEProm update

#define DisableChrono	UpdateChronoDisplay(FALSE)			//no EEProm update
#define EnableChrono	UpdateChronoDisplay(TRUE);			//no EEProm update

typedef struct {
	byte x0, x1, x2, x3;
} Vect_t;
#include "vectsamples.inc"

void PrintDisplayableChar(PGM_P p)
{
	char ch;
	do {
		ch = pgm_read_byte(p);
		if (ch == 0) break;
		DisplayableChar(ch);
		++p;
	} while (TRUE);
}

//
// Wait n seconds while maintaining image and events.
// Issue: These wait functions are all called from UsrDemo, which may be called from the
//		 Menu while a TC1 timeout is running. At this point, the Menu Timeout test
//		 is never done while UsrDemo is running.
//
void DemoWait(byte nSecs)
{
	SetTC1Countdown(nSecs);
	do {
		ClockWorks();
		DiscardGPSInput();
	} while (TC1CountdownActive() && !(GPIOR0 & _BV(fUPS2)));
	GPIOR0 &= ~_BV(fUPS2);									//clear S2 up action if necessary
	SetTC1Countdown(0);										//not really necessary
}

void Wait2(void)
{
	DemoWait(2);
}

void Wait3(void)
{
	DemoWait(3);
}

void Wait5(void)
{
	 DemoWait(5);
}

void DemoSetUsrName(PGM_P pName)
{
	byte cnt = 0;
	char *pUsrNameBuf = UsrNameBuf;
	while ((*pUsrNameBuf++ = pgm_read_byte(pName++)) != 0) {
		++cnt;
	}
	EEConfigData.UsrNameCnt = cnt;
	EEConfigData.UsrNameVal = USRNAMEON;
	SetDisplay();
}

void DemoSetDayOfTheWeek(void)
{
	EEConfigData.UsrNameVal = USRNAMEDAY;
	SetDisplay();
}

void SetDefaultDemoState(void)
{
	Enable12HRDial;
	Enable12HRNumbers;
}

void DemoNumericField(void)
{
	Enable12HRNumbers;
	Wait2();
	EnableDateNumbers;
	Wait2();
	Enable24HRNumbers;
	Wait2();
	EnableChrono;
	Wait2();
	DisableChrono;
	EnableHexNumbers;
	Wait2();
	Enable12HRNumbers;
	Wait2();
	DisableNumbers;
	Wait2();
	EnableDateNumbers;
}

void SaveAndSetState(void)
{
//
// Assume for now that the "Push to Start" message is not used in Demo Mode
//
	ScanTblPush->pVect = (VectorTblEntry_t *)&TMenuDemo;	//Demo Message
	ScanTblPush->Xoffset = DEMOHPOS;
	ScanTblPush->Yoffset = DEMOVPOS;			

	DisablePlayModes();
	DisableChrono;
	fpCurrentRefresh = ClockRefresh;				//set Clock Refresh func ptr (word address)
	ScanTblNum->Yoffset = NUMERICVPOS;				//Vertical Position for Numeric Field. Identical for all modes.
	AVRTimeDateBlk.Hrs = 15;						//3:44PM, to make sure the 24hr clock is displayed
	AVRTimeDateBlk.Mins = 42;						//also to highlight Hex Display
	SetDialDisplay();
	DemoSetUsrName(MsgPszLogo);
	SetDefaultDemoState();
}

BOOL UsrDemo(char c)
{
	BOOL saveBurnIn;
	Flags.DemoEnabled = _BV(fDemoGoing);
	SaveAndSetState();
	UsrPrintNewLine();
	UARTPrintfProgStr(UsrDemoHelpText);
	DemoNumericField();
	DemoSetUsrName(TYourName);
	Enable12HRDial;
	Wait3();
	EnableMinDial;
	Wait3();
	Enable24HRDial;					//depends on the time
	Wait3();
	EnableRomanDial;
	Wait3();
	DemoSetUsrName(TDigClock);
	EnableDigitalDial;
	DemoNumericField();

	DemoSetUsrName(TBinClock);
	EnableBinDial;
	DemoNumericField();

	DemoSetUsrName(TBiHexClock);
	EnableHexNumbers;
	Wait3();

	Enable12HRDial;
	Enable12HRNumbers;
	DemoSetUsrName(T37Combinations);
	Wait3();
//
// mix and match
//  comment in Binary Clock and hex display
//
	DemoSetUsrName(TDayoftheWeek);
	Wait2();
	DemoSetDayOfTheWeek();
	Wait2();
	EnableChrono;
	Enable24HRDial;
	Wait3();
	DisableChrono;
	EnableDateNumbers;
	Wait3();
	EnableRomanDial;
	Wait3();
	EnableDigitalDial;
	Wait3();
	EnableBinDial;
	Enable12HRNumbers;
	Wait3();
	EnableMinDial;
	EnableHexNumbers;
	Wait3();
	Enable12HRNumbers;
	Enable12HRDial;

	DemoSetUsrName(TLedClock);
	LedMorseTime;
	Wait5();
	DemoSetUsrName(TCalScreen);
	Wait2();
	fpCurrentRefresh = CalibrationRefresh;				//Refresh func ptr in Calibration Mode (word address)
	Wait3();
	fpCurrentRefresh = ClockRefresh;					//set default Refresh func ptr (word address)
	DemoSetUsrName(TTermScreen);
	Wait2();
	fpCurrentRefresh = TermRefresh;						//Refresh func ptr in Terminal Mode (word address)
	DisableNumbers;										//Or update Numeric Buffer location for Terminal Mode
	DisableUsrName;										//Or update UsrName Buffer location for Terminal Mode
	Wait5();
	fpCurrentRefresh = ClockRefresh;					//set default Refresh func ptr (word address)
	Enable12HRNumbers;
	DemoSetUsrName(TVGScreen);
	Wait2();
	fpCurrentRefresh = TermRefresh;						//Refresh func ptr in Terminal Mode (word address)
	// turn off Numeric Display, Burnin
	ProcessCmd1(TermSetClkLine, 0);
	ProcessCmd1(TermSetBurnIn, 0);
	// clrscreen
	ProcessCmd1(TermCtrl, 0);
	// add vectors
	for (byte i = 0; i < 101; ++i) {
		args[0] = i;
		args[1] = pgm_read_byte(&VectSamples[i].x0);
		args[2] = pgm_read_byte(&VectSamples[i].x1);
		args[3] = pgm_read_byte(&VectSamples[i].x2);
		args[4] = pgm_read_byte(&VectSamples[i].x3);
		ProcessCmd3(VectEntry, args);
	}
	// set vectorrange
	ProcessCmd2(VectRange, 0, 101);
	// add Score:  23300
	ProcessCmd2(TermSetCursor, 0, MAXSCREENLINES-1);
	PrintDisplayableChar(TScoreLine);
	Wait5();
	ProcessCmd1(TermSetClkLine, 1);
	ProcessCmd1(TermSetBurnIn, 1);
	fpCurrentRefresh = ClockRefresh;					//set default Refresh func ptr (word address)

	DemoSetUsrName(THelpScreen);
	Wait2();
	DisplayHelpscreen();
	DemoSetUsrName(TScreenMenu);
	Wait2();
	CMenuSetTable();
	StartMenuStateFlash(MScanTbl_NumVal);				//First field to flash
	fpCurrentRefresh = MenuRefresh;						//Refresh func ptr in Menu Mode (word address)
	Wait5();
	fpCurrentRefresh = ClockRefresh;					//set default Refresh func ptr (word address)

//
// Fun stuff
//
	DemoSetUsrName(TFunClock);
	UpdPlayMode(FUNFASTF);
	Wait3();
	DisablePlayModes();
	Wait2();
	UpdPlayMode(FUNREV);
	Wait3();
	UpdPlayMode(FUNFASTR);
	Wait3();
	DisablePlayModes();

	DemoSetUsrName(TWarnClock);
	BatteryStartFlashing(FLASHPERSEC*3);
	Wait3();
	Flags.DemoEnabled = (_BV(fDemoPPS)|_BV(fDemoGoing));
	PPSStartFlashing(FLASHPERSEC*3);
	Wait3();
	Flags.DemoEnabled = _BV(fDemoGoing);	//turn off fDemoPPS
	GPSStartFlashing(FLASHPERSEC*3, FALSE);
	Wait3();
//
// This code depends on the fact that the Dial normally cannot flash. May change.
//
	(void)AddFlashItem(ScanTbl_Dial, FLASHPERSEC*3, NULL);					//3 seconds
	Wait3();

	DemoSetUsrName(TBurnClock);
	Flags.DemoEnabled = (_BV(fDemoBurnIn)|_BV(fDemoGoing));
	saveBurnIn = Flags.RotationDisabled;
	Flags.RotationDisabled = FALSE;
	RotationIndex = 0;								//EnableBurnInPerSecond
	Wait5();
	Flags.RotationDisabled = saveBurnIn;
	Flags.DemoEnabled = _BV(fDemoGoing);			//turn off fDemoBurnIn

	DemoSetUsrName(TNotShown);
	Wait3();
	DemoSetUsrName(TGPSClock);
	Wait3();
	DemoSetUsrName(TDaylighClock);
	Wait3();
	DemoSetUsrName(TAlarm);
	Wait3();

//
// Demo Change Time and Date
// Demo Automatic Daylight Saving Time adjustment
// Demo GPS NMEA parsing
// Demo Name: first Dutchtronix on,off,on. Then set "Your name here"
// Set to to PM for 24hr dial
//
//	DisableNumbers();
//	Enable12HRDial();
//	StartFlashingHRHand();
//	Wait3();
//	StartFlashingMinHand();
//	Wait3();
//	StartFlashingSecHand();
//	Wait3();
//	EnableBinDial();
//
	SetDefaultDemoState();
	DemoSetUsrName(TThanksClock);
	Wait5();
//	RestoreState();						//not needed since a reboot follows
	InitiateSysReset();					//reboot
	return TRUE;
}

