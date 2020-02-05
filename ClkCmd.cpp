// Dutchtronix AVR Oscilloscope Clock
//
//  Copyright @ 2010 Johannes P.M. de Rie
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
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "./ClkConfig.h"
#include "./ClkData.h"
#include "./ClkISR.h"
#include "./ClkDebug.h"
#include "./ClkFlash.h"
#include "./ClkSupport.h"

extern void SetDisplay(void);
extern void DSTRetro(BOOL fAllowDSTFix);
extern void DSTRetroApply(void);
extern void DSTRetroNotApply(void);
extern void UpdateTimeAndDisplay(void);
extern void UpdateDateAndDisplay(void);
extern void UpdateNumericDisplay(byte newVal);
extern void UpdateDialDisplay(byte newVal);
extern void UpdPlayMode(byte newVal);
extern void UpdateDSTMode(byte newVal);
extern void UpdateLedOptions(byte newVal);
extern void RestoreUsrName(void);
extern void UpdateUsrNameMode(byte newMode);
extern void SaveUsrName(void);
extern void ClockWorks(void);
extern BOOL UpdateAlarm(byte newMode);
extern void UpdateBurnInOptions(byte newVal);
extern BOOL UpdateGPSSetting(byte newGPSVal);
extern BOOL UpdateChronoDisplay(byte newMode);
extern BOOL UsrChangeLedOptions(char c);
extern BOOL UpdateVerbose(byte newMode);
extern BOOL UpdatePPSTrigger(byte newMode);
extern BOOL UsrDemo(char c);
extern BOOL UpdateIOCTL(uint w);
extern byte SetDialDisplay(void);

//extern BOOL (*CmdMapping[])(char c) PROGMEM;
extern char UsrHelpText[] PROGMEM;
extern char MsgPszCommand[] PROGMEM;
extern char MsgPszCancel[] PROGMEM;
extern char MsgPszOneNumericCommand[] PROGMEM;
extern char MsgPsz2Digit[] PROGMEM;
extern char MsgPszHourCommand[] PROGMEM;
extern char MsgPszMinCommand[] PROGMEM;
extern char MsgPszSecCommand[] PROGMEM;
extern char MsgPszYearCommand[] PROGMEM;
extern char MsgPszMonthCommand[] PROGMEM;
extern char MsgPszDayCommand[] PROGMEM;
extern char MsgPszCopyright[] PROGMEM;
extern char MsgPszLargeNumericCommand[] PROGMEM;
extern char MsgPszGetName[] PROGMEM;
extern char MsgPszIOCTLCommand[] PROGMEM;

char *DstReportVectors(char *pBuf);
char *UsrShowNumericByteStatus(char *pBuf, char status, byte val);
char *UsrInsertSpace(char *pBuf);
void UsrPrintNewLine(void);
void UsrSendByte(char ch);
BOOL SpecialUARTRB(char *pChar);
BOOL UsrGetOneNum(byte nMax, byte *pVal);
BOOL UsrGetTwoNum(PGM_P pMsg, byte *pVal);
void UsrShowStatus(void);

BOOL UsrSetTime(void);
BOOL UsrSetDate(void);
BOOL UsrChangeApp(void);
BOOL UsrShowCopyright(void);
BOOL UsrResetEEprom(void);
BOOL UsrShow(void);
BOOL UsrGetName(void);
BOOL UsrCancel(void);
BOOL UsrSetIOCTL(void);
BOOL UsrDisplayHelp(void);
BOOL ToggleGPSMode(void);
BOOL UsrSetDelayValue(void);

char ToggleCommands[] PROGMEM = "CMVK";
BOOL (*CmdToggleMapping[])(byte v) PROGMEM = {
	UpdateAlarm,
	UpdatePPSTrigger,
	UpdateVerbose,
	UpdateChronoDisplay
};
pByte CmdToggleMapping2[] PROGMEM = {
	&EEConfigData.AlarmDisabled,
	&EEConfigData.PPSTrigger,
	&EEConfigData.Verbose,
	&EEConfigData.ChronoEnabled
};
//
// Process a Toggle Command
// cmd is offset in ToggleCommands
//
BOOL UsrToggleCmd(byte cmd)
{
	BOOL (*pF)(byte v);
	pByte pCfg;
	ASSERT(cmd < 4);
	pF = (BOOL (*)(byte v))pgm_read_word(&CmdToggleMapping[(byte)cmd]);
	pCfg = (pByte)pgm_read_word(&CmdToggleMapping2[(byte)cmd]);
	return pF(*pCfg ^ 1);
}

char ChangeCommands[] PROGMEM = "NFPULB";
void (*CmdChangeMapping[])(byte v) PROGMEM = {
	UpdateNumericDisplay,
	UpdateDialDisplay,
	UpdPlayMode,
	UpdateDSTMode,
	UpdateLedOptions,
	UpdateBurnInOptions
};
byte CmdChangeMapping2[] PROGMEM = {
	NUMLAST-NUM12HR,
	DIALLAST-DIAL12HR,
	FUNLAST-FUNNORM,
	DSTLAST-0,
	LEDLAST-LEDENABLED,
	9-0
};
#if DEBUG
byte CmdChangeMapping3[] PROGMEM = {
	NUM12HR,
	DIAL12HR,
	FUNNORM,
	0,
	LEDENABLED,
	0
};
#endif
//
// Process a Change Command
// cmd is offset in ChangeCommands
//
BOOL UsrChangeCmd(byte cmd)
{
	byte maxValue, baseValue = 0, val;
	void (*pF)(byte v);
	ASSERT(cmd < 6);
	pF = (void (*)(byte v))pgm_read_word(&CmdChangeMapping[(byte)cmd]);
	maxValue = (byte)pgm_read_byte(&CmdChangeMapping2[(byte)cmd]);
#if DEBUG
	baseValue = (byte)pgm_read_byte(&CmdChangeMapping3[(byte)cmd]);
#endif
	if (UsrGetOneNum(maxValue, &val)) {
		if (cmd == 5) {		//'B'
			//must map to EEProm range 
			if (val >= 5) val -= 5; else val += 5;
		}
		pF(val + baseValue);
		return TRUE;
	}
	return FALSE;
}

char OtherCommands[] PROGMEM = "DTAHIORSWG";
BOOL (*CmdOtherMapping[])(void) PROGMEM = {
	UsrSetDate,
	UsrSetTime,
	UsrChangeApp,
	UsrDisplayHelp,
	UsrSetIOCTL,
	UsrShowCopyright,
	UsrResetEEprom,
	UsrShow,
	UsrGetName,
	ToggleGPSMode
};
//
// Process an 'Other' Command
// cmd is offset in OtherCommands
//
BOOL UsrOtherCmd(byte cmd)
{
	BOOL (*pF)(void);
	ASSERT(cmd < 12);
	pF = (BOOL (*)(void))pgm_read_word(&CmdOtherMapping[(byte)cmd]);
	return pF();
}
//
// Toggle the mode of the Serial IN port between
// commands and NMEA records
// This includes changing the baudrate of the serial port.
// Used by command line interpreter
//
BOOL ToggleGPSMode(void)
{
	EEConfigData.GPSInEnabled ^= 1;			//toggle
	byte newGPSVal;
	if (EEConfigData.GPSInEnabled) {
		newGPSVal = GPSOFSZERO;
	} else {
		newGPSVal = NOGPS;
	}
	(void)UpdateGPSSetting(newGPSVal);
	return TRUE;
}

char * MsgAddNewLine(char *p)
{
	return strcpy_P(p, MsgPszNewLine) + strlen(p);
}

void ShowClockVersion(void)
{
	UsrPrintNewLine();
	UARTPrintfProgStr(MsgPszLogo);
	UARTPrintfProgStr(MsgPszSignOn);;
	UARTPrintfProgStr(MsgPszVersion);
}

BOOL UsrDisplayHelp(void)
{
	ShowClockVersion();
	UARTPrintfProgStr(UsrHelpText);
	return TRUE;
}

char * DstReportVectors(char *pBuf)
{
	pBuf = UsrShowNumericByteStatus(pBuf, 'f', EEConfigData.DSTFYears);		//forward vector
	return UsrShowNumericByteStatus(pBuf, 'b', EEConfigData.DSTBYears);		//backward vector
}

void DSTReportAction(void)
{
	char LBuf[64], *p;
	UARTPrintfProgStr(MsgPszDSTApplied);
	p = DstReportVectors(LBuf);
	p = MsgAddNewLine(p);
	ASSERT(*p == 0);
	ASSERT(strlen(LBuf) <64);
	UARTPrintfStr(LBuf);
}
//
// Show Status with Numeric Field.
//
char *UsrShowNumericByteStatus(char *pBuf, char status, byte val)
{
	pBuf[0] = status;
	pBuf = FormatHex(pBuf+1, val);
	return UsrInsertSpace(pBuf);


}
char *UsrShowNumericWordStatus(char *pBuf, char status, uint w)
{
	pBuf = UsrShowNumericByteStatus(pBuf, status, w >> 8);
	pBuf = FormatHex(pBuf-1, w & 0xff);
	return UsrInsertSpace(pBuf);
}
//
// Add a SPACE to output text buffer
//
char *UsrInsertSpace(char *pBuf)
{
	pBuf[0] = ' ';
	return pBuf+1;
}
//
// convert char to upper case
//
char ToUpper(char ch)
{
	if ((ch >= 'a') && (ch <= 'z')) {
		ch -= ('a' - 'A');				//rescale
	}
	return ch;
}
//
// Special Versions of the output functions.
// These call the main pump (ClockWorks) while waiting for
// the serial port to be ready.
// Issue: If ClockWorks wants to do output while being called from one
//		 of these. Recursion may occur and order of output may be messed up.
//
void SpecialWait4UART(void)
{
	while (!IsUARTOutReady()) {
		ClockWorks();
	}
}
//
// send char 
//
void UsrSendByte(char ch)
{
	UARTSendByte(ch, TRUE);
}
//
// Get char from serial port. FALSE means timeout occurred
//
BOOL SpecialUARTRB(char *pChar)
{
	byte cnt = AVRTimeDateBlk.Secs + INP_TIMEOUT;		//timeout seconds
	if (cnt >= 60) {
		cnt -= 60;			//mod 60
	}
	while (!UARTHasData()) {
		ClockWorks();
		if (AVRTimeDateBlk.Secs == cnt) {				//if same, timeout
			return FALSE;
		}
	}
	*pChar = UARTReceiveByte();
	return TRUE;
}
//
// Send Newline
//
void UsrPrintNewLine(void)
{
	UARTPrintfProgStr(MsgPszNewLine);
}
//
// Serial (RS-232) User Interface
//
BOOL UsrCommand(void)
{
	char cmd, oldCmd;
	BOOL b;
	PGM_P pCmd;
//	BOOL (*pF)(char c);

//
// no need to loop here since we know a char is waiting
//
	cmd = UARTReceiveByte();
	cmd = ToUpper(cmd);													//ignore case
	if ('X' == cmd) {
//
//	user asked attention by typing 'x' or 'X'
//	prompt for command. Answers are case insensitive.
//
		do {
			UARTPrintfProgStr(MsgPszCommand);
			if (!SpecialUARTRB(&cmd)) {									//get command
				return FALSE;											//timeout
			}
			UsrSendByte(cmd);											//echo
			oldCmd = ToUpper(cmd);
			cmd = oldCmd - 'A';											//rebase
			UsrSendByte(' ');
		} while (cmd >= 'Z' - 'A' + 1);
		Flags.AppChanged = FALSE;
		if ((pCmd = strchr_P(ToggleCommands, oldCmd)) != NULL) {
			b = UsrToggleCmd(pCmd - ToggleCommands);
		} else if ((pCmd = strchr_P(ChangeCommands, oldCmd)) != NULL) {
			b = UsrChangeCmd(pCmd - ChangeCommands);
		} else if ((pCmd = strchr_P(OtherCommands, oldCmd)) != NULL) {
			b = UsrOtherCmd(pCmd - OtherCommands);
		} else {
			b = FALSE;
		}
		if (b == FALSE) {
			UARTPrintfProgStr(MsgPszCancel);
		}
		UsrPrintNewLine();
	}
	UsrShowStatus();
//
// force complete clock redraw (except the dial)
//
	SetDisplay();
	return Flags.AppChanged;											//return App change status
}
//
// check if the argument (ptr to char) is a valid digit (0-9, A-F if hex)
// return FALSE if invalid, else return binary value in argument
//
BOOL UsrChkDigit(pByte pVal, byte radix)
{
	char ch = *pVal;
	if ((ch >= '0') & (ch <= '9')) {
		*pVal = (ch - '0');
		return TRUE;
	} else if (radix == 16) {
		ch = ToUpper(ch);
		if ((ch >= 'A') & (ch <= 'F')) {
			*pVal = (ch - 55);
			return TRUE;
		} else {
			return FALSE;
		}
	} else {
		ASSERT(radix == 10);
		return FALSE;
	}
}
//
//	get 1 digit from the Serial Port. Decimal or hex. Internal use
//
BOOL UsrGetOneNumCommon(byte *pVal, byte radix)
{
	byte val;
	do {
		if (!SpecialUARTRB((char *)(&val))) {	//get digit
			return FALSE;						//timeout
		}
		UsrSendByte((char)val);				//echo
		if ('X' == ToUpper(val)) {
			return FALSE;
		}
		if (FALSE == UsrChkDigit(&val, radix)) {
			continue;
		}
	} while (FALSE);
	*pVal = val;
	return TRUE;
}
//
//	Get a 1 digit from the Serial Port.
//	Also validates that digit is in range
//
BOOL UsrGetOneNum(byte nMax, byte *pVal)
{
	byte val;
	UARTPrintfProgStr(MsgPszOneNumericCommand);
	UsrSendByte('0' + nMax);
	UsrSendByte(')');
	UsrSendByte(':');
	if (UsrGetOneNumCommon(&val, 10) && (val <= nMax)) {
		*pVal = val;
		return TRUE;
	}
	return FALSE;
}
//
//	get a 2 digit number from the Serial Port
//
BOOL UsrGetTwoNum(PGM_P pMsg, byte *pVal)
{
	byte hval, lval;
	UARTPrintfProgStr(MsgPsz2Digit);
	UARTPrintfProgStr(pMsg);
	if (UsrGetOneNumCommon(&hval, 10) && UsrGetOneNumCommon(&lval, 10)) {
		*pVal = (hval * 10) + lval;
		return TRUE;
	}
	return FALSE;
}
//
//	get a 4 digit hex number from the Serial Port
//
BOOL UsrGetFourHexNum(PGM_P pMsg, uint *pVal)
{
	byte hhval, hlval, lhval, llval;
	UARTPrintfProgStr(pMsg);
	if (UsrGetOneNumCommon(&hhval, 16) && UsrGetOneNumCommon(&hlval, 16) &&
		UsrGetOneNumCommon(&lhval, 16) && UsrGetOneNumCommon(&llval, 16)) {
		*pVal = (((hhval << 4) + hlval) << 8) | ((lhval << 4) + llval);
		return TRUE;
	}
	return FALSE;
}

BOOL UsrShow(void)
{
	if (EEConfigData.Verbose) {
		RTCReadTime(FALSE);			//update Time & Date
		RTCReadDate();
		SetDisplay();
	}
	ShowClockVersion();			//status will be printed on exit
	return TRUE;
}

BOOL UsrSetIOCTL(void)
{
	uint w;
	if (UsrGetFourHexNum(MsgPszIOCTLCommand, &w)) {
		return UpdateIOCTL(w);
	}
	return FALSE;
}

BOOL UsrSetTime(void)
{
	byte val;
	if (UsrGetTwoNum(MsgPszHourCommand, &val)) {
		if (AVRSetHours(val) == val) {
			UsrPrintNewLine();
			if (UsrGetTwoNum(MsgPszMinCommand, &val)) {
				if (AVRSetMins(val) == val) {
					UsrPrintNewLine();
					if (UsrGetTwoNum(MsgPszSecCommand, &val)) {
						if (AVRSetSecs(val) == val) {
							RTCSetTime();		//update the RTC with these new values
//
// check if the new time is in a DST window. If so, set the proper bitvector flag
// but do not apply a DST correction.
//
							DSTRetroNotApply();
							SetDisplay();		//show the new time properly
							return TRUE;
						}
					}
				}
			}
		}
	}
	RTCReadTime(FALSE);							//restore from RTC
	return FALSE;
}

BOOL UsrSetDate(void)
{
	byte val;
	if (UsrGetTwoNum(MsgPszYearCommand, &val)) {
		if (AVRSetYear(val) == val) {
			UsrPrintNewLine();
			if (UsrGetTwoNum(MsgPszMonthCommand, &val)) {
				if (AVRSetMonth(val) == val) {
					UsrPrintNewLine();
					if (UsrGetTwoNum(MsgPszDayCommand, &val)) {
						if (AVRSetDay(val) == val) {
							RTCSetDate();			//update the RTC with these new values
//
// check if the new time is in a DST window. If so, set the proper bitvector flag
//
							DSTRetroApply();		//will update RTC if doing DST fix.
							SetDisplay();			//show the new time properly
							return TRUE;
						}
					}
				}
			}
		}
	}
	UsrPrintNewLine();
	RTCReadDate();									//restore from RTC

	return FALSE;
}

BOOL UsrShowCopyright(void)
{
	UARTPrintfProgStr(MsgPszCopyright);
	return TRUE;
}

BOOL UsrResetEEprom(void)
{
	ClearEEConfiguration();
	InitiateSysReset();
	return TRUE;
}

//
// Space Saving Suggestion:
//	Allocate a static SRam buffer with status fields. These fields
//	need to be dynamically updated as they change.
//
void UsrShowStatus(void)
{
	char LBuf[75], *p;
	p = FormatAVRDate(LBuf);
	p = UsrInsertSpace(p);
	p = FormatAVRTime(p);
	p = UsrInsertSpace(p);
	p = strcpy_P(p, (void *)pgm_read_word(&WeekDays[DayOfTheWeek(&AVRTimeDateBlk)])) + strlen(p);
	p = UsrInsertSpace(p);
//
// display 'L' if Led is on
//
	if (!EEConfigData.LedOption != LEDDISABLED) {
		p = UsrShowNumericByteStatus(p, 'L', EEConfigData.LedOption);
	}
//
// display 'V' if verbose mode
//
	if (EEConfigData.Verbose) {
		*p++ = 'V';
	}
//
// display 'U' or 'E' if Daylight Saving Time is on
//
	if (DSTUS == EEConfigData.DSTMode) {
		*p++ = 'U';
	} else if (DSTEU == EEConfigData.DSTMode) {
		*p++ = 'E';
	}
//
// display 'K' if Chrono Display is on
//
	if (EEConfigData.ChronoEnabled) {
		*p++ = 'K';
	}
//
// display 'B' + val if BurninPrevention on
//
	if (EEConfigData.BurninVal != BURNINOFFVAL) {
		p = UsrShowNumericByteStatus(p, 'B', (EEConfigData.BurninVal + 5) % 10);
	}
#if DEBUG		// only output command line available if GPS input enabled
//
// display 'G' if GPS Input is on
//
	if (EEConfigData.GPSInEnabled) {
		p = UsrShowNumericByteStatus(p, 'G', EEConfigData.GPSOffset);	//add local time zone offset
	}	
#endif
//
// display 'F' if any Fun Mode is enabled
//
	if (FUNNORM != CurrPlayMode) {
		*p++ = 'F';
	}
//
// display 'C' if Calibration is on
//
	if (CurrAppMode == APPCALIBRATE) { 
		*p++ = 'C';
	}
	p = UsrInsertSpace(p);
//
// display DST bitvectors
//
	p = DstReportVectors(p);
//
// display 'M' Trigger Mode
//
	p = UsrShowNumericByteStatus(p, 'M', EEConfigData.PPSTrigger);
//
// Display IOCTL word
//
	p = UsrShowNumericWordStatus(p, 'I', EEConfigData.ioctl);
//
	p = MsgAddNewLine(p);
	ASSERT(*p == 0);
	ASSERT(strlen(LBuf) <75);
	UARTPrintfStr(LBuf);
}

void UsrShowAll(void)
{
	ShowClockVersion();
	UsrPrintNewLine();
	UsrShowStatus();
}
//
// UsrGetName
//
// Request 20 characters, or 2 consecutive spaces or CR or LF
//	Immediate CR or LF means "toggle username display"
//
BOOL UsrGetName(void)
{
	char ch, *pBuf;
	byte charCnt, anyCharCnt;
	BOOL fBlank;
	UARTPrintfProgStr(MsgPszGetName);
	charCnt = anyCharCnt = 0;
	fBlank = FALSE;
	pBuf = UsrNameBuf;
	do {
		if (!SpecialUARTRB(&ch)) {		//get char
			RestoreUsrName();			//timeout
			return FALSE;
		}
		UsrSendByte(ch);				//echo
		charCnt++;
		if ((ch == CR) | (ch == LF)) {	// CR or LF will terminate the string
			break;
		}
		++anyCharCnt;
		*pBuf++ = ch;
		if (' ' == ch) {
			if (fBlank) {				//second blank?
				break;					//yes, done
			} else {					//no set blank flag
				fBlank = TRUE;			//first blank found
			}
		} else {						//not a special char	
			fBlank = FALSE;
		}
	} while (charCnt < 20);
	if ((ch != CR) & (ch != LF)) {
//
// remove trailing spaces
//
		while (*--pBuf == ' ') {
			charCnt--;
		}
		++pBuf;							//terminate
	}
	if (0 == anyCharCnt) {				//any input at all
//
// no input. Toggle username display. Don't touch UsrNameBuf
//
		if (EEConfigData.UsrNameVal == USRNAMEON) {
			UpdateUsrNameMode(USRNAMEOFF);	//toggle
		} else {
			UpdateUsrNameMode(USRNAMEON);	//toggle
		}
		return TRUE;					//done
	}
	*pBuf = 0;
	EEConfigData.UsrNameCnt = charCnt;	//save count
	UsrNameDflt = FALSE;				//not default any more
	SaveUsrName();						//write to EEProm and update display
	return TRUE;
}

BOOL UsrChangeApp(void)
{
	byte val;
	if (!UsrGetOneNum(APPLAST - APPCLOCK, &val)) {
		return FALSE;
	}
	val += APPCLOCK;					//base
//
// Did we change application? We are running Clock or Calibrate
//
	ASSERT((CurrAppMode == APPCLOCK) || (CurrAppMode == APPCALIBRATE));
//
// code duplication from CMenuSaveAppVal(void)
//
	if (val != CurrAppMode) {
		CurrAppMode = val;
		// Only Term mode saved to EEprom
		// (no change from Clock to Clock, Clock to Cal, Cal to Clock, 
		// From Terminal to Calibrate using Menu, then to Clock using UsrChangeApp(). Need to save Clock too.
		if ((CurrAppMode == APPTERM) || (CurrAppMode == APPCLOCK)) {
			UpdateEEprom(EEPROM_STARTAPP, CurrAppMode - APPCLOCK);
		}
		//	change application/ do special function.
		Flags.AppChanged = TRUE;
	}
	return TRUE;
}

char MsgPszSignOn[] PROGMEM =		" AVR Oscilloscope Clock ";
char MsgPszOneNumericCommand[] PROGMEM = "1 digit value (0-";
char MsgPsz2Digit[] PROGMEM =		"2 digit ";
char MsgPszHourCommand[] PROGMEM =	"hrs: ";
char MsgPszMinCommand[] PROGMEM = 	"mins: ";
char MsgPszSecCommand[] PROGMEM =	"secs: ";
char MsgPszYearCommand[] PROGMEM =	"year: ";
char MsgPszMonthCommand[] PROGMEM =	"month: ";
char MsgPszDayCommand[] PROGMEM =	"day: ";
char MsgPszCancel[] PROGMEM =		"Cancel/Timeout";
char MsgPszGetName[] PROGMEM =		"Name ";
char MsgPszCopyright[] PROGMEM =	"(C)2010 Jan de Rie";
char MsgPszNewLine[] PROGMEM =		{CR,LF,0};
char MsgPszIOCTLCommand[] PROGMEM = "4 hex digit IOCTL: ";

char MsgPszDSTApplied[] PROGMEM = 	"DST fix ";

char UsrHelpText[] PROGMEM = 
		" Help:\r\n"\
		"A: Application Selection (0..5)\r\n"\
		"B: Burn-in prevention\r\n"\
		"C: Alarm on/off\r\n"\
		"D: Change Date\r\n"\
		"F: Clock Face Mode (0..5)\r\n"\
		"G: GPS Input\r\n"\
		"I: IOCTL\r\n"\
		"K: Chrono Display\r\n"\
		"L: LED blinking\r\n"\
		"M: PPS Trigger Mode\r\n"\
		"N: Numeric Display Mode (0..5)\r\n"\
		"P: Play Mode (0..3)\r\n"\
		"R: Reset EEProm Config\r\n"\
		"S: Display Status\r\n"\
		"T: Change Time\r\n"\
		"U: Change DST mode\r\n"\
		"V: Verbose Output\r\n"\
		"W: Change Name";

char MsgPszCommand[] PROGMEM  = "Command(H for help):";

char MsgPszLogo[] PROGMEM =		"Dutchtronix";
#if ds1307==1
char MsgPszAltLogo[] PROGMEM =	"Sparkfun";
#endif
#if MajorVersion==4
#if MinorVersion==0
#if ds1307==1
char MsgPszVersion[] PROGMEM = "V4.0ds";
#else
char MsgPszVersion[] PROGMEM = "V4.0";
#endif
#else
	UNSUPPORTED
#endif
#else
	UNSUPPORTED
#endif
