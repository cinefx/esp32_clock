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
#include "./ClkFlash.h"
#include "./ClkISR.h"
#include "./ClkSupport.h"

extern void SetBaudVal(byte newBaudVal);
extern BOOL TestPlayModes(void);
extern BOOL DSTGPSInWindow(void);
extern void DSTRetroNotApply(void);
extern void SpecialUARTSB(char ch);
extern void UsrPrintNewLine(void);

char NMEA_GPRMC[] PROGMEM = "$GPRMC,";

void ProcessGPSRecord(void);

void GPSStartFlashing(byte cnt, BOOL fSolid)
{
	//
	// Helpscreen timer interferes with Flags.GPSOKToUpdate. Skip if solid "G" message still active
	//
	if (NULL == CheckFlashItem(ScanTblStatG, (VectorTblEntry_t*)CharG)) {
		FlashTblEntry_t* pFlash;
		ASSERT(ScanTblStatG->pVect == NULL);
		if (fSolid) {
			ScanTblStatG->pVect = (VectorTblEntry_t*)CharG;
		}
		pFlash = AddFlashItem(ScanTblStatG, cnt, (VectorTblEntry_t*)CharG);
		if (fSolid) {
			pFlash->pVectOrig = NULL;					//make sure CharG is removed when done flashing
		}
	}
}
//
// Locate a substr in the GPS NMEA record,
// The record is terminated by 0 (guaranteed)
//
char *GPSInBufLocate(char *pBuf, byte field)
{
	char *p = pBuf;
#if DEBUG
	byte cnt = 0;
#endif
	char ch;
	++field;					//it's 0 based
	while (--field) {
		do {
			ch = *p++;
			if (0 == ch) return pBuf;	//end of record. Return original buffer ptr. to prevent NULL ptr references
#if DEBUG
			if (++cnt >= GPSINBUFLENGTH) return pBuf;	//return original buffer ptr. to prevent NULL ptr references
#endif
		} while (ch != ',');
	}
	return p;
}	
//
//	Compare SRam based string with Flash based String
//	',' is termination char
//
BOOL StrCmp_S_F(char *p1, PGM_P p2)
{
	char ch1, ch2;
	do {
		ch1 = *p1++;
		ch2 = pgm_read_byte(p2++);
		if (ch1 != ch2) return FALSE;
	} while (ch1 != ',');
	return TRUE;
}

#if EXTDEBUG
//
// Display Current Date and Time based on UTC Variables
// Create the text in the Transmit Buffer, then send
//
// Buffer overflow checked at code time (fixed format)
//
char MsgPszGPS[] PROGMEM = "GPS: ";

void DisplayUTCDateTime(char *pNMEA)
{
	char LBuf[32];				//watch stack use
	char *p = LBuf;
	byte l;
	UARTPrintfProgStr(MsgPszGPS);
	p = FormatUTCDate(p);
	p[0] = ' ';
	p = FormatUTCTime(p+1);
	p[0] = ' ';
	if (GPSUpdateRTCTime) {
		*p++ = 'T';
	}
	if (GPSUpdateRTCDate) {
		*p++ = 'D';
	}
	*p++ = 0;
	l = strlen(LBuf);
	ASSERT(l <64);
	for (byte i = 0; i < l; ++i) {
		SpecialUARTSB(LBuf[i]);
	}
#if EXTDEBUG
	SpecialUARTSB(' ');
	SpecialUARTSB('(');
	p = pNMEA;
	while ((*p != CR) && (*p != LF) && (*p != 0)) {
		++p;
	}
	*p = 0;
	UARTPrintfStr(pNMEA);
	SpecialUARTSB(')');
#endif
	UsrPrintNewLine();
}
#endif
//
// convert 2 digit ascii from p1
// buffer may be inconsistent (e.g. when serial in overflow occurs)
//
byte GPSGetBinVal(char *p1)
{
	byte v = ((p1[0] - '0') * 10);
	v += (p1[1] - '0');
	if (v >= 60) {
		v = 1;
	}
	return v;
}
//
//
BOOL UpdateGPSSetting(byte newGPSVal)
{
	if (newGPSVal != NOGPS) {
		// Desired value is On. Compute desired local time zone offset
		byte ofs = newGPSVal - (NOGPS+13);
		if (EEConfigData.GPSInEnabled && (ofs == EEConfigData.GPSOffset)) {
			//Was enabled and time offset identical: no change
			return FALSE;
		}
		EEConfigData.GPSOffset = ofs;
		EEConfigData.GPSInEnabled = TRUE;
		EEConfigData.PreGPSBaudVal = EEConfigData.BaudVal;				//save current 0-based internal code
		SetBaudVal(BAUDVAL4800 - BAUDVALFIRST);
	} else {
		// MenuData.GPSVal == NOGPS
		if (!EEConfigData.GPSInEnabled) {
			//Actual value was off, done
			return FALSE;
		}
		EEConfigData.GPSOffset = 0;
		EEConfigData.GPSInEnabled = FALSE;
		SetBaudVal(EEConfigData.PreGPSBaudVal);							//is 0-based internal code
	}
	UpdateEEprom(EEPROM_GPSMODE, EEConfigData.GPSInEnabled);
	UpdateEEprom(EEPROM_GPSOFFSET, EEConfigData.GPSOffset);
	UpdateEEprom(EEPROM_BAUDVAL, EEConfigData.BaudVal);
	UpdateEEprom(EEPROM_PREGPSBAUDVAL, EEConfigData.PreGPSBaudVal);
	return TRUE;
}
//
void DiscardGPSInput(void)
{
	if (!Flags.RXLFDetected) {
		return;
	}
	ASSERT(EEConfigData.GPSInEnabled);
	UARTFlushRecord();
	Flags.RXLFDetected = FALSE;
}
//
// While in normal Clock mode, process incoming GPS records
//
void CheckGPSInput(void)
{
//
// Check if a LF was detected in the UART input buffering.
// This method requires that the input buffer is large enough
// to hold a complete NMEA record
//
	if (!Flags.RXLFDetected) {						//complete buffer received?
		return;
	}

	ASSERT(EEConfigData.GPSInEnabled);			//are we processing GPS info
	if (!Flags.GPSOKToUpdate || TestPlayModes()) {	//time to process GPS record?
		DiscardGPSInput();
	} else {
		ProcessGPSRecord();
	}
}

#if EXTDEBUG
char m1[] PROGMEM = "Year\r\n";
char m2[] PROGMEM = "Month\r\n";
char m3[] PROGMEM = "Day\r\n";
char m4[] PROGMEM = "Hour\r\n";
char m5[] PROGMEM = "Min\r\n";
char m6[] PROGMEM = "Sec ";
void Special_UARTPfu08dec(byte b);
#endif

void ProcessGPSRecord(void)
{
	char GPSInBuf[GPSINBUFLENGTH];
//
// new records may come in while we copy the current record
//
	Flags.RXLFDetected = FALSE;												//clear record complete flag
	if (UARTReceiveLine(GPSInBuf, GPSINBUFLENGTH-1) > (GPSINBUFLENGTH-1)) {
		//copy failed due to buffer overflow.
		return;
	}
//
// Parse GPS line. This is just as good a time as any
// Look for "$GPRMC" record
//
	if (StrCmp_S_F(GPSInBuf, NMEA_GPRMC) == FALSE) {					//compare Sram and Flash based string
		return;
	}
//
// found "$GPRMC" record
//
	char *p;
	p = GPSInBufLocate(GPSInBuf, GPSStatusField);						//locate Status substr
	if (*p != 'A') {													//check for valid time flag
		return;
	}
//
// 'A' flag found. We have a valid record. Show this on the clock
//  Start timer for next record.
//
	Flags.GPSOKToUpdate = FALSE;										//do not process GPS record
	SetTC1Countdown(4);													//Timer shared with Demo/Menu/Helpscreen Mode
//
// get UTC time
//
	p = GPSInBufLocate(GPSInBuf, GPSTimeField);			//locate Time substr
	UTCTimeDateBlk.Hrs = GPSGetBinVal(p);
	p += 2;
	UTCTimeDateBlk.Mins = GPSGetBinVal(p);
	p += 2;
	UTCTimeDateBlk.Secs = GPSGetBinVal(p);
//
// get UTC date
//
	p = GPSInBufLocate(GPSInBuf, GPSDateField);			//locate Date substr
	UTCTimeDateBlk.Day = GPSGetBinVal(p);
	p += 2;
	UTCTimeDateBlk.Month = GPSGetBinVal(p);
	p += 2;
	UTCTimeDateBlk.Year = GPSGetBinVal(p);
//
// Convert UTC time to local time.
// The time zone offset needs to be entered by the user.
// If Automatic Daylight Saving Time (DST) is active, the
// clock will do the correction (for the supported years)
//
// We only support Hour offsets.
//
// Note that this conversion may include a change to the current date,
// including possibly the current year, which in turn may affect the
// DST computations. Since this year change can only happen around
// 12/31 - 1/1, DST will not be active.
//
	UTCTimeDateBlk.Hrs += EEConfigData.GPSOffset;		//UTCHrs was time from incoming NMEA record
	// Could use signed chars here
	if (UTCTimeDateBlk.Hrs >127) {	//underflow
		UTCTimeDateBlk.Hrs += 24;
		UTCDecrementDate();
	} else if (UTCTimeDateBlk.Hrs >= 24) {
		UTCTimeDateBlk.Hrs -= 24;
		UTCIncrementDate();
	}
//
// Now check if a Daylight Saving Time adjustment of 1 hour needs to be applied
// to the UTC time/date variables
//
	if (DSTGPSInWindow() == TRUE) {		//if in DST range, advance time 1 hr
		UTCAdvanceTimeOneHr();
	}

#if DEBUG
	GPSUpdateRTCDate = GPSUpdateRTCTime = FALSE;
#endif
	if (memcmp(&UTCTimeDateBlk, &AVRTimeDateBlk, sizeof(TimeDate_t)) != 0) {
#if EXTDEBUG
		if (UTCTimeDateBlk.Year != AVRTimeDateBlk.Year) UARTPrintfProgStr(m1);
		if (UTCTimeDateBlk.Month != AVRTimeDateBlk.Month) UARTPrintfProgStr(m2);
		if (UTCTimeDateBlk.Day != AVRTimeDateBlk.Day) UARTPrintfProgStr(m3);
		if (UTCTimeDateBlk.Hrs != AVRTimeDateBlk.Hrs) UARTPrintfProgStr(m4);
		if (UTCTimeDateBlk.Mins != AVRTimeDateBlk.Mins) UARTPrintfProgStr(m5);
		if (UTCTimeDateBlk.Secs != AVRTimeDateBlk.Secs) {
// Biggest problem is that seconds transition and GPS time are not synchronized
// would need to reprogram the RTC. Of course by the time we are here, finding any
// difference, the actual seconds transition happened way earlier.
			Special_UARTPfu08dec(AVRTimeDateBlk.Secs);
			SpecialUARTSB(':');
			Special_UARTPfu08dec(TimeTicks);
			UARTPrintfProgStr(m6);
		}
#endif
//
// Optional improvement if time differs in seconds only.
//		look at TimeTicks which tracks the current second in 1/200 increments.
//		If the AvrTime is within 90%
//		if UTCsecs == AvrSecs+1 and (TimeTicks > 180) no adjustment
//		if UTCsecs == AvrSecs-1 and (TimeTicks < 20) no adjustment
//
		AVRTimeDateBlk = UTCTimeDateBlk;
		RTCSetDate();
		RTCSetTime();
		Flags.DisplayRedraw = TRUE;
#if DEBUG
		GPSUpdateRTCDate = 	GPSUpdateRTCTime = TRUE;
#endif
	}
#if EXTDEBUG
//
// send UTC info to serial port. Only available in Extended Debug Mode
//
	DisplayUTCDateTime(GPSInBuf);
	GPSUpdateRTCDate = 	GPSUpdateRTCTime = 0;
#endif
//
// Using the Flags.DisplayRedraw is somewhat flawed
// since it won't get cleared until the next seconds transition.
// We shouldn't get any interesting NMEA records until then though
//
	if (Flags.DisplayRedraw) {	//Date or Time updated?
		// Inform the DST subsystem
		DSTRetroNotApply();				//no fix but set bitvector flags
		GPSStartFlashing(FLASHPERSEC*5, FALSE);
	} else {
		//
		// No change in Time/Date. Just show we received a valid NMEA record by
		// showing a non-flashing G for 1 second.
		// Set the "G" in the warning field (Use the Flash system in non-flashing mode)
		// If we update the time or date, this same field will be flashing. Just
		// skip in that case.
		// Use the LED control () to bypass this "valid record received" indicator.
		//
		if (EEConfigData.LedOption != LEDDISABLED) {				//if Led Disabled, bypass "G" indicator
			GPSStartFlashing(1*FLASHPERSEC, TRUE);
		}
	}
}
