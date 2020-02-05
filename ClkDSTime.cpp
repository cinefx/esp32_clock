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
#include "./ClkSupport.h"
#include "./ClkISR.h"
//
// after reading time and date from RTC, get DSTForward and DSTBack for current year
//
// DST jump will be missed if clock is not running at transition.
// Flags to check at startup time:
// Bitvector for 7 years: was forward DST applied, was backward DST applied.
// If DSTYear and date/time window and Forward flag not set  >= Forward date/time, inc hr, may overflow date!
// If DSTYear and after date/time window and Forward flag set and Backward flag NOT set ->
//	back one hour. May set date back!
// flags cleared if date before Forward date
// Year change-> redo DST data
//
// Source: http://webexhibits.org/daylightsaving/b.html
// Consistency check: if date/time is prewindow and DST Forw was applied?
// If time is set and we are INSIDE a DST window, mark DST Forward as set
// It time is set and we are AFTER a DST window, mark DST Backward as set
//
// Regular DST correction is kind of useless when using NMEA data to set the clock
//
// Time was changed by the user (Menu or Serial Interface).
// Do NOT apply a DST correction (if applicable) but mark the bitvectors
// that a correction was applied (if applicable).
// The theory here is that the user knows what he or she is doing.
//

//
// Daylight Saving Time Tables
//
// Source: http://webexhibits.org/daylightsaving/b.html
//
// Supported years: 2008-2014 (7 years. Can't do 8 years with bitvectors in EEProm)
// Data for 2008 (index 0) to 2014 (index 6).
// High nibble is date in March.
// Low nibble is date in November (US) or October (EU)
// EU dates are base 25
//
byte DSTUSData[8] PROGMEM = {9*16+2, 8*16+1, 14*16+7, 13*16+6, 11*16+4, 10*16+3, 9*16+2, 0};
byte DSTEUData[8] PROGMEM = {5*16+1, 4*16+0,  3*16+6,  2*16+5,  0*16+3,  6*16+2, 5*16+1, 0};

extern void DSTReportAction(void);

#define FORWARD 1
#define BACKWARD 2

void DSTRetro(BOOL fAllowDSTFix);
void DSTRetroApply(void);
void DSTRetroNotApply(void);
BOOL DSTInWindow(TimeDate_t *p);
BOOL DSTGPSInWindow(void);
void DSTInit(TimeDate_t *p);
BOOL DSTWasApplied(TimeDate_t *p, byte fDirection);
BOOL DSTMarkApplied(TimeDate_t *p, byte fDirection);
BOOL DSTPostWindow(TimeDate_t *p);
BOOL DSTPostWindow(TimeDate_t *p);

extern byte _BVMap[8] PROGMEM;

//
// DSTMakeMask --
//	Build the bit mask corresponsing to the normalized year
//
// IN:	normalized year
// OUT:	bitvector mask
//
inline byte DSTMakeMask(byte yr)
{
	ASSERT(yr < 8);
	byte msk;
	msk = pgm_read_byte(&_BVMap[yr]);
	return msk;
}

void DSTRetroNotApply(void)
{
	DSTRetro(FALSE);
}
//
// DSTRetroApply--
//	After the clock starts up, or after the user changed the date, check
//	if the new date requires a DST fixup.
//
void DSTRetroApply(void)
{
	DSTRetro(TRUE);
}

void DSTRetro(BOOL fAllowDSTFix)
{
//
//initialize Daylight Saving Time variables
//
	DSTInit(&AVRTimeDateBlk);
	if (DSTInWindow(&AVRTimeDateBlk) == TRUE) {
		//
		// in window
		// Time should have been advanced. Was it done?
		//	
		if (DSTWasApplied(&AVRTimeDateBlk, FORWARD)) {		//applied to current year?
			return;
		}
		//
		// Advance time one hour. May change date. Cannot change year.
		// Same issue with GPS input if time is in the last hr of the window.
		// time will be advanced one hour (but only once), then set back at the next minute.
		//
		if (fAllowDSTFix) {								//Fix allowed?
			AVRAdvanceTimeOneHr();
		}
		DSTMarkApplied(&AVRTimeDateBlk, FORWARD);
		//
		// Time may have advanced beyond the DST window (if we were in the last hour of the window)
		// and we were allowed to apply a fix. Check again to see if we have to apply a backward
		// fix. Recursion is ok here since the Forward bitvector was set, implying we won't do
		// another forward fix (and check again etc)
		//
		DSTRetro(TRUE);					//recurse to see if we advanced beyond the DST window.
	} else {
		//
		// not in window
		// check if the clock is after the DST window and DST Forward was applied
		//
		if (DSTPostWindow(&AVRTimeDateBlk) == FALSE) {
			return;
		}
		if (DSTWasApplied(&AVRTimeDateBlk, FORWARD) == FALSE) {		//applied to current year?
			return;													//No, done
		}
		//
		// Time should have been put back. Was it done?
		//	
		if (DSTWasApplied(&AVRTimeDateBlk, BACKWARD) == TRUE) {		//applied to current year?
			return;													//Yes, done
		}
		//
		//	put back time one hour. May change date. Cannot change year
		//
		if (fAllowDSTFix) {								//Fix allowed?
			AVRRetreatTimeOneHr();
			DSTMarkApplied(&AVRTimeDateBlk, BACKWARD);
		}
	}
}
//
// Check is DST is enabled and if the year in the time block
//	pointed to by arg is a supported year
//
// Supported year: 2008-2014 (7 years. Can't do 8 years with bitvectors in EEProm)
//
//		out - normalized year (base 2008).  0xff means non-supported year
//
byte DSTPossible(TimeDate_t *p)
{
	if ((EEConfigData.DSTMode != DSTNONE) &&		//OFF Value
		((p->Year >= 8) && (p->Year <= 14))) {		//supported year?
		return p->Year - 8;							//normalized year
	}
	return 0xff;
}
//
//
inline void ClrDSTVars(void)
{
	DSTForwardHr = 0;
	DSTForwardDay = 0;
	DSTBackDay = 0;
	DSTBackMonth = 0;
}
//
//
void DSTInit(TimeDate_t *p)
{
	ClrDSTVars();
	byte yr = DSTPossible(p);
	if (yr == 0xff) {
		return;
	}
	if (EEConfigData.DSTMode == DSTUS) {
		//
		// US:
		// DST Forward Month is March, DST Forward Time is 02:00. If match, set time to 3:00
		// DST Back Month is November, DST Back Time is 02:00. If match, set to 1:00 (but only once)
		// Mark jump back in EEprom. Clear this flag at 02:01
		//
		DSTForwardHr = DSTUSFORWARDHR;
		DSTBackMonth = DSTUSBACKMONTH;
		byte v = pgm_read_byte(DSTUSData + yr);
		DSTForwardDay = v >> 4;					//start date
		DSTBackDay = (v & 0x0f);
	} else if (EEConfigData.DSTMode == DSTEU) {
		//
		// E.U.
		// DST Forward Month is March, DST Forward Time is 01:00. If match, set time to 2:00
		// DST Back Month is November, DST Back Time is 02:00. If match, set to 1:00 (but only once)
		// Mark jump back in EEprom. Clear this flag at 02:01
		//
		DSTForwardHr = 1;
		DSTBackMonth = DSTEUBACKMONTH;
		byte v = pgm_read_byte(DSTEUData + yr);
		DSTForwardDay = (v >> 4) + 25;					//start date
		DSTBackDay = (v & 0x0f) + 25;
	}
}
//
// DSTWasApplied
//	Was Daylight Savings Time Forward/Backward applied to the year in the time block
//	pointed to by arg ptr
//
BOOL DSTWasApplied(TimeDate_t *p, byte fDirection)
{
	byte yr = DSTPossible(p); 						//returns year (normalized)
	if (yr == 0xff) {
		return FALSE;
	}
	byte v;
	if (fDirection == FORWARD) {
		v = EEConfigData.DSTFYears;					//flags bitvector
	} else {
		ASSERT(fDirection == BACKWARD);
		v = EEConfigData.DSTBYears;					//flags bitvector	
	}
	while (yr) {
		--yr;
		v = v >> 1;
	}
	return (BOOL)(v & 0x01);
}
//
// DSTFMarkApplied:
// Apply Daylight Savings Time Forward/Backward Mark to the year in the time block
//	pointed to by arg ptr 
//
BOOL DSTMarkApplied(TimeDate_t *p, byte fDirection)
{
	byte yr = DSTPossible(p);	 //returns year (normalized)
	if (yr == 0xff) {
		return FALSE;
	}
	byte msk = DSTMakeMask(yr);
	if (fDirection == FORWARD) {
		EEConfigData.DSTFYears |= msk;	//update Forward flags bitvector	
		UpdateEEprom(EEPROM_DSTFVEC, EEConfigData.DSTFYears);
	} else {
		ASSERT(fDirection == BACKWARD);
		EEConfigData.DSTBYears |= msk;	//update Forward flags bitvector	
		UpdateEEprom(EEPROM_DSTBVEC, EEConfigData.DSTBYears);
	}
	DSTReportAction();					//not completely correct since we may set the Mark
										//without applying the fix (User Time Change)
	return TRUE;
}
//
// DSTGPSInWindow --
//	Check if the UTC Year, Month and Day are in the DST window
//
// Need to re-init to make sure proper data are used. Since
// the UTC date will become the AVR date (if different) this is ok.
//
// Also, return FALSE if we are in the last hour of the window since
// we do NOT need a correction in that case
//
// OUT: FALSE: out of range	TRUE: in range
//
BOOL DSTGPSInWindow(void)
{
	DSTInit(&UTCTimeDateBlk);
	
	if (DSTInWindow(&UTCTimeDateBlk) == FALSE) {
		return FALSE;
	}
//
// inside the window. Check for last hr.
//
	if (GPSLastHrFlag == FALSE) { 	//were we in the last hour?
		return TRUE;
	}
//
// We're going to return FALSE (not in window) even though we
// actually are in the last hour of the window. This is fine
// for GPS purposes (we don't want to advance the time outside of
// the DST window) but as a result, the Backward bitvector won't be
// set. Time will be corrected at 2:00AM (set to 1:00) and the
// backward bitvector will be set then, but time will be off until corrected
// by GPS (or the user)
// To prevent this we need to set the Backward bitvector now (just once
// of course), under the assumption that the new GPS time will
// be propagated to the AVR.
// Note that this is only useful if we turn on GPS (or the clock) while
// in the last hour of the window. If we transition into it, the clock
// will fix at 2:00 AM back to 1:00AM, equal to GPS time
//
// This code also assumes that the DST sub-system was initialized
// with UTC data, which we did above.
//	
	if (DSTWasApplied(&UTCTimeDateBlk, BACKWARD) == FALSE) {		//was B flag applied?
		DSTMarkApplied(&UTCTimeDateBlk, BACKWARD);					//set B flag
	}
	return FALSE;
}
//
// DSTInWindow --
//	Check if the Year, Month and Day in the time block
//	pointed to by arg ptr are in the DST window
//
//	out: FALSE - out of range
//		 TRUE  - in range
//
BOOL DSTInWindow(TimeDate_t *p)
{
	byte yr = DSTPossible(p);
	if (yr == 0xff) {
		return FALSE;
	}
	if (p->Month < DSTFORWARDMONTH) {
		return FALSE;
	}
	if (p->Month == DSTFORWARDMONTH) {
		//
		// First Month in window. Check Day
		//
		if (p->Day < DSTForwardDay) {
			return FALSE;
		}
		if (p->Day != DSTForwardDay) {
			return TRUE;
		}
		//
		// First day of window. Are we after the transistion hr
		//
		if (p->Hrs >= DSTForwardHr) {
			return TRUE;
		}
		return FALSE;			//no but first day so we're done
	}
	if (p->Month < DSTBackMonth) {
		return TRUE;		//in range, return true (no carry)
	}
	if (p->Month != DSTBackMonth) {
		return FALSE;		//not last month, then out of range
	}
	//
	// Last month of window. Check Day
	//
	if (p->Day < DSTBackDay) {
		return TRUE;		//in range, return true (no carry)
	}
	if (p->Day != DSTBackDay) {
		return FALSE;		//not last day, then out of range
	}
	//
	// last day of window. Are we BEFORE the transition hr
	//
	if (p->Hrs >= DSTBACKHOUR) {
		return FALSE;		//last day so we're done
	}
	//
	// last day of window, before transition hr
	//
	// Are we in the LAST HOUR before the transition hr.
	// This is relevant for the GPS parsing/DST combination
	// since in that last hour, we should NOT advance time 1 hr
	//
	GPSLastHrFlag = FALSE;					//clear special flag
	if (p->Hrs >= DSTBACKHOUR - 1) {		//are we inside the last hour window?
		//
		// Inside the LAST HOUR of the window.
		//
		GPSLastHrFlag = TRUE;				//set special flag
	}
	return TRUE;							// Month and Day in window
}
//
// DSTPostWindow --
//	Check if the Year, Month and Day in the time block
//	pointed to by arg ptr are post the DST windows
//
// OUT: CS out of range	CC in range
//	out: FALSE - out of range
//		 TRUE  - in range
//
BOOL DSTPostWindow(TimeDate_t *p)
{
	byte yr = DSTPossible(p);
	if (yr == 0xff) {
		return FALSE;
	}
	if (DSTBackMonth < p->Month) {
		return TRUE;			//post range, return true
	}
	if (DSTBackMonth != p->Month) {
		return FALSE;			//not last month, then out of range
	}
	//
	// Last month of window. Check Day
	//
	if (DSTBackDay < p->Day) {
		return TRUE;			//post range, return true
	}
	if (DSTBackDay != p->Day) {
		return FALSE;			//not last day, then out of range
	}
	//
	// last day of window. Are we after the transition hr
	//
	if (p->Hrs >= DSTBACKHOUR) {
		return TRUE;			//yes, return true (no carry)
	}
	return FALSE;
}
