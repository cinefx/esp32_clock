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
#include <avr/io.h>
#include <stdio.h>
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

extern byte bintobcd(byte val);
extern void BatteryWarning(void);
extern BOOL TestPlayModes(void);

#include "i2cmaster.h"

#define	DS1307ID		0b11010000
#define	PCF8563ID		0b10100010

#if ds1307
#define	RTCID		DS1307ID
#else
#define	RTCID		PCF8563ID
#endif

char MonthsText[12][4] PROGMEM = {
	{'J', 'a', 'n', 31},
	{'F', 'e', 'b', 28},
	{'M', 'a', 'r', 31},
	{'A', 'p', 'r', 30},
	{'M', 'a', 'y', 31},
	{'J', 'u', 'n', 30},
	{'J', 'u', 'l', 31},
	{'A', 'u', 'g', 31},
	{'S', 'e', 'p', 30},
	{'O', 'c', 't', 31},
	{'N', 'o', 'v', 30},
	{'D', 'e', 'c', 31}
};

//
// convert nibble value to Hex Char
//
char Hex2Asc(byte v)
{
	return (char)pgm_read_byte(ASC_TAB + v);
}


byte I2CRndWrite(byte adr, byte val)
{
	byte failcnt = 0;
	byte ret;

	while (--failcnt) {
	    ret = i2c_start(RTCID+I2C_WRITE);       // set device address and write mode
	    if ( ret ) {
	        // failed to issue start condition, possibly no device found
	        i2c_stop();
	        continue;
	    }
        // issuing start condition ok, device accessible
        ret = i2c_write(adr);                       // ret=0 -> Ok, ret=1 -> no ACK 
	    if ( ret ) {
	        // failed to write.
	        i2c_stop();
	        continue;
	    }
        ret = i2c_write(val);                       // ret=0 -> Ok, ret=1 -> no ACK 
	    if ( ret ) {
	        // failed to write.
	        i2c_stop();
	        continue;
	    }
        i2c_stop();                            // set stop conditon = release bus
		break;
	}
	if (failcnt == 0) {
		Flags.RTCNotPresent = TRUE;
		ret = 0xff;
	}
	return ret;        
}

byte I2CRndRead(byte adr)
{
	byte failcnt = 0;
	byte ret;

	while (--failcnt) {
	    ret = i2c_start(RTCID+I2C_WRITE);     		// set device address and write mode
	    if ( ret ) {
	        // failed to issue start condition, possibly no device found
	        i2c_stop();
	        continue;
	    }
        // issuing start condition ok, device accessible
        ret = i2c_write(adr);                       // ret=0 -> Ok, ret=1 -> no ACK 
	    if ( ret ) {
	        // failed to write.
	        i2c_stop();
	        continue;
	    }
        ret = i2c_rep_start(RTCID+I2C_READ); 		// set device address and read mode
	    if ( ret ) {
	        // failed to start
	        i2c_stop();
	        continue;
	    }
        ret = i2c_readNak();                  		// read one byte
        i2c_stop();                           		// set stop conditon = release bus
		break;
	}
	if (failcnt == 0) {
		Flags.RTCNotPresent = TRUE;
		ret = 0xff;
	}
	return ret;        
}

#if ds1307
void StartRTCInt(void)
{
	byte v;
	if (EEConfigData.PPSMode == ONEHZCODE) {
		v = 0b10010000;								//1 Hz
	} else {
		v = 0b10010001;								//4096 Hz
	}
	I2CRndWrite(RTCCNT, v);							//set SQWE and frequency value
}

void InitRTCInt(void)
{
	StartRTCInt();
}
//
// This function is used in calibration mode.
// Also used to determine if RTC is present at all. (Flags.RTCNotPresent).
// If not present, the code assumes that the AVR is being programmed
// and the ClockEverRun EEProm flag is not set.
// Does not apply to ds1307 clock since there is no clockout pin
// and the ICs are always present, even in programming mode.
//
void StartRTCClockOut(void)
{
#if EXTDEBUG
	I2CRndWrite(RTCCNT,	0b10010011);			//set SQWE and frequency value of 32KHz
#endif
}

void StopRTCClockOut(void)
{
#if EXTDEBUG
	StartRTCInt();
#endif
}

#if EXTDEBUG
char MsgPszCHBitClearedMessage[] PROGMEM = "CH bit cleared";
#endif
//
// If the CH bit is set in the seconds field, the RTC needs
// to be reset by writing a 0 to this bit
//
// IN: r24 current value of RTCSEC field
// OUT: corrected RTCSEC value
//
// Some observations:
//	We could use a flag value in the nvram to indicate whether the RTC is initialized.
// 	It will stay initialized as long as it has power (incl. battery). With power removed,
//	the flag value will be gone too.
//
//	We don't preserve the value of the CH and 12/24 bit when writing to the SEC and HOUR registers
//	because both are known to be 0 when running.
//	Valid seconds and hours value will always write 0 to these bits.
//
byte ResetDs1307(byte RTCSecs)
{
	I2CRndWrite(RTCSEC, RTCSecs & 0b01111111);		//start oscillator by writing a 0 to the CH bit
//
// Also set the mode bit to 24hr
//
	byte val = I2CRndRead(RTCHOUR);					//set 24 hour mode
	I2CRndWrite(RTCHOUR, val & 0b10111111);			//write updated value with bit 6 cleared
#if EXTDEBUG
	DebugSerialOutStr(MsgPszCHBitClearedMessage);
#endif
	return RTCSecs & 0b01111111;
}

#else	//pcf8563

void StartRTCClockOut(void)
{
	I2CRndWrite(RTCCLK, 0x80);					//enable ClockOut at 32.768 KHz
}

void StopRTCClockOut(void)
{
	I2CRndWrite(RTCCLK, 0);						//disable ClockOut (power saver)
}

void StartRTCInt(void)
{
	byte v1, v2;
	if (EEConfigData.PPSMode == ONEHZCODE) {
		v1 = 64;								//setup for 1 Hz
		v2 = 0b10000001;
	} else {
		v1 = 1;									//setup for 4096 Hz
		v2 = 0b10000000;
	}
	I2CRndWrite(RTCCNT, v1);					//set countdown value
	I2CRndWrite(RTCTCR, v2);					//update Timer/Control Register: enable TE, set source Clock frequency
	I2CRndWrite(RTCCS2, 0b00010001);			//update Control/Status 2: clear TF, set TI/TP set TIE
}

void InitRTCInt(void)
{
	StopRTCClockOut();
	StartRTCInt();
}
#endif

void I2CInit(void)
{
    i2c_init();                                // init I2C interface
}

byte IncrementDay(TimeDate_t* p)
{
	return SetDay(p, ++p->Day);
}

byte IncrementMonth(TimeDate_t* p)
{
	return SetMonth(p, ++p->Month);
}

byte IncrementYear(TimeDate_t* p)
{
	return SetYear(p, ++p->Year);
}

void IncrementDate(TimeDate_t* p)
{
	ASSERT(p);
	byte d = IncrementDay(p);
	if (1 == d) {		//wrapped
		byte m = IncrementMonth(p);
		if (1 == m) {	//wrapped
			IncrementYear(p);
		}
	}
}

void DecrementDate(TimeDate_t* p)
{
	ASSERT(p);
	if (0 == --(p->Day)) {
		if (0 == --(p->Month)) {
			p->Month = 12;		//set to December
			DecrementYear(p);
		}
		p->Day = GetDaysInMonth(p);
	}
}

byte DecrementYear(TimeDate_t* p)
{
	if (p->Year == 0) {
		p->Year = 99;
	} else {
		--p->Year;
	}
	return p->Year;
}

byte IncrementSecs(TimeDate_t* p)
{
	byte s = ++p->Secs;
	if (s >= 60) {
		s = 0;
	}
	p->Secs = s;
	return s;
}

byte IncrementMins(TimeDate_t* p)
{
	byte m = ++p->Mins;
	if (m >= 60) {
		m = 0;
	}
	p->Mins = m;
	return m;
}

byte IncrementHrs(TimeDate_t* p)
{
	byte h = ++p->Hrs;
	if (h >= 24) {
		h = 0;
	}
	p->Hrs = h;
	return h;
}

byte SetDay(TimeDate_t* pTimeDate, byte newDay)
{
	TimeDate_t *p = pTimeDate;
	if ((CheckDaysInMonth(p, newDay) == 0)) {		//CheckDaysInMonth tests for 0 value
		newDay = 1;									//wrap
	}
	p->Day = newDay;
	return newDay;
}

byte SetMonth(TimeDate_t* p, byte newMonth)
{
	if ((newMonth == 0) || (newMonth > 12)) {
		newMonth = 1;
	}
	p->Month = newMonth;
	return newMonth;
}

byte SetYear(TimeDate_t* p, byte newYear)
{
	if (newYear > 99) {
		newYear = 0;
	}
	p->Year = newYear;
	return newYear;
}

byte AVRSetSecs(byte newSecs)
{
	if (newSecs < 60) {
		AVRTimeDateBlk.Secs = newSecs;
		return newSecs;
	}
	return 0;
}

byte AVRSetMins(byte newMins)
{
	if (newMins < 60) {
		AVRTimeDateBlk.Mins = newMins;
		return newMins;
	}
	return 0;
}

byte AVRSetHours(byte newHours)
{
	if (newHours < 24) {
		AVRTimeDateBlk.Hrs = newHours;
		return newHours;
	}
	return 0;
}

void UTCAdvanceTimeOneHr(void)
{
	if (++UTCTimeDateBlk.Hrs >= 24) {
		UTCTimeDateBlk.Hrs = 0;
		IncrementDate(&UTCTimeDateBlk);
	}
}

//
// AdvanceTimeOneHr --
//	Increment the current AvrHr as part of Daylight Saving Time
//
void AVRAdvanceTimeOneHr(void)
{
	if (++AVRTimeDateBlk.Hrs >= 23) {		//update Hours & check for overflow
		AVRTimeDateBlk.Hrs = 0;
		IncrementDate(&AVRTimeDateBlk);
		RTCSetDate();						//update RTC date
	}
	RTCSetTime();							//update RTC time
}
//
// RetreatTimeOneHr --
//	Decrement the current AvrHr as part of Daylight Saving Time
//
void AVRRetreatTimeOneHr(void)
{
	if (--AVRTimeDateBlk.Hrs == 0xff) {		//update Hours & check for overflow
		AVRTimeDateBlk.Hrs = 23;
		DecrementDate(&AVRTimeDateBlk);
		RTCSetDate();						//update RTC date
	}
	RTCSetTime();							//update RTC time
}

char *FormatTime(char *pBufIn, TimeDate_t *pTimeDate)
{
	char *pBuf;
	pBuf = FormatDecimal(pBufIn, pTimeDate->Hrs);
	*pBuf++ = ':';
	pBuf = FormatDecimal(pBuf,pTimeDate->Mins);
	*pBuf++ = ':';
	return FormatDecimal(pBuf, pTimeDate->Secs);
}

char *FormatDate(char *pBufIn, TimeDate_t *pTimeDate)
{
	char *pBuf;
	pBuf = FormatDecimal(pBufIn, pTimeDate->Day);
	pBuf[0] = '-';
	//crafted for smallest code
	byte i = pTimeDate->Month - 1;
	prog_char *p = &MonthsText[i][0];
	pBuf[1] = pgm_read_byte(p);
	pBuf[2] = pgm_read_byte(p+1);
	pBuf[3] = pgm_read_byte(p+2);
	pBuf[4] = '-';
	pBuf[5] = '2';				//Display Year
	pBuf[6] = '0';
	return FormatDecimal(pBuf + 7, pTimeDate->Year);
}

char *FormatAVRTime(char *pBuf)
{
	return FormatTime(pBuf, &AVRTimeDateBlk);
}

char *FormatAVRDate(char *pBuf)
{
	return FormatDate(pBuf, &AVRTimeDateBlk);
}

char *FormatDecimal(char *pBuf, byte val)
{
	if (val < 100) {
		*pBuf++ = '0' + (val / 10);
		*pBuf++ = '0' + (val % 10);
	}
	return pBuf;
}

char *FormatHex(char *pBuf, byte val)
{
	pBuf[0] = Hex2Asc(val >> 4);
	pBuf[1] = Hex2Asc(val & 0x0f);
	return pBuf+2;
}
//
//	Set time to 00:00:00 and date to 01/01/2000
//	Mark RTC as initialized
//
void CleanInitRTC(void)
{
	AVRTimeDateBlk.Secs = AVRTimeDateBlk.Mins = AVRTimeDateBlk.Hrs = 0;
	RTCSetTime();
	AVRTimeDateBlk.Day = AVRTimeDateBlk.Month = 1;
	AVRTimeDateBlk.Year = 0;
	RTCSetDate();
	UpdateEEprom(EEPROM_RTCINIT, 1);
}

//
// Update RTC with data from AVRTimeDateBlk
//
void RTCSetTime(void)
{
	if (Flags.DemoEnabled == 0) {
		if (AVRTimeDateBlk.Secs < 60) {										//sanity check
			I2CRndWrite(RTCSEC, bintobcd(AVRTimeDateBlk.Secs));
		}
		if (AVRTimeDateBlk.Mins < 60) {										//sanity check
			I2CRndWrite(RTCMIN, bintobcd(AVRTimeDateBlk.Mins));
		}
		if (AVRTimeDateBlk.Hrs < 24) {										//sanity check
			I2CRndWrite(RTCHOUR, bintobcd(AVRTimeDateBlk.Hrs));
		}
	}
}

void RTCSetDate(void)
{
	if (Flags.DemoEnabled == 0) {
		//validate day
		if (CheckDaysInMonth(&AVRTimeDateBlk, AVRTimeDateBlk.Day) != 0) {
			I2CRndWrite(RTCDAY, bintobcd(AVRTimeDateBlk.Day));
		} else {
			RTCReadDate();													//restore AVR date from RTC
		}
		if (AVRTimeDateBlk.Month < 13) {									//sanity check
			I2CRndWrite(RTCMONTH, bintobcd(AVRTimeDateBlk.Month));
		}
		I2CRndWrite(RTCYEAR, bintobcd(AVRTimeDateBlk.Year));
	}
}
//
//
//
void RTCReadSecs(BOOL fActivate)
{
//
// get time units in BCD form from RTC
//
	byte val  = I2CRndRead(RTCSEC);
	if (fActivate) {
#if ds1307
		if (val & _BV(7)) {
			ResetDs1307(val);								//if CH bit is on, reset RTC
		}
#endif
	} else {
#if ds1307
		ASSERT((val & 0x80) == 0);							//CH bit
#else
		if (val & 0x80) {									//VL bit
			BatteryWarning();
		}
#endif
	}
	val = bcdtobin(val & 0x7f);							//convert
	if (val >= 60) {
		val = 0;
	}
	AVRTimeDateBlk.Secs = val;
}
//
// Update Time Variables in AVRTimeDateBlk
//
void RTCReadTime(BOOL fActivate)
{
//
// get time units in BCD form from RTC
//
	RTCReadSecs(fActivate);
	byte val = bcdtobin(I2CRndRead(RTCMIN) & 0x7f);			//convert
	if (val >= 60) {
		val = 0;
	}
	AVRTimeDateBlk.Mins = val;
	val = bcdtobin(I2CRndRead(RTCHOUR) & 0x3f);				//convert
	if (val >= 24) {
		val = 0;
	}
	AVRTimeDateBlk.Hrs = val;
}
//
// Update Date Variable in AVRTimeDateBlk.
//
void RTCReadDate(void)
{
	byte val;
//
// get date units in BCD form from RTC
//
	val = bcdtobin(I2CRndRead(RTCDAY) & 0x3f);			//convert
	if (val > 31) {
		val = 1;
	}
	AVRTimeDateBlk.Day = val;
	val = bcdtobin(I2CRndRead(RTCMONTH) & 0x1f);			//convert
	if ((val == 0) | (val > 12)) {
		//
		// value is too small or too large
		//
		val = 1;
	}
	AVRTimeDateBlk.Month = val;
	val = bcdtobin(I2CRndRead(RTCYEAR));					//convert
	if (val > 99) {
		val = 0;
	}
	AVRTimeDateBlk.Year = val;
//
// now validate Day for current month
//
	if (CheckDaysInMonth(&AVRTimeDateBlk, AVRTimeDateBlk.Day) == 0) {
		AVRTimeDateBlk.Day = 1;
	}
}
//
// Clear all Configuration Data
//
void ClearEEConfiguration(void)
{
	byte *p = (pByte)&EEConfigData;
	for (byte i = 0; i < sizeof(EEConfig_t); ++i) {
		UpdateEEprom(EEPROM_FIRST + i, 0xff);
		*p++ = 0;										//Reset memory too
	}
}
//
// Read Configuration Data as a block on startup
//
void ReadEEConfiguration(void)
{
	byte *p = (pByte)&EEConfigData;
	byte v;
	for (byte i = 0; i < sizeof(EEConfig_t); ++i) {
		v = EepromRead(EEPROM_FIRST + i);
		if (v == 0xff) v = 0;							//0xFF means not-initialized
		*p++ = v;
	}
}
//
// convert BCD number to binary
//
byte bcdtobin(byte bcdval)
{
	return (bcdval >> 4) * 10 + (bcdval & 0x0f);
}
//
//  IN:	 TimeDate ptr
//	OUT: #of days
//
byte GetDaysInMonth(TimeDate_t *p)
{
	byte v;
	//crafted for smallest code
	byte i = p->Month - 1;
	prog_char *pt = &MonthsText[i][0];
	v = pgm_read_byte(&pt[3]);
	if ((v == 28) && ((p->Year & 0x03) == 0)) {
//
// Possible leap year.
//
// Gregorian Calendar:
// A year will be a leap year if it is divisible by 4 but not by 100.
// If a year is divisible by 4 and by 100, it is not a leap year
// unless it is also divisible by 400.
// Thus years such as 1996, 1992, 1988 and so on are leap years because
// they are divisible by 4 but not by 100. For century years, the 400
// rule is important. Thus, century years 1900, 1800 and 1700 while all
// still divisible by 4 are also exactly divisible by 100. As they are
// not further divisible by 400, they are not leap years.
// The year 2000 is the only year of interest for us (we support 2000..2099)
// and it IS a leap year. Therefore no special test needed.
//
		v = 29;					//leap year, return 29
	}
	return v;
}
//
//  IN:	 TimeDate ptr, day to check (1..31)
//	OUT: 0 means error else input value
//
byte CheckDaysInMonth(TimeDate_t *p, byte day)
{
	if ((day > 0) && (day <= GetDaysInMonth(p))) {
		return day;
	} else {
		return 0;
	}
}

char MsgPszSunday[] PROGMEM = "Sunday";
char MsgPszMonday[] PROGMEM = "Monday";
char MsgPszTuesday[] PROGMEM = "Tuesday";
char MsgPszWednesday[] PROGMEM = "Wednesday";
char MsgPszThursday[] PROGMEM = "Thursday";
char MsgPszFriday[] PROGMEM = "Friday";
char MsgPszSaturday[] PROGMEM = "Saturday";

char *WeekDays[] PROGMEM = {
 MsgPszSunday,
 MsgPszMonday,
 MsgPszTuesday,
 MsgPszWednesday,
 MsgPszThursday,
 MsgPszFriday,
 MsgPszSaturday
};
//
// Lewis Carroll's algorithm
//
byte AuxTable[12] PROGMEM = {0,3,3,6,1,4,6,2,5,0,3,5};

byte DayOfTheWeek(TimeDate_t *p)
{
	byte v,x;
//Century 20
	v = 6;
// divide by 12, add this to remainder, add number of 4’s in the remainder
 	v = (v + (((byte)(p->Year/12))+(p->Year%12)+((byte)((p->Year%12)/4)))) % 7;
	v = (v + pgm_read_byte(&AuxTable[p->Month-1])) % 7;
// if the year is a leap year, and the month is January or February then minus 1.
// problem year 2000 IS a leap year (see above)
	x = p->Day;
	if (((p->Year & 0x03) == 0) && ((p->Month == 1) || (p->Month == 2))) {
		--x;
	}
	v = (v + x) % 7;
	return v;
}
//
// nDays calculation
// from: http://alcor.concordia.ca/~gpkatch/gdate-algorithm.html
//
uint32_t g(uint y,byte m,byte d)
{
	m = (m + 9) % 12;
	y = y - m/10;
	return 365*y + y/4 - y/100 + y/400 + (m*306 + 5)/10 + ( d - 1 );
}

char * print4digits(char *pStr, uint16_t w, BOOL fZeros)
{
	char *p = pStr;
	if (fZeros) {
		if (w < 1000) *p++ = '0';
		if (w < 100) *p++ = '0';
		if (w < 10) *p++ = '0';
	}
	utoa(w, p, 10);
	return pStr + strlen(pStr);
}
//
// from Douglas W. Jones: http://www.cs.uiowa.edu/~jones/bcd/decimal.html
//
void putdec(char *pStr, uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3)
{
	char *p = pStr;
    uint32_t q;		//d4
	uint32_t d00, d11, d22, d33;

    d00 = 656 * (uint32_t)d3 + 7296 * (uint32_t)d2 + 5536 * (uint32_t)d1 + (uint32_t)d0;
    q = d00 / 10000;
    d00 = d00 % 10000;

    d11 = q + 7671 * (uint32_t)d3 + 9496 * (uint32_t)d2 + 6 * (uint32_t)d1;
    q = d11 / 10000;
    d11 = d11 % 10000;

    d22 = q + 4749 * (uint32_t)d3 + 42 * (uint32_t)d2;
    q = d22 / 10000;
    d22 = d22 % 10000;

    d33 = q + 281 * (uint32_t)d3;
    q = d33 / 10000;
    d33 = d33 % 10000;

//  d4 = q;

//
// Result is range bound in this application. Somewhere between 9 trillion and 150 trillion
// Therefore the highest 4 digits (d4) are always 0.
// d33 is >= 9
// d22, d11, d00 are between 0 and 10,000.
//
	ASSERT(q == 0);
	*p = '$';
	p = print4digits(p+1, (uint16_t)d33, FALSE);
	p = print4digits(p, (uint16_t)d22, TRUE);
	p = print4digits(p, (uint16_t)d11, TRUE);
	(void)print4digits(p, (uint16_t)d00, TRUE);
	ASSERT(strlen(pStr) < MAXUSRNAMELEN);
}
//
// from: http://perotcharts.com/cgi-bin/nationaldebtclock.pl
//
void TotalUSDebt(TimeDate_t *pTD, char *pStr)
{
	char cbuf[MAXUSRNAMELEN];
	byte i,j, k;
//
// Base date: October 22, 2007. No data before that date
// Reference date: September 29, 2010. Daily increase based on
// USDebt increase between these 2 dates
//
	uint32_t TodayDays = g(pTD->Year + 2000, pTD->Month, pTD->Day);
	uint32_t BaseDays = g(2007, 10, 22);
	if (TodayDays < BaseDays) {
		*pStr = 0;
		return;
	}
	uint nDays = TodayDays - BaseDays;
	uint nMins = (pTD->Hrs * 60) + pTD->Mins;

	union {
		uint64_t q;
		uint16_t w[4];
	} USDebt;

	// Start with published debt on October 22, 2007, 12:00 PM
	USDebt.q = 9054233572201ULL - (12 * 60) * 2855467ULL ;
	USDebt.q += nDays * 4111872171ULL + nMins * 2855467ULL;
	if ((EEConfigData.ioctl & _BV(IOCTL_SECSOMODE)) != 0) {
		USDebt.q += pTD->Secs * 47591;
	}

	putdec(cbuf, USDebt.w[0], USDebt.w[1], USDebt.w[2], USDebt.w[3]);
	i = strlen(cbuf) - 1;
	j = i + 4;			//need 4 commas
	pStr[j + 1] = 0;
	ASSERT((j + 1) < (MAXUSRNAMELEN + 2));
	k = 0;
	do {
		pStr[j--] = cbuf[i--];
		if (i == 0xFF) {
			break;
		}
		if (((++k % 3) == 0) && (i != 0)){
			pStr[j--] = ',';
		}
	 
	} while (TRUE);
}

//
// Morse Led Output.
//
// A table with Morse States (On or Off) is prepared every 20 seconds,
// which is the max time is takes to send the complete table.
// Every 100 msec (interval could be customized), the next state is
// transferred to the Led.
//
void ClrLedMorseData(void)
{
	LedMorseHead = 0;		//first free to add.
	LedMorseTail = 0;		//next to transfer out.
	TimeLedTicks = 0;
	memset(LedMorseData, 0, LedMorseDataMaxCnt / 8);	//preset so assumed value is FALSE
}

void AddLedMorseAtom(BOOL b)
{
	byte byteOfs, bitMsk;
	if (b) {
		byteOfs = LedMorseHead >> 3;
		bitMsk = pgm_read_byte(&_BVMap[LedMorseHead & 0x07]);
		LedMorseData[byteOfs] |= bitMsk;
	}
	++LedMorseHead;
	ASSERT(LedMorseHead < LedMorseDataMaxCnt);
}

void AddLedMorsePause(byte cnt)
{
	for (byte i = 0; i < cnt; ++i) {
		AddLedMorseAtom(FALSE);
	}
}

void AddLedMorseDot(void)
{
	AddLedMorseAtom(TRUE);
	AddLedMorseAtom(FALSE);	//Separator
}

void AddLedMorseDash(void)
{
	AddLedMorseAtom(TRUE);
	AddLedMorseAtom(TRUE);
	AddLedMorseAtom(TRUE);
	AddLedMorseAtom(FALSE); //Separator
}

void AddLedMorseDigit(byte dig)
{
	byte *p = (byte *)&LedMorseDigits[dig];
	for (byte i = 0; i < LedMorseDigitCnt; ++i) {
		byte b = pgm_read_byte(p+i);
		if (b) AddLedMorseDash(); else AddLedMorseDot();
	}
}

void AddLedMorseToken(byte tok)
{
	if (tok == MorseStart) {
		AddLedMorseDash();
		AddLedMorseDot();
		AddLedMorseDash();
		AddLedMorseDot();
		AddLedMorseDash();
		AddLedMorseDot();
	} else if (tok == MorseColon) {
		AddLedMorseDash();
		AddLedMorseDash();
		AddLedMorseDash();
		AddLedMorseDot();
		AddLedMorseDot();
		AddLedMorseDot();
	} else {
		ASSERT(FALSE);		//none defined
	}
}
//
// If Morse Output on Led active, update the state.
// Morse output is prepared every 22 seconds, which
// is longer than the longest possible code (216 units of 100 mSec.)
//
void LedMorseUpdate(void)
{
	++LedMorseSecs;
	if (LedMorseSecs == 22) {
		LedMorseSecs = 0;
		if ((EEConfigData.LedOption == LEDMORSE) && !TestPlayModes()){
			//
			// This assertion got hit when in Demo mode and hitting S2 to
			// shorten the pauses. General problem related to PlayMode,
			// which call LedMorseUpdate more than once a second, but
			// the emission of Morse code is not sped up.
			// Also got hit after stopping the clock (time change mode)
			// Fixed by resetting the Morse State when playmode changes
			// and when restarting the clock.
			//
			ASSERT(!(LedMorseHead > LedMorseTail));
			ClrLedMorseData();
			AddLedMorseToken(MorseStart);
			AddLedMorsePause(6);		//One spacing unit already added
			AddLedMorseDigit(AVRTimeDateBlk.Hrs / 10);
			AddLedMorsePause(2);		//One spacing unit already added
			AddLedMorseDigit(AVRTimeDateBlk.Hrs % 10);
			AddLedMorsePause(6);		//One spacing unit already added
			AddLedMorseToken(MorseColon);
			AddLedMorsePause(6);		//One spacing unit already added
			AddLedMorseDigit(AVRTimeDateBlk.Mins / 10);
			AddLedMorsePause(2);		//One spacing unit already added
			AddLedMorseDigit(AVRTimeDateBlk.Mins % 10);
			AddLedMorsePause(6);		//One spacing unit already added
			AddLedMorseToken(MorseColon);
			AddLedMorsePause(6);		//One spacing unit already added
			AddLedMorseDigit(AVRTimeDateBlk.Secs / 10);
			AddLedMorsePause(2);		//One spacing unit already added
			AddLedMorseDigit(AVRTimeDateBlk.Secs % 10);
			AddLedMorsePause(6);		//One spacing unit already added
		}
	}
}

