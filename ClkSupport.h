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
//
// ClkSupport
//
void StopRTCClockOut(void);
void CleanInitRTC(void);
void InitRTCInt(void);
void StartRTCClockOut(void);
void I2CInit(void);
void StartRTCInt(void);


#define AVRSetDay(newDay)		 SetDay(&AVRTimeDateBlk, newDay)
#define	AVRSetMonth(newMonth)	 SetMonth((&AVRTimeDateBlk), newMonth)
#define	AVRSetYear(newYear)		 SetYear(&AVRTimeDateBlk, newYear)
#define AVRIncrementDate()		 IncrementDate(&AVRTimeDateBlk)
#define AVRIncrementDay()		 IncrementDay(&AVRTimeDateBlk)
#define AVRIncrementMonth()		 IncrementMonth(&AVRTimeDateBlk)
#define AVRIncrementYear()		 IncrementYear(&AVRTimeDateBlk)
#define AVRIncrementSecs()		 IncrementSecs(&AVRTimeDateBlk)
#define AVRIncrementMins()		 IncrementMins(&AVRTimeDateBlk)
#define AVRIncrementHrs()		 IncrementHrs(&AVRTimeDateBlk)
#define AVRDecrementDate()		 DecrementDate(&AVRTimeDateBlk)
#define UTCIncrementDate()		 IncrementDate(&UTCTimeDateBlk)
#define	UTCDecrementDate()		 DecrementDate(&UTCTimeDateBlk)

void IncrementDate(TimeDate_t* p);
void DecrementDate(TimeDate_t* p);
byte DecrementYear(TimeDate_t* p);
byte IncrementSecs(TimeDate_t* p);
byte IncrementMins(TimeDate_t* p);
byte IncrementHrs(TimeDate_t* p);
byte IncrementDay(TimeDate_t* p);
byte IncrementMonth(TimeDate_t* p);
byte IncrementYear(TimeDate_t* p);

byte SetDay(TimeDate_t* p, byte newDay);
byte SetMonth(TimeDate_t* p, byte newMonth);
byte SetYear(TimeDate_t* p, byte newYear);

byte GetDaysInMonth(TimeDate_t* p);
BOOL CheckDaysInMonth(TimeDate_t* p, byte newDay);
byte SetDay(TimeDate_t* p, byte newDay);
byte SetMonth(TimeDate_t* p, byte newMonth);
byte SetYear(TimeDate_t* p, byte newYear);
void RTCReadSecs(BOOL fActivate);
void RTCReadTime(BOOL fActivate);
void RTCReadDate(void);
void RTCSetTime(void);
void RTCSetDate(void);

void UTCAdvanceTimeOneHr(void);
void AVRAdvanceTimeOneHr(void);
void AVRRetreatTimeOneHr(void);

byte AVRSetSecs(byte newSecs);
byte AVRSetMins(byte newMinutes);
byte AVRSetHours(byte newHours);

char *FormatTime(char *pBuf, TimeDate_t *pTimeDate);
char *FormatDate(char *pBuf, TimeDate_t *pTimeDate);
char *FormatDecimal(char *pBuf, byte val);
char *FormatAVRTime(char *pBuf);
char *FormatAVRDate(char *pBuf);

inline char *FormatUTCTime(char *pBuf)
{
	return FormatTime(pBuf, &UTCTimeDateBlk);
}

inline char *FormatUTCDate(char *pBuf)
{
	return FormatDate(pBuf, &UTCTimeDateBlk);
}
char *FormatHex(char *pBuf, byte val);

byte bcdtobin(byte val);
char Hex2Asc(byte v);
void ClearEEConfiguration(void);
void ReadEEConfiguration(void);

byte DayOfTheWeek(TimeDate_t *p);
extern char *WeekDays[7];
void TotalUSDebt(TimeDate_t *p, char *pStr);

void ClrLedMorseData(void);
void LedMorseUpdate(void);
