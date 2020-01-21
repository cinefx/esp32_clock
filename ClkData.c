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
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "./ClkConfig.h"
#include "./ClkData.h"

//
// Clock Display Scan Table
//
// Each table entry consists of a pointer to program memory containing
// a list of vectors positions (or a string) plus an Xoffset and Yoffset
// field (to be added to the vector/string position)
//
ScanTblEntry_t ScanTbl[SCANTABLEN + SCANSTRTABLEN + SCANSSTRTABLEN];
MScanTblEntry_t MScanTbl[MENUSCANTABLEN];			//Could be allocated dynamically. Used as work area.

unsigned char NumDispBuf[NUMDISPBUFLEN + 1];
unsigned char NumFlashDispBuf[NUMDISPBUFLEN + 1];
char *pChronoDispBuf;
#if DEBUG
char UsrNameBuf[MAXUSRNAMELEN+1+4];
#else
char UsrNameBuf[MAXUSRNAMELEN+1];
#endif
char UsrNameFlashBuf[MAXUSRNAMELEN+2];
char SRAMBuf[MAXUSRNAMELEN+2];

//
// Flash Control Table
//
// Each entry describes a ScanTbl entry that needs to flash/blink:
//	ScanTbl ptr, VectorTbl ptr, On/Off status, FlashCnt, Original VectorTbl ptr, Alternate VectorTbl ptr
// Original VectorTbl ptr table entry is to deal with errors
// Issue: should all flash fields be in sync, i.e. all NULL at the same time?
//
FlashTblEntry_t	FlashTbl[MaxFlashTblEntries+1];
TimeDate_t	UTCTimeDateBlk;
TimeDate_t	AVRTimeDateBlk;
EEConfig_t	EEConfigData;

//
// Scope Terminal SRam Variables.
//
ScanTblEntry_t	TermScanTblStr[TERMSCANSSTRTABLEN];
//
// Terminal Screen Data Storage
// add one positon for NULL terminator plus one for Debug Marker
//
char	TermLines[MAXSCREENLINES][MAXLINESIZE+2];
BOOL	LinesEmpty[MAXSCREENLINES];
char	TermFlashBuffer[MAXLINESIZE+2];
ScanTblEntry_t    *TermTextBuf;
byte	CurLine;
byte	CurPosition;
byte	VectorStartIdx, VectorRngLen;
#if EXTDEBUG
byte	nSerialInMissed;
byte	LastGoodByteIdx;
#endif
byte	nScreenLines;

ClkFlags_t Flags;
TermFlags_t TermFlags;
BOOL	fDispNumBuf;
BOOL	ShowDialDots;
BOOL	fClrAlarm;
BOOL	fTimeOut;
BOOL	fMiniDDS;
BOOL	fClockIsRunning;

#if DEBUG
volatile byte OldTimeTicks;
#endif

volatile byte TimeTicks;
volatile byte TimeLedTicks;
volatile byte Time2Ticks;
byte	PPSTimeOutCnt;			//in case PPS signal fails
byte	ReloadPPSTimeOutCnt;
byte	CurScanTblLen;			//current length of Display Scan Table
byte	FlashCount;				//countdown for flash fields
byte	RotationIndex;			//current index in Grid Offset Rotation. 0xff means disabled.
byte	RotationCountdown;		//Countdown for RotateGrid
byte	CModState;
byte	CMenuState;
FlashTblEntry_t *CModFlashPtr1;
FlashTblEntry_t *CModFlashPtr2;
FlashTblEntry_t *CMenuFlashPtr;
FlashTblEntry_t *CTextFlashPtr;
FlashTblEntry_t *PPSFlashPtr;
FlashTblEntry_t *DialFlashPtr;
void	(*fpCurrentRefresh)(void);

byte	UsrNameEditIdx;
byte	CurrPlayMode;
byte	TextFlashField;			//Buf Field Mod Flag
pByte	CurrDial12DigitsPtr;
pByte	CurrDial24DigitsPtr;
//
// Daylight Saving Time data
//
byte	DSTForwardHr;
byte	DSTBackMonth;
byte	DSTForwardDay;
byte	DSTBackDay;
byte	GPSLastHrFlag;
FlashTblEntry_t *BatFlashPtr;
byte	UsrNameDflt;
uint	RefreshCnt;
uint	LastRefreshCnt;
byte	CurrAppMode;		//range APPCLOCK..APPGEN
byte	MenuAppVal;

#if EXTDEBUG
uint	ShortSecCnt;
uint	LongSecCnt;
#endif

byte	BeamDotX0;
byte	BeamDotY0;
byte	BeamDotX1;
byte	BeamDotY1;
byte	BeamDotX2;
byte	BeamDotY2;
byte	BeamDotX3;
byte	BeamDotY3;

#if DEBUG
BOOL	GPSUpdateRTCTime;
BOOL	GPSUpdateRTCDate;
byte	RestartCounter;
byte	TC0DebugCnt;
byte	SecondCnt;
uint	ItemCnt;
byte	ExtDbg1;
byte	ExtDbg2;
byte	ExtraTimer;
byte	DBGStage;
#endif

#if NETFREQUENCY
byte	NetCycle;
byte	NetCycleCnt;
byte	NetCyclePhase;
byte	NetCycleRefresh;
#endif

pByte	CurWaveTblPtr;
uint32_t	UpTime;

byte LedMorseData[LedMorseDataMaxCnt/8];
byte LedMorseHead, LedMorseTail;
byte LedMorseSecs;

byte UsrNameSkipCnt;

BOOL fUpdateLed;
BOOL FirstInString;

LedMorseDigit_t LedMorseDigits[10] PROGMEM = {
	{{TRUE, TRUE, TRUE, TRUE, TRUE}},		//0
	{{FALSE, TRUE, TRUE, TRUE, TRUE}},	//1
	{{FALSE, FALSE, TRUE, TRUE, TRUE}},	//2
	{{FALSE, FALSE, FALSE,TRUE, TRUE}},	//3
	{{FALSE, FALSE, FALSE, FALSE, TRUE}},	//4
	{{FALSE, FALSE, FALSE, FALSE, FALSE}},//5
	{{TRUE, FALSE, FALSE, FALSE, FALSE}},	//6
	{{TRUE, TRUE, FALSE, FALSE, FALSE}},	//7
	{{TRUE, TRUE, TRUE, FALSE, FALSE}},	//8
	{{TRUE, TRUE, TRUE, TRUE, FALSE}}		//9
};

#include	"./vecttable.inc"
#include	"./font8x5.inc"


