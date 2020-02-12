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
// Scope Clock data declarations
//
typedef	unsigned char BOOL;
typedef unsigned char byte;
typedef unsigned int uint;
typedef byte* pByte;
#define PGM_A const prog_uint16_t *

typedef struct VectorTblEntry {
	byte				v[4];
} VectorTblEntry_t;

typedef struct ScanTblEntry {
	VectorTblEntry_t*	pVect;
	unsigned char		Xoffset;
	unsigned char 		Yoffset;
} ScanTblEntry_t;

typedef struct MScanTblEntry {
	char			*	pVect;
	unsigned char		Xoffset;
	unsigned char 		Yoffset;
} MScanTblEntry_t;

typedef struct FlashTblEntry {
	ScanTblEntry_t*		pScan;
	VectorTblEntry_t*	pVect;
	unsigned char		stat;
	unsigned char		cnt;
	VectorTblEntry_t*	pVectOrig;
	VectorTblEntry_t*	pVectAlt;
} FlashTblEntry_t;

//
// Fixed Struct. These Variables are read as a block from EEProm
//
typedef struct EEConfig {
	byte	oldLedDisabled;				//if true, disable LED blinking (power saver)
	byte	Verbose;					//spit out more info
	byte	PreGPSBaudVal;		
	byte	LedOption;					//on, morse, debug, off
	byte	AlarmDisabled;
	byte	ClockEverRun;		
	byte	BurninVal;					//minutes between a screen move
	byte	PPSMode;					//1Hz or 4096Hz mode
	byte	CurrNumVal;			
	byte	CurrDialVal;			
	byte	PPSTrigger;					//Positive or Negative going edge
	byte	DSTMode;					//Daylight Saving Time mode (none, US, EU)
	byte	DSTFYears;					//bit array marking years for which DST Forward has been applied
	byte	DSTBYears;					//bit array marking years for which DST Backward has been applied
	byte	GPSInEnabled;		
	byte	GPSOffset;					//Local Time Zone Offset
	byte	UsrNameVal;					//Display UsrName or Weekday or O
	byte	UsrNameCnt;					//UsrName length in EEprom
	byte	Spare;						//UNUSED but reserved
	byte	ChronoEnabled;		
	byte	BaudVal;		
	byte	StartApp;
	byte	AlarmSecs;					//Alarm Time
	byte	AlarmMins;
	byte	AlarmHrs;
	uint	ioctl;

} EEConfig_t;
//
//
typedef struct TimeDate {
	byte	Secs;
	byte	Mins;
	byte	Hrs;
	byte	Day;
	byte	Month;
	byte	Year;
} TimeDate_t;

typedef struct CModStateTblEntry {
	byte			f1,f2;
	void			(*S1Action)(void);
	ScanTblEntry_t *f4;
} CModStateTblEntry_t;

typedef struct CMenuStateTblEntry {
	void			(*S1Action)(void);
	void	 		(*S2Action)(void);
	MScanTblEntry_t *FlashField;
} CMenuStateTblEntry_t;

typedef struct CMenuVals {
	byte		AppVal;
	byte		UsrNameCnt;
	byte		AlarmVal;
	byte		LedVal;
	byte		BurninVal;
	byte		DialVal;
	byte		NumVal;
	byte		PlayVal;
	byte		ChronoVal;
	byte		DSTMode;
	byte		UsrNameVal;
	byte		PPSMode;
	byte		BaudVal;
	byte		EEInitVal;
	byte		GPSVal;
} CMenuVals_t;

typedef struct PushButtonState {
	byte	TC0Cntr;
	byte	Position;
	byte	KeyState;
	BOOL	fAutorepeat;
	byte	autorepeatcnt;
	BOOL	okToRepeat;
} PushButtonState_t;

typedef struct ClkFlags {
	BOOL	SerialInOverflow;
	BOOL	DisplayRedraw;
	BOOL	GPSOKToUpdate;
	BOOL	BatteryLow;					//RTC reported low battery
	BOOL	RTCNotPresent;
	BOOL	RXLFDetected;
	BOOL	UseDblHand;
	BOOL	AppChanged;				//may be set by UsrChangeApp
	BOOL	UpdMin;
	BOOL	UpdHour;
	BOOL	InputActive;
	byte	DemoEnabled;
	BOOL	RotationDisabled;
#if DEBUG
	BOOL	DBGT;
#endif
} ClkFlags_t;

typedef struct TermFlags {
	BOOL	LFPending :1 ;
	BOOL	CRPending :1 ;
	BOOL	DispCursor :1 ;
	BOOL	ProgClkLineSet :1 ;
	BOOL	ProgClkLine :1 ;
	BOOL	ProgBurnInSet :1 ;
	BOOL	ProgBurnIn :1 ;
} TermFlags_t;

extern BOOL	fClrAlarm;
extern BOOL	fTimeOut;
extern BOOL	fMiniDDS;
extern BOOL	fClockIsRunning;

//
// These flags are referenced from ClkRender.s
//
extern BOOL	ShowDialDots;
extern BOOL	fDispNumBuf;

enum {
	CMENUIDLE			= 79,
	CMENUNUM,
	CMENUDIAL,
	CMENUGPS,
	CMENUDST,
	CMENUAPP,
	CMENUDEMO,
	CMENUCHRONO,
	CMENUCAL,
	CMENULED,
	CMENUBURN,
	CMENUFUN,
	CMENUINIT,
	CMENUBAUD,
	CMENUNAME,
	CMENUNAMEDIT
};

#define CMENULAST       CMENUNAMEDIT

//
// EEPROM layout
//
// Certain status flags are saved in the EEPROM area.
// These flags must have a default (normal) value of 0
// since the EEProm is 0xFF when unprogrammed which will be
// mapped to 0 when the EEProm is read at startup.
//
// the EEPROM Configuration area is read all at once at startup.
// Therefore, this enumeration must match "EEConfig_t"
//
enum {
	EEPROM_FIRST			= 0,
	EEPROM_VERBOSE,
	EEPROM_PREGPSBAUDVAL,	//Baudval before switching to GPS mode
	EEPROM_LED,
	EEPROM_ALARM,
	EEPROM_RTCINIT,
	EEPROM_BURN,
	EEPROM_PPSMODE,			//PPS mode
	EEPROM_NUMERIC,			//numeric display mode
	EEPROM_DIAL,			//clock dial mode
	EEPROM_PPSTRIG,			//PPS Trigger mode
	EEPROM_DSTMODE,			//Daylight Savings Time mode
	EEPROM_DSTFVEC,			//Daylight Savings Time Forward bit vector
	EEPROM_DSTBVEC,			//Daylight Savings Time Backward bit vector
	EEPROM_GPSMODE,			//GPSEnabled mode (Can't use composite value for GPS)
	EEPROM_GPSOFFSET,		//Local Time Offset from UTC/GPS
	EEPROM_USRNAMEVAL,	
	EEPROM_USRNAMECNT,		//USRNAME char count (0 means no UsrName)
	EEPROM_SPARE,			//Unused
	EEPROM_CHRONO,			//Chronometer enabled
	EEPROM_BAUDVAL,			//Current Baudval (0-based code)
	EEPROM_STARTAPP,		//Start application
	EEPROM_ALARMSECS,		//Alarm Value
	EEPROM_ALARMMINS,
	EEPROM_ALARMHRS,
	EEPROM_IOCTLL,			//IOCTL control low/high
	EEPROM_IOCTLH,
	EEPROM_MAX				= EEPROM_IOCTLH
};

#define EEPROM_USRNAME 30	//Assume no more than 30 flag entries
#define MAXUSRNAMELEN 20


enum {
#if DEBUG
	DIAL12HR	= 10,
#else
	DIAL12HR,
#endif
	DIAL24HR,
	DIALROMAN,
	DIALDIG,
	DIALBIN,
	DIALMIN,
	DIALLAST = DIALMIN
};

enum {
#if DEBUG
	BAUDVAL19200	= 74,
#else
	BAUDVAL19200,
#endif
	BAUDVAL28800,
	BAUDVAL38400,
	BAUDVAL57600,
	BAUDVAL115200,
	BAUDVAL250000,
	BAUDVAL4800,
	BAUDVAL96200,
	BAUDVAL14400,
	BAUDVALLAST = BAUDVAL14400
};
#define BAUDVALFIRST		BAUDVAL19200

enum {
#if DEBUG
	APPCLOCK	= 110,
#else
	APPCLOCK,
#endif
	APPCALIBRATE,
	APPTERM,
	APPGEN,
	APPDEMO,
	APPBOOT,
	APPLAST		= APPBOOT
};

enum {
#if DEBUG
	NUM12HR		= 20,
#else
	NUM12HR,
#endif
	NUM24HR,
	NUMHEX,
	NUMDATE,
	NUMALARM,
	NUMOFF,
	NUMLAST		= NUMOFF
};

enum {
#if DEBUG
	PPSBASEVAL		= 100,	// 1Hz or 4096Hz
#else
	PPSBASEVAL,				// 1Hz or 4096Hz
#endif
	PPS1HZVAL,				//  or PPS4096HZVAL
	PPSMIN4VAL,
	PPSMIN3VAL,
	PPSMIN2VAL,
	PPSMIN1VAL,
	PPSPLUS1VAL,
	PPSPLUS2VAL,
	PPSPLUS3VAL,
	PPSPLUS4VAL,
	PPSLASTVAL		= PPSPLUS4VAL
};

enum {
	DSTUS,
	DSTEU,
	DSTNONE,
	DSTLAST = DSTNONE
};

enum {
	LEDENABLED,
	LEDMORSE,
	LEDDEBUG,
	LEDDISABLED,
	LEDLAST = LEDDISABLED
};

enum {
	USRNAMEON,
	USRNAMEOFF,
	USRNAMEEDIT,
	USRNAMEDAY,
	USRNAMEO,
	USRNAMELAST		= USRNAMEO
};

enum {
	IOCTL_EASYOMODE,		//bit 0 0x0001
	IOCTL_SAVEOMODE,		//bit 1 0x0002
	IOCTL_NODIALDOTS,		//bit 2 0x0004
	IOCTL_12HRDIG,			//bit 3 0x0008
	IOCTL_NUMOFFTERM,		//bit 4 0x0010
	IOCTL_USRNAMETERM,		//bit 5 0x0020
	IOCTL_BURNINTERM,		//bit 6 0x0040
	IOCTL_DONOTUSE0,		//prevent 0xff value
	IOCTL_SECSOMODE,		//bit 8 0x0100
	IOCTL_SPARE9,
	IOCTL_SPARE10,
	IOCTL_SPARE11,
	IOCTL_SPARE12,
	IOCTL_SPARE13,
	IOCTL_SPARE14,
	IOCTL_DONOTUSE1			//prevent 0xff value
};

typedef enum {
	VectNone,
	VectEntry = 0xf0,		//5 arg bytes
	VectRange = 0xf1,		//2 arg bytes
	TermSetCursor = 0xf2,	//2 arg bytes
	TermSetCursorCtl = 0xf3,//1 arg byte
	TermSetClkLine = 0xf4,	//1 arg byte
	TermSetBurnIn = 0xf5,	//1 arg byte
	TermCtrl = 0xf6,		//1 arg byte. 0 = Clear Screen
	TermMaxCmd = TermCtrl
}_TermCmd;

extern _TermCmd TermCmd;

#define ScanTbl_Dial (&ScanTbl[stDialMarks])
#define	ScanTbl_Full (&ScanTbl[stDialDigits])
#define	ScanTbl_SecHand (&ScanTbl[stSecHand])
#define	ScanTbl_SecHand2 (&ScanTbl[stSecHand2])
#define	ScanTbl_MinHand (&ScanTbl[stMinHand])
#define	ScanTbl_MinHand2 (&ScanTbl[stMinHand2])
#define	ScanTbl_HrHand (&ScanTbl[stHrHand])
#define	ScanTbl_HrHand2 (&ScanTbl[stHrHand2])
#define ScanTblPush (&ScanTbl[stPushtoStart])
#define ScanTblStatG (&ScanTbl[stMsgG])
#define ScanTblStatB (&ScanTbl[stMsgB])
#define ScanTblStatP (&ScanTbl[stMsgP])
#define ScanTblSig (&ScanTbl[stMsgSig])
//
// Pointer(s) to SRam Space based Text:
//	Usr Name Display,	Numeric Text Display
//
#define	ScanTblUsrName (&ScanTbl[stUsrName])		//User Name Display
#define	ScanTblNum (&ScanTbl[stNumBuf])				//Numeric Text Display

#define MScanTbl_NumVal  (&MScanTbl[3])
#define MScanTbl_DialVal  (&MScanTbl[5])
#define MScanTbl_GPSVal  (&MScanTbl[7])
#define MScanTbl_DSTVal  (&MScanTbl[9])
#define	MScanTbl_AppVal  (&MScanTbl[11])
#define MScanTbl_PPSVal  (&MScanTbl[13])
#define MScanTbl_ChronoVal (&MScanTbl[15])
#define MScanTbl_AlarmVal (&MScanTbl[17])
#define MScanTbl_LedVal (&MScanTbl[19])
#define MScanTbl_BurninVal (&MScanTbl[21])
#define MScanTbl_PlayVal (&MScanTbl[23])
#define MScanTbl_InitVal (&MScanTbl[25])
#define MScanTbl_BaudVal (&MScanTbl[27])
#define MScanTbl_UsrNameVal (&MScanTbl[29])
#define MScanTbl_UsrNameTxt (&MScanTbl[30])

extern ScanTblEntry_t TermScanTblStr[];
extern char	TermLines[MAXSCREENLINES][MAXLINESIZE+2];
extern BOOL	LinesEmpty[MAXSCREENLINES];
extern char	TermFlashBuffer[MAXLINESIZE+2];
extern ScanTblEntry_t *TermTextBuf;
extern byte	CurLine;
extern byte	CurPosition;
extern byte VectorStartIdx, VectorRngLen;
extern byte nScreenLines;

#if EXTDEBUG
extern byte nSerialInMissed;
extern byte LastGoodByteIdx;
#endif

extern ClkFlags_t Flags;
extern TermFlags_t TermFlags;
extern byte DialData[];
extern byte DialDots[];
extern VectorTblEntry_t VectorTbl[];
//
// Clock Display Scan Table
//
// Each table entry consists of a pointer to program memory containing
// a list of vectors positions (or a string) plus an Xoffset and Yoffset
// field (to be added to the vector/string position)
//
extern ScanTblEntry_t ScanTbl[SCANTABLEN + SCANSTRTABLEN + SCANSSTRTABLEN];		//Clock Vectors
extern MScanTblEntry_t MScanTbl[];
extern byte NumDispBuf[];
extern byte NumFlashDispBuf[];
extern char *pChronoDispBuf;
extern char UsrNameBuf[];
extern char UsrNameFlashBuf[];
extern char SRAMBuf[];
extern FlashTblEntry_t	FlashTbl[];

#if DEBUG
extern BOOL	GPSUpdateRTCTime;
extern BOOL	GPSUpdateRTCDate;
extern volatile byte OldTimeTicks;
#endif
extern volatile byte TimeTicks;
extern volatile byte TimeLedTicks;
extern volatile byte Time2Ticks;
extern TimeDate_t UTCTimeDateBlk;
extern TimeDate_t AVRTimeDateBlk;
extern byte	PPSTimeOutCnt;				//in case PPS signal fails
extern byte	ReloadPPSTimeOutCnt;
extern byte	CurScanTblLen;				//current length of Display Scan Table
extern byte	FlashCount;					//countdown for flash fields
extern byte	RotationIndex;				//current index in Grid Offset Rotation. 0xff means disabled.
extern byte	RotationCountdown;			//Countdown for RotateGrid

extern byte	CModState;
extern byte	CMenuState;
extern FlashTblEntry_t *CModFlashPtr1;
extern FlashTblEntry_t *CModFlashPtr2;
extern FlashTblEntry_t *CMenuFlashPtr;
extern FlashTblEntry_t *CTextFlashPtr;
extern FlashTblEntry_t *PPSFlashPtr;
extern FlashTblEntry_t *DialFlashPtr;
extern void	(*fpCurrentRefresh)(void);

extern byte	UsrNameEditIdx;
extern byte	CurrPlayMode;
extern byte	TextFlashField;			//Buf Field Mod Flag
extern pByte CurrDial12DigitsPtr;
extern pByte CurrDial24DigitsPtr;

extern byte	CurrAppMode;			//range APPCLOCK..APPGEN

extern FlashTblEntry_t *BatFlashPtr;

extern byte	BeamDotX0;
extern byte	BeamDotY0;
extern byte	BeamDotX1;
extern byte	BeamDotY1;
extern byte	BeamDotX2;
extern byte	BeamDotY2;
extern byte	BeamDotX3;
extern byte	BeamDotY3;

extern byte	UsrNameDflt;
extern byte UsrNameSkipCnt;

#if EXTDEBUG
extern uint	ShortSecCnt;
extern uint	LongSecCnt;
#endif

extern uint	RefreshCnt;
extern uint	LastRefreshCnt;

#if DEBUG
extern byte	RestartCounter;
extern byte	TC0DebugCnt;
extern byte	SecondCnt;
extern uint	ItemCnt;
extern byte	ExtDbg1;
extern byte	ExtDbg2;
extern byte	ExtraTimer;
extern byte	DBGStage;
#endif
extern byte	DSTForwardHr;
extern byte	DSTBackMonth;
extern byte	DSTForwardDay;
extern byte	DSTBackDay;
extern byte	GPSLastHrFlag;


#if NETFREQUENCY
extern byte	NetCycle;
extern byte	NetCycleCnt;
extern byte	NetCyclePhase;
extern byte	NetCycleRefresh;
#endif

extern EEConfig_t EEConfigData;

extern unsigned char BinaryFrameData[] PROGMEM;
extern unsigned char DialDigits12[] PROGMEM;
extern unsigned char DialDigits24[] PROGMEM;
extern unsigned char DialDigitsRoman[] PROGMEM;
extern unsigned char DialDigitsMin[] PROGMEM;
extern char CharB[] PROGMEM;
extern char CharP[] PROGMEM;
extern char CharG[] PROGMEM;
extern char ASC_TAB[] PROGMEM;
extern byte RotationTable[] PROGMEM;
extern byte CalibrationData[] PROGMEM;

extern char MsgPszLogo[] PROGMEM;
extern char MsgPszAltLogo[] PROGMEM;
extern char MsgPszSignOn[] PROGMEM;
extern char MsgPszVersion[] PROGMEM;
extern char MsgPszNewLine[] PROGMEM;
extern char MsgPszDSTApplied[] PROGMEM;
extern char TermMsgPszSignOn[] PROGMEM;
extern char TermMsgPszWelcome[] PROGMEM;

//
// VectorTbl & WaveTbl use the same SRAM memory
//
extern byte WaveTbl[];
extern pByte CurWaveTblPtr;

extern CModStateTblEntry_t CModStateTable[26] PROGMEM;
extern CMenuStateTblEntry_t CMenuStateTbl[] PROGMEM;

extern byte SecPtrData[] PROGMEM;
extern byte MinPtrData[] PROGMEM;
extern byte HrPtrData[] PROGMEM;
extern byte *DigNums[] PROGMEM;
extern byte DigSeparator[] PROGMEM;

#define LedMorseDataMaxCnt 216
#define LedMorseDigitCnt 5
#define MorseStart 0x81
#define MorseColon 0x82
extern byte LedMorseData[];
extern byte LedMorseHead, LedMorseTail;
extern byte LedMorseSecs;
extern byte _BVMap[8];

extern uint32_t	UpTime;
extern byte LedMorseData[];
extern byte LedMorseHead, LedMorseTail;
extern byte LedMorseSecs;
typedef struct {
	byte data[5];
} LedMorseDigit_t;

extern BOOL fUpdateLed;

extern LedMorseDigit_t LedMorseDigits[] PROGMEM;
