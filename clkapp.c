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
#include "./ClkISR.h"
#include "./ClkDebug.h"
#include "./ClkFlash.h"
#include "./ClkSupport.h"

extern void MainMenuProcessing(BOOL firstTime);
extern void MainCModProcessing(void);
extern BOOL RunCalibrate(void);
extern void RunTerm(void);
extern void RunFuncGen(void);
extern void UsrDemo(void);
extern void CheckGPSInput(void);
extern BOOL UsrCommand(void);
extern void SetMem(void);
extern byte bintobcd(byte val);
extern void TurnOffLed(void);
extern void SetActualTime(void);
extern void	HelpScreenRefresh(void);
extern void CalibrationRefresh(void);
extern void	ClockRefresh(void);
extern void	MenuRefresh(void);
extern void UsrShowStatus(void);
extern void UpdateCModFlashEntry(ScanTblEntry_t* pScanPtr, VectorTblEntry_t* pVectArg);
extern void SetUsrNameField(byte newVPos);
extern void DSTRetroApply(void);
extern void DACinit(void);
extern void InitTerm(void);
extern void UsrPrintNewLine(void);
extern void UsrShowAll(void);
extern void UsrWait4Print(void);
extern BOOL UpdateGPSSetting(byte newGPSVal);
extern char *SetAsciiFromBin(char *p, byte v);
extern void DiscardGPSInput(void);
extern void setLed(byte v);
extern void StartKeyDebouncing(void);
extern uint	IntrpCnt;

pByte Led2Tbl[] ;
pByte Led3Tbl[] ;
pByte Led4Tbl[] ;
byte AMPMMapping[] ;

BOOL RunClock(void);
void InitData(void);
void SetInt0Countdown(void);
void SetRotation(void);
void ReadEEUsrName(void);
void SetBaudVal(byte newBaudVal);
byte SetDialDisplay(void);
void SetDisplay(void);
void UpdateSecs(void);
void ReverseSecs(void);
void ClockWorks(void);
void SetActualTime(void);
void ResetPPSCntr(void);
void SetPPSMode(byte newMode);
byte EnablePPSTimeOut(void);
BOOL TestPlayModes(void);
void UpdateChronoBuf(void);
byte ValidateNumericDisplay(void);
byte ValidateDialDisplay(void);
void SetAnalogClockPtrs(void);
void SetBinaryClockPtrs(void);
void SetDigitalClockPtrs(void);
void SetNumDisplay(void);
void SetSecsHand(void);
void SetMinsHand(void);
void SetHrsHand(void);
void SetDialDigits(void);
void SetClockHands(void);
byte ComputeSRamHPos(char *p);
byte GetCharWidth(char);
void RotateXYOffset(void);
void CalibrationOn(void);
void CalibrationOff(void);
void PPSMissingOn(void);
void PPSMissingOff(void);
void SetHelpScanTbl(void);
void DisplayHelpscreen(void);

char MsgPszSerialInOverflow[];

#define GetNumDisplayMode ValidateNumericDisplay

byte _BVMap[8] PROGMEM = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

void __attribute__ ((naked, section (".init1")))	// called before .do_copy_data_start
SetMem_init (void) 
{
	SetMem();
}

// __heap_start is defined in the linker script
extern uint8_t __heap_start;

#if DEBUG
//		 Check for 0xff (in DEBUG mode) as set by SetMem();
uint16_t
GetMemUnused (void)
{
	uint16_t unused = 0;
    
    // Get end of static allocated RAM space (.data, .bss, .noinit, ...)
	uint8_t *p = &__heap_start;
	
	do
	{
        // Value written in SetMem() still intact?
		if (*p++ != 0xff)
			break;
			
		unused++;
	} while (p <= (uint8_t *) RAMEND);

	return unused;
}
#endif
//
//
int main(void)
{
	WDTimerOff();
	PRR = _BV(PRADC);									// Disable ADC part of the AVR
	GPIOR0 = 0;
	GXOffset = GYOffset = 0;
	InitData();
	I2CInit();
	InitTC0();											//does NOT run until enabled
	InitTC1();											//runs immediately
	InitTC2();											//does NOT run until enabled
	DACinit();

#if CLOCKLEDENABLED
	InitLed();
	TurnOffLed();
#endif
	InitSwitch();
	InitFlashTbl();
	ReadEEConfiguration();									// Read from EEProm
	(void)ValidateNumericDisplay();
	(void)ValidateDialDisplay();
	InitINT0();												//does NOT run until enabled. Uses EEConfigData
	SetInt0Countdown();										//set Int0CntdownStart, before ResetPPSCntr()
	ResetPPSCntr();
	SetRotation();											//initializes countdown
	ASSERT(RotationCountdown == ((EEConfigData.BurninVal + 5) % 10));
	ReadEEUsrName();										//read user name from EEprom
	//Validate Baud rate value
	if (EEConfigData.BaudVal > (BAUDVALLAST - BAUDVALFIRST)) {
		EEConfigData.BaudVal = BAUDVAL19200 - BAUDVALFIRST;
		UpdateEEprom(EEPROM_BAUDVAL, EEConfigData.BaudVal);
	}
		
	UARTInit(EEConfigData.BaudVal);
	byte tmp = EEConfigData.StartApp;
	if (tmp > (APPGEN-APPCLOCK)) {							//validate
		tmp = APPCLOCK-APPCLOCK;
		UpdateEEprom(EEPROM_STARTAPP, tmp);					//reset to default
	}
	CurrAppMode = tmp + APPCLOCK;							//rebase
	InitTerm();												//Initialize the Terminal Application
	ClrLedMorseData();
//
// Show the Help Screen while synchronizing the RTC and INT0
//
	SetHelpScanTbl();										//No need to set RefreshCnt & UpTime strings
	fpCurrentRefresh = HelpScreenRefresh;					//set Splash Refresh func ptr (word address)
//
// See if the RTC was ever initialized. This requires that the EEPROM is
// not erased when programming the flash (Set EESAVE Fuse).
// Test first if an RTC is present. If not, skip this
// otherwise the EEPROM will be set when programming the AVR,
// even in parallel programming mode.
//
	StopRTCClockOut();										//dummy call to test RTC presence
	if (!Flags.RTCNotPresent && !EEConfigData.ClockEverRun) {		//was the clock run before
		CleanInitRTC();
	}
	RTCReadTime(TRUE);										//this will activate the ds1307 if needed
	RTCReadDate();
//
// If RTC not present, also skip the 1PPS signal synchronization.
//
	if (!Flags.RTCNotPresent) {
		//
		// Synchronize to the next second boundary before starting the 1 PPS signal
		//
		byte tmp;
		tmp = AVRTimeDateBlk.Secs;
		do {
			ClockWorks();									//interrupts still off i.e. time not updated
			RTCReadSecs(TRUE);
		} while (tmp == AVRTimeDateBlk.Secs);	
		--AVRTimeDateBlk.Secs;								//UpdateSecs will increment again
		UpdateSecs();										//we just ran a second off the clock
	}
//
// Start the Clock
//
	EnableINT0();											//start accepting RTC interrupts
	fClockIsRunning = TRUE;
#if NETFREQUENCY
	EnableTC2();
#endif
	sei();
//
// check if the clock started up in a DST window (RTC doesn't know about DST)
//
	DSTRetroApply();										//needs interrupts for message
//
// setup the clock screen. Do this before calling ClockWorks()
//
	(void)SetDialDisplay();
//
// Still showing the Splash Screen
//
	UsrShowAll();
//
// Forever loop
//
	BOOL firstRun = TRUE;
	do {
		fpCurrentRefresh = MenuRefresh;						//Refresh func ptr in Menu Mode (word address)
		MainMenuProcessing(firstRun);						//no menu on firstRun
		firstRun = FALSE;
//
// force complete clock redraw (except the dial)
//
		SetDisplay();

		if (CurrAppMode == APPCLOCK) {
			if (RunClock()) {								//TRUE means we changed app in UsrCmd
				firstRun = TRUE;							//if so, skip menu again
			}
		} else if (CurrAppMode == APPCALIBRATE) {
			if (RunCalibrate()) {							//TRUE means we changed app in UsrCmd
				firstRun = TRUE;							//if so, skip menu again
			}
		} else if (CurrAppMode == APPTERM) {
			RunTerm();
		} else if (CurrAppMode == APPGEN) {
			RunFuncGen();									//This won't ever return. MiniDDS terminated by reboot only
		} else if (CurrAppMode == APPDEMO) {
			UsrDemo();										//Calls ClockWorks(). This won't ever return. Demo ends in reboot
		} else if (CurrAppMode == APPBOOT) {
			InitiateSysReset();								//Restart will reload CurrAppMode properly
		}
	} while (TRUE);

	return 1;
}

BOOL RunClock(void)
{
	// keep in mind that selecting the menu exits this function and restarts when continuing.
	// Therefore, no general init at the beginning of RunClock()
	fpCurrentRefresh = ClockRefresh;							//Refresh func ptr in Clock Mode (word address)
	ScanTblNum->Yoffset = NUMERICVPOS;							//Vertical Position for Numeric Field. Identical for all modes.

	do {
		ClockWorks();											//show the clock and process events

		if (GPIOR0 & _BV(fUPS2)) {
			DisplayHelpscreen();
		}
		//
		// We are in IDLE state
		// Did we receive an S1 Button Up Message (Menu mode)?
		//
		if (GPIOR0 & _BV(fUPS1)) {
			GPIOR0 &= ~_BV(fUPS1);
			return FALSE;
		}
		//
		// We are in CMODIDLE state
		// Did we receive an S1 Button TimeOut Message (Change Time/Date)?
		//
		if (GPIOR0 & _BV(fTimeoutS1)) {							//TC0 flag
			GPIOR0 &= ~_BV(fTimeoutS1);
			MainCModProcessing();
		}

		if (EEConfigData.GPSInEnabled) {						//are we processing GPS info
			CheckGPSInput();									//process GPS records, if any
		} else {												//no, command line input only
			//
			// Was a char received on the UART
			//
			if (UARTHasData()) {								//Command Line Input processing
				//
				// UART char received. Process command line input
				//
				if (UsrCommand()) {								//ClockWorks can be called from here!
					return TRUE;								//returns TRUE is we changed app
				}
			}
		}
	}
	while (TRUE);
}
//
// Suggestion: merge code with RunClock()
//
BOOL RunCalibrate(void)
{
	fpCurrentRefresh = CalibrationRefresh;						//Refresh func ptr in Calibration Mode (word address)
	do {
		ClockWorks();											//show the clock and process events

		if (GPIOR0 & _BV(fUPS2)) {
			DisplayHelpscreen();
		}
		//
		// We are in IDLE state
		// Did we receive an S1 Button Up Message (Menu mode)?
		//
		if (GPIOR0 & _BV(fUPS1)) {
			GPIOR0 &= ~_BV(fUPS1);
			return FALSE;
		}
		if (EEConfigData.GPSInEnabled) {						//are we processing GPS info
			CheckGPSInput();									//process GPS records, if any
		} else {												//no, command line input only
			//
			// Was a char received on the UART
			//
			if (UARTHasData()) {								//Command Line Input processing
				//
				// UART char received. Process command line input
				//
				if (UsrCommand()) {								//ClockWorks can be called from here!
					return TRUE;								//returns TRUE is we changed app
				}
			}
		}
	}
	while (TRUE);
}

//
// Code common between the Clock/Menu/Change/Terminal/Calibration mode of operation
//
void ClockWorks(void)
{
	ASSERT(fpCurrentRefresh != NULL);
	// executed even if chrono is not visible. Test for NUMHEX or NUM24Hr or make a fChronoActive flag.
	if (EEConfigData.ChronoEnabled) {
	//
	//  Could use a more predictable timer here
	//
		uint8_t v = RefreshCnt & 0x07;
		if (v == 0) {											//every 8 refresh cycles
			UpdateChronoBuf();
		}
	}
	fpCurrentRefresh();											//refresh the display.
	//
	// Check if the push button switch S1 is closed. Skip if we are already in debouncing mode
	// if HIGH (set), button is UP. If LOW (clear), button is DOWN
	//
	if (!(GPIOR0 & _BV(fDebouncing))) {			//skip test if already debouncing
		//check if any push button switch pin is low (meaning down)
		if ((SWPin	& (_BV(SW1Bit)|_BV(SW2Bit))) != (_BV(SW1Bit)|_BV(SW2Bit))) {
			StartKeyDebouncing();
		}
	}
	//
	// Check if the push button switch S2 is closed.
	// Reboot if it is.
	//
	if (GPIOR0 & _BV(fTimeoutS2)) {					//did SW2 timeout
		GPIOR0 &= ~_BV(fTimeoutS2);
		InitiateSysReset();						//inline function to prevent too many register saves
	}

#if CLOCKLEDENABLED
	if (fUpdateLed) {
		ASSERT(EEConfigData.LedOption == LEDMORSE);
		byte byteOfs, bitMsk;
		byteOfs = LedMorseTail >> 3;
		bitMsk = pgm_read_byte(&_BVMap[LedMorseTail & 0x07]);
		setLed(LedMorseData[byteOfs] & bitMsk);
#if EXTDEBUG
		if (LedMorseData[byteOfs] & bitMsk) {
			UARTSendByte('1', TRUE);
		} else {
			UARTSendByte('0', TRUE);
		}
#endif
		++LedMorseTail;
		fUpdateLed = FALSE;
}
#endif


	if (GPIOR0 & _BV(fUpdSecond)) {								//Do we need to advance a second
		//
		// code executed once a second
		//
#if EXTDEBUG
		//
		// The fUpdSecond code is executed AFTER the 1PPS signal interrupt occured
		// since we need to finish the Screen Refresh. Experimentation (on Atmega32)
		// shows that we are at most 10 mSecs delayed (in clock mode)
		//
		if (TimeTicks >= 2) {
			__asm__ __volatile__ ("nop" ::);					//allow for a breakpoint
		}
		byte v1 = PPSTimeOutCnt;
		byte v2 = EnablePPSTimeOut();							//reloads PPSTimeOutCnt 
		if (((v2 - v1) != 0) && (v2 < 30) && !TestPlayModes()) { //if same, PPSTimeOutCnt was reloaded && less than 30 ticks passed?
			// we have a very short second (<30 ticks)
			++ShortSecCnt;
			ShowSecSkippedMsg();
		} else
#endif
		{
			++UpTime;
			if (CurrPlayMode & RUNCLOCKREVERSE) {
				ReverseSecs();
			} else {
				UpdateSecs();
			}
		}
		if (Flags.DemoEnabled & _BV(fDemoGoing)) {
			//
			// if doing PPS warning demo, don't turn off PPSMissing
			//
			if (!(Flags.DemoEnabled & _BV(fDemoPPS))) {
				PPSMissingOff();		//Turn off PPS missing warning (if on)
			}
			//
			// if doing Burn-In Demo, move grid once a second
			//
			if (Flags.DemoEnabled & _BV(fDemoBurnIn)) {
				RotateXYOffset();
			}
		} else {
			PPSMissingOff();								//Turn off PPS missing warning
		}
		if ((CurrAppMode == APPCLOCK) && 
			(AVRTimeDateBlk.Hrs == EEConfigData.AlarmHrs) &&
			(AVRTimeDateBlk.Mins == EEConfigData.AlarmMins) &&
			(AVRTimeDateBlk.Secs == EEConfigData.AlarmSecs) &&
			(EEConfigData.AlarmDisabled == FALSE) &&
			(Flags.DemoEnabled == 0) &&
			(DialFlashPtr == NULL) &&
			//Next test assumes ALARM states are the highest states
			(CModState < CMODPREALARMHR)) {
			// Alarm time reached. Flash DialMarks/DialDots/Frame iff clock is being displayed.
			// Won't work if FAST playmode is active since it relies on Flags.TimeOut
			DialFlashPtr = AddFlashItem(ScanTbl_Dial, FLASHPERSEC*60, NULL);			//1 minute
			SetAlarmCountdown();				//needed to clr out DialFlashPtr after 1 minute.
		}
		//
		// Add more items due at the seconds boundary here
		//
#if DEBUG
//		ShowCnt(IntrpCnt);
		IntrpCnt = 0;		//may be interrupted. Ignore.
//		ShowCnt(RefreshCnt);
//		ShowCnt((uint)OldTimeTicks);
//		UARTSendByte('.', TRUE);
#endif
		LastRefreshCnt = RefreshCnt;
		RefreshCnt = 0;
		GPIOR0 &= ~_BV(fUpdSecond);							//clear Seconds flag
		ASSERT(GetMemUnused() > 50);						//at least 50 bytes unused memory.
	}
	// See if the 1 PPS signal is missing
	if (fTimeOut) {											//Did we get a Timeout on the PPS signal
#if NETFREQUENCY
#if DEBUG
		ShowCnt(NetCycleCnt);
#endif
		NetCycleCnt = 0;
#endif
		TimeTicks = 0;										//re-initialize 100Hz counter (INT0 is not executed)
		PPSMissingOn();										//Turn on the PPS missing warning

#if EXTDEBUG
		++LongSecCnt;
#endif
		if (CurrPlayMode & RUNCLOCKREVERSE) {
			ReverseSecs();
		} else {
			UpdateSecs();
		}
		fTimeOut = FALSE;
	}			
	if (GPIOR0 & _BV(fFLASH)) {								//Do we need to flash any fields
		//
		// Table Based Flashing
		//
		ProcessFlashTbl();
		GPIOR0 &= ~_BV(fFLASH);								//Turn off Flash bit
	}
	//
	// Time to process GPS records again?
	//
	if (TC1CountdownActive() == FALSE) {
		Flags.GPSOKToUpdate = TRUE;							//ok to process GPS record
	}

	if (Flags.UpdMin) {										//Skip tests for hours (fHR) boundary since it's only set if UpdMin is set
		//
		// Check rotate Grid Offset
		//
		if (--RotationCountdown == 0) {
			RotateXYOffset();
			RotationCountdown = ((EEConfigData.BurninVal + 5) % 10);	//initialize again
		}
		//
		if (EEConfigData.Verbose) {
			UsrShowStatus();
		}
		Flags.UpdMin = FALSE;								//turn off Minute Flag

		if (Flags.UpdHour) {								//Are we on an hour boundary
			Flags.UpdHour = FALSE;							//turn off Hour Flag
		}
	}
	// No need to stop flashing here since dialflashing times out automatically
	if (fClrAlarm && DialFlashPtr) {
		DialFlashPtr = NULL;
		fClrAlarm = FALSE;
	}

	if (Flags.SerialInOverflow) {
		if (EEConfigData.LedOption == LEDDEBUG) {		//if LedOption is Debug, use LED to signal overflow
			LEDPin |= _BV(LEDBit);						//toggle the LED
		}
#if DEBUG
		UARTPrintfProgStr(MsgPszSerialInOverflow);
#endif
		Flags.SerialInOverflow = FALSE;
	}
}

void InitData(void)
{
	S1.KeyState = -1;			//this prevents an initial UP
	S2.KeyState = -1;			//this prevents an initial UP
	Flags.BatteryLow = -1;			//special value!
	CModState = CMODIDLE;
	CMenuState = CMENUIDLE;
//
// These are regular initializations but unneeded in RELEASE mode
// since all memory is set to 0 at startup.
//
#if DEBUG
	fUpdateLed = FALSE;
	ExtDbg1 = 0;
	ExtDbg2 = 0;
	SecondCnt = 0;
	RestartCounter = 0;
	DBGStage = 0;
	ExtraTimer = 0;
	LastRefreshCnt = RefreshCnt = 0;
	UpTime = 0;
#if EXTDEBUG
	ShortSecCnt = 0;
	LongSecCnt = 0;
#endif
	Flags.RTCNotPresent = FALSE;
	Flags.DisplayRedraw = FALSE;
	fClockIsRunning = FALSE;
	Flags.UseDblHand = FALSE;
	Flags.SerialInOverflow = FALSE;
	Flags.DemoEnabled = 0;
	fClrAlarm = FALSE;
	ShowDialDots = FALSE;
	S1.Position = 0;			//S1 is UP
	S2.Position = 0;			//S2 is UP
	S1.fAutorepeat = 0;
	S1.okToRepeat = 0;
	S2.fAutorepeat = 0;
	S2.okToRepeat = 0;
	DialFlashPtr = NULL;
	PPSFlashPtr = NULL;
	BatFlashPtr = NULL;
	CModFlashPtr1 = NULL;
	CModFlashPtr2 = NULL;
	CMenuFlashPtr = NULL;
	Flags.RotationDisabled = FALSE;
	RotationIndex = 0;
	TextFlashField = 0;
	CTextFlashPtr = 0;
	UsrNameDflt = 0;
	TimeTicks = 0;
	Time2Ticks = 0;
	GPSLastHrFlag = 0;
	CurWaveTblPtr = NULL;
#if EXTDEBUG
	nSerialInMissed = 0;
	LastGoodByteIdx = 0;
#endif
	pChronoDispBuf = NULL;
//
// Preset ScanTbl with default data
//
	byte i;
	pByte p = (pByte)ScanTbl;
	for (i = 0; i < (SCANTABLEN + SCANSTRTABLEN + SCANSSTRTABLEN)* sizeof(ScanTblEntry_t); ++i) {
		*p++ = 0;
	}
	memset(&AVRTimeDateBlk, 0, sizeof(AVRTimeDateBlk));
//
// Initialize GPS related data
//
	EEConfigData.GPSInEnabled = 0;			//Default Off
	memset(&UTCTimeDateBlk, 0, sizeof(UTCTimeDateBlk));
	EEConfigData.GPSOffset = 0;
//
// True debug code
//
// clear UsrName buffer
//
	p = (pByte)UsrNameBuf;
	for (i = 0; i < (MAXUSRNAMELEN+1) * sizeof(byte); ++i) {
		*p++ = 0;
	}
	// p now points to UsrNameBufGuard
	*p     = 255;
	*(p+1) = 255;
	*(p+2) = 255;
	*(p+3) = 255;
#endif		// DEBUG

#if NETFREQUENCY
	NetCycle = 1;
	NetCycleCnt = 0;
	NetCycleRefresh = 0;
#endif

	CurScanTblLen = SCANTABLEN;
	ScanTbl[stDialMarks].pVect = (VectorTblEntry_t*)DialData;	//Dial Data
	CurrPlayMode = FUNNORM;
	Flags.GPSOKToUpdate = TRUE;									//time to process GPS record
	PPSTimeOutCnt = ReloadPPSTimeOutCnt = TIMEOUTTICKS;			//initialize tick counter for timeout
	FlashCount = FLASHTICKS;									//initialize tick counter for flashing

	//Make sure EEPROM data don't overlap
	ASSERT(EEPROM_MAX < EEPROM_USRNAME);

	LedMorseSecs = 20;
}

void UpdateSecs(void)
{
#if CLOCKLEDENABLED
	BlinkLed();
	LedMorseUpdate();
#endif
	//
	// Update Time
	//
	if (++AVRTimeDateBlk.Secs < 60) {			//check Seconds
		//
		// Update Display Pointers.
		// Seconds Hands only unless Flags.DisplayRedraw is set
		//
		if (Flags.DisplayRedraw || ((EEConfigData.ioctl & _BV(IOCTL_SECSOMODE)) != 0)) {
			SetDisplay();
		} else {
			SetSecsHand();
			//
			// Don't update numeric field if we're modifying the date; the
			// user may be updating the value we're changing here.
			// This assumes that the numeric field is in DATE mode when modifying
			// the date.
			//
			if (!(CModState & CMODDATE)) {
				SetNumDisplay();
			}
		}
		return;
	}
	// a Minute has passed. Set Flag and keep time
	Flags.UpdMin = TRUE;							//set minutes flag
	AVRTimeDateBlk.Secs = 0;						//reset Seconds
	if (++AVRTimeDateBlk.Mins >= 60) {				//update minutes
		// an Hour has passed.
		Flags.UpdHour = TRUE;						//set Hours flag
		AVRTimeDateBlk.Mins = 0;					//reset minutes
		if (++AVRTimeDateBlk.Hrs >= 24) {			//update Hours
			//
			// 24 hour overflow.
			// Skip the RTC time update at the Hour transition so we don't miss a date transition
			// (if 00 hr set before internal RTC transition from 24->00, then no date update)
			// Manually inc date to get a proper display (don't know exactly when RTC date update happens)
			//
			AVRTimeDateBlk.Hrs = 0;
			// if modifying the date, don't change
			if (!(CModState & CMODDATE)) {
				IncrementDate(&AVRTimeDateBlk);		//no need to update the RTC since it will follow shortly
			}
			//
			// Finally, apply any DST adjustments (if enabled etc.)
			// This seems unnecessary. Is there EVER a DST adjustment on a 24HR transition?
			//
			if (CurrPlayMode & RUNCLOCKFAST) {
				DSTRetroApply();
			}
			SetDisplay();
			return;
		}
	}
	// Time has been updated. Now update the rtc which is the backup time.
	if (CurrPlayMode & RUNCLOCKFAST) {
		SetDisplay();									//Fast Mode: exit
		return;
	}
	ASSERT(!(CurrPlayMode & RUNCLOCKREVERSE));			//we should NOT get here in reverse mode
	RTCSetTime();
	//
	// if modifying the date, don't change the RTC, nor do DST correction.
	// the menu code will do it explicitly.
	//
	if (!(CModState & CMODDATE)) {
		RTCReadDate();										//sync date
		// Finally, apply any DST adjustments (if enabled etc.)
		DSTRetroApply();
	}
	SetDisplay();
}
//
// Set the clock back one second. Available for Play Mode. RTC not updated.
//
void ReverseSecs(void)
{
#if CLOCKLEDENABLED
	BlinkLed();
#endif
	LedMorseUpdate();
	//Reverse Time
	if (--AVRTimeDateBlk.Secs != 0xff) {
		// Update Display Pointers. Seconds Hands only unless Flags.DisplayRedraw is set
		if (Flags.DisplayRedraw == TRUE) {
			SetDisplay();
		} else {
			SetSecsHand();
			//
			// Don't update numeric field if we're modifying the date
			// This assumes that the numeric field is in DATE mode when modifying
			//
			if (!(CModState & CMODDATE)) {
				SetNumDisplay();
			}
		}
		return;
	}
	// Minute boundary
	AVRTimeDateBlk.Secs = 59;
	Flags.UpdMin = TRUE;								//set minutes flag
	if (--AVRTimeDateBlk.Mins == 0xff) {				//update minutes
		// an Hour has passed.
		AVRTimeDateBlk.Mins = 59;						//reset minutes
		Flags.UpdHour = TRUE;							//set Hours flag
		if (--AVRTimeDateBlk.Hrs == 0xff) {				//update Hours
			// a Day has passed.
			AVRTimeDateBlk.Hrs= 23;
		}
	}
	SetDisplay();
}
//
// Update Display Addresses for clock hands
//
void SetClockHands(void)
{
	SetSecsHand();
	SetMinsHand();
	SetHrsHand();
}

void SetDisplay(void)
{
	Flags.DisplayRedraw = FALSE;
	SetDialDigits();
	SetClockHands();
	SetNumDisplay();
	SetUsrNameField(ScanTblUsrName->Yoffset);
}
//
// Reset the clock to the actual time after being in fun mode
// Only do this if both Fast and Reverse modes are off
//
void SetActualTime(void)
{
	if (!TestPlayModes()) {
		RTCReadTime(FALSE);
		RTCReadDate();
		SetDisplay();
	}
}
//
// Update Display Addresses for Seconds hand
//

void SetBinaryClock(byte idx, byte v, PGM_A tbl1, PGM_A tbl2)
{
	byte d = bintobcd(v);
	ScanTbl[idx].pVect = (VectorTblEntry_t*)pgm_read_word(&(tbl1[d >> 4]));			//High BCD digit
	UpdateCModFlashEntry(&(ScanTbl[idx]), ScanTbl[idx].pVect);
	ScanTbl[idx + 1].pVect = (VectorTblEntry_t*)pgm_read_word(&(tbl2[d & 0x0f]));	//Low BCD digit
	UpdateCModFlashEntry(&(ScanTbl[idx + 1]), ScanTbl[idx + 1].pVect);
}

void SetDigitalClock(byte idx, byte v)
{
	byte d = bintobcd(v);
	ScanTbl[idx].pVect = (VectorTblEntry_t*)pgm_read_word(&DigNums[d >> 4]);			//High BCD digit
	UpdateCModFlashEntry(&(ScanTbl[idx]), ScanTbl[idx].pVect);
	ScanTbl[idx + 1].pVect = (VectorTblEntry_t*)pgm_read_word(&DigNums[d & 0x0f]);	//Low BCD digit
	UpdateCModFlashEntry(&(ScanTbl[idx + 1]), ScanTbl[idx + 1].pVect);
}

void SetSecsHand(void)
{
	if (EEConfigData.CurrDialVal == DIALDIG) {
		SetDigitalClock(stSecHand, AVRTimeDateBlk.Secs);
		byte *p = &DialDots[AVRTimeDateBlk.Secs * DialDotsDataSize];
		BeamDotX0 = pgm_read_byte(p);
		BeamDotY0 = pgm_read_byte(p+1);
		BeamDotX1 = pgm_read_byte(p+2);
		BeamDotY1 = pgm_read_byte(p+3);
		BeamDotX2 = pgm_read_byte(p+4);
		BeamDotY2 = pgm_read_byte(p+5);
		BeamDotX3 = pgm_read_byte(p+6);
		BeamDotY3 = pgm_read_byte(p+7);
	} else if (EEConfigData.CurrDialVal == DIALBIN) {
		SetBinaryClock(stSecHand, AVRTimeDateBlk.Secs, (PGM_A)Led3Tbl, (PGM_A)Led4Tbl);
	} else {
		ScanTbl[stSecHand].pVect = (VectorTblEntry_t*)(SecPtrData + (AVRTimeDateBlk.Secs * SecHandsDataSize));
		UpdateCModFlashEntry(&(ScanTbl[stSecHand]), ScanTbl[stSecHand].pVect);		//this checks BOTH cmodflash pointers
	}
}
//
// Update Display Addresses for Minutes hand
//
void SetMinsHand(void)
{
	if (EEConfigData.CurrDialVal == DIALDIG) {
		SetDigitalClock(stMinHand, AVRTimeDateBlk.Mins);
	} else if (EEConfigData.CurrDialVal == DIALBIN) {
		SetBinaryClock(stMinHand, AVRTimeDateBlk.Mins, (PGM_A)Led3Tbl, (PGM_A)Led4Tbl);
	} else {
		ScanTbl[stMinHand].pVect = (VectorTblEntry_t*)(MinPtrData + (AVRTimeDateBlk.Mins * MinHandsDataSize));
		UpdateCModFlashEntry(&(ScanTbl[stMinHand]), ScanTbl[stMinHand].pVect);
	}
}
//
// Update Display Addresses for Hours hand
//
// Hours are counted on a 0..23 scale.
// compute Hours positions based on 12 minutes granularity:
// pos = (((hours % 12) * 60) + min) / 12
//
void SetHrsHand (void)
{
	if (EEConfigData.CurrDialVal == DIALDIG) {
		if ((EEConfigData.ioctl & _BV(IOCTL_12HRDIG)) != 0) {
			SetDigitalClock(stHrHand, pgm_read_byte(AMPMMapping + AVRTimeDateBlk.Hrs));
		} else {
			SetDigitalClock(stHrHand, AVRTimeDateBlk.Hrs);
		}
	} else if (EEConfigData.CurrDialVal == DIALBIN) {
		SetBinaryClock(stHrHand, AVRTimeDateBlk.Hrs, (PGM_A)Led2Tbl, (PGM_A)Led4Tbl);
	} else {
		ScanTbl[stHrHand].pVect = (VectorTblEntry_t*)(HrPtrData + ((AVRTimeDateBlk.Hrs % 12) * 5 + (AVRTimeDateBlk.Mins / 12)) * HrHandsDataSize);
		UpdateCModFlashEntry(&(ScanTbl[stHrHand]), ScanTbl[stHrHand].pVect);
	}
}
//
//
char *SetAsciiFromBin(char *p, byte v)
{
	if (EEConfigData.CurrNumVal == NUMHEX) {
		return FormatHex(p, v);
	} else {
		return FormatDecimal(p, v);
	}
}
//
// Compute Numeric Display Buffer
//
void SetNumDisplay(void)
{
	char *p = (char *)NumDispBuf;
	byte displayMode = EEConfigData.CurrNumVal;
	// NULL string if no Numeric Display
	if (displayMode != NUMOFF) {
		// date formatted string if NUMDATE Display
		if (displayMode == NUMDATE) {
			p = FormatAVRDate(p);
		} else if (displayMode == NUMALARM) {
			p = SetAsciiFromBin(p, EEConfigData.AlarmHrs);		//convert to (hexa)decimal
			p[0] = ':';
			p = SetAsciiFromBin(p + 1, EEConfigData.AlarmMins);	//convert to (hexa)decimal
			p[0] = ':';
			p = SetAsciiFromBin(p + 1, EEConfigData.AlarmSecs);	//convert to (hexa)decimal
			p[0] = HALFSPACE;
			p[1] = 'A';
			p[2] = 'L';
			p += 3;
		} else {
			byte hrs = AVRTimeDateBlk.Hrs;
			// Convert to AM/PM hours if NUM12HR mode
			if (displayMode == NUM12HR) {
				hrs = pgm_read_byte(AMPMMapping + hrs);
			}
			p = SetAsciiFromBin(p, hrs);						//convert to (hexa)decimal
			p[0] = ':';
			p = SetAsciiFromBin(p + 1, AVRTimeDateBlk.Mins);	//convert to (hexa)decimal
			p[0] = ':';
			p = SetAsciiFromBin(p + 1, AVRTimeDateBlk.Secs);	//convert to (hexa)decimal
			if (displayMode == NUM12HR) {						//Are we displaying AM/PM
				p[0] = HALFSPACE;
				p[1] = (AVRTimeDateBlk.Hrs < 12) ? 'A' : 'P';
				p[2] = 'M';
				p += 3;
			} else if (EEConfigData.ChronoEnabled && ((displayMode == NUM24HR) || (displayMode == NUMHEX))) {
				// add chronometer display
				*p++ = ':';
				pChronoDispBuf = p;
				UpdateChronoBuf();
				p += 2;
			}
		}
	}
	*p = 0;				//terminate. Critical if NUMOFF
	ScanTblNum->pVect = (VectorTblEntry_t*)NumDispBuf;								//Update Scan Table
	ScanTblNum->Xoffset = ComputeSRamHPos((char *)NumDispBuf);						//center
}
//
// Set Flash Field and update Numeric Display Field
// Build an Alternate Text Buffer for flashing
//
void SetNumFlashField(byte v)
{
	TextFlashField = v;
	SetNumDisplay();
	if (TextFlashField) {
		char *p = (char *)NumFlashDispBuf;
		memcpy(p, NumDispBuf, NUMDISPBUFLEN+1);											//also copy zero terminator
		if (TextFlashField == MinInString) {
			p += 3;
		} else if (TextFlashField == SecInString) {
			p += 6;
		} else if (TextFlashField == YearInString) {
			// flash Year field: 4 digits
			p += 7;
			*p++ = ' ';
			*p++ = ' ';
		} else if (TextFlashField == MonthInString) {
			p += 3;
			*p++ = ' ';
		} else {
			ASSERT(TextFlashField == HourInString);										//Same as DayInString
		}
		*p++ = ' ';
		*p++ = ' ';
	}
}

void UpdateChronoBuf(void)
{
	ASSERT(EEConfigData.ChronoEnabled);
	if ((EEConfigData.CurrNumVal == NUM24HR) || (EEConfigData.CurrNumVal == NUMHEX)) {
//
// TimeTicks was moved to Timer1, it will run from 0..59
//
		byte t = TimeTicks >> 1;						//time 1.65625
		byte t2 = t >> 2;
		byte t3 = t2 >> 2;
		t += (t2 + t3 + TimeTicks);
		//
		// TimeTicks can overflow (get over 100) if the 1PPS signal is missing and
		// we're faking the seconds transition after 1 1/30 a seconds
		//
		if (t >= 100) t -= 100;
		if (CurrPlayMode & RUNCLOCKREVERSE) {
		// clock is running reverse. display 99-x
			t = 99 - t;
		}
		char *p;
		if (pChronoDispBuf) {
			p = SetAsciiFromBin(pChronoDispBuf, t);
			*p = 0;
		}
	}
}
//
// Set the Dial Digits Ptr to use
//
void SetDialDigits(void)
{
	pByte p;
	ScanTbl[stDialDigits].Xoffset = 0;
	if (EEConfigData.CurrDialVal == DIALBIN) {
		p = NULL;
	} else if (EEConfigData.CurrDialVal == DIALDIG) {
		p = DigSeparator;
		ScanTbl[stDialDigits].Xoffset = DigSepHPOS;
	} else if ((EEConfigData.CurrNumVal == NUMOFF) || (EEConfigData.CurrNumVal == NUMDATE)) {
		// Numeric Display is either Off or Date. Use 24 hrs display
		p = CurrDial12DigitsPtr;										//normal dial
		if (AVRTimeDateBlk.Hrs >= 12) {									//current hours value
			p = CurrDial24DigitsPtr;
		}
	} else {
		p = CurrDial12DigitsPtr;										//normal dial
	}
	ScanTbl[stDialDigits].pVect = (VectorTblEntry_t *)p;				//store dialdigits
}
//
//	byte ValidateNumericDisplay()
//
// 	returns CurrNumVal
//
byte ValidateNumericDisplay(void)
{
	if ((EEConfigData.CurrNumVal < NUM12HR) || (EEConfigData.CurrNumVal > NUMLAST)) {
		EEConfigData.CurrNumVal = NUM12HR;
	}
	return EEConfigData.CurrNumVal;
}
//
//	byte ValidateDialDisplay(void)
//
//	returns CurrDialVal
//
byte ValidateDialDisplay(void)
{
	if ((EEConfigData.CurrDialVal < DIAL12HR) || (EEConfigData.CurrDialVal > DIALLAST)) {
		EEConfigData.CurrDialVal = DIAL12HR;
	}
	return EEConfigData.CurrDialVal;
}
//
// UpdateNumericDisplay(byte newNumVal). Called after a change
//
void UpdateNumericDisplay(byte newNumVal)
{
	EEConfigData.CurrNumVal = newNumVal;
	byte v = GetNumDisplayMode();			//returns validated CurrNumVal
	UpdateEEprom(EEPROM_NUMERIC, v);
	SetDialDigits();
	SetNumDisplay();
}
//
// SetDialDisplay
// returns valid CurrDialVal
//
byte SetDialDisplay(void)
{
	byte v = ValidateDialDisplay();
	if (v == DIALBIN) {
		//Update status field location
		ScanTblStatG->Xoffset = STATUSGHPOSBIN;
		ScanTblStatG->Yoffset = STATUSVPOSBIN;
		ScanTblStatB->Xoffset = STATUSBHPOSBIN;
		ScanTblStatB->Yoffset = STATUSVPOSBIN;
		ScanTblStatP->Xoffset = STATUSPHPOSBIN;
		ScanTblStatP->Yoffset = STATUSVPOSBIN;
		SetBinaryClockPtrs();
	} else 	if (v == DIALDIG) {
		ScanTblStatG->Xoffset = STATUSGHPOSDIGITAL;
		ScanTblStatG->Yoffset = STATUSVPOSDIGITAL;
		ScanTblStatB->Xoffset = STATUSBHPOSDIGITAL;
		ScanTblStatB->Yoffset = STATUSVPOSDIGITAL;
		ScanTblStatP->Xoffset = STATUSPHPOSDIGITAL;
		ScanTblStatP->Yoffset = STATUSVPOSDIGITAL;
		SetDigitalClockPtrs();
	} else {
		if (v == DIAL12HR) {
			// set Default  Digits Display
			CurrDial12DigitsPtr = DialDigits12;		//Current Dial Digits Display
			CurrDial24DigitsPtr = DialDigits12;		//Current 24 Hours Dial Digits Display
		} else if (v == DIAL24HR) {
			//set Default  Digits Display
			CurrDial12DigitsPtr = DialDigits12;		//Current Dial Digits Display
			//set 24 Hours Digits Display
			CurrDial24DigitsPtr = DialDigits24;		//Current 24 Hours Dial Digits Display
		} else if (v == DIALROMAN) {
			//set Roman Numerals Display
			CurrDial12DigitsPtr = DialDigitsRoman;	//Current Dial Digits Display
			CurrDial24DigitsPtr = DialDigitsRoman;	//Current 24 Hours Dial Digits Display
		} else {
			ASSERT(v == DIALMIN);
			//set Minimum Digits Display
			CurrDial12DigitsPtr = DialDigitsMin;	//Current Dial Digits Display
			CurrDial24DigitsPtr = DialDigitsMin;	//Current 24 Hours Dial Digits Display
		}
		ScanTblStatG->Xoffset = STATUSGHPOSANALOG;
		ScanTblStatG->Yoffset = STATUSVPOSANALOG;
		ScanTblStatB->Xoffset = STATUSBHPOSANALOG;
		ScanTblStatB->Yoffset = STATUSVPOSANALOG;
		ScanTblStatP->Xoffset = STATUSPHPOSANALOG;
		ScanTblStatP->Yoffset = STATUSVPOSANALOG;
		SetAnalogClockPtrs();
	}
	SetDisplay();
	return v;
}
//
// UpdateDialDisplay. Called after a change
//
// in:	New DialVal
//
void UpdateDialDisplay(byte newDialVal)
{
	EEConfigData.CurrDialVal = newDialVal;
	byte v = SetDialDisplay();								//returns validated CurrDialVal
	UpdateEEprom(EEPROM_DIAL, v);
	SetDialDigits();
	SetClockHands();
}

void SetAnalogClockPtrs(void)
{
//
// Clear Scan Table Ptrs and Horizontal values for Analog Clock Display
//
	for (byte i= stSecHand; i <= stHrHand2; ++i) {
		ScanTbl[i].pVect = NULL;
		ScanTbl[i].Xoffset = 0;								//WHY BOTHER IF pVect == NULL??
	}
//
// Update Text Display Field, if enabled
// Set once only except VPOS. Current code saves space
//
	SetUsrNameField(USRNAMEACLOCKVPOS);
//
// Set DialDigits and Dial itself
//
	ScanTbl_Dial->pVect = (VectorTblEntry_t*)DialData;		//Dial Marks
	SetDialDigits();
	ShowDialDots = FALSE;
}
//
// SetBinaryClockPtrs
//
// set Horizontal values for Binary Clock Display
// the Vertical values are included in the tables
//
void SetBinaryClockPtrs(void)
{
	ScanTbl[stSecHand].Xoffset = LedSHHPOS;
	ScanTbl[stSecHand2].Xoffset = LedSLHPOS;
	ScanTbl[stMinHand].Xoffset = LedMHHPOS;
	ScanTbl[stMinHand2].Xoffset = LedMLHPOS;
	ScanTbl[stHrHand].Xoffset = LedHHHPOS;
	ScanTbl[stHrHand2].Xoffset = LedHLHPOS;
//
// Update Text Display Field, if enabled
//
	SetUsrNameField(USRNAMEBCLOCKVPOS);
//
// Disable DialDigits and Dial itself
//
	ScanTbl_Dial->pVect = (VectorTblEntry_t*)BinaryFrameData;
	SetDialDigits();
	ShowDialDots = FALSE;
}
//
// SetDigitalClockPtrs
//
// set Horizontal values for Digital Clock Display
// the Vertical values are included in the tables
//
void SetDigitalClockPtrs(void)
{
	ScanTbl[stSecHand].Xoffset = DigSHHPOS;
	ScanTbl[stSecHand2].Xoffset = DigSLHPOS;
	ScanTbl[stMinHand].Xoffset = DigMHHPOS;
	ScanTbl[stMinHand2].Xoffset = DigMLHPOS;
	ScanTbl[stHrHand].Xoffset = DigHHHPOS;
	ScanTbl[stHrHand2].Xoffset = DigHLHPOS;
//
// Update Text Display Field, if enabled
//
	SetUsrNameField(USRNAMEDCLOCKVPOS);
//
// Set DialDigits and Dial itself
//
	ScanTbl_Dial->pVect = (VectorTblEntry_t*)DialDots;		//Dial Dots
	SetDialDigits();
	ShowDialDots = ((EEConfigData.ioctl & _BV(IOCTL_NODIALDOTS)) == 0);
}

//
// ComputeSRamHPos
//	Compute Horizontal Pos necessary to center text
//
//	In:	ptr to SRam based text
//
//	Returns HPos pixel position, 0 if overflow
//
byte ComputeSRamHPos(char * pSRamText)
{
	uint w = 0;
	char ch;
	while ((ch = *pSRamText++) != 0) {
		w += GetCharWidth(ch);
	}
	if (w > 255) {				//overflow
		return 0;
	}
	return (255 - (byte)w) >> 1;
}
//
// byte GetCharWidth(char ch)
//
// returns char width in pixels
//
byte GetCharWidth(char ch)
{
	byte v;
	if (ch == ':') {
		v = COLONCHARWIDTH;
	} else if (ch == HALFSPACE) {
		v = HALFSPACECHARWIDTH;
	} else if (ch == '-') {
		v = DASHCHARWIDTH;
	} else if (ch == ',') {
		v = COMMACHARWIDTH;
	} else {
		v = DEFCHARWIDTH;
	}
	return v;
}

void UpdateUsrNameMode(byte newMode)
{
	ASSERT(newMode <= USRNAMELAST);
	ASSERT(newMode != USRNAMEEDIT);
	EEConfigData.UsrNameVal = newMode;
	if ((newMode != USRNAMEO) || ((EEConfigData.ioctl & _BV(IOCTL_SAVEOMODE)) != 0)) {
		UpdateEEprom(EEPROM_USRNAMEVAL, newMode);
	}
	//
	// need to call SetDialDisplay (not SetDisplay) since Usrname screen position
	// may have changed e.g. when booted in OFF position (ypos == 0), then change to ON in clock dial.
	//
	SetDialDisplay();
}
//
// Update Verbose setting
//
BOOL UpdateVerbose(byte newMode)
{
	EEConfigData.Verbose = newMode;
	UpdateEEprom(EEPROM_VERBOSE, EEConfigData.Verbose);
	return TRUE;
}
//
// UpdateChronoDisplay
//
BOOL UpdateChronoDisplay(byte val)
{
	EEConfigData.ChronoEnabled = val;
	UpdateEEprom(EEPROM_CHRONO, val);
	SetNumDisplay();
	return TRUE;
}
//
// UpdateLedOptions
//
BOOL UpdateLedOptions(byte val)
{
	EEConfigData.LedOption = val;
	if (val != LEDENABLED) {
		TurnOffLed();
	}
	ClrLedMorseData();
	LedMorseSecs = 20;					//if Morse enabled. Start quickly
	UpdateEEprom(EEPROM_LED, val);
	return TRUE;
}
//
// UpdateAlarm
//
BOOL UpdateAlarm(byte newVal)
{
	EEConfigData.AlarmDisabled = newVal;						//Store new Mode
	UpdateEEprom(EEPROM_ALARM, newVal);
	return TRUE;
}
//
// UpdateBurnInOptions
//
BOOL UpdateBurnInOptions(byte newVal)
{
	EEConfigData.BurninVal = newVal;
	UpdateEEprom(EEPROM_BURN, newVal);
	SetRotation();
	return TRUE;
}

BOOL UpdateIOCTL(uint w)
{
	EEConfigData.ioctl = w;
	UpdateEEprom(EEPROM_IOCTLL, w & 0xff);
	UpdateEEprom(EEPROM_IOCTLH, w >> 8);
	return TRUE;
}
//
// UpdateDSTMode
//
void UpdateDSTMode(byte newVal)
{
	EEConfigData.DSTMode = newVal;								//must be proper NUMERIC value (0..2)
	UpdateEEprom(EEPROM_DSTMODE, newVal);
//
// check if the new mode requires DST adjustments
//
	DSTRetroApply();
}
//
// Update PPSTrigger
//
BOOL UpdatePPSTrigger(byte newVal)
{
	EEConfigData.PPSTrigger = newVal;							//Store new Mode
	UpdateEEprom(EEPROM_PPSTRIG, newVal);
	InitiateSysReset();
	return TRUE;
}
//
// Result: TRUE if any Fun Mode is active
//
BOOL TestPlayModes(void)
{
	return (CurrPlayMode != FUNNORM);
}
//
// Disable all Fun Modes
//
void DisablePlayModes(void)
{
	if (CurrPlayMode & RUNCLOCKFAST) { 						//was clock running fast
		ReloadPPSTimeOutCnt = PPSTimeOutCnt = TIMEOUTTICKS;			//fast was on, turn off
	}
	CurrPlayMode = FUNNORM;
	SetActualTime();
}
//
// Update Fun Mode.
//
void UpdPlayMode(byte newMode)
{
	byte v = CurrPlayMode ^ newMode;
	if (v & RUNCLOCKFAST) {
		//
		// change in Fast Mode.
		//
		if (newMode	& RUNCLOCKFAST) {
			v = FASTTIMEOUTTICKS;					//Change to Fast
		} else {
			v = TIMEOUTTICKS;						//normal
		}
		ReloadPPSTimeOutCnt = PPSTimeOutCnt = v;
	}
	ClrLedMorseData();
	LedMorseSecs = 20;								//if Morse enabled. Start quickly
	CurrPlayMode = newMode;
	SetActualTime();
}
//
// Set Optional User Name Field
//
// IN: vertical position (Y) to use
//
void SetUsrNameField(byte vpos)
{
	if (EEConfigData.UsrNameVal == USRNAMEOFF) {		//0 means ON here
		ScanTblUsrName->pVect = NULL;
		return;
	}
	if (EEConfigData.UsrNameVal == USRNAMEON) {
		ScanTblUsrName->pVect = (VectorTblEntry_t *)UsrNameBuf;
	} else if (EEConfigData.UsrNameVal == USRNAMEDAY) {
		// use SRAMBuf to get the day of the week in RAM
		strcpy_P(SRAMBuf, (void *)pgm_read_word(&WeekDays[DayOfTheWeek(&AVRTimeDateBlk)]));
		ScanTblUsrName->pVect = (VectorTblEntry_t *)SRAMBuf;
	} else if (EEConfigData.UsrNameVal == USRNAMEO) {
		TotalUSDebt(&AVRTimeDateBlk, SRAMBuf);
		ScanTblUsrName->pVect = (VectorTblEntry_t *)SRAMBuf;
	} else {
		ASSERT(FALSE);
	}
	// Set User Name Display Field
	ScanTblUsrName->Xoffset = ComputeSRamHPos((char *)ScanTblUsrName->pVect);
	ScanTblUsrName->Yoffset = vpos;
}
//
// Read UsrName from EEprom and store in UsrNameBuf
//
void ReadEEUsrName(void)
{
	PGM_P pSrc;
	if ((EEConfigData.UsrNameCnt == 0) || (EEConfigData.UsrNameCnt > MAXUSRNAMELEN)){
		//if 0 or invalid, use default name
#if ds1307
		pSrc = MsgPszAltLogo;					//Sparkfun string
		EEConfigData.UsrNameCnt = 8;			//length
#else
		pSrc = MsgPszLogo;						//Dutchtronix string
		EEConfigData.UsrNameCnt = 11;			//length
#endif
		strcpy_P(UsrNameBuf, pSrc);				//pSrc is 0-terminated
		UsrNameDflt = TRUE;
	} else {
		for (byte i = 0; i < EEConfigData.UsrNameCnt; ++i) {
			UsrNameBuf[i] = EepromRead(EEPROM_USRNAME + i);
#if DEBUG
			if (UsrNameBuf[i] == 0xff) {
				UsrNameBuf[i] = ' ';
			}
#endif
		}
		UsrNameBuf[EEConfigData.UsrNameCnt] = 0;
	}
}
//
// Store UsrNameBuf in EEprom
//
void WriteEEUsrName(void)
{
	if ((Flags.DemoEnabled != 0) | UsrNameDflt) {		//No Update in Demo Mode or when using the default name
		return;
	}
	if (EEConfigData.UsrNameCnt == 0) {
	//if 0, means ReadEEUsrName was never called
	// or user selected an empty name. Save a single space
		UsrNameBuf[0] = ' ';
		UsrNameBuf[1] = 0;					//terminate
		EEConfigData.UsrNameCnt = 1;
	}
	UpdateEEprom(EEPROM_USRNAMECNT, EEConfigData.UsrNameCnt);
	for (byte i = 0; i < EEConfigData.UsrNameCnt; ++i) {
		UpdateEEprom(EEPROM_USRNAME + i, UsrNameBuf[i]);
	}
}
//
// Set Baudrate using 0 based internal value. Don't update the EEprom
//
void SetBaudVal(byte newBaudVal)
{
	ASSERT(newBaudVal <= (BAUDVALLAST - BAUDVALFIRST));
	EEConfigData.BaudVal = newBaudVal;
	UARTKillPendingOutput();
	UARTSetBaudVal(newBaudVal);
}
//
// Display a flashing 'B' for n seconds
//
// IN --  Flash Count
//
// OUT -- FlashPtr
//
FlashTblEntry_t *BatteryStartFlashing(byte flashCnt)
{
	return AddFlashItem(ScanTblStatB, flashCnt, (VectorTblEntry_t *)CharB);
}
//
// The RTC reports a Low Battery (VL flag). This only has meaning when 
// the AVR starts up (battery should not go low while running from 5V)
//
void BatteryWarning(void)
{
	if (Flags.BatteryLow == 0xff) {				//was this reported earlier. If so, ignore
		Flags.BatteryLow = TRUE;				//set low battery flag
		if (BatFlashPtr == NULL) {
			BatFlashPtr = BatteryStartFlashing(FLASHPERSEC*60);			//60 seconds max 'B' warning
		}
	}
}
//
// Disable the PPS timeout
//
void DisablePPSTimeOut(void)
{
	PPSTimeOutCnt = 0;
}
//
// Enable the PPS timeout
//
// OUT: new PPSTimeOutCnt
//
byte EnablePPSTimeOut(void)
{
	PPSTimeOutCnt = ReloadPPSTimeOutCnt;		 //reinitialize tick counter for timeout
	return ReloadPPSTimeOutCnt;
}
//
// Display a flashing 'P' for n seconds
//
// IN  -- Flash Count
//
// OUT -- FlashPtr
//
FlashTblEntry_t *PPSStartFlashing(byte flashCnt)
{
	return AddFlashItem(ScanTblStatP, flashCnt, (VectorTblEntry_t *)CharP);
}
//
// The PPS signal is missing. Turn on the warning field.
// This warning overrules the 'B' warning
//
// No warning in Fast Mode since we use the timeout to run fast!
//
void PPSMissingOn(void)
{
	if (CurrPlayMode & RUNCLOCKFAST) {			//are we in Fast Mode
		return;									//yes, ignore
	}
	if (PPSFlashPtr == NULL) {					//skip if already active
		PPSFlashPtr = PPSStartFlashing(0xff);	//forever
	}
}
//
// The PPS signal is back. Turn off the warning field
//
void PPSMissingOff(void)
{
	RemoveFlashItem(&PPSFlashPtr);
}
//
// Rotate the X/Y offset values
//
void RotateXYOffset(void)
{
	if (Flags.RotationDisabled) {
		return;
	}
	RotationIndex = (RotationIndex + 1) & 0x07;
	uint offset = pgm_read_word(&RotationTable[RotationIndex << 1]);
	GXOffset = offset & 0xff;
	GYOffset = offset >> 8;
}
//
// Enable Grid Rotation
// Don't enable RotationIndex if Burn-in Protection is disabled
//
void EnableRotation(void)
{
	if (EEConfigData.BurninVal != BURNINOFFVAL) {
		Flags.RotationDisabled = FALSE;
		RotationCountdown = ((EEConfigData.BurninVal + 5) % 10);	//initialize
		RotateXYOffset();
	}
}
//
// Disable Grid Rotation
//
void DisableRotation(void)
{
	Flags.RotationDisabled = TRUE;
	GXOffset = 0;
	GYOffset = 0;
	RotationCountdown = 0;
}
//
// Set RotationIndex
//
void SetRotation(void)
{
	if (EEConfigData.BurninVal == BURNINOFFVAL) {
		DisableRotation();
	} else {
		EnableRotation();
	}
}
//
// Change PPS mode
//
void SetPPSMode(byte newMode)
{
	cli();
	EEConfigData.PPSMode = newMode;
	StartRTCInt();
	SetInt0Countdown();
	ResetPPSCntr();
	EnablePPSTimeOut();
	sei();
}
//
// Reset PPS Interrupt Counter. Not needed when in 1Hz mode but just as easy.
//
void ResetPPSCntr(void)
{
	Int0CntDown = Int0CntdownStart;
}
//
// Initialize Int0CntdownStart after reading Eeprom configuration data
//	or after a Menu change.
//
void SetInt0Countdown(void)
{
	int	v = INT0CNTDOWNVAL;							//start with default value
	if (EEConfigData.PPSMode != ONEHZCODE) {		// in1Hz mode Int0CntdownStart is not really used
		if (EEConfigData.PPSMode != F4096HZCODE) {	//in 4096Hz mode, no correction needed
			int i = EEConfigData.PPSMode;
			if (EEConfigData.PPSMode & 0x80) {		//sign extend PPSMode
				i |= 0xff00;
			}
			v += i;
		}
	}
	Int0CntdownStart = v;
}
char CharB[] PROGMEM = "B";
char CharP[] PROGMEM = "P";
char CharG[] PROGMEM = "G";

char ASC_TAB[16] PROGMEM = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
//
// Vector drawing tables
// Make sure to NOT use the ESCAPE char
//
byte BinaryFrameData[] PROGMEM = {
//	12,0xf8, 12,BINVPOS0-35,0xf8, BINVPOS0-35,0xf8,0xf8, 12,0xf8, 0xff,0
	12,0xf8, 12,BINVPOS0-35,0xf8, BINVPOS0-35,0xf8,0xf8, 12,0xf8, 0xff
};
byte CalibrationData[] PROGMEM = {
//	0x01,0xfe,0x01,0x01,0xfe,0x01,0xfe,0xfe,0x01,0xfe,0xfe,0x01,0x01,0x01,0xfe,0xfe,0x01,0xfe,0x01,0x01,0xfe,0x01,0xfe,0xfe, 0xff,0
	0x01,0xfe,0x01,0x01,0xfe,0x01,0xfe,0xfe,0x01,0xfe,0xfe,0x01,0x01,0x01,0xfe,0xfe,0x01,0xfe,0x01,0x01,0xfe,0x01,0xfe,0xfe, 0xff
};

byte Led2Tbl_0[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x4F,0xff
};
byte Led2Tbl_1[] PROGMEM = {
	ESCAPE,BINVPOS0,0x7F,ESCAPE,BINVPOS1,0x4F,0xff
};
byte Led2Tbl_2[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x7F,0xff
};
pByte Led2Tbl[] PROGMEM = {
	Led2Tbl_0,Led2Tbl_1,Led2Tbl_2
};
byte Led3Tbl_0[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x4F,0xff
};
byte Led3Tbl_1[] PROGMEM = {
	ESCAPE,BINVPOS0,0x7F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x4F,0xff
};
byte Led3Tbl_2[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x7F,ESCAPE,BINVPOS2,0x4F,0xff
};
byte Led3Tbl_3[] PROGMEM = {
	ESCAPE,BINVPOS0,0x7F,ESCAPE,BINVPOS1,0x7F,ESCAPE,BINVPOS2,0x4F,0xff
};
byte Led3Tbl_4[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x7F,0xff
};
byte Led3Tbl_5[] PROGMEM = {
	ESCAPE,BINVPOS0,0x7F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x7F,0xff
};
pByte Led3Tbl[] PROGMEM = {
	Led3Tbl_0,Led3Tbl_1,Led3Tbl_2,Led3Tbl_3,Led3Tbl_4,Led3Tbl_5
};
byte Led4Tbl_0[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x4F,ESCAPE,BINVPOS3,0x4F,0xff
};
byte Led4Tbl_1[] PROGMEM = {
	ESCAPE,BINVPOS0,0x7F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x4F,ESCAPE,BINVPOS3,0x4F,0xff
};
byte Led4Tbl_2[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x7F,ESCAPE,BINVPOS2,0x4F,ESCAPE,BINVPOS3,0x4F,0xff
};
byte Led4Tbl_3[] PROGMEM = {
	ESCAPE,BINVPOS0,0x7F,ESCAPE,BINVPOS1,0x7F,ESCAPE,BINVPOS2,0x4F,ESCAPE,BINVPOS3,0x4F,0xff
};
byte Led4Tbl_4[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x7F,ESCAPE,BINVPOS3,0x4F,0xff
};
byte Led4Tbl_5[] PROGMEM = {
	ESCAPE,BINVPOS0,0x7F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x7F,ESCAPE,BINVPOS3,0x4F,0xff
};
byte Led4Tbl_6[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x7F,ESCAPE,BINVPOS2,0x7F,ESCAPE,BINVPOS3,0x4F,0xff
};
byte Led4Tbl_7[] PROGMEM = {
	ESCAPE,BINVPOS0,0x7F,ESCAPE,BINVPOS1,0x7F,ESCAPE,BINVPOS2,0x7F,ESCAPE,BINVPOS3,0x4F,0xff
};
byte Led4Tbl_8[] PROGMEM = {
	ESCAPE,BINVPOS0,0x4F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x4F,ESCAPE,BINVPOS3,0x7F,0xff
};
byte Led4Tbl_9[] PROGMEM = {
	ESCAPE,BINVPOS0,0x7F,ESCAPE,BINVPOS1,0x4F,ESCAPE,BINVPOS2,0x4F,ESCAPE,BINVPOS3,0x7F,0xff
};
pByte Led4Tbl[] PROGMEM = {
	Led4Tbl_0,Led4Tbl_1,Led4Tbl_2,Led4Tbl_3,Led4Tbl_4,Led4Tbl_5,Led4Tbl_6,Led4Tbl_7,Led4Tbl_8,Led4Tbl_9
};

//
// map 24-hour hours to AM/PM hours
//
byte AMPMMapping[24] PROGMEM = {
	12,1,2,3,4,5,6,7,8,9,10,11,12,1,2,3,4,5,6,7,8,9,10,11
};

byte RotationTable[16] PROGMEM = {
	0,0, 1,0, 1,1, 0,1, -1,1, -1,0, -1,-1, 0,-1		//	,+1,-1 Too many data. May have reversed XOffset and YOffset
};
char MsgPszSerialInOverflow[] PROGMEM = "Serial In Overflow \n\r";

char HelpLine0[] PROGMEM = "Normal";
char HelpLine1[] PROGMEM = "Short S1   Menu";
char HelpLine2[] PROGMEM = "Long  S1   Change";
char HelpLine3[] PROGMEM = "Short S2   Help";
char HelpLine4[] PROGMEM = "Long  S2   Reboot";
char HelpLine5[] PROGMEM = "Change/Menu";
char HelpLine6[] PROGMEM = "S1         Increment";
char HelpLine7[] PROGMEM = "S2         Advance";
char HelpLine8[] PROGMEM = "Refresh:";
char HelpLine9[] PROGMEM = "UpTime:";

// note DEFCHARWIDTH = 12
#define TOPLINE 205
#define MIDDLELINE 95
#define STATLINE 25
#define LINEDIST 21
MScanTblEntry_t HelpScanTblImage[HELPSCANTABLEN + HELPSCANEXTTABLEN] PROGMEM = {
	{MsgPszLogo,LOGOHPOS, LOGOVPOS},
	{MsgPszVersion,LOGOHPOS+12*DEFCHARWIDTH, LOGOVPOS},
	{HelpLine0, 85, TOPLINE - 0*LINEDIST},
	{HelpLine1,	10, TOPLINE - 1*LINEDIST},
	{HelpLine2,	10,	TOPLINE - 2*LINEDIST},
	{HelpLine3,	10,	TOPLINE - 3*LINEDIST},
	{HelpLine4,	10,	TOPLINE - 4*LINEDIST},
	{HelpLine5,	60,	MIDDLELINE - 0*LINEDIST},
	{HelpLine6,	10,	MIDDLELINE - 1*LINEDIST},
	{HelpLine7,	10,	MIDDLELINE - 2*LINEDIST},
	{HelpLine8,	10,	STATLINE - 0*LINEDIST},
	{HelpLine9,	10, STATLINE - 1*LINEDIST},
	{NULL,		142,STATLINE - 0*LINEDIST},
	{NULL,		142,STATLINE - 1*LINEDIST}
};

//
// Use the MenuScanTbl to do the Help Screen
//
void SetHelpScanTbl(void)
{
	ASSERT(HELPSCANTABLEN <= MENUSCANTABLEN);
	memcpy_P(MScanTbl, HelpScanTblImage, sizeof(HelpScanTblImage));
}
//
// Use the MenuScanTbl to do the Help Screen
//
void DisplayHelpscreen(void)
{
	char UpTimeBuf[12];				//max uint32_t + S + 0.
	char RateBuf[8];				//max uint16_t + Hz + 0.
	void	(*SavedfpCurrentRefresh)(void);
	GPIOR0 &= ~_BV(fUPS2);									//clear S2 up action
	// stop alarm if active
	if (DialFlashPtr) {
		RemoveFlashItem(&DialFlashPtr);
		return;
	}
	SavedfpCurrentRefresh = fpCurrentRefresh;
	SetHelpScanTbl();
	utoa(LastRefreshCnt, RateBuf, 10);
	byte l = strlen(RateBuf);
	RateBuf[l] = 'H';
	RateBuf[l+1] = 'z';
	RateBuf[l+2] = 0;
	MScanTbl[HELPSCANTABLEN].pVect = RateBuf;
	ultoa(UpTime, UpTimeBuf, 10);
	l = strlen(UpTimeBuf);
	UpTimeBuf[l] = 'S';
	UpTimeBuf[l+1] = 0;
	MScanTbl[HELPSCANTABLEN+1].pVect = UpTimeBuf;

	fpCurrentRefresh = HelpScreenRefresh;					//set Help Screen Refresh func ptr (word address)
	SetTC1Countdown(4);
	do {
		ClockWorks();
		DiscardGPSInput();									//could call CheckGPSInput, if desired
	} while (TC1CountdownActive() && !(GPIOR0 & _BV(fUPS1)) && !(GPIOR0 & _BV(fUPS2)));
	GPIOR0 &= ~_BV(fUPS2);									//clear S2 up action if necessary
	SetTC1Countdown(0);										//if necessary
	fpCurrentRefresh = SavedfpCurrentRefresh;
}

