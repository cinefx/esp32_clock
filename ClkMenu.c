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
#include "./ClkFlash.h"
#include "./ClkSupport.h"
#include "./ClkDebug.h"

extern BOOL UpdateGPSSetting(byte newGPSVal);

CMenuVals_t	MenuData;
//
// forward declarations
//
#define TMenumin4  TMenumin4hr
#define TMenumin3  TMenumin3hr
#define TMenumin2  TMenumin2hr
#define TMenumin1  TMenumin1hr
#define TMenuplus1  TMenuplus1hr
#define TMenuplus2  TMenuplus2hr
#define TMenuplus3  TMenuplus3hr
#define TMenuplus4  TMenuplus4hr
#define OnOffValTbl UsrNameValTbl
char * OffOnValTbl[] ;
char * DialValTbl[] ;
char * NumValTbl[] ;
char * LedValTbl[];
char * PlayValTbl[] ;
char * DSTValTbl[] ;
char * BaudValTbl[] ;
char * AppValTbl[] ;
char * PPSValTbl[] ;
byte PPSToMenuMap[] ;
byte PPSMenuMap[] ;
char * UsrNameValTbl[] ;
char TMenuGPSVals[] ;
char TMenuBurninVals[] ;
char TPushToStart[] ;
char TMenuApp		[] ;
char TMenuPPS		[] ;
char TMenuAlarm		[] ;
char TMenuNumeric	[] ;
char TMenuLed		[] ;
char TMenuBurnin	[] ;
char TMenuDial		[] ;
char TMenuPlay		[] ;
char TMenuUsrName	[] ;
char TMenuGPS		[] ;
char TMenuDST		[] ;
char TMenuChrono	[] ;
char TMenuReset		[] ;
char TMenuBaud		[] ;
extern MScanTblEntry_t MScanTblImage[MENUSCANTABLEN];


extern void UpdateNumericDisplay(byte newVal);
extern void UpdateDialDisplay(byte newVal);
extern void UpdateAlarm(byte newVal);
extern void UpdateLedOptions(byte newVal);
extern void UpdateBurnInOptions(byte newVal);
extern void UpdPlayMode(byte newVal);
extern void UpdateDSTMode(byte newVal);
extern void UpdateChronoDisplay(byte newVal);
extern byte MapMenu2PPSMode(void);
extern void SetPPSMode(byte newVal);
extern void UpdatePPSMode_EEPprom(byte newVal);
extern void SetBaudVal(byte newBaudVal);
extern void ReadEEUsrName(void);
extern void WriteEEUsrName(void);
extern void SetDialDisplay(void);
extern void UpdateUsrNameMode(byte newMode);
extern byte ComputeSRamHPos(char *UsrNameBuf);
extern void SetDisplay(void);
extern void DSTRetroApply(void);
extern void DSTRetroNotApply(void);
extern void SetNumFlashField(byte n);
extern char *MsgAddNewLine(char *p);
extern void ClockWorks(void);
extern void DiscardGPSInput(void);
extern void CalibrationOff(void);
extern void UsrShowStatus(void);
extern void EnablePPSTimeOut(void);
extern void DisablePPSTimeOut(void);
extern void SpecialUARTSB(char ch);

void SetCMENULAST(void) __attribute__ ((noinline));
void SetCMENULAST(void)
{
	CMenuState = CMENULAST;
}
void CmenuFlashUsrName(void);

//
// The vector ptr in the current State Flash Entries
// will be changed as values get updated. Update the vector ptrs
// currently in the Flash Table.
// Do nothing if currently not flashing.
//
void UpdateCModFlashEntry(ScanTblEntry_t* pScanPtr, VectorTblEntry_t* pVectArg)
{
	UpdateFlashEntry(CModFlashPtr1, pScanPtr, pVectArg);
	UpdateFlashEntry(CModFlashPtr2, pScanPtr, pVectArg);
}

//
// Options: 12HR, 24HR, HEX, DATE, OFF
//
void CMenuSetNumVal(byte newVal)
{
	MenuData.NumVal = newVal;
	MScanTbl_NumVal->pVect = (char *)pgm_read_word(&NumValTbl[newVal-NUM12HR]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_NumVal, (VectorTblEntry_t*)MScanTbl_NumVal->pVect);
}

void CMenuNextNumVal(void)
{
	byte i = MenuData.NumVal + 1;
	if (i > NUMLAST) i = NUM12HR;
	CMenuSetNumVal(i);
}

void CMenuSaveNumVal(void)
{
	if (MenuData.NumVal != EEConfigData.CurrNumVal) {
		UpdateNumericDisplay(MenuData.NumVal);
		SetCMENULAST();
	}
}
//
// Options: 12HR, 24HR, ROMAN, BINARY, OFF
//
void CMenuSetDialVal(byte newVal)
{
	MenuData.DialVal = newVal;
	MScanTbl_DialVal->pVect = (char *)pgm_read_word(&DialValTbl[newVal-DIAL12HR]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_DialVal, (VectorTblEntry_t*)MScanTbl_DialVal->pVect);
}

void CMenuNextDialVal(void)
{
	byte i = MenuData.DialVal + 1;
	if (i > DIALMIN) i = DIAL12HR;
	CMenuSetDialVal(i);
}

void CMenuSaveDialVal(void)
{
	if (MenuData.DialVal != EEConfigData.CurrDialVal) {
		UpdateDialDisplay(MenuData.DialVal);
		SetCMENULAST();
	}
}

void CMenuSetAlarmVal(byte newVal)
{
	MenuData.AlarmVal = newVal;
	MScanTbl_AlarmVal->pVect = (char *)pgm_read_word(&OnOffValTbl[newVal]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_AlarmVal, (VectorTblEntry_t*)MScanTbl_AlarmVal->pVect);
}

void CMenuNextAlarmVal(void)
{
	CMenuSetAlarmVal(MenuData.AlarmVal ^ 1);			//flip
}

void CMenuSaveAlarmVal(void)
{
	if (MenuData.AlarmVal != EEConfigData.AlarmDisabled) {
		UpdateAlarm(MenuData.AlarmVal);
		SetCMENULAST();
	}
}

void CMenuSetLedVal(byte newVal)
{
	MenuData.LedVal = newVal;
	MScanTbl_LedVal->pVect = (char *)pgm_read_word(&LedValTbl[newVal]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_LedVal, (VectorTblEntry_t*)MScanTbl_LedVal->pVect);
}

void CMenuNextLedVal(void)
{
	if (++MenuData.LedVal > LEDLAST) {
		MenuData.LedVal = LEDENABLED;
	}
	CMenuSetLedVal(MenuData.LedVal);
}

void CMenuSaveLedVal(void)
{
	if (MenuData.LedVal != EEConfigData.LedOption) {
		UpdateLedOptions(MenuData.LedVal);
		SetCMENULAST();
	}
}

void CMenuSetBurninVal(byte newVal)
{
	MenuData.BurninVal = newVal;
	MScanTbl_BurninVal->pVect = (char *)(&TMenuBurninVals[newVal * 2]);	//2 bytes per string
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_BurninVal, (VectorTblEntry_t*)MScanTbl_BurninVal->pVect);
}

void CMenuNextBurninVal(void)
{
	MenuData.BurninVal += 1;
	if (MenuData.BurninVal > MAXBURNINVAL) {
		MenuData.BurninVal = 0;
	}
	CMenuSetBurninVal(MenuData.BurninVal);
}

void CMenuSaveBurninVal(void)
{
	if (MenuData.BurninVal != EEConfigData.BurninVal) {
		UpdateBurnInOptions(MenuData.BurninVal);
		SetCMENULAST();
	}
}

void CMenuSetPlayVal(byte newVal)
{
	MenuData.PlayVal = newVal;
	MScanTbl_PlayVal->pVect = (char *)pgm_read_word(&PlayValTbl[newVal - FUNNORM]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_PlayVal, (VectorTblEntry_t*)MScanTbl_PlayVal->pVect);
}

void CMenuNextPlayVal(void)
{
	byte i = MenuData.PlayVal + 1;
	if (i > FUNLAST) i = FUNNORM;
	CMenuSetPlayVal(i);
}

void CMenuSavePlayVal(void)
{
	if (MenuData.PlayVal != CurrPlayMode) {
		UpdPlayMode(MenuData.PlayVal);
		SetCMENULAST();
	}
}

void CMenuSetDSTVal(byte newVal)
{
	MenuData.DSTMode = newVal;
	MScanTbl_DSTVal->pVect = (char *)pgm_read_word(&DSTValTbl[newVal - DSTUS]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_DSTVal, (VectorTblEntry_t*)MScanTbl_DSTVal->pVect);
}

void CMenuNextDSTVal(void)
{
	byte i = MenuData.DSTMode + 1;
	if (i > DSTNONE) i = DSTUS;
	CMenuSetDSTVal(i);
}

void CMenuSaveDSTVal(void)
{
	if (MenuData.DSTMode != EEConfigData.DSTMode) {
		UpdateDSTMode(MenuData.DSTMode);
		SetCMENULAST();
	}
}

void CMenuSetBaudVal(byte newVal)
{
	MenuData.BaudVal = newVal;
	MScanTbl_BaudVal->pVect = (char *)pgm_read_word(&BaudValTbl[newVal-BAUDVALFIRST]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_BaudVal, (VectorTblEntry_t*)MScanTbl_BaudVal->pVect);
}

void CMenuNextBaudVal(void)
{
	byte i = MenuData.BaudVal + 1;
	if (i > BAUDVAL14400) i = BAUDVALFIRST;
	CMenuSetBaudVal(i);
}

void CMenuSaveBaudVal(void)
{
	byte i = MenuData.BaudVal - BAUDVALFIRST;
	if (i != EEConfigData.BaudVal) {
		SetBaudVal(MenuData.BaudVal - BAUDVALFIRST);
		UpdateEEprom(EEPROM_BAUDVAL, i);
		SetCMENULAST();
	}
}

void CMenuSetChronoVal(byte newVal)
{
	MenuData.ChronoVal = newVal;
	MScanTbl_ChronoVal->pVect = (char *)pgm_read_word(&OffOnValTbl[newVal]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_ChronoVal, (VectorTblEntry_t*)MScanTbl_ChronoVal->pVect);
}

void CMenuNextChronoVal(void)
{
	CMenuSetChronoVal(MenuData.ChronoVal ^ 1);			//flip
}

void CMenuSaveChronoVal(void)
{
	if (MenuData.ChronoVal != EEConfigData.ChronoEnabled) {
		UpdateChronoDisplay(MenuData.ChronoVal);
		SetCMENULAST();
	}
}

void CMenuSetInitVal(byte newVal)
{
	MenuData.EEInitVal = newVal;
	MScanTbl_InitVal->pVect = (char *)pgm_read_word(&OffOnValTbl[newVal]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_InitVal, (VectorTblEntry_t*)MScanTbl_InitVal->pVect);
}

void CMenuNextInitVal(void)
{
	CMenuSetInitVal(MenuData.EEInitVal ^ 1);			//flip
}

void CMenuSaveInitVal(void)
{
	if (MenuData.EEInitVal) {
		cli();
		ClearEEConfiguration();
		InitiateSysReset();								//reboot
		SetCMENULAST();
	}
}

void CMenuSetAppVal(byte newVal)
{
	MenuData.AppVal = newVal;
	MScanTbl_AppVal->pVect = (char *)pgm_read_word(&AppValTbl[newVal - APPCLOCK]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_AppVal, (VectorTblEntry_t*)MScanTbl_AppVal->pVect);
}

void CMenuNextAppVal(void)
{
	byte i = MenuData.AppVal + 1;
	if (i > APPLAST) i = APPCLOCK;
	CMenuSetAppVal(i);
}

void CMenuSaveAppVal(void)
{
	if (MenuData.AppVal != CurrAppMode) {
		CurrAppMode = MenuData.AppVal;
		// Only Clock or Term mode saved to EEprom
		if ((CurrAppMode == APPCLOCK) || (CurrAppMode == APPTERM)) {
			UpdateEEprom(EEPROM_STARTAPP, CurrAppMode - APPCLOCK);
		}
		//	change application/ do special function.
		// The actual change takes place in the main loop,
		// after MainMenuProcessing() returns.
		SetCMENULAST();
	}
}

void CMenuSetPPSVal(byte newVal)
{
	MenuData.PPSMode = newVal;
	MScanTbl_PPSVal->pVect = (char *)pgm_read_word(&PPSValTbl[newVal - PPSBASEVAL]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_PPSVal, (VectorTblEntry_t*)MScanTbl_PPSVal->pVect);
}

void CMenuNextPPSVal(void)
{
	byte i = MenuData.PPSMode + 1;
	if (i > PPSLASTVAL) i = PPSBASEVAL;
	CMenuSetPPSVal(i);
}

void CMenuSavePPSVal(void)
{
	byte i = MapMenu2PPSMode();
	if (i != EEConfigData.PPSMode) {
		//
		// update PPS mode. Map value properly
		// This also requires stopping interrupts
		//
		UpdateEEprom(EEPROM_PPSMODE,i);
		SetPPSMode(i);
		SetCMENULAST();
	}
}

void CMenuSetGPSVal(byte newVal)
{
	MenuData.GPSVal = newVal;
	MScanTbl_GPSVal->pVect = (char *)(TMenuGPSVals + (MenuData.GPSVal - NOGPS) * 4);		// 4 char for each string
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_GPSVal, (VectorTblEntry_t*)(TMenuGPSVals + (MenuData.GPSVal - NOGPS) * 4));
}

void CMenuNextGPSVal(void)
{
	byte i = MenuData.GPSVal + 1;
	if (i > GPSOFSPLUS12) i = NOGPS;
	CMenuSetGPSVal(i);
}

void CMenuSaveGPSVal(void)
{
	if (!UpdateGPSSetting(MenuData.GPSVal)) {
		return;														//no change
	}
	SetCMENULAST();
}
//
// MenuData.PPSMode values are:
// 0: 4096H  1: 1Hz	(ds1307)
// or
// 0: 1Hz	1: 4096H (pcf8563)
// 2, 3, 4, 5:  -4, -3, -2, -1
// 6, 7, 8, 9:  +1, +2, +3, +4
//
byte MapMenu2PPSMode(void)
{
	ASSERT(MenuData.PPSMode <= PPSLASTVAL);
	return pgm_read_byte(&PPSMenuMap[MenuData.PPSMode - PPSBASEVAL]);
}

byte MapPPSMode2Menu(void)
{
	byte val;
	if (EEConfigData.PPSMode == F4096HZCODE) {
		val = MENUF4096HZCODE;
	} else if (EEConfigData.PPSMode == ONEHZCODE) {
		val = MENUONEHZCODE;
	} else {
		val = EEConfigData.PPSMode + 4;					//rescale to 0 based
		ASSERT(val < 9);
		val = pgm_read_byte(&PPSToMenuMap[val]);
	}
	return (val + PPSBASEVAL);
}

//
// Start Text field flashing to show it is being updated
//
void StartStateTextFlash(void)
{
	ASSERT(ScanTblNum->pVect == (VectorTblEntry_t*)NumDispBuf);
	CTextFlashPtr = AddFlashItem(ScanTblNum, 0xff, (VectorTblEntry_t*)NumFlashDispBuf);
}

//
// Start Menu SRam Text field flashing to show it is being updated
//
void StartMenuTextFlash(void)
{
	ASSERT(MScanTbl_UsrNameTxt->pVect == UsrNameBuf);
	CTextFlashPtr = AddFlashItem((ScanTblEntry_t *)MScanTbl_UsrNameTxt, 0xff, (VectorTblEntry_t*)UsrNameFlashBuf);
}
//
void StopTextFlash(void)
{
	RemoveFlashItem(&CTextFlashPtr);
	TextFlashField = 0;
}
//
// Start field flashing to show it is being updated
///
FlashTblEntry_t* StartStateFlash(ScanTblEntry_t *pTbl)
{
	return AddFlashItem(pTbl, 0xff, NULL);
}
//
//
void NextCModState(byte state)
{
	CModState = state;			//move to next state
	RemoveFlashItem(&CModFlashPtr1);
	RemoveFlashItem(&CModFlashPtr2);
}
//
//
void SetIdleCModState(void)
{
	NextCModState(CMODIDLE);
}
//
// Start Menu field flashing to show it is being updated
//
void StartMenuStateFlash(MScanTblEntry_t *pTbl)
{
	CMenuFlashPtr = StartStateFlash((ScanTblEntry_t *)pTbl);
}
//
void NextCMenuState(byte state)
{
	CMenuState = state;
	RemoveFlashItem(&CMenuFlashPtr);
}
//
void RestoreUsrName(void)
{
	ReadEEUsrName();				//Restore original UsrName
	SetDialDisplay();				//re-display user name
}

void SaveUsrName(void)
{
	WriteEEUsrName();				//Also writes UsrNameCnt
	SetDialDisplay();				//re-display user name
}
//
// Timeout UsrName edit mode
//
void TimeOutEditUsrName(void)
{
	if (MenuData.UsrNameVal == USRNAMEEDIT) {
		ASSERT(CMenuState == CMENUNAMEDIT);
		StopTextFlash();
		RestoreUsrName();
	}
}

void CMenuSetUsrNameVal(byte newVal)
{
	MenuData.UsrNameVal = newVal;
	MScanTbl_UsrNameVal->pVect = (char *)pgm_read_word(&UsrNameValTbl[newVal - USRNAMEON]);
	UpdateFlashEntry(CMenuFlashPtr, (ScanTblEntry_t *)MScanTbl_UsrNameVal, (VectorTblEntry_t*)MScanTbl_UsrNameVal->pVect);
}

void CMenuNextUsrNameVal(void)
{
	byte i = MenuData.UsrNameVal + 1;
	if ((i == USRNAMEO) && ((EEConfigData.ioctl & _BV(IOCTL_EASYOMODE)) == 0)) {
		if (++UsrNameSkipCnt < 5) {
			++i;					//skip
		} else {
			UsrNameSkipCnt = 0;
		}
	}
	if (i > USRNAMELAST) i = USRNAMEON;
	CMenuSetUsrNameVal(i);
}

void CMenuSaveUsrNameVal(void)
{
	if (MenuData.UsrNameVal == USRNAMEEDIT) {
//
// Edit the UsrName. Make sure CMenuState is advanced to CMENUNAMEDIT
// Start Flashing the first char in the edit buffer
// Do NOT update UsrNameVal (leave display mode as it was)
//
		UsrNameEditIdx = 0;				//set current edit char
		CmenuFlashUsrName();
		StartMenuTextFlash();
	} else if (MenuData.UsrNameVal != EEConfigData.UsrNameVal) {
		UpdateUsrNameMode(MenuData.UsrNameVal);
		SetCMENULAST();
	}
}
//
// center the UsrName Text line
//
void CMenuCenterUsrName(void)
{
	UsrNameBuf[MenuData.UsrNameCnt] = 0;		//Must be NULL terminated
	MScanTbl_UsrNameTxt->Xoffset = ComputeSRamHPos(UsrNameBuf);
}
//
// Build the Alternate buffer for flashing
//
void CmenuFlashUsrName(void)
{
	memcpy(UsrNameFlashBuf, UsrNameBuf, MAXUSRNAMELEN+1);		//also copy zero terminator
	UsrNameFlashBuf[UsrNameEditIdx] = '_';
}
//
//
void CMenuIncUserNameChar(void)
{
	char ch = UsrNameBuf[UsrNameEditIdx];
	ASSERT(ch != 0);
	if (++ch & 0x80) {			// ch > 127
		ch = ' ';
	}
	UsrNameBuf[UsrNameEditIdx] = ch;
	CmenuFlashUsrName();
}
//
// CMenuUsrNameIdx is 0..MenuData.UsrNameCnt-1
//
void CMenuNextUserNameChar(void)
{
	BOOL done = FALSE;
	UsrNameEditIdx++;
	if ((UsrNameEditIdx + 1) >= MenuData.UsrNameCnt) {
		MenuData.UsrNameCnt = UsrNameEditIdx + 1;
	}
	if (UsrNameEditIdx >= MAXUSRNAMELEN) {
//
// Buffer full, exit edit mode. Remove any trailing spaces
//
		while (--UsrNameEditIdx > 0) {
			if (UsrNameBuf[UsrNameEditIdx] != ' ') break;
		}
		UsrNameEditIdx++;
		done = TRUE;
	} else {
//
// buffer not full. See if we had 2 consecutive spaces
//
		byte tmpIdx = UsrNameEditIdx - 1;
		if (tmpIdx != 0) {
			tmpIdx--;
			if ((UsrNameBuf[tmpIdx] == ' ') & (UsrNameBuf[tmpIdx + 1] == ' ')) {
				UsrNameEditIdx = tmpIdx;
				done = TRUE;
			}
		}
	}
	if (done) {
//
// Done. Terminate UsrName. Exit edit Mode
//
		EEConfigData.UsrNameCnt = UsrNameEditIdx;		//reset length
		UsrNameBuf[UsrNameEditIdx] = 0;		//terminate UsrName
//
// Assume name was changed. Could optimize here in case there was no change.
//
		UsrNameDflt = FALSE;				//not default any more
		SaveUsrName();						//write to EEProm and update display
		StopTextFlash();
		SetCMENULAST();
	} else {
//
// not done. Check next char to see if it is 0
// Make sure we stay in the CMENUNAMEDIT mode
//
		char ch = UsrNameBuf[UsrNameEditIdx];
		ASSERT(ch != 0xff);						//should never happen
		if (ch == 0) {							//0 (name extension)
			UsrNameBuf[UsrNameEditIdx] = ' ';	//start at SPACE to exit easier
		}
		CMenuCenterUsrName();
		CmenuFlashUsrName();
		ASSERT(CMenuState == CMENUNAMEDIT);
		CMenuState = CMENUNAMEDIT-1;
	}
}

//
// Populate Menu Table Value fields with current values
// Flags retain their original interpretation as much as possible.
//
void CMenuSetTable(void)
{
	// Copy fixed part of MScanTbl every time we enter the menu, then use the MScanTbl memory when menu not active
	memcpy_P(MScanTbl, MScanTblImage, sizeof(MScanTblImage));

	CMenuSetAlarmVal(EEConfigData.AlarmDisabled);
	CMenuSetNumVal(EEConfigData.CurrNumVal);
	CMenuSetLedVal(EEConfigData.LedOption);
	CMenuSetBurninVal(EEConfigData.BurninVal);
	CMenuSetDialVal(EEConfigData.CurrDialVal);
	CMenuSetPlayVal(CurrPlayMode);
	CMenuSetAppVal(CurrAppMode);
	CMenuSetUsrNameVal(EEConfigData.UsrNameVal);
	UsrNameSkipCnt = 0;
	CMenuSetChronoVal(EEConfigData.ChronoEnabled);		//Boolean
	
	MenuData.UsrNameCnt = EEConfigData.UsrNameCnt;
	UsrNameEditIdx = 0;						//start edit at char 0
	CMenuCenterUsrName();

	if (EEConfigData.GPSInEnabled) {					//GPS currently enabled?
//
// get local time zone offset rebased to NOGPS range
//
		ASSERT(((byte)(EEConfigData.GPSOffset + (NOGPS + 13))) < (GPSOFSPLUS12+1));
		CMenuSetGPSVal(EEConfigData.GPSOffset + (NOGPS + 13));
	} else {
		CMenuSetGPSVal(NOGPS);
	}

	CMenuSetDSTVal(EEConfigData.DSTMode);				//Composite Value: US, EU, none
	CMenuSetInitVal(0);
	CMenuSetBaudVal(EEConfigData.BaudVal + BAUDVALFIRST); //rebase from range 0..7. INCONSISTENT
	CMenuSetPPSVal(MapPPSMode2Menu());
}
//
//  Table based Cmenu Processing
//
void CMenuStateAdvance(void)
{
	void (*pF)(void);

	ASSERT(CMenuState != CMENUIDLE);
	if ((CMenuState == CMENUNAMEDIT) && (MenuData.UsrNameVal != USRNAMEEDIT)) {
//
// This only works if CMENUNAMEDIT is the last Menu state
//
		NextCMenuState(CMENUIDLE);
		return;
	}
	if (GPIOR0 & _BV(fUPS1)) {			//short push S1
		GPIOR0 &= ~_BV(fUPS1);			//clear
		pF = (void (*)(void))pgm_read_word(&(CMenuStateTbl[CMenuState - (CMENUIDLE + 1)].S1Action));		//word address
		pF();							//S1 action
		SetTC1Countdown(30);
		return;
	}
	if (GPIOR0 & _BV(fUPS2)) {			//short push S2
		GPIOR0 &= ~_BV(fUPS2);			//clear
		pF = (void (*)(void))pgm_read_word(&(CMenuStateTbl[CMenuState - (CMENUIDLE + 1)].S2Action));		//word address
		pF();
		ScanTblEntry_t *p = (ScanTblEntry_t *)pgm_read_word(&(CMenuStateTbl[CMenuState - (CMENUIDLE + 1)].FlashField));	//word address
		byte i = ++CMenuState;
		if (i > CMENULAST) i = CMENUIDLE;
		NextCMenuState(i);
		if (p) {
			CMenuFlashPtr = StartStateFlash(p);
		}
	}
}
//
// Main Program Loop for Menu Processing
//
void MainMenuProcessing(BOOL firstRun)
{
	// stop alarm if active
	if (DialFlashPtr) {
		RemoveFlashItem(&DialFlashPtr);
	}

	if (firstRun) {
		return;
	}
//
// Start flashing the first entry
//
	CMenuSetTable();									//populate Menu table with current values
	NextCMenuState(CMENUNUM);							//first state
	StartMenuStateFlash(MScanTbl_NumVal);				//First field to flash
	SetTC1Countdown(30);								//start timeout timer
	S1.okToRepeat = TRUE;								//ok to auto repeat
	do {
		ClockWorks();									//show the menu and process events
		DiscardGPSInput();								//could call CheckGPSInput, if desired
		CMenuStateAdvance();							//Menu Change State Machine
		if (TC1CountdownActive() == FALSE) {			//still going?
			//
			// Timing Out a CMENU state. Clean up any flash ptrs
			//
			TimeOutEditUsrName();						//only when in User Name Edit mode
			NextCMenuState(CMENUIDLE);					//force back to CMENUIDLE
			break;
		}
	} while (CMenuState != CMENUIDLE);					//back to IDLE?
	ASSERT(CMenuState == CMENUIDLE);
	Flags.GPSOKToUpdate = TRUE;								//ok to process GPS record again
	S1.okToRepeat = FALSE;								//stop auto repeat
}
//
//	returns updated state, Possibly GPIOR0 modified
//
byte StateCheck4Flags(byte state)
{
	if (state & NOPUSH) {
		GPIOR0 |= _BV(fUPS1);
		state &= ~NOPUSH;
	}
	if (state & DBLHAND) {
		Flags.UseDblHand = TRUE;
		state &= ~DBLHAND;
	}
	return state;
}
//
// display "PUSH TO START"
//
void SetPushToStart(void)
{
	ScanTblEntry_t *p = ScanTblPush;
	p->pVect = (VectorTblEntry_t*)TPushToStart;
	p->Xoffset = P2SHPOS;
	p->Yoffset = P2SVPOS;
}
//
// Note. StartStateTextFlash setups the numFlashDispBuf in the
//		flash tables, even though numFlashDispBuf has not been
//		initialized yet. Setting fTimeoutS1/fUPS2 causes SetNumFlashField
//		to be called, which does the initialization before
//		the field is being flashed
//
void StateSetHRNR(void)
{
	StartStateTextFlash();	//Numeric Text String to flash
	GPIOR0 |= _BV(fUPS2);	//fake an S2 push to advance to the next state
}

void StateSetHRHAND(void)
{
	GPIOR0 |= _BV(fUPS2);	//fake an S2 push to advance to the next state
}

void StateSetHRBIN(void)	//same as StateSetHRHAND
{
	GPIOR0 |= _BV(fUPS2);	//fake an S2 push to advance to the next state
}

#define StateSetAlarmHr	StateSetHRNR
#define StateSetYEAR	StateSetHRNR

void StateSetMODSET(void)
{
	SetPushToStart();
	StopTextFlash();
	GPIOR0 |= _BV(fUPS2);	//fake an S2 push to advance to the next state
}
//
// Increment Seconds
//
void StateIncSec(void)
{
	AVRIncrementSecs();
	SetDisplay();
}
//
// Increment Minutes
//
void StateIncMin(void)
{
	AVRIncrementMins();
	SetDisplay();
}
//
// Increment Hours
//
void StateIncHr(void)
{
	AVRIncrementHrs();
	SetDisplay();
	SetNumFlashField(HourInString);	//needed to update AM/PM
}

void StateSetTime(void)
{
	RTCSetTime();					//update the RTC
//
// check if the new time is in a DST window. If so, set the proper bitvector flag
//  but do not apply a DST correction.
//
	DSTRetroNotApply();
	SetIdleCModState();
	ScanTblPush->pVect = NULL;		//clear PUSH TO START message
	SetDisplay();
}
//
// Increment Alarm Seconds
//
void StateIncAlarmSec(void)
{
	byte s = ++EEConfigData.AlarmSecs;
	if (s >= 60) {
		s = 0;
	}
	EEConfigData.AlarmSecs = s;
	SetDisplay();
}
//
// Increment Alarm Minutes
//
void StateIncAlarmMin(void)
{
	byte m = ++EEConfigData.AlarmMins;
	if (m >= 60) {
		m = 0;
	}
	EEConfigData.AlarmMins = m;
	SetDisplay();
}
//
// Increment Alarm Hours
//
void StateIncAlarmHr(void)
{
	byte h = ++EEConfigData.AlarmHrs;
	if (h >= 24) {
		h = 0;
	}
	EEConfigData.AlarmHrs = h;
	SetDisplay();
}

void StateSetAlarm(void)
{
	StopTextFlash();
	UpdateEEprom(EEPROM_ALARMSECS, EEConfigData.AlarmSecs);
	UpdateEEprom(EEPROM_ALARMMINS, EEConfigData.AlarmMins);
	UpdateEEprom(EEPROM_ALARMHRS, EEConfigData.AlarmHrs);
	SetIdleCModState();
	SetDisplay();
}


void CTimeDateCancel(void)
{
	RTCReadTime(FALSE);				//reload AVR time from RTC
	RTCReadDate();					//reload AVR date from RTC
	SetIdleCModState();
	ScanTblPush->pVect = NULL;		//clear PUSH TO START message
	SetDisplay();
}

void StateIncDay(void)
{
	AVRIncrementDay();
	SetDisplay();
}
//
// Validate the day field based on the newly selected month/year
//
void StateSetDay(void)
{
	if (!CheckDaysInMonth(&AVRTimeDateBlk, AVRTimeDateBlk.Day)) {
		AVRTimeDateBlk.Day = GetDaysInMonth(&AVRTimeDateBlk);
	}
	GPIOR0 |= _BV(fUPS2);		//fake an S2 push to advance to the next state
}

void StateIncMonth(void)
{
	AVRIncrementMonth();
	SetDisplay();
}

void StateIncYear(void)
{
	AVRIncrementYear();
	SetDisplay();
}
//
// Internal Date has been updated by the user.
// Now propagate this to the RTC
//
void StateSetDate(void)
{
	StopTextFlash();
	RTCSetDate();			//This will validate the new date
//
// check if the new date is in a DST window
//
	DSTRetroApply();		//will update RTC if doing DST fix. Should test
	SetDisplay();
	GPIOR0 |= _BV(fUPS2);	//fake an S2 push to advance to the next state
}

void CModStateAdvance(void)
{
	byte cnt = 0;
	CModStateTblEntry_t *pCMod = &CModStateTable[0];			//state actions table
	Flags.UseDblHand = FALSE;
	CModState = StateCheck4Flags(CModState);					//may set fUPS1
	while (CModState != pgm_read_byte(&(pCMod->f1))) {
		++pCMod;
		++cnt;
		ASSERT(cnt < ((sizeof(CModStateTable)/sizeof(CModStateTblEntry_t)))+1);
	}
//
// we've located the table entry for the current state
// see if an S1 push was issued. If so: advance field value
// This may be a fake S1 push  (NOPUSH attribute)
//
	if (GPIOR0 & _BV(fUPS1)) {
#if 0&DEBUG
		ShowCModState(CModState, cnt);
#endif
		BOOL (*pF)(void);
		GPIOR0 &= ~_BV(fUPS1);
		pF = (BOOL (*)(void))pgm_read_word(&(pCMod->S1Action));
		(void)pF();
		SetTC1Countdown(30);
	}
//
// See if an S2 push was issued; could be a fake push
// If so: advance to the next field.
//
	else if (GPIOR0 & _BV(fUPS2)) {
#if 0&DEBUG
		ShowCModState(CModState, cnt);
#endif
		GPIOR0 &= ~_BV(fUPS2);
		byte i = pgm_read_byte(&(pCMod->f2));
		NextCModState(StateCheck4Flags(i));
		uint w = pgm_read_word(&(pCMod->f4));
		if (w) {
			if (w & NUMFIELDFLASH) {
				SetNumFlashField(w & 0x7f);
			} else {
				ScanTblEntry_t *pScan = (ScanTblEntry_t *)w;
				CModFlashPtr1 = StartStateFlash(pScan);
				if (Flags.UseDblHand) {
					CModFlashPtr2 = StartStateFlash(pScan+1);
				}
			}
		}
		SetTC1Countdown(30);
	}
}
void MainCModProcessing(void)
{
	byte initialState;
	// stop alarm if active
	if (DialFlashPtr) {
		RemoveFlashItem(&DialFlashPtr);
	}
	if (EEConfigData.CurrNumVal == NUMDATE) {
		//
		// Enter Date Set Mode. Flash current date
		// Don't stop clock when modifying the date, but stop updating the Numeric field
		// while in Date Set Mode.
		//
		initialState = CMODPREYEAR|NOPUSH;					//start at YEAR Field
	} else if (EEConfigData.CurrNumVal == NUMALARM) {
		//
		// Enter Alarm Time Set Mode. Flash current Alarm Time
		// Don't stop clock when modifying the Alarm Time
		//
		initialState = CMODPREALARMHR|NOPUSH;				//start at HOUR Field
	} else {
		//
		// Time Change Modes
		//
		fClockIsRunning = FALSE;							//stop the clock
		DisablePPSTimeOut();								//prevent timeout
		//
		// Enter Time Set Mode. Flash number, hands or leds depending on current state
		//
		if (EEConfigData.CurrNumVal != NUMOFF) {			//if showing numbers, flash numbers
			ASSERT(EEConfigData.CurrNumVal != NUMDATE);
			initialState = CMODPREHRNR|NOPUSH;	//start at numeric hour mode
		} else {
			//
			// No Numeric Field. Flash hands/leds
			//
			if ((EEConfigData.CurrDialVal == DIALBIN) ||
				(EEConfigData.CurrDialVal == DIALDIG)) {
				initialState = CMODPREHRBIN|NOPUSH;			//start at binary hour mode
			} else {
				initialState = CMODPREHRHAND|NOPUSH;		//start at hour hands mode
			}
		}
	}
	NextCModState(initialState);
	SetTC1Countdown(30);
	S1.okToRepeat = TRUE;									//ok to auto repeat
	do {
		ClockWorks();										//show the clock and process events
		DiscardGPSInput();		
		CModStateAdvance();									//Time/Date Change State Machine
		if (TC1CountdownActive() == FALSE) {
			//
			// CModState has timed out
			//
			CTimeDateCancel();								//cleanup
			ASSERT(CModFlashPtr2 == NULL);
			StopTextFlash();
		}
	} while (CModState != CMODIDLE);
//
// done with CMOD states
// skip Clock Restart if we were setting the date
//
	if (EEConfigData.CurrNumVal != NUMDATE) {
		ClrLedMorseData();
		LedMorseSecs = 20;									//if Morse enabled. Start quickly
		fClockIsRunning = TRUE;								//restart the clock
		EnablePPSTimeOut();
	}

	SetTC1Countdown(0);
	Flags.GPSOKToUpdate = TRUE;								//ok to process GPS record again
	S1.okToRepeat = FALSE;									//stop auto repeat
	UsrShowStatus();
}

char TMenuApp		[] PROGMEM =	"App";
char TMenuClock		[] PROGMEM =	"Clock";
char TMenuCalibrate	[] PROGMEM =	"Cal";
char TMenuTerm		[] PROGMEM =	"Term";
char TMenuGen		[] PROGMEM =	"Gen";
char TMenuBoot		[] PROGMEM =	"Boot";
char TMenuDemo		[] PROGMEM =	"Demo";
char TMenuPPS		[] PROGMEM =	"PPS";
char TMenuNumeric	[] PROGMEM =	"Num";
char TMenuLed		[] PROGMEM =	"Led";
char TMenuBurnin	[] PROGMEM =	"Burnin";
char TMenuDial		[] PROGMEM =	"Dial";
char TMenuOn		[] PROGMEM =	"On";
char TMenuOff		[] PROGMEM =	"Off";
char TMenu12HR		[] PROGMEM =	"12hr";
char TMenu24HR		[] PROGMEM =	"24hr";
char TMenuHex		[] PROGMEM =	"Hex";
char TMenuDate		[] PROGMEM =	"Date";
char TMenuAlarm		[] PROGMEM =	"Alarm";
char TMenuDigital	[] PROGMEM =	"Dig";
char TMenuBinary	[] PROGMEM =	"Bin";
char TMenuRoman		[] PROGMEM =	"Rom";
char TMenuMin		[] PROGMEM =	"Min";
char TMenuMorse		[] PROGMEM =	"Morse";
char TMenuDebug		[] PROGMEM =	"Debug";
char TMenuPlay		[] PROGMEM =	"Play";
char TMenuNorm		[] PROGMEM =	"Norm";
char TMenuRev		[] PROGMEM =	"Rev";
char TMenuFastF		[] PROGMEM =	"FF";
char TMenuFastR		[] PROGMEM =	"FR";
char TMenuUsrName	[] PROGMEM =	"Name";
char TMenuEdit		[] PROGMEM =	"Edit";
char TMenuDay		[] PROGMEM =	"Day";
char TMenuO			[] PROGMEM =	"O";
char TPushToStart	[] PROGMEM =	"Push S1 to Start";
char TMenuGPS		[] PROGMEM =	"GPS";
char TMenuGPSVals	[] PROGMEM =	"Off";
char TMenumin12hr	[] PROGMEM =	"-12";
char TMenumin11hr	[] PROGMEM =	"-11";
char TMenumin10hr	[] PROGMEM =	"-10";
char TMenumin9hr	[] PROGMEM =	"-9 ";
char TMenumin8hr	[] PROGMEM =	"-8 ";
char TMenumin7hr	[] PROGMEM =	"-7 ";
char TMenumin6hr	[] PROGMEM =	"-6 ";
char TMenumin5hr	[] PROGMEM =	"-5 ";
char TMenumin4hr	[] PROGMEM =	"-4 ";
char TMenumin3hr	[] PROGMEM =	"-3 ";
char TMenumin2hr	[] PROGMEM =	"-2 ";
char TMenumin1hr	[] PROGMEM =	"-1 ";
char TMenumin0hr	[] PROGMEM =	"0  ";
char TMenuplus1hr	[] PROGMEM =	"+1 ";
char TMenuplus2hr	[] PROGMEM =	"+2 ";
char TMenuplus3hr	[] PROGMEM =	"+3 ";
char TMenuplus4hr	[] PROGMEM =	"+4 ";
char TMenuplus5hr	[] PROGMEM =	"+5 ";
char TMenuplus6hr	[] PROGMEM =	"+6 ";
char TMenuplus7hr	[] PROGMEM =	"+7 ";
char TMenuplus8hr	[] PROGMEM =	"+8 ";
char TMenuplus9hr	[] PROGMEM =	"+9 ";
char TMenuplus10hr	[] PROGMEM =	"+10";
char TMenuplus11hr	[] PROGMEM =	"+11";
char TMenuplus12hr	[] PROGMEM =	"+12";
char TMenuDST		[] PROGMEM =	"DST";
char TMenuChrono	[] PROGMEM =	"Chrono";
char TMenuReset		[] PROGMEM =	"Reset";
char TMenu1Hz		[] PROGMEM =	"1Hz";
char TMenu4096Hz	[] PROGMEM =	"4096Hz";
char TMenuBaud		[] PROGMEM =	"Baud";
char TMenuBaud19	[] PROGMEM =	"19200 ";
char TMenuBaud28	[] PROGMEM =	"28800 ";
char TMenuBaud38	[] PROGMEM =	"38400 ";
char TMenuBaud57	[] PROGMEM =	"57600 ";
char TMenuBaud115	[] PROGMEM =	"115200";
char TMenuBaud250	[] PROGMEM =	"250000";
char TMenuBaud48	[] PROGMEM =	"4800  ";
char TMenuBaud96	[] PROGMEM =	"9600  ";
char TMenuBaud14	[] PROGMEM =	"14400 ";
char TMenuUSA		[] PROGMEM =	"USA";
char TMenuEU		[] PROGMEM =	"EU";

char TMenuBurninVals[20] PROGMEM = 	{'5', 0, '6', 0, '7', 0, '8', 0, '9', 0, '0', 0, '1', 0, '2', 0, '3', 0, '4', 0};

char * DialValTbl[6] PROGMEM = {
	TMenu12HR,
	TMenu24HR,
	TMenuRoman,
	TMenuDigital,
	TMenuBinary,
	TMenuMin
};

char * NumValTbl[6] PROGMEM = {
	TMenu12HR,
	TMenu24HR,
	TMenuHex,
	TMenuDate,
	TMenuAlarm,
	TMenuOff
};

char * LedValTbl[4] PROGMEM = {
	TMenuOn,
	TMenuMorse,
	TMenuDebug,
	TMenuOff
};

char * BaudValTbl[9] PROGMEM = {
	TMenuBaud19,
	TMenuBaud28,
	TMenuBaud38,
	TMenuBaud57,
	TMenuBaud115,
	TMenuBaud250,
	TMenuBaud48,
	TMenuBaud96,
	TMenuBaud14
};

char * OffOnValTbl[2] PROGMEM = {
	TMenuOff,
	TMenuOn
};

char * PPSValTbl[10] PROGMEM = {
	TMenu1Hz,
	TMenu4096Hz,
	TMenumin4,
	TMenumin3,
	TMenumin2,
	TMenumin1,
	TMenuplus1,
	TMenuplus2,
	TMenuplus3,
	TMenuplus4
};

byte PPSMenuMap[10] PROGMEM = {
	ONEHZCODE, F4096HZCODE,-4,-3,-2,-1,1,2,3,4
};

byte PPSToMenuMap[10] PROGMEM = {
	2,3,4,5,0xff,6,7,8,9,0
};

char * PlayValTbl[4] PROGMEM = {
	TMenuNorm,
	TMenuRev,
	TMenuFastF,
	TMenuFastR
};

char * UsrNameValTbl[5] PROGMEM = {
	TMenuOn,
	TMenuOff,
	TMenuEdit,
	TMenuDay,
	TMenuO
};

char * DSTValTbl[3] PROGMEM = {
	TMenuUSA,
	TMenuEU,
	TMenuOff
};

char * AppValTbl[6] PROGMEM = {
	TMenuClock,
	TMenuCalibrate,
	TMenuTerm,
	TMenuGen,
	TMenuDemo,
	TMenuBoot
};
//
// Menu Change Table: Action, Finalize, Flash
//
CMenuStateTblEntry_t CMenuStateTbl[] PROGMEM = {
	{		CMenuNextNumVal,		CMenuSaveNumVal,	MScanTbl_DialVal },		//Numeric Field
	{		CMenuNextDialVal,		CMenuSaveDialVal,	MScanTbl_GPSVal	},		//Dial field
	{		CMenuNextGPSVal,		CMenuSaveGPSVal,	MScanTbl_DSTVal	},		//GPS field
	{		CMenuNextDSTVal,		CMenuSaveDSTVal,	MScanTbl_AppVal	},		//DST field
	{		CMenuNextAppVal,		CMenuSaveAppVal,	MScanTbl_PPSVal	},		//App field
	{		CMenuNextPPSVal,		CMenuSavePPSVal,	MScanTbl_ChronoVal },
	{		CMenuNextChronoVal,		CMenuSaveChronoVal,	MScanTbl_AlarmVal },	//Chrono field
	{		CMenuNextAlarmVal,		CMenuSaveAlarmVal,	MScanTbl_LedVal },
	{		CMenuNextLedVal,		CMenuSaveLedVal,	MScanTbl_BurninVal },
	{		CMenuNextBurninVal,		CMenuSaveBurninVal, MScanTbl_PlayVal },
	{		CMenuNextPlayVal,		CMenuSavePlayVal,	MScanTbl_InitVal },
	{		CMenuNextInitVal,		CMenuSaveInitVal,	MScanTbl_BaudVal },		//Init field
	{		CMenuNextBaudVal,		CMenuSaveBaudVal,	MScanTbl_UsrNameVal },	//Baud rate field
	{		CMenuNextUsrNameVal,	CMenuSaveUsrNameVal, MScanTbl_UsrNameTxt },
	{		CMenuIncUserNameChar,	CMenuNextUserNameChar,	NULL}
};
//
// The NOPUSH attribute causes an immediate S1 push
// The DBLHAND attribute causes 2 ScanTbl entries to flash
//
CModStateTblEntry_t CModStateTable[26] PROGMEM = {
	{	CMODPREHRNR,CMODHRNR, StateSetHRNR, (ScanTblEntry_t *)(NUMFIELDFLASH|HourInString) },
	{	CMODHRNR, CMODMINNR, StateIncHr, (ScanTblEntry_t *)(NUMFIELDFLASH|MinInString) },
	{	CMODMINNR, CMODSECNR, StateIncMin, (ScanTblEntry_t *)(NUMFIELDFLASH|SecInString) },
	{	CMODSECNR, CMODPRESET|NOPUSH, StateIncSec, 0 },			//advance immediately
	{	CMODPREHRHAND, CMODHRHAND, StateSetHRHAND, ScanTbl_HrHand },
	{	CMODHRHAND, CMODMINHAND, StateIncHr,	ScanTbl_MinHand },
	{	CMODMINHAND, CMODSECHAND, StateIncMin, ScanTbl_SecHand },
	{	CMODSECHAND, CMODPRESET|NOPUSH, StateIncSec, NULL },		//advance immediately
	{	CMODPREHRBIN, CMODHRBIN|DBLHAND, StateSetHRBIN, ScanTbl_HrHand },		//use 2 ScanTbl Entries
	{	CMODHRBIN, CMODMINBIN|DBLHAND, StateIncHr, ScanTbl_MinHand },		//use 2 ScanTbl Entries
	{	CMODMINBIN, CMODSECBIN|DBLHAND, StateIncMin, ScanTbl_SecHand },		//use 2 ScanTbl Entries
	{	CMODSECBIN, CMODPRESET|NOPUSH, StateIncSec, 0 },		//advance immediately
	{	CMODPREYEAR, CMODYEAR, StateSetYEAR, (ScanTblEntry_t *)(NUMFIELDFLASH|YearInString) },
	{	CMODYEAR, CMODMONTH, StateIncYear, (ScanTblEntry_t *)(NUMFIELDFLASH|MonthInString) },
	{	CMODMONTH, CMODPREDAY|NOPUSH, StateIncMonth, NULL },		//advance immediately
	{	CMODPREDAY, CMODDAY, StateSetDay, (ScanTblEntry_t *)(NUMFIELDFLASH|DayInString) },
	{	CMODDAY, CMODDATESET|NOPUSH, StateIncDay, 0 },				//advance immediately
	{	CMODDATESET, CMODIDLE, StateSetDate, NULL },
	{	CMODPREALARMHR, CMODALARMHR, StateSetAlarmHr, (ScanTblEntry_t *)(NUMFIELDFLASH|HourInString) },
	{	CMODALARMHR, CMODALARMMIN, StateIncAlarmHr, (ScanTblEntry_t *)(NUMFIELDFLASH|MinInString) },
	{	CMODALARMMIN, CMODALARMSEC, StateIncAlarmMin, (ScanTblEntry_t *)(NUMFIELDFLASH|SecInString) },
	{	CMODALARMSEC, CMODALARMSET|NOPUSH, StateIncAlarmSec, 0 },				//advance immediately
	{	CMODALARMSET, CMODIDLE, StateSetAlarm, NULL },
	{	CMODPRESET, CMODSET, StateSetMODSET, ScanTblPush },
	{	CMODSET, CMODCANCEL|NOPUSH, StateSetTime, 0 },			//advance immediately
	{	CMODCANCEL,0, CTimeDateCancel, 0 }
};

MScanTblEntry_t MScanTblImage[MENUSCANTABLEN] PROGMEM = {
	{	MsgPszLogo,		LOGOHPOS,LOGOVPOS},
	{	MsgPszVersion,	LOGOHPOS+12*DEFCHARWIDTH,LOGOVPOS},
	{	TMenuNumeric,	MENUHPOS,MENU0VPOS},
	{	0,				MENUHPOS+4*DEFCHARWIDTH,MENU0VPOS},
	{	TMenuDial	,	MENUH2POS,MENU0VPOS},
	{	0			,	MENUH2POS+5*DEFCHARWIDTH,MENU0VPOS},
	{	TMenuGPS	,	MENUHPOS,MENU1VPOS},
	{	0			,	MENUHPOS+4*DEFCHARWIDTH,MENU1VPOS},
	{	TMenuDST	,	MENUH2POS,MENU1VPOS},
	{	0			,	MENUH2POS+4*DEFCHARWIDTH,MENU1VPOS},
	{	TMenuApp	,	MENUHPOS,MENU2VPOS},
	{	0			,	MENUHPOS+4*DEFCHARWIDTH,MENU2VPOS},
	{	TMenuPPS	,	MENUH2POS,MENU2VPOS},
	{	0			,	MENUH2POS+4*DEFCHARWIDTH,MENU2VPOS},
	{	TMenuChrono	,	MENUHPOS,MENU3VPOS},
	{	0			,	MENUHPOS+7*DEFCHARWIDTH,MENU3VPOS},
	{	TMenuAlarm	,	MENUH2POS,MENU3VPOS},
	{	0			,	MENUH2POS+6*DEFCHARWIDTH,MENU3VPOS},
	{	TMenuLed	,	MENUHPOS,MENU4VPOS},
	{	0			,	MENUHPOS+4*DEFCHARWIDTH,MENU4VPOS},
	{	TMenuBurnin	,	MENUH2POS,MENU4VPOS},
	{	0			,	MENUH2POS+7*DEFCHARWIDTH,MENU4VPOS},
	{	TMenuPlay	,	MENUHPOS,MENU5VPOS},
	{	0			,	MENUHPOS+5*DEFCHARWIDTH,MENU5VPOS},
	{	TMenuReset	,	MENUH2POS,MENU5VPOS},
	{	0			,	MENUH2POS+6*DEFCHARWIDTH,MENU5VPOS},
	{	TMenuBaud	,	MENUHPOS,MENU6VPOS},
	{	0			,	MENUHPOS+4*DEFCHARWIDTH+MENUSPACINGWIDTH,MENU6VPOS},
	{	TMenuUsrName,	MENUH2POS,MENU6VPOS},
	{	0			,	MENUH2POS+5*DEFCHARWIDTH,MENU6VPOS},
	{	UsrNameBuf	,	MENUHPOS, MENU7VPOS}
};
