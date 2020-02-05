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

#include "./ClkConfig.h"
#include "./ClkData.h"
#include "./ClkDebug.h"
#include "./ClkFlash.h"
#include "./ClkISR.h"

enum {
	StateChar,
	StateNoCmd,
	StateCmd
} TermState;
//
// define Vertical Position (Y) zero (0) in input as top line.
// map to bottom line for internal processing
//

byte argidx;
byte args[5];

char TermMsgPszWelcome[] PROGMEM =	" Terminal";
#if DEBUG
char TermMsgPszSignOn[] PROGMEM =	" AVR Oscilloscope Terminal";
#endif

void ProcessCmd1(byte cmd, byte arg);
void ProcessCmd2(byte cmd, byte arg1, byte arg2);
void ProcessCmd3(byte cmd, byte *pArgs);
char FillTermLine(char *p, char ch);
void SetTermScreen(void);
void ClearTermScreen(void);
void SetWelcomeTermScreen(void);
void TermSerialInput(void);
void ClockRefresh(void);
void TermRefresh(void);
char *ClearTermLine(char *p);
void ScrollTermScreen(void);
char *GetTermLine(byte line);
void StartCursorFlashing(void);
void DisplayableChar(char ch);
void DoTermLF(void);
void SubInitTerm(void);
void StopCursorFlashing(void);

extern byte SetDialDisplay(void);
extern void SetNumDisplay(void);
extern void UsrPrintNewLine(void);
extern void ClockWorks(void);
extern void UsrShowStatus(void);
extern void UsrSendByte(char ch);
extern void DisableRotation(void);
extern void EnableRotation(void);
extern void DisplayHelpscreen(void);
extern byte ComputeSRamHPos(char *p);

void InitTerm(void)
{
	SubInitTerm();
	VectorStartIdx = 0;
	VectorRngLen = 0;
	TermState = StateChar;
	TermFlags.DispCursor = TRUE;
	TermFlags.ProgClkLineSet = FALSE;
	TermFlags.ProgClkLine = 0;
	TermFlags.ProgBurnInSet = FALSE;
	TermFlags.ProgBurnIn = 0;
	if (EEConfigData.ioctl & _BV(IOCTL_NUMOFFTERM)) {			//Default 0 means ON
		nScreenLines = MAXSCREENLINES;
	} else {
		nScreenLines = DEFSCREENLINES;
	}
	SetTermScreen();					//initialize ScanTbl
	ClearTermScreen();
	SetWelcomeTermScreen();
}

void SubInitTerm(void)
{
	CurLine = 0;
	CurPosition = 0;							//beginning of line
	TermFlags.LFPending = FALSE;
	TermFlags.CRPending = FALSE;
}
//
// Configure dynamic options, i.e. those that may change after InitTerm() was called.
// Mostly the IOCTL options.
//
void SetDynamicTermOptions(void)
{
	// Value set by program takes priority
	if (TermFlags.ProgClkLineSet) {
		if (TermFlags.ProgClkLine == FALSE) {
			fDispNumBuf = FALSE;
			nScreenLines = MAXSCREENLINES;
		} else {
			fDispNumBuf = TRUE;
			nScreenLines = DEFSCREENLINES;
			// cursor position should be in range, if set by program
			ASSERT(CurLine < DEFSCREENLINES);
		}
	} else {
		if (EEConfigData.ioctl & _BV(IOCTL_NUMOFFTERM)) {			//Default 0 means ON
			fDispNumBuf = FALSE;
			nScreenLines = MAXSCREENLINES;
		} else {
			fDispNumBuf = TRUE;
			nScreenLines = DEFSCREENLINES;
			// correct cursor position, if needed
			if (CurLine >= DEFSCREENLINES) {
				CurLine = DEFSCREENLINES - 1; 
			}
		}
	}
	if (EEConfigData.ioctl & _BV(IOCTL_USRNAMETERM)) {			//Default 0 means Numeric Line
		TermTextBuf = ScanTblUsrName;
		ScanTblUsrName->Xoffset = ComputeSRamHPos((char *)ScanTblUsrName->pVect);
		ScanTblUsrName->Yoffset = TERMNUMERICVPOS;				//Vertical Position for Numeric Field
	} else {
		TermTextBuf = ScanTblNum;
		ScanTblNum->Yoffset = TERMNUMERICVPOS;					//Vertical Position for Numeric Field
	}
	// BurnIn always enabled (though may be turned off) when coming here.
	// Value set by program takes priority
	if (TermFlags.ProgBurnInSet) {
		if (TermFlags.ProgBurnIn == FALSE) {
			DisableRotation();
		}
	} else {
		if ((EEConfigData.ioctl & _BV(IOCTL_BURNINTERM)) == 0) {	//Default 0 means Disable Rotation
			DisableRotation();
		}
	}
}

void RunTerm(void)
{
	// keep in mind that selecting the menu exits this function and restarts when continuing.
	// Therefore, no general init at the beginning of RunTerm().
	// There could be an init call before starting the application from main()
	fpCurrentRefresh = TermRefresh;							//Refresh func ptr in Terminal Mode (word address)
	SetDynamicTermOptions();
	//
	// RunTerm Program Forever Loop.
	//
	Time2Ticks = TERMTIMEOUTTICKS;							//Time before we start to flash the cursor
	do {
		ClockWorks();										//Update the display and process events

		if (GPIOR0 & _BV(fUPS2)) {
			StopCursorFlashing();
			DisplayHelpscreen();
		}
		//
		// Did we receive an S1 Button Up Message (Menu)?
		//
		if (GPIOR0 & _BV(fUPS1)) {
			GPIOR0 &= ~_BV(fUPS1);
			StopCursorFlashing();
			EnableRotation();								//set burnin screen movement setting.
			SetDialDisplay();								//undo any changes made in SetDynamicTermOptions()
			return;
		}
		//
		// Was a char received on the Serial Input
		//
		if (UARTHasData()) {
			//
			// Process Bunched Data
			//
			StopCursorFlashing();
			byte cnt = 175; //MAXBUNCHEDBYTES;					//max chars per bunch
			do {
				TermSerialInput();							//process one char
				if (!UARTHasData()) {						//no more input waiting
					break;
				}
			} while (--cnt);
			//
			// Show a blinking cursor at the current char position
			// but only after no input was received for at least 1 second,
			// to prevent wasting too much time setting up the flash line.
			// Also used to timeout the parser state.
			//
			Time2Ticks = TERMTIMEOUTTICKS;
			continue;
		}
		if (Time2Ticks == 0) {
			if (TermFlags.DispCursor) {
				StartCursorFlashing();						//iff we're not flashing yet!
			}
			// reset parser state
			TermState = StateChar;
		}
	} while (TRUE);
//
// end of Main Program Loop
//
}
//
//	TermSerialInput
//
//	Accepts a buffered serial input char and update display memory
//
void TermSerialInput(void)
{
	ASSERT(CTextFlashPtr == NULL);				//Flashing stopped before calling this function
	byte b = UARTReceiveByte();					//get byte
	if (TermState == StateNoCmd) {				//Binary mode, waiting for cmd
		if (b <= TermMaxCmd) {					//Valid cmd?
			TermCmd = b;
			TermState = StateCmd;
			argidx = 0;
		} else {
			TermState = StateChar;
		}
	} else if (TermState == StateCmd) {			//Binary mode, collecting args
		args[argidx] = b;
		++argidx;
		if ((argidx == 1) && (TermCmd >= TermSetCursorCtl)) {
			ProcessCmd1(TermCmd, b);
			TermState = StateChar;
		} else if ((argidx == 2) && (TermCmd >= VectRange)) {
			ProcessCmd2(TermCmd, args[0], b);
			TermState = StateChar;
		} else if (argidx == 5) {
			if (TermCmd == VectEntry) {			//Validate
				ProcessCmd3(VectEntry, args);
			}
			TermState = StateChar;				//stop at 5 arguments, whatever TermCmd is.
		}
	} else {
		ASSERT(TermState == StateChar);
		if (b == TERMESCAPE) {
			TermState = StateNoCmd;
			TermCmd = VectNone;
			return;
		}
		if (b <= 0x1f) {							//could still be >127
			if (b == BS) {							//Backspace		
				if (CurPosition != 0) {
					--CurPosition;
				}
				return;
			}
			if (b == CR) {
				if (TermFlags.CRPending) {
					DoTermLF();
				}
				if (TermFlags.LFPending) {
					// was this a real LF?
					if (CurPosition < MAXLINESIZE) {
						return;						//Yes, ignore this CR
					}
					//
					// we received a CR after a line overflow. Clear the line overflow status
					//
					TermFlags.LFPending = FALSE;
				}
				// no, pend the CR
				TermFlags.CRPending = TRUE;
				return;
			}
			if (b == TAB) {
				DisplayableChar(' ');
				return;
			}
			if (b == LF) {
				// If its a linefeed char then set linefeed pending flag
				// Handle multiple LFs.  LF implies CR
				// <CR><LF> is identical to <LF>
				if (TermFlags.LFPending) {
					DoTermLF();
				}
				TermFlags.LFPending = TRUE;
				TermFlags.CRPending = FALSE;
				return;
			}
			// Display any other control char as "^<letter>"
			DisplayableChar('^');
			DisplayableChar(b + 'A');
			return;
		} else {
			if (b > 127) {
				return;								//ignore
			}
		}
		DisplayableChar(b);
	}
}
//
// Displayable char routine
//
void DisplayableChar(char ch)
{
	ASSERT((ch > 0x1f) & (ch <= 127));
//
//	Do line feed if pending
//
	if (TermFlags.LFPending) {
		DoTermLF();								//will reset CurPosition
	}
	if (TermFlags.CRPending) {
		TermFlags.CRPending = FALSE;
		CurPosition = 0;
	}
	char *p = GetTermLine(CurLine);
	ASSERT(CurPosition < MAXLINESIZE);
	*(p + CurPosition) = ch;					//store char in display memory
	if (ch != ' ') {
		LinesEmpty[CurLine] = FALSE;
	}
	if (++CurPosition >= MAXLINESIZE) {
		TermFlags.LFPending = TRUE;
	}
}

#if DEBUG
void TermShowStatus(void)
{
	UARTPrintfProgStr(MsgPszLogo);							//Dutchtronix
	UARTPrintfProgStr(TermMsgPszSignOn);					//signon
	UsrSendByte(' ');
	UARTPrintfProgStr(MsgPszVersion);
	UsrPrintNewLine();
	UsrShowStatus();
}
#endif

void SetWelcomeTermScreen(void)
{
	strcpy_P(TermLines[0], MsgPszLogo);						//Dutchtronix
	strcpy_P(TermLines[0] + strlen(TermLines[0]), TermMsgPszWelcome);
	LinesEmpty[0] = FALSE;
	CurLine = 1;
	char ch = '1';
	for (byte i = 8; i < nScreenLines; ++i) {
		ch = FillTermLine(TermLines[i], ch);
		LinesEmpty[i] = FALSE;
	}
}

void ClearTermScreen(void)
{
	char *p = TermLines[0];
	for (byte i = 0; i < nScreenLines; ++i) {
		p = ClearTermLine(p);
		LinesEmpty[i] = TRUE;
	}
	SubInitTerm();
}
//
// char FillTermLine(char *p, char ch);
//
char FillTermLine(char *p, char ch)
{
	for (byte i = 0; i < MAXLINESIZE; ++i) {
		*p++ = ch;
		if (++ch > 127) {
			ch = ' ';
		}
	}
	*p = 0;
	return ch;
}

void SetTermScreen(void)
{
	byte vpos = TOPLINEVPOS;
	for (byte i = 0; i < MAXSCREENLINES; ++i) {
		TermScanTblStr[i].pVect = (VectorTblEntry_t *)TermLines[i];
		TermScanTblStr[i].Xoffset = 7;
		TermScanTblStr[i].Yoffset = vpos;
		vpos -= LINEVSIZE;
		TermLines[i][MAXLINESIZE+1] = 0xff;					//debug marker
	}
}
//
// Scroll One Line
//
void ScrollTermScreen(void)
{
	byte i;
	VectorTblEntry_t *pLine = TermScanTblStr[0].pVect;
	for (i = 0; i < (nScreenLines - 1); ++i) {
		TermScanTblStr[i].pVect = TermScanTblStr[i+1].pVect ;
		LinesEmpty[i] = LinesEmpty[i+1];
	}
	ASSERT(i == nScreenLines - 1);
	TermScanTblStr[i].pVect = pLine;
	ClearTermLine((char *)pLine);
	LinesEmpty[i] = TRUE;
}
//
// Clear Terminal Data Line - fill it with spaces
//
char *ClearTermLine(char *p)
{
	for (byte i = 0; i < MAXLINESIZE; ++i) {
		*p++ = ' ';
	}
	*p = 0;
	return p + 2;
}

void DoTermLF(void)
{
	if (++CurLine >= nScreenLines) {
		ScrollTermScreen();
		CurLine = nScreenLines - 1;
	} else {
		//ScrollTermScreen already clears the last line. Use else
		(void)ClearTermLine(GetTermLine(CurLine));
		LinesEmpty[CurLine] = TRUE;
	}
	CurPosition = 0;
	TermFlags.LFPending = FALSE;
	TermFlags.CRPending = FALSE;
}

char *GetTermLine(byte line)
{
	if (line >= nScreenLines) {				//out of range?
		line = 0;
	}
	return (char *)TermScanTblStr[line].pVect;
}
//
// Display a flashing cursor. This is done by building an alternate
// buffer for the current Display Line and mark the current line
// for flashing
//
void StartCursorFlashing(void)
{
	byte pos;
	if (CTextFlashPtr != NULL) {
		return;
	}
	char *pSrc = GetTermLine(CurLine);
	char *pDst = TermFlashBuffer;
	while ((*pDst++ = *pSrc++) != 0) ;					//copy line to TermFlashBuffer
//
// compute current position in TermFlashBuffer
//
	pos = CurPosition;
	if (pos >= MAXLINESIZE) {
		pos = MAXLINESIZE - 1;
	} else if (TermFlags.LFPending && (pos != 0)) {
		--pos;
	}
	TermFlashBuffer[pos] = '_';
	CTextFlashPtr = AddFlashItem(&TermScanTblStr[CurLine], 0xff, (VectorTblEntry_t*)TermFlashBuffer);
	LinesEmpty[CurLine] = FALSE;
}
//
//
void StopCursorFlashing(void)
{
	RemoveFlashItem(&CTextFlashPtr);
}
//
//
void ProcessCmd1(byte cmd, byte arg)
{
	if (cmd == TermSetCursorCtl) {
		TermFlags.DispCursor = !(arg == 0);
		// if FALSE, removing current cursor not needed. Already done before calling TermSerialInput()
		ASSERT(CTextFlashPtr == NULL);
	} else if (cmd == TermSetClkLine) {
		TermFlags.ProgClkLineSet = TRUE;
		TermFlags.ProgClkLine = arg;
		if (arg == 0) {
			fDispNumBuf = FALSE;
			nScreenLines = MAXSCREENLINES;
		} else {
			fDispNumBuf = TRUE;
			nScreenLines = DEFSCREENLINES;
			// correct cursor position, if needed
			if (CurLine >= DEFSCREENLINES) {
				CurLine = DEFSCREENLINES - 1;
			}
		}
	} else if (cmd == TermSetBurnIn) {
		TermFlags.ProgBurnInSet = TRUE;
		TermFlags.ProgBurnIn = arg;
		if (arg == 0) {
			DisableRotation();
		} else {
			EnableRotation();
		}
	} else if (cmd == TermCtrl) {
		if (arg == 0) {
			ClearTermScreen();
		}
	}
}

void ProcessCmd2(byte cmd, byte arg1, byte arg2)
{
	if (cmd == VectRange) {
		//set range in VectGraphicsTable to display. arg2 is length and may be 0
		if ((arg1 + arg2) > MAXVECTENTRIES) {
			arg2 = 0;
		}
		VectorStartIdx = arg1;
		VectorRngLen = arg2;
	} else if (cmd == TermSetCursor) {
		//new cursor position. arg1 is horizontal (x), arg2 is vertical (y). These are char coordinates, not pixel coordinates
		if (arg1 < MAXLINESIZE) {
			CurPosition = arg1;
		} else {
			CurPosition = 0;							// make sure we have a valid position
		}
		if (arg2 < nScreenLines) {
			CurLine = arg2;
		}
		TermFlags.LFPending = FALSE;
		TermFlags.CRPending = FALSE;
	}
}

void ProcessCmd3(byte cmd, byte *pArgs)
{
	ASSERT(cmd == VectEntry);
	byte idx = pArgs[0];
	if (idx < MAXVECTENTRIES) {							// ignore if index out of range
		VectorTbl[idx].v[0] = pArgs[1];
		VectorTbl[idx].v[1] = ~(pArgs[2]);				//invert y0
		VectorTbl[idx].v[2] = pArgs[3];
		VectorTbl[idx].v[3] = ~(pArgs[4]);				//invert y1
	}
}
