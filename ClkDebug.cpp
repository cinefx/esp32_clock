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
#include <util/delay.h>

#include "./ClkConfig.h"
#include "./ClkData.h"
#include "./ClkISR.h"
#include "./ClkSupport.h"
#include "./ClkDebug.h"

extern char MsgShortSecond[] PROGMEM;
extern char MsgPszShortSecCnt[] PROGMEM;
extern char MsgPszLongSecCnt[] PROGMEM;
extern char MsgPszAssert[] PROGMEM;

extern void UsrSendByte(char ch);
extern void UARTPrintfProgStr(PGM_P pBuf);
extern void UsrPrintNewLine(void);
extern char * MsgAddNewLine(char *p);

void DebugSerialOut(char ch);
void DebugSerialOutStrFlash(PGM_P pSrc);
void DebugSerialOutStr(char *pSrc);
void DebugSerialu08(byte b);
void DebugSerialu16(uint w);
void DebugSerialu16dec(uint w);

#if DEBUG | ASCIILINK
//
// print byte value as 2 hex chars
//
void Special_UARTPfu08(byte b)
{
	UsrSendByte(Hex2Asc(b >> 4));
	UsrSendByte(Hex2Asc(b & 0x0f));
}
#endif

#if DEBUG
//
// print byte value in decimal
//
void Special_UARTPfu08dec(byte b)
{
	char buf[4];
	byte i=2;
	do {
		buf[i] = '0' + (b % 10);
		b = b / 10;
		--i;
	} while (b);
	while (i != 255) {
		buf[i] = ' ';
		--i;
	}
	buf[3] = 0;
	UARTPrintfStr(buf);
}
//
// print word value as 4 hex chars
//
void Special_UARTPfu16(uint w)
{
	Special_UARTPfu08(w >> 8);
	Special_UARTPfu08(w & 0xff);
}
//
// print word value in decimal
//
void Special_UARTPfu16dec(uint w)
{
	char buf[6];
	itoa(w, buf, 10);
	buf[5] = 0;
	UARTPrintfStr(buf);
}

void ShowCnt(uint w)
{
	Special_UARTPfu16dec(w);
	UsrSendByte(' ');
}

void Special_UARTPfu08(byte b);
void Special_UARTPfu16(uint w);

void DebugAssert(uint line)
{
	DebugSerialOutStrFlash(MsgPszAssert);
	DebugSerialu16dec(line);
	DebugSerialOutStrFlash(MsgPszNewLine);
	cli();
#if 0
	_delay_ms(1000);
	do {
		//check if any push button switch pin is low (meaning down)
		if ((SWPin	& (_BV(SW1Bit)|_BV(SW2Bit))) != (_BV(SW1Bit)|_BV(SW2Bit))) {
			break;
		}
	} while (TRUE);
	sei();
#else
	do {} while (TRUE);
#endif
}

#if 0&DEBUG
char MsgPszCMod[] PROGMEM = "CModState = ";

void ShowCModState(byte state, byte val)
{
	char LBuf[128], *p;
	p = strcpy_P(LBuf, MsgPszCMod)  + strlen(LBuf);
	p = FormatHex(p, state);
	*p++ = ' ';
	p = FormatHex(p, val);
	p = MsgAddNewLine(p);
	ASSERT(*p == 0);
	byte l = strlen(LBuf);
	ASSERT(l <128);
	for (byte i = 0; i < l; ++i) {
		UsrSendByte(LBuf[i]);
	}
}
#endif
//
// print long value as 8 hex chars
//

void UART_Printfu32(unsigned long l)
{
	Special_UARTPfu16(l >> 16);
	Special_UARTPfu16(l & 0xffff);
}
#endif

//
// UNIMPLEMENTED
//
BOOL UsrDebugSetting0(char c)
{
	return TRUE;
}

#if DEBUG | ds1307
//
// DebugSerialOut(char ch)
//
void DebugSerialOut(char ch)
{
	while ((UCSR0A & _BV(UDRE0)) == 0) {}
	UDR0 = ch;
//	while ((UCSR0A & _BV(TXC0)) == 0) {}	//optional
}
//
// DebugSerialOutStrFlash: Use when interrupts not enabled.
//
// IN: flash based string to send to serial port
//
void DebugSerialOutStrFlash(PGM_P pSrc)
{
	char ch;
	while ((ch = pgm_read_byte(pSrc++)) != 0) {
		DebugSerialOut(ch);
	}
}
//
// DebugSerialOutStr: Use when interrupts not enabled.
//
// IN: ram based string to send to serial port
//
void DebugSerialOutStr(char *pSrc)
{
	char ch;
	while ((ch = *pSrc++) != 0) {
		DebugSerialOut(ch);
	}
}
#endif

#if DEBUG
//
// print byte value as 2 hex chars. Works with interrupts off.
//
void DebugSerialu08(byte b)
{
	DebugSerialOut(Hex2Asc(b >> 4));
	DebugSerialOut(Hex2Asc(b & 0x0f));
}

//
// print word value as 4 hex chars. Works with interrupts off.
//
void DebugSerialu16(uint w)
{
	DebugSerialu08(w >> 8);
	DebugSerialu08(w & 0xff);
}
//
// print word value in decimal.  Works with interrupts off.
//
void DebugSerialu16dec(uint w)
{
	char buf[6];
	itoa(w, buf, 10);
	buf[5] = 0;
	DebugSerialOutStr(buf);
}
#endif

#if DEBUG
char MsgPszAssert[] PROGMEM = "Assert line: ";
#endif

#if EXTDEBUG
char MsgShortSecond[] PROGMEM = "Short Second ignored";
char MsgPszLongSecCnt[] PROGMEM = "LongSecondsCount: ";
char MsgPszShortSecCnt[] PROGMEM = "ShortSecondsCount: ";

//
// Report Internal Variables
//
BOOL UsrDebugReport(char c)
{
	UARTPrintfProgStr(MsgPszShortSecCnt);
	Special_UARTPfu16(ShortSecCnt);
	UsrPrintNewLine();
	UARTPrintfProgStr(MsgPszLongSecCnt);
	Special_UARTPfu16(LongSecCnt);
	UsrPrintNewLine();
	return TRUE;
}

void ShowSecSkippedMsg(void)
{
	char ch, *p = MsgShortSecond;
	while ((ch = pgm_read_byte(p++)) != 0) {
		UsrSendByte(ch);
	} 
}

BOOL UsrDebugSetting(byte mode)
{
	if (mode == 1) {
		ExtDbg1 ^= 1;
	} else if (mode == 2) {
		ExtDbg2 ^= 1;
	} else if (mode == 3) {
		ExtDbg1 ^= 1;
		ExtDbg2 ^= 1;
	}
	return TRUE;
}
#else	//!EXTDEBUG

BOOL UsrDebugReport(char c)
{
	return TRUE;
}
#endif
