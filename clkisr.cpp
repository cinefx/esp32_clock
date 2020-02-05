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
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "./ClkConfig.h"
#include "./ClkData.h"
#include "./ClkDebug.h"
#include "./ClkISR.h"

PushButtonState_t S1;
PushButtonState_t S2;
uint	Timer1Counter;
uint	TC1AlarmCounter;
uint	Int0CntDown;
uint	Int0CntdownStart;

extern void InitRTCInt(void);
extern void ResetPPSCntr(void);
extern void InitiateSysReset(void);
extern void FuncGenProcessInput(void);

void setLed(byte v);

#if !NETFREQUENCY
//
// Initialize INT0
//
// TIMEMARK signal is Positive Going Pulse
//
void InitINT0(void)
{
	//
	// configure the INT0 pin with a pull-up resistor
	// to prevent random interrupts if unconnected
	//
	PPSPort |= _BV(PPSBit);					//enable pull-up
	//
	//set ISC00 and ISC01 to INT0 on rising (PPSTrigger) or falling (!PPSTrigger) edge
	//
	EICRA = EEConfigData.PPSTrigger ? ((1<<ISC01)|(1 <<ISC00)) : ((1<<ISC01)|(0 <<ISC00));
}

void EnableINT0(void)
{
	InitRTCInt();							//setup for Interrupt
	EIMSK |= _BV(INT0);						//enable INT0
}

void DisableINT0(void)
{
	EIMSK &= ~_BV(INT0);					//disable INT0
}
//
// Handler for External Interrupt 0 (TimeMark signal)
//
// Triggered by RTC (or external source)
//
// Sets flag to be acted upon in ClockWorks
//
ISR(INT0_vect)
{
	if (fClockIsRunning) {
		BOOL fAdvance = TRUE;
		if (EEConfigData.PPSMode != ONEHZCODE) {	//Make sure PPSMode only changes with interrupts OFF
			//
			//	4096Hz. Actual PPSMode value may be offset by -4..+4
			//
			if (--Int0CntDown == 0) {
				Int0CntDown = Int0CntdownStart;		//ResetPPSCntr();
			} else {
				fAdvance = FALSE;
			}
		}
		if (fAdvance) {
			GPIOR0 |= _BV(fUpdSecond);				//Tell ClockWorks we need to advance a second
#if DEBUG
			OldTimeTicks = TimeTicks;
#endif
			TimeTicks = 0;							//start fractional seconds count
			PPSTimeOutCnt = ReloadPPSTimeOutCnt;	//reinitialize tick counter for timeout
		}
	}
}
#endif	// !NETFREQUENCY
//
// TC0 is used to program the push buttons
//
// It tracks the duration of a down push and it does key debouncing
//
// Initialize TC0
//
// Set prescaler to clkio/1024. At 20 Mhz, this is 51.2 uSec
// Using a Timer0CountUp value of 127 yields an match every 6.502 mSec
//
void InitTC0(void)
{
	TCCR0A = 0;
	TCCR0B = (1<<CS02)|(0<<CS01)|(1<<CS00);
	OCR0A = Timer0CountUp;
}
//
// Enable TC0 timer for key debouncing
//
void StartKeyDebouncing(void)
{
	GPIOR0 |= _BV(fDebouncing);
	S1.TC0Cntr = 0;
	S2.TC0Cntr = 0;
	S1.KeyState = S2.KeyState = 0xff;
	S1.Position = S2.Position = FALSE;
#if DEBUG
	TC0DebugCnt = 150;
#endif
	TIFR0 = _BV(OCF0A);				//clear pending match
	TIMSK0 = _BV(OCIE0A);			//Enable TC0.matchA
}
//
// Disable TC0 timer for key debouncing
//
void DisableTC0(void)
{
	TIMSK0 = (0<<OCIE0A);			//Disable TC0.matchA
}
//
//
void ProcessButton(PushButtonState_t *pSW, byte ButtonMsk, byte fUPMsk, byte fTimeoutMsk, BOOL fDemoEnabled)
{
//
// Decrement Timer0Counter iff != 0
//
	if (pSW->TC0Cntr && (--pSW->TC0Cntr == 0)) {
		//
		// the 8-bit Timer0S1Counter field is now 0
		//
		if (!fDemoEnabled) {								//ignore if in demo mode
			GPIOR0 |= fTimeoutMsk;							//mark TC0 timeout in flags
			pSW->Position = FALSE;							//Pretend Button is up (ignore next fUPS1)
			pSW->fAutorepeat = FALSE;
		}
	}
//
// bool_t DebounceSwitch2() Jack Ganssle embedded.com
//    static uint16_t State = 0// // Current debounce status
//    State=(State<<1) | !RawKeyPressed() | 0xe000//
//    if(State==0xf000)return TRUE//
//    return FALSE// 
//
	byte v = SWPin & ButtonMsk;
	if (v) v = 0b0000001;
	pSW->KeyState = (uint8_t)(pSW->KeyState << 1) | v;		//0 bit value means down
	if (pSW->KeyState == 0b10000000) {						//7 downs in a row
		if (pSW->Position == FALSE) {
			pSW->Position = TRUE;
			pSW->TC0Cntr = (TIMER0TIMEOUT & 0xff);
		}
	} else if (pSW->KeyState == 0b01111111) {				//7 ups in a row. Could be Button Up or failed push (<7 downs in a row)
		if (pSW->fAutorepeat) {
			pSW->fAutorepeat  = FALSE;
			GPIOR0 &= ~fTimeoutMsk;
		} else if (pSW->Position != FALSE) {				//If Button already UP (FALSE), ignore this event
			GPIOR0 |= fUPMsk;
		}
		pSW->Position = FALSE;
		pSW->TC0Cntr = 0;
	} else if (pSW->KeyState == 0) {
		//
		// pSW->KeyState==0 indicates key has been down for at least 8 checks.
		// Start autorepeating (if allowed) after S1 has had a timeout (fTimeoutS1)
		//
		if (pSW->okToRepeat && (GPIOR0 & fTimeoutMsk)) {
			if (pSW->fAutorepeat == FALSE) {
				pSW->Position = TRUE;		//Undo "Pretend Button is up" (we want the next fUPS1)
				pSW->autorepeatcnt = AUTOREPEATVAL;
				pSW->fAutorepeat = TRUE;
			} else {
				if (--pSW->autorepeatcnt == 0) {
					//
					// reached 0. Insert an fUPS1 event.
					//
					GPIOR0 |= fUPMsk;	//set UP in flags
					pSW->autorepeatcnt = AUTOREPEATVAL;
				}
			}
		}
	}
}
//
// Timer0 Match Handler. Used to debounce/interpret S1 and S2
//
ISR(TIMER0_COMPA_vect)
{
	TCNT0 = 0;
	sei();											//stay as accurate as possible
#if DEBUG
	if (--TC0DebugCnt == 0) {
		Flags.DBGT = TRUE;				//set DBG Timeout in flags
		TC0DebugCnt = 150;
	}
#endif
	ProcessButton(&S1, _BV(SW1Bit), _BV(fUPS1), _BV(fTimeoutS1), Flags.DemoEnabled);
	ProcessButton(&S2, _BV(SW2Bit), _BV(fUPS2), _BV(fTimeoutS2), FALSE);
	if ((S1.KeyState & S2.KeyState) == 0xff) {		//all ones means UP. done if both switches are all ones
		GPIOR0 &= ~_BV(fDebouncing);
		DisableTC0();								//stop key debouncing timer
	}
}
//
// TC1 is the general purpose timer
//
// Used for 1 PPS timeout checking, field flashing timing
//
// We use the 16-bit timer for this purpose since it will
// get us closest to precisely 1 second.
//
// Set prescaler to clkio/256. At 20 Mhz, this is 12.8 uSec 
//
void EnableTC1(void)
{
	Timer1Counter = 0;
	TC1AlarmCounter = 0;
	TCNT1 = 0;
	TIMSK1 |= _BV(OCIE1A);				//Enable TC1.matchA
}
//
//
void DisableTC1(void)
{
	TIMSK1 &= ~_BV(OCIE1A);				//Disable TC1.matchA
}
//
//
void InitTC1(void)
{
	TCCR1B = 0b00000100;				//TC1.ck = 20Mhz/256 = 12.8 uSec
	OCR1A = Timer1CountUp;
	EnableTC1();
}
//
// SetTC1Countdown(byte nSecs)
//
void SetTC1Countdown(byte nSecs)
{
	byte flags = SREG;
	cli();
	Timer1Counter = nSecs * TIMER1TIMEOUT1SEC;
	SREG = flags;
}
//
// SetAlarmCountdown. Timer used to terminate Alarm function.
//
void SetAlarmCountdown(void)
{
	TC1AlarmCounter = 60 * TIMER1TIMEOUT1SEC;
	fClrAlarm = FALSE;
}
//
// Timer1 Compare Match Handler.
//
// Operates at 60 Hz (16.666 mSecs)
//
// Use the 16 bit counter since it approaches 1 second closest.
//
// Maintains a timer in case INT0 fails.  Also used as a flash counter
//
// Other timers to be added (1 byte counter yields max 4.2 seconds delay)
//
// Side effect of every timer is that the beam stops when executing the timer,
// causing a slightly brighter spot.
//
ISR(TIMER1_COMPA_vect)
{
	byte beam = (DACCTLPort & _BV(DACZXBit));		//save BEAM status
	BEAMOFF;
	TCNT1 = 0;
//
// Decrement 16 bit Timer1Counter iff != 0
//
	if (Timer1Counter) {
		--Timer1Counter;
	}
//
// Decrement 16 bit TC1AlarmCounter iff != 0
//
	if (TC1AlarmCounter) {
		--TC1AlarmCounter;
		if (TC1AlarmCounter == 0) {
			fClrAlarm = TRUE;
		}
	}
//
//  countdown in case INT0 fails
//
	if (PPSTimeOutCnt && (--PPSTimeOutCnt == 0)) {
		fTimeOut = TRUE;						//set TimeOut Flag
		PPSTimeOutCnt = ReloadPPSTimeOutCnt;	//reinitialize tick counter for timeout
	}
//
// Countdown for flashing fields
//
	if (--FlashCount == 0) {
		GPIOR0 |= _BV(fFLASH);					//set Flash-Bit
		FlashCount = FLASHTICKS;				//reload
	}
	if (Time2Ticks) {
		--Time2Ticks;
	}
	if (fClockIsRunning) {
#if CLOCKLEDENABLED
		// TimeTicks is reset by INT0, likely before reaching the 60 value (ideally 59)
		// Postpone the "++TimeTicks" step until after this test.
		// But don't emit if TimeTicks == 60, since that would generate an extra event
		// fUpdateLed causes the LED update to be done in ClockWorks()
		// because CharPixelNew wants to cache PORTD and the LED update here changes PORTD
		if ((LedMorseHead > LedMorseTail)) {
			if ((TimeLedTicks  >= 6) && (TimeTicks < 60)) {
				ASSERT(EEConfigData.LedOption == LEDMORSE);
				fUpdateLed = TRUE;
				TimeLedTicks = 0;
			}
			++TimeLedTicks;
		}
#endif
		++TimeTicks;
	}
	DACCTLPort |= beam;		//restore BEAM status
}
//
// TC2 is used to run a 200Hz clock. Normally does NOT run.
//
// Initialize and Enable TC2
// Set prescaler to clkio/1024. At 20 Mhz, this is 51.2 uSec
// Using a Timer2CountUp value of 97 yields an overflow every 4.99 mSec (about 200 Hz)
//
void EnableTC2(void)
{
	TCNT2 = 0;
	TIFR2 = _BV(OCF2A);							//clear pending match
	TIMSK2 = _BV(OCIE2A);						//Enable TC2.Match A
}

void DisableTC2(void)
{
	TIMSK2 &= ~_BV(OCIE2A);						//Disable TC2.Match A
}

void InitTC2(void)
{
	TCCR2A = 0;
	TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);
	OCR2A = Timer2CountUp;
}

ISR(TIMER2_COMPA_vect)
{
	TCNT2 = 0;
	//this call forces many registers to be saved, incl Z so it cannot be changed by FuncGenProcessInput()
	if (fMiniDDS) {								//If MiniDDS mode
		FuncGenProcessInput();
		return;
	}
#if NETFREQUENCY
	byte v1 = 0;								//preset
	byte v2;
	if (PIND & _BV(NET_BIT)) {
		v = 1;
	}
	v2 = NetCycle;								//state of last sample
	NetCycle = v1;								//current sample
	if (v1 ^ v2) {								//check for state change
		// one half cycle completed.
		if (NetCyclePhase == 0) { 				//if 0, first change. Need 2 for a cycle
			NetCyclePhase = 1;
		} else {								// one cycle completed
			NetCycleRefresh = 1;				//time to refresh the screen
			NetCyclePhase = 0;					//reset
			if (++NetCycleCnt >= 60) {			//Seconds Transition, based on Net Frequency
				ASSERT(EEConfigData.PPSMode == ONEHZCODE);
				if (fClockIsRunning) {
					GPIOR0 |= _BV(fUpdSecond);	//Tell ClockWorks we need to advance a second
					TimeTicks = 0;				//start fractional seconds count
					PPSTimeOutCnt = ReloadPPSTimeOutCnt;	//reinitialize tick counter for timeout
				}
				NetCycleCnt = 0;
			}
		}
	}
#endif
}

#if CLOCKLEDENABLED
void InitLed(void)
{
	LEDDDR |= _BV(LEDBit);					//set as output
	LEDPort &= ~_BV(LEDBit);				//turn off
}
//
void BlinkLed(void)
{
	if (EEConfigData.LedOption == LEDENABLED) {
		LEDPin |= _BV(LEDBit);				//toggle the LED
	}
}
//
void TurnOnLed(void)
{
	LEDPort |= _BV(LEDBit);
}
//
void TurnOffLed(void)
{
	LEDPort &= ~_BV(LEDBit);
}
//
void setLed(byte v)
{
	if (v != 0) TurnOnLed(); else TurnOffLed();
}

#endif
//
void InitSwitch(void)
{
	SWDDR &= ~(_BV(SW1Bit)|_BV(SW2Bit));	//set as input
	SWPort |= (_BV(SW1Bit)|_BV(SW2Bit));	//enable pull-up resistor
}
//
//	Execute a read on the EEPROM
//	arg: eeprom address to read
//	out: data byte
//	no address increment
//
byte EepromRead(uint adr)
{
	do {} while (EECR & _BV(EEPE));
	EEAR = adr;
	EECR |= _BV(EERE);							//read command
	return EEDR;
}
//
// EepromWrite(uint EEpromadr, byte val)
//
void EepromWrite(uint adr, byte val)
{
	do {} while (EECR & _BV(EEPE));
	EEAR = adr;
	EEDR = val;
	byte flags = SREG;
	cli();
	EECR |= _BV(EEMPE);							//write command
	EECR |= _BV(EEPE);
	SREG = flags;
}
//
//UpdateEEprom(uint EEpromadr, byte val)
//
void UpdateEEprom(uint adr, byte val)
{
	if (Flags.DemoEnabled == 0) {						//No Update in Demo Mode
		EepromWrite(adr, val);
	}
}
//
// Disable Watchdog Timer Completely
//
void WDTimerOff(void)
{
	wdt_reset();
	MCUSR &= ~_BV(WDRF);
	WDTCSR |= (_BV(WDCE) | _BV(WDE));
	WDTCSR = 0;
}
//
// Initiate a System Reset
//
void inline InitiateSysReset(void)
{
	cli();
	WDTCSR |= (_BV(WDCE) | _BV(WDE));
	WDTCSR = (_BV(WDE)|_BV(WDP1));				//Watchdog System Reset Enable, 64 mSec timeout
	while (TRUE);
}

typedef struct BaudInfo {
	uint	val;
	byte	twox;
} BaudInfo_t;

// values reflect a 20 Mhz system clock.
BaudInfo_t MapBaudVal[9] PROGMEM = {
	{ 64, 0 },	// 19,200
	{ 86, 1	},	// 28,800
	{ 64, 1	},	// 38,400
	{ 42, 1	},	// 57,600
	{ 10, 0	},	// 115,200
	{ 4,  0	},	// 250,000
	{ 520,1	},	// 4800
	{ 129,0	},	// 9600
	{ 86, 0	}	// 14,400
};

char TXBuffer[MAXTXBUFFERLEN];
char RXBuffer[MAXRXBUFFERLEN];
volatile BOOL UARTOutReady;
volatile byte TXBufferHead;
volatile byte TXBufferTail;
volatile byte RXBufferHead;
volatile byte RXBufferTail;
uint	IntrpCnt;

void UARTInit(byte newBaud)
{
	UCSR0A = 0;
// set rx interrupt, tx interrupt, clear data empty interrupt
	UCSR0B = _BV(TXCIE0)|_BV(TXEN0)|_BV(RXCIE0)|_BV(RXEN0);
// set default 8 bits, no parity, 1 stop
	UCSR0C = (_BV(UCSZ01)|_BV(UCSZ00));
	UARTSetBaudVal(newBaud);
	UARTOutReady = TRUE;
#if DEBUG
//
// all memory cleared in RELEASE mode
//
	RXBufferHead = 0;
	RXBufferTail = 0;
	Flags.RXLFDetected = 0;
	TXBufferHead = 0;
	TXBufferTail = 0;
	IntrpCnt = 0;
#endif
}
//
// Input: internal baud rate code
//
// Map internal baud rate code to baud rate value for AVR
// and set in UART
//
void UARTSetBaudVal(byte newBaud)
{
	//Overkill.
	byte sreg = SREG;
	cli();
	UBRR0 = pgm_read_word(&MapBaudVal[newBaud].val);
	if (pgm_read_byte(&MapBaudVal[newBaud].twox)) {
		UCSR0A |= _BV(U2X0);
	} else {
		UCSR0A &= ~_BV(U2X0);
	}
	SREG = sreg;
}

void UARTStop(void)
{
	UCSR0A = 0;
	UCSR0B = 0;
	UCSR0C = 0;
	UARTOutReady = FALSE;
}
//
// UART Transmit Complete Interrupt
//
ISR(USART_TX_vect)
{
	byte tail = TXBufferTail;				//remove volatile attribute since ints are off
	if (TXBufferHead == tail) {
		UARTOutReady = TRUE;				// no more buffered data
		return;
	}
	UDR0 = TXBuffer[tail];
	TXBufferTail = ((tail + 1) & TXBUFFERMASK);
}
//
// UART Receive Complete Interrupt
//
ISR(USART_RX_vect)
{
	byte beam = (DACCTLPort & _BV(DACZXBit));		//save BEAM status
	BEAMOFF;
	byte newHead = (RXBufferHead + 1) & RXBUFFERMASK;
	byte b = UDR0;
	if (RXBufferTail == newHead) {
		//	No room for incoming character in RingBuffer.
		// Ignoring an incoming byte may cause serious problems for the binary cmd mode of the
		// terminal. Some thoughts:
		//  .set SerialInOverflow Flag indicating there is a problem with the stream.
		//  .count nr of missed bytes
		//  .mark the position of the last valid incoming byte
		//  .This happens all the time when displaying the Help Screen or changing to Menu Mode (in Terminal)
#if EXTDEBUG
		++nSerialInMissed;
		LastGoodByteIdx = RXBufferHead;
#endif
		Flags.SerialInOverflow = TRUE;
		if (EEConfigData.GPSInEnabled) {
			Flags.RXLFDetected = TRUE;	// make sure we empty the buffer
		}
	} else {
		RXBuffer[RXBufferHead] = b;
		GPIOR0 |= _BV(fRcvdChar);
		if ((b == LF) && (EEConfigData.GPSInEnabled)) {
			Flags.RXLFDetected = TRUE;						// make sure we empty the buffer
		}
		RXBufferHead = newHead;
	}
	// use TC2 to force FuncGen to process the input
	if (fMiniDDS && !Flags.InputActive) {
		TIMSK2 = _BV(OCIE2A);		//Enable TC2.Match A
	}
	++IntrpCnt;
	DACCTLPort |= beam;		//restore BEAM status
}
//
// void UARTSendByte(char ch, BOOL fForce)
//
// Assumes interrupts are ON. Do not call from an ISR.
//
void UARTSendByte(byte b, BOOL fForce)
{
	if (UARTOutReady) {										//if ready, send out
		UARTOutReady = FALSE;
		UDR0 = b;
	} else {
		byte newHead = (TXBufferHead + 1) & TXBUFFERMASK;
		if (TXBufferTail == newHead) {						//if equal, there is no room
			if (fForce) {
				do {
					fpCurrentRefresh();						//draw the display.
				} while (UARTOutReady == FALSE);
				UARTOutReady = FALSE;
				UDR0 = b;
			}
		} else {
			TXBuffer[TXBufferHead] = b;
			TXBufferHead = newHead;
		}
	}
}
//
// Print String
//
void UARTPrintfStr(char *pBuf)
{
	char ch;
	while ((ch = *pBuf++) != 0) {
		UARTSendByte(ch, TRUE);
	}
}
//
// Print flash based text
//
void UARTPrintfProgStr(PGM_P pBuf)
{
	char ch;
	while ((ch = pgm_read_byte(pBuf)) != 0) {
		UARTSendByte(ch, TRUE);
		++pBuf;
	}
}
//
// RXBufferHead may change while executing this function
//
byte UARTReceiveByte(void)
{
	byte b;
	while (!(GPIOR0 & _BV(fRcvdChar))) ;
	ASSERT(RXBufferHead != RXBufferTail);
	b = RXBuffer[RXBufferTail];
	RXBufferTail = (RXBufferTail + 1) & RXBUFFERMASK;
	if (RXBufferHead != RXBufferTail) {
		return b;						// more chars available
	}
	GPIOR0 &= ~_BV(fRcvdChar);
	return b;
}
//
//
BOOL UARTReceiveLine(char *pDst, byte maxLength)
{
	char ch;
	byte head, tail, l = 0;
	byte sreg = SREG;
	cli();
	ASSERT(GPIOR0 & _BV(fRcvdChar));
	head = RXBufferHead;
	tail = RXBufferTail;
	do {
		ch = RXBuffer[tail];
		tail = (tail + 1) & RXBUFFERMASK;
		if (++l <= maxLength) {
			*pDst++ = ch;
		}
	} while ((ch != LF) && (head != tail));
//
// Change last character to NULL. It's an LF or the line was incomplete.
//
	*--pDst = 0;
	if (head == tail) {					//is buffer now empty?
		GPIOR0 &= ~_BV(fRcvdChar);		//buffer now empty
	}
	RXBufferTail = tail;
	SREG = sreg;
	return l;
}
//
// Flush remainder of current record (until LF or end)
//
void UARTFlushRecord(void)
{
	char ch;
	byte head, tail;
	if (GPIOR0 & _BV(fRcvdChar)) {
		byte sreg = SREG;
		cli();
		head = RXBufferHead;
		tail = RXBufferTail;
		do {
			ch = RXBuffer[tail];
			tail = (tail + 1) & RXBUFFERMASK;
		} while ((ch != LF) && (head != tail));
		if (head == tail) {						//is buffer now empty?
			GPIOR0 &= ~_BV(fRcvdChar);			//yes
		}
		RXBufferTail = tail;
		SREG = sreg;
	}
}
//
//
void UARTKillPendingOutput(void)
{
	TXBufferHead = 0;
	TXBufferTail = 0;
	//wait for current send, if any, to complete
	while (!UARTOutReady) ;
}
