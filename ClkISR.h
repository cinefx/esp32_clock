// ClkISR.h -- Dutchtronix ISR Support
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

#define MAXTXBUFFERLEN	32
#define MAXRXBUFFERLEN	256
#define TXBUFFERMASK	0b00011111	//2^5-1
#define RXBUFFERMASK	0b11111111	//2^8-1

extern PushButtonState_t S1;
extern PushButtonState_t S2;
extern uint	Timer1Counter;
extern uint	Int0CntDown;
extern uint	Int0CntdownStart;

#if !NETFREQUENCY
extern void InitINT0(void);
extern void EnableINT0(void);
extern void DisableINT0(void);
#endif
extern void InitTC0(void);
extern void DisableTC0(void);
extern void EnableTC1(void);
extern void DisableTC1(void);
extern void InitTC1(void);
extern void SetTC1Countdown(byte nSecs);
extern void SetAlarmCountdown(void);

//
// return FALSE if counter is NULL (inactive), else return TRUE (active)
//
inline BOOL TC1CountdownActive(void)
{
	return (Timer1Counter != 0);
}
extern void EnableTC2(void);
extern void DisableTC2(void);
extern void InitTC2(void);

#if CLOCKLEDENABLED
extern void InitLed(void);
extern void BlinkLed(void);
extern void TurnOffLed(void);

#if DEBUG
extern void DBGTurnLedOn(void);
extern void DBGTurnLedOff(void);
#endif
#endif

extern void InitSwitch(void);
extern byte EepromRead(uint adr);
extern void UpdateEEprom(uint adr, byte val);
extern void WDTimerOff(void);
extern void InitiateSysReset(void);

// Serial Port (UART) declarations

extern volatile BOOL UARTOutReady;
extern void UARTKillPendingOutput(void);
extern void UARTClrRXBuffer(void);
extern BOOL UARTReceiveLine(char *pDst, byte maxLength);
extern void UARTFlushRecord(void);
extern void UARTSendByte(byte b, BOOL fWait);
extern void UARTPrintfStr(char *pString);
extern void UARTPrintfProgStr(PGM_P pString);
extern byte UARTReceiveByte(void);
extern void UARTSetBaudVal(byte newBaud);
extern void UARTInit(byte newBaud);
extern void UARTSet115200(void);
inline BOOL UARTHasData(void)
{
	return ((GPIOR0 & _BV(fRcvdChar)) != 0);
}
inline BOOL IsUARTOutReady(void)
{
	return UARTOutReady;
}

#define BEAMON	DACCTLPort |= _BV(DACZXBit)
#define BEAMOFF	DACCTLPort &= ~_BV(DACZXBit)
