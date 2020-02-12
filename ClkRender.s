; ClkRender.s -- source for Dutchtronix Oscilloscope Clock
;
;  Copyright @ 2010 Johannes P.M. de Rie
;
;  All Rights Reserved
;
;  This file is part of the Dutchtronix Oscilloscope Clock Distribution.
;  Use, modification, and re-distribution is permitted subject to the
;  terms in the file named "LICENSE.TXT", which contains the full text
;  of the legal notices and should always accompany this Distribution.
;
;  This software is provided "AS IS" with NO WARRANTY OF ANY KIND.
;
;  This notice (including the copyright and warranty disclaimer)
;  must be included in all copies or derivations of this software.
;

#include "avr.h"
#include "ClkConfig.h"

	.extern	ScanTbl, MScanTbl, TermScanTblStr
#define	ScanTblStrFlash (ScanTbl + stStrFlash * 4)
#define	ScanTblStrRam (ScanTbl + stStrSRam * 4)
#define	ScanTblNumBuf (ScanTbl + stNumBuf * 4)

#define BEAMON	sbi DACZX
#define BEAMOFF	cbi	DACZX

	.section .bss
	.global	VectorTbl
	.global	WaveTbl
//
//	VectorTbl & WaveTbl use the same SRAM memory
//
//
//	WaveTbl needs to be at a 256 byte boundary
//
	.balign 256
VectorTbl:
WaveTbl:
	.skip	256						//256 bytes for WaveTbl
	.skip	256						//512 bytes for VectorTbl	

	.text
	.global	SetMem
	.global	DACinit
	.global	ClockRefresh
	.global	MenuRefresh
	.global	TermRefresh
	.global bintobcd
	.global	HelpScreenRefresh
	.global	CalibrationRefresh
	.global	TC2MatchHandler
	.global	TC1MatchHandler

	.balign 2

SetMem:
#if DEBUG
	ser		r18
#else
	clr		r18
#endif
	ldiw	Z,SRAM_START			;Preset RAM
	in		r19,(_SFR_IO_ADDR(SPL))
	in		r20,(_SFR_IO_ADDR(SPH))
1:
	st		Z+,r18
	cp		ZL,r19
	cpc		ZH,r20
	brne	1b
	ret

#define	X0					r24
#define	Y0					r23
#define	X1					r18
#define	Y1					r19
#define	XOffset				r9
#define	YOffset				r10
;
DACinit:
;
; Upper 4 bits can be safely ignored here.
; PB4, PB5 are used for push buttons; set as input ports
; PB6, PB7 are unconnected
;  
	in		r24,DACLODDR
	ori		r24,0b00001111
	out		DACLODDR,r24
;
; Upper 4 bits can be safely ignored here.
; PC4, PC5 are used for I2C access to the RTC but are set as input ports otherwise
; PC6, PC7 are unconnected/non-existant
;
	in		r24,DACHIDDR
	ori		r24,0b00001111
	out		DACHIDDR,r24

	in		r24,DACCTLDDR
	ori		r24,0b00111000
	out		DACCTLDDR,r24

	sbi		DACCTL,DACLDACBit
	sbi		DACSELECT		;select DACB (active high)
	sbi		DACZXDDR		;Z-Axis control
	sbi		DACCTL,DACWRBit	;hold
	BEAMOFF
	ret
;
; DACXYPos		X position in X0, Y position in Y0
;
; USED:	r22
;
DACXYPos:
;
; X first (DACA)
;
	mov		r22,X0			;preserve X0
#if INVERT
	neg		r22				;map X origin
#endif
	add		r22,XOffset		;Scan Table offset
	cbi		DACSELECT		;select DACA (active low)
	out		DACLODATA,r22
	swap	r22
	out		DACHIDATA,r22	;no need to preserve High nibble of DACHIDATA
	cbi		DACCTL,DACWRBit			;latch DACA data
	sbi		DACCTL,DACWRBit			;hold
;
; Y second (DACB)
;
	mov		r22,Y0			;preserve Y0 by using r22
#if INVERT
	neg		r22				;map Y origin
#endif
	add		r22,YOffset		;Scan Table offset
	sbi		DACSELECT		;select DACB (active high)
	out		DACLODATA,r22
	swap	r22
	out		DACHIDATA,r22	;no need to preserve High nibble of DACHIDATA
	cbi		DACCTL,DACWRBit			;latch DACB data
	sbi		DACCTL,DACWRBit			;hold
	cbi		DACCTL,DACLDACBit		;update DAC channels
	sbi		DACCTL,DACLDACBit
#if FORCEDELAY
	lds		r22,DelayValue
	dec		r22
1:
	brne	1b
#endif
	ret
;
; DACYXPos		Y position in Y0, X position in X0
;
; USED:	r22
;
DACYXPos:
;
; Y first (DACA)
;
	mov		r22,Y0			;save Y0
#if INVERT
	neg		r22				;map Y origin
#endif
	add		r22,XOffset		;Scan Table offset
	cbi		DACSELECT		;select DACA (active low)
	out		DACLODATA,r22
	swap	r22
	out		DACHIDATA,r22	;no need to preserve High nibble of DACHIDATA
	cbi		DACCTL,DACWRBit			;latch DACA data
	sbi		DACCTL,DACWRBit			;hold
;
; X second (DACB)
;
	mov		r22,X0			;preserve X0 by using r22
#if INVERT
	neg		r22				;map X origin
#endif
	add		r22,YOffset		;Scan Table offset
	sbi		DACSELECT		;select DACB (active high)
	out		DACLODATA,r22
	swap	r22
	out		DACHIDATA,r22	;no need to preserve High nibble of DACHIDATA
	cbi		DACCTL,DACWRBit			;latch DACB data
	sbi		DACCTL,DACWRBit			;hold
	cbi		DACCTL,DACLDACBit		;update DAC channels
	sbi		DACCTL,DACLDACBit
#if FORCEDELAY
	lds		r22,DelayValue
1:	dec		r22
	brne	1b
#endif
	ret

DoDialMarks:
	lds		r24,ShowDialDots
	and		r24,r24
	breq	2f
	ldi		r25,8
1:
	clr		XOffset
	clr		YOffset
	lds		X0,BeamDotX0
	lds		Y0,BeamDotY0
	lds		X1,BeamDotX1
	lds		Y1,BeamDotY1
	rcall	DrawVector
	lds		X0,BeamDotX2
	lds		Y0,BeamDotY2
	lds		X1,BeamDotX3
	lds		Y1,BeamDotY3
	rcall	DrawVector
	dec		r25
	brne	1b
2:
	ret

HelpScreenRefresh:
	ldi		r25,HELPSCANTABLEN
	ldiw	X,MScanTbl				;table pointer. MScanTbl reused
	rcall	TextTableScan
;
; Process any Strings, located in SRam
; X continues where it left off
;
	ldi		r25,HELPSCANEXTTABLEN		;count
	rcall	TextTableScanSRam
	ldsw	X,RefreshCnt
	adiw	XL,1
	stsw	RefreshCnt,X
	ret

CalibrationRefresh:
	ldiw	Z,CalibrationData
	clr		XOffset					;do NOT add Global Offset in Calibration Mode
	clr		YOffset
	rcall	DrawVectorTable
	ldsw	X,RefreshCnt
	adiw	XL,1
	stsw	RefreshCnt,X
	ret
;
; ClockRefresh: draw one image on the CRT
;
; r25 used as the ScanTbl index, pointing to next entry to process
; r24, r23, r20, r19 are coordinates
; Z: current ScanTbl entry pointer
;
; No registers saved by this function. This is the
; responsibility of the caller.
;
ClockRefresh:
	lds		r25,CurScanTblLen		;get current scan table length
	ldiw	X,ScanTbl				;compute pointer
ClockRefresh100:
	ld		ZL,X+					;get new coordinates pointer LO,HI
	ld		ZH,X+
//XOffset and YOffset are only used in Binary Clock Mode. Xoffset only actually.
	ld		XOffset,X+
	add		XOffset,GXOffset
	ld		YOffset,X+
	add		YOffset,GYOffset
	mov		r0,ZL					;NULL Ptr check
	or		r0,ZH
	breq	1f
	rcall	DrawVectorTable			;argument Z. preserve r25, X
1:
	dec		r25
	brne	ClockRefresh100
	rcall	DoDialMarks
;
; now process any Strings, located in Flash
;
	ldiw	X,ScanTblStrFlash			;compute pointer
	ldi		r25,SCANSTRTABLEN		;count
	rcall	TextTableScan
;
; Process any Strings, located in SRam
;
	ldiw	X,ScanTblStrRam			;compute pointer
	ldi		r25,SCANSSTRTABLEN		;count
	rcall	TextTableScanSRam
;
; Finished scanning the Display Table
;
RefreshExit:
	ldsw	X,RefreshCnt
	adiw	XL,1
	stsw	RefreshCnt,X
	BEAMOFF
	ret
;
; IN: r25, cnt   X, Table Ptr
;
TextTableScan:
	ld		ZL,X+					;get new coordinates pointer LO,HI
	ld		ZH,X+
	ld		XOffset,X+				;unneeded is Z == 0 but need to update X
	add		XOffset,GXOffset
	ld		YOffset,X+
	add		YOffset,GYOffset
	mov		r0,ZL					;skip NULL entry
	or		r0,ZH
	breq	TextTableScan200
	rcall	DrawTextStringfromFlash	;preserve r25,X
TextTableScan200:
	dec		r25
	brne	TextTableScan
	ret
;
; IN: r25, cnt   X, Table Ptr
;
TextTableScanSRam:
	ld		ZL,X+					;get new coordinates pointer LO,HI
	ld		ZH,X+
	ld		XOffset,X+				;unneeded is Z == 0 but need to update X
	add		XOffset,GXOffset
	ld		YOffset,X+
	add		YOffset,GYOffset
	mov		r0,ZL					;skip NULL entry
	or		r0,ZH
	breq	TextTableScanSRamL200
	rcall	DrawTextStringfromSRam	;preserve r25,X
TextTableScanSRamL200:
	dec		r25
	brne	TextTableScanSRam
	ret
;
; MenuRefresh: draw one image on the CRT
;
; r25 used as the ScanTbl index, pointing to next entry to process
;
; Registers r3, r4, r6 are used but permanently reserved (C compiler won't use them)
;
MenuRefresh:
	ldiw	X,MScanTbl				;table pointer
	ldi		r25,MENUSCANTABLEN-1	;count. Postpone last entry
	rcall	TextTableScan
;
; Finished scanning the Flash Text Pointers part of the Menu Table
; X points to next table entry: SRam Text Pointer
;
	ldi		r25,1
	rcall	TextTableScanSRam
	ldsw	X,RefreshCnt
	adiw	XL,1
	stsw	RefreshCnt,X
	ret
;
; TermRefresh: draw one image on the CRT
;
; r25 used as the ScanTbl index, pointing to next entry to process
; Z: current ScanTbl entry pointer
;
; No registers saved by this function. This is the
; responsibility of the caller.
;
TermRefresh:
//
// Process SRam based String Data.  Skips empty (all blanks) lines.
//
	ldiw	X,TermScanTblStr		;First Pointer
	lds		r25,nScreenLines		;count
	pushw	Y
	ldiw	Y,LinesEmpty			;Blank Line Flags array
TermRefreshL100:
	ld		ZL,X+					;get new coordinates pointer LO,HI
	ld		ZH,X+
	ld		XOffset,X+				;unneeded is Z == 0 but need to update X
	add		XOffset,GXOffset
	ld		YOffset,X+
	add		YOffset,GYOffset
#if DEBUG
	mov		r0,ZL					;skip NULL entry. Should NOT happen in TermRefresh
	or		r0,ZH
	brne	2f
1:
	rjmp	1b
2:
#endif
	ld		r0,Y+					;does Line contain all blanks?
	tst		r0						;if so (TRUE), skip char drawing.
	brne	3f
	rcall	DrawTextStringfromSRam	;preserve r25,X,Y
3:
	dec		r25
	brne	TermRefreshL100
	popw	Y
;
; display numeric buffer
;
	lds		r25,fDispNumBuf			;ok to display
	and		r25,r25
	breq	1f						;no
	ldsw	X,TermTextBuf			;Source Pointer
	ldi		r25,1					;count
	call	TextTableScanSRam
1:
;
; Process Vectors, if any
;
	lds		r24,VectorStartIdx		;start index
	clr		r25
	mov		XOffset,GXOffset
	mov		YOffset,GYOffset
	ldiw	X,VectorTbl				;start of table
	lsl		r24						;index * 4
	rol		r25
	lsl		r24
	rol		r25
	add		XL,r24					;compute table ptr
	adc		XH,r25
	lds		r25,VectorRngLen		;# of vectors to display
	and		r25,r25
	breq	TermRefreshL200			;none, done
1:
	ld		X0,X+					;load vector
	ld		Y0,X+
	ld		X1,X+
	ld		Y1,X+
	rcall	DrawVector				;draw a vector from X0,Y0 to X1,Y1.Preserve X, r25
	dec		r25						;more vectors to draw?
	brne	1b						;yes, continue
TermRefreshL200:
	ldsw	X,RefreshCnt
	adiw	XL,1
	stsw	RefreshCnt,X
	ret
;
; DrawVectorTable
;
; Z points to Flash based table of vector coordinates
;
; R24,R23 holds X0,Y0. R18,R19 holds X1,Y1
;
; start a new coordinates pair
;
; Need to preserve r25, X
;
; Registers r2..r8 are used but permanently reserved (C compiler won't use them)
;
DrawVectorTable:
	lpm		X0,Z+			;get first byte from code space
DrawVTL150:
	tst		X0				;but skip a possible extra 0 byte here
	breq	DrawVectorTable
	cpi		X0,ESCAPE		;special
	breq	DrawVTChar		
	lpm		Y0,Z+			;get next byte from code space
;
; process coordinate pairs
;
DrawVTLoop:
	lpm		X1,Z+			;get first byte from code space
	tst		X1				;test the byte we got
	brne	DrawVTL200		;0 means "move to the next coordinate"
	rjmp	DrawVectorTable
DrawVTL200:
	cpi		X1,0xff			;0xff means "end of coordinates" for this table 
	breq	DrawVTExit
;
; Normal DrawVector using X,Y Data
;
	lpm		Y1,Z+			;get second byte from code space
;
; Draw vector: r24:x0 r23:y0 r20:x1 r19:y1
;
	push	Y1				;save Y1,X1
	push	X1
	rcall	DrawVector		;draw a vector from X0,Y0 to X1,Y1
	pop		X0				;move X1,Y1 to X0,Y0
	pop		Y0
	rjmp	DrawVTLoop
;
; Support for characters in the vector tables.
; Used to display binary clock efficiently
;
; Format: ESCAPE VPOS CHAR
;
DrawVTChar:
	lpm		YOffset,Z+			;get VPOS from code space
	lpm		r24,Z+				;get char
	mov		r18,XOffset			;save in X1
	rcall	DrawChar			;preserves r25,r18,X,Z. Uses r24 (X0)
	mov		XOffset,r18			;restore
	lpm		X0,Z+				;get next byte from code space
	cpi		X0,0xff				;end of coordinates
	breq	DrawVTExit
	rjmp	DrawVTL150
;
; Finished scanning this table
;
DrawVTExit:
	ret
;
;	Vector Drawing Function: Bresenham's Algorithm
;
;	Arguments:
;	   r24 - X0-Start
;	   r23 - Y0-Start
;	   r18 - X1-End
;	   r19 - Y1-End
;
; Not used:
;
;	unprotected "Call Saved" registers r11..r17, Y
;	X, Z(saved)
;
; When done, X0,Y0 and X1,Y1 are identical
;
; registers r9/r10 are Xoffset/Yoffset. Don't change
; registers r7/r8 are GXOffset/GYOffset. Don't change
; Don't use r25 (used in ClockRefresh, VectorRefresh)
; Register r22 used by pixel set function (DACXYPos, DACYXPos)
;
; r20,r21 is used to save Z
; Z is used to hold the plot function to call
;
; register r0 used; work registers r2..r6 defined here:
;
#define	LineErrorL			r2
#define	LineErrorH			r3
#define	DeltaX				r4
#define	DeltaY				r5
#define	YStep				r6

#if 0
void (*pfPlot)(x,y);
function line(x0, y0, x1, y1)
     boolean steep := abs(y1 - y0) > abs(x1 - x0)
     if steep then
	 	pfPlot = DACYXPOS;
		swap(x0, y0)
		swap(x1, y1)
     if x0 > x1 then
	 	pfPlot = DACXYPos;
		swap(x0, x1)
		swap(y0, y1)
     int deltax := x1 - x0
     int deltay := abs(y1 - y0)
     int LineError := 0
     int YStep
     int y := y0
     if y0 < y1 then YStep := 1 else YStep := -1
     for x from x0 to x1
		pfPlot();
        LineError := LineError + deltay
        if 2xLineError >= deltax then
             y := y + YStep
             LineError := LineError - deltax
#endif
DrawVector:
	movw	r20,ZL
;     boolean steep := abs(y1 - y0) > abs(x1 - x0)
	ldiw	Z,pm(DACXYPos)		;preset word address
	mov		DeltaY,Y1
	sub		DeltaY,Y0
	brcc	Line100
	neg		DeltaY				;Take absolute value Y1-Y0 -> Dx
Line100:
	mov		DeltaX,X1
	sub		DeltaX,X0
	brcc	Line200
	neg		DeltaX				;Take absolute value X1-X0 ->  Dy
Line200:
	cp		DeltaX,DeltaY		;Inverted: if (Dx >= Dy) jmp
	brsh	Line300
	ldiw	Z,pm(DACYXPos)		; preset word address
;     if steep then
;         swap(x0, y0)
;         swap(x1, y1)
;XOR swap algorithm
	eor		X0,Y0				;swap X0,Y0
	eor		Y0,X0
	eor		X0,Y0
	eor		X1,Y1				;swap X1, Y1
	eor		Y1,X1
	eor		X1,Y1
Line300:
;     if x0 > x1 then
;         swap(x0, x1)
;         swap(y0, y1)
	cp		X1,X0				;inverted: if (x1 >= x0) jmp
	brsh	Line400
	eor		X0,X1				;swap X0,X1
	eor		X1,X0
	eor		X0,X1
	eor		Y0,Y1				;swap Y0,Y1
	eor		Y1,Y0
	eor		Y0,Y1
Line400:
	mov		DeltaX,X1			;int deltax := x1 - x0
	sub		DeltaX,X0
	mov		DeltaY,Y1			;int deltay := abs(y1 - y0)
	sub		DeltaY,Y0
	brcc	Line500
	neg		DeltaY				;Take absolute value
Line500:
	clr		LineErrorL			;int LineError := 0
	clr		LineErrorH
	clr		YStep				;int YStep
	cp		Y0,Y1				;if y0 < y1 then ystep := 1 else ystep := -1
	brsh	Line600
	inc		YStep
	rjmp	Line750
Line600:
	dec		YStep
;     int y := y0
;     for x from x0 to x1
Line750:
	cp		X1,X0
	brlo	Line1100
	BEAMON
Line775:
;		pfPlot();
	icall
;         LineError := LineError + deltay
	add		LineErrorL,DeltaY
	adc		LineErrorH,__zero_reg_
Line997:
;         if 2xLineError >= deltax then
	mov		r0,LineErrorL
	add		r0,LineErrorL
	cp		r0,DeltaX
	cpc		LineErrorH,__zero_reg_
Line998:
	brlt	 Line1000
;             y := y + ystep
	add		Y0,YStep
;             LineError := LineError - deltax
	sub		LineErrorL,DeltaX
	sbc		LineErrorH,__zero_reg_
Line1000:
	inc		X0
	breq	Line1100		;X0 overflows so we're done. Handles the X1==0xff case
	cp		X1,X0			;inverted test
	brsh	Line775
Line1100:
	BEAMOFF
	movw	ZL,r20
	ret
#undef	LineErrorL
#undef	LineErrorH
#undef	DeltaX
#undef	DeltaY
#undef	YStep
#undef	X0
#undef	Y0
#undef	X1
#undef	Y1
;
; Draw a String
;
; in Z: ptr to 0-terminated string
;
; Must preserve r25,X
;
DrawTextStringfromSRam:
	// Move the beam to the correct position first
	ldi		r22,1
	sts		FirstInString,r22
1:
	ld		r24,Z+				;get next Text Code from SRam
	tst		r24					;end of string
	breq	DrawTSL500			;done
	rcall	DrawChar
	rjmp	1b
DrawTSL500:
	ret

DrawTextStringfromFlash:
	// Move the beam to the correct position first
	ldi		r22,1
	sts		FirstInString,r22
1:
	lpm		r24,Z+				;get next Text Code from flash
	tst		r24					;end of string
	breq	DrawTSL700			;done
	rcall	DrawChar
	rjmp	1b
DrawTSL700:
	ret
//
// Changing the DAC to automatic mode deteriorates
// the image quality significantly. Code disabled.
//
#define DACAUTOMATIC 0
;
; DrawChar
;
; IN:	r24	-- char to draw. Valid range is 32..127 + HALFSPACE
;
; USES: r4,r5,r6,r19,r20,r21,r22,r23,r24
; CharPixel uses: r0, r2, r3. Expects r17,r19,r20,r21 preloads
;
; Preserves: r7, r8, r9, r10, r18, r25, X, Z
;
; Also called from DrawVTChar in VectorTable Drawing,
; which uses a lot of registers
;
DrawChar:
	movw	r4,ZL				;save
	sbrc	r24,7				;bit 1 high is out of range
	rjmp	DrawCL900			;includes HALFSPACE
	subi	r24,FIRST_CHAR+1	;rebase
	brcc	1f
	rjmp	DrawCL900			;char was out of range, includes SPACE
1:
	mov		r20,r24				;compute 8 * char index
	clr		r21
	lsl		r20
	rol		r21
	lsl		r20
	rol		r21
	lsl		r20
	rol		r21
	ldiw	Z,Newfonttbl
	add		ZL,r20
	adc		ZH,r21
	ldi		r22,CHAR_HEIGHT		;First Y pos, Also Row Counter
	push	r17
//
// Draw the first pixel with BEAMOFF iff FirstInString is TRUE
//
	lds		r6,FirstInString
	and		r6,r6
	breq	DrawCL175
//
// Combine bit actions on DACCTL port
// This requires caching the value of port D, but the LED bit
// may be changed in an ISR (LED Morse code). Solved by using
// fUpdateLed and doing the LED change in ClockWorks()
//
//	BEAMOFF
	in		r20,PD_IN
	andi	r20,~(_BV(DACZXBit))				//clear one bit:BEAMOFF
	mov		r19,r20
	andi	r19,~(_BV(DACLDACBit))				//clear one bit
	mov		r21,r20
	andi	r21,~(_BV(DACSELBit)|_BV(DACWRBit))	//clear two bits
	mov		r17,r20
	andi	r17,~(_BV(DACWRBit))				//clear one bit
	pushw	Z
DrawCL100:
	lpm		r6,Z+								//bits for current row
	clr		r23									//start x pos, 6 pixels per row
	sbrc	r6,7
	rcall	CharPixelTemp						//will return to DrawCL150
	ldi		r23,1
	sbrc	r6,6
	rcall	CharPixelTemp						//will return to DrawCL150
	ldi		r23,2
	sbrc	r6,5
	rcall	CharPixelTemp						//will return to DrawCL150
	ldi		r23,3
	sbrc	r6,4
	rcall	CharPixelTemp						//will return to DrawCL150
	ldi		r23,4
	sbrc	r6,3
	rcall	CharPixelTemp						//will return to DrawCL150
	ldi		r23,5
	sbrc	r6,2
	rcall	CharPixelTemp						//will return to DrawCL150
	dec		r22									//next Y pos, row counter
	brne	DrawCL100							//finish all rows
DrawCL150:
// reload Z and r22
	popw	Z
	ldi		r22,CHAR_HEIGHT						//First Y pos, Also Row Counter

	sts		FirstInString,__zero_reg_

DrawCL175:
//
// Combine bit actions on DACCTL port
// This requires caching the value of port D, but the LED bit
// may be changed in an ISR (LED Morse code). Solved by using
// fUpdateLed and doing the LED change in ClockWorks()
//
//	BEAMON
	in		r20,PD_IN
	ori		r20,_BV(DACZXBit)					//set one bit:BEAMON
	mov		r19,r20
	andi	r19,~(_BV(DACLDACBit))				//clear one bit
	mov		r21,r20
	andi	r21,~(_BV(DACSELBit)|_BV(DACWRBit))	//clear two bits
	mov		r17,r20
	andi	r17,~(_BV(DACWRBit))				//clear one bit
DrawCL200:
	lpm		r6,Z+				;bits for current row
	;
	; early out test for r6 == 0. Timing shows slight improvement.
	;
	tst		r6
	breq	1f
	clr		r23					;start x pos, 6 pixels per row
	sbrc	r6,7
	rcall	CharPixel
	ldi		r23,1
	sbrc	r6,6
	rcall	CharPixel
	ldi		r23,2
	sbrc	r6,5
	rcall	CharPixel
	ldi		r23,3
	sbrc	r6,4
	rcall	CharPixel
	ldi		r23,4
	sbrc	r6,3
	rcall	CharPixel
	ldi		r23,5
	sbrc	r6,2
	rcall	CharPixel
1:
	dec		r22					;next Y pos, row counter
	brne	DrawCL200			;finish all rows
	pop		r17
DrawCL900:
	BEAMOFF

	movw	ZL,r4			;restore
;
; compute GetCharWidth, r24 holds char
;
	cpi		r24,25		;':'-(FIRST_CHAR+1)
	brne	GetCWL100
	ldi		r24,COLONCHARWIDTH
	rjmp	GetCWL800
GetCWL100:
	cpi		r24,128		;HALFSPACE (not rebased)
	brne	GetCWL200
	ldi		r24,HALFSPACECHARWIDTH
	rjmp	GetCWL800
GetCWL200:
	cpi		r24,12		;'-'-(FIRST_CHAR+1)
	brne	GetCWL300
	ldi		r24,DASHCHARWIDTH
	rjmp	GetCWL800
GetCWL300:
	cpi		r24,11		;','-(FIRST_CHAR+1)
	brne	GetCWL400
	ldi		r24,COMMACHARWIDTH
	rjmp	GetCWL800
GetCWL400:
	ldi		r24,DEFCHARWIDTH
GetCWL800:

	add		XOffset,r24
	ret

CharPixelTemp:
	rcall	CharPixel
	pop		r23				;remove return address
	pop		r23
	rjmp	DrawCL150
;
; in r22,r23
; preloads: r17,r19,r20,r21
; uses r0,r2,r3,r23
; don't use r4,r5,r6,r24
;
CharPixel:
	mov		r0,r22
;
; X first (DACA)
;
	lsl		r23						;times 2
	add		r23,XOffset 			;Scan Table offset
	mov		r3,r23
	swap	r3
;
; Y second (DACB)
;
	lsl		r22						;times 2
	add		r22,YOffset 			;Scan Table offset
	mov		r2,r22
	swap	r2
//
// 52 cycles per fat pixel: 2.6 uSec. Could run with interrupts OFF since
// Timer1 may interrupt, causing flicker. Need to save interrupt status first.
// Reduced time spent in Timer1.
//
#if 0
	in		r1,_SFR_IO_ADDR(SREG)
	cli
#endif
	out		DACLODATA,r23			;no need to preserve High nibble of DACLODATA
	out		DACHIDATA,r3			;no need to preserve High nibble of DACHIDATA
	// select DACA (active low). latch DACA data
	out		DACCTL,r21				;001	DACSELBit low DACWRBit low
	// hold latch. select DACB (active high)
	out		DACCTL,r20				;111	DACSELBit high DACWRBit high
	out		DACLODATA,r22			;no need to preserve High nibble of DACLODATA
	out		DACHIDATA,r2			;no need to preserve High nibble of DACHIDATA
	//latch DACB data
	out		DACCTL,r17				;101	DACWRBit low
	//hold
	out		DACCTL,r20				;111
	//update DAC channels
	out		DACCTL,r19				;110	DACLDACBit low
	//hold
	out		DACCTL,r20				;111
	inc		r23
	mov		r3,r23
	swap	r3
	out		DACLODATA,r23			;no need to preserve High nibble of DACLODATA
	out		DACHIDATA,r3			;no need to preserve High nibble of DACHIDATA
	// select DACA (active low). latch DACA data
	out		DACCTL,r21				;001	DACSELBit low DACWRBit low
	// hold latch. select DACB (active high)
	out		DACCTL,r20				;111
	out		DACLODATA,r22			;no need to preserve High nibble of DACLODATA
	out		DACHIDATA,r2			;no need to preserve High nibble of DACHIDATA
	//latch DACB data
	out		DACCTL,r17				;101	DACWRBit low
	//hold
	out		DACCTL,r20				;111
	//update DAC channels
	out		DACCTL,r19				;110	DACLDACBit low
	//hold
	out		DACCTL,r20				;111
	dec		r23
	mov		r3,r23
	swap	r3
	inc		r22
	mov		r2,r22
	swap	r2
	out		DACLODATA,r23			;no need to preserve High nibble of DACLODATA
	out		DACHIDATA,r3			;no need to preserve High nibble of DACHIDATA
	// select DACA (active low). latch DACA data
	out		DACCTL,r21				;001	DACSELBit low DACWRBit low
	// hold latch. select DACB (active high)
	out		DACCTL,r20				;111
	out		DACLODATA,r22			;no need to preserve High nibble of DACLODATA
	out		DACHIDATA,r2			;no need to preserve High nibble of DACHIDATA
	//latch DACB data
	out		DACCTL,r17				;101	DACWRBit low
	//hold
	out		DACCTL,r20				;111
	//update DAC channels
	out		DACCTL,r19				;110	DACLDACBit low
	//hold
	out		DACCTL,r20				;111
	inc		r23
	mov		r3,r23
	swap	r3
	out		DACLODATA,r23			;no need to preserve High nibble of DACLODATA
	out		DACHIDATA,r3			;no need to preserve High nibble of DACHIDATA
	// select DACA (active low). latch DACA data
	out		DACCTL,r21				;001	DACSELBit low DACWRBit low
	// hold latch. select DACB (active high)
	out		DACCTL,r20				;111
	out		DACLODATA,r22			;no need to preserve High nibble of DACLODATA
	out		DACHIDATA,r2			;no need to preserve High nibble of DACHIDATA
	//latch DACB data
	out		DACCTL,r17				;101	DACWRBit low
	//hold
	out		DACCTL,r20				;111
	//update DAC channels
	out		DACCTL,r19				;110	DACLDACBit low
	//hold
	out		DACCTL,r20				;111
	mov		r22,r0
#if 0
	out		_SFR_IO_ADDR(SREG),r1
	clr		__zero_reg_
#endif
	ret
#undef	XOffset
#undef	YOffset
;
; Very fast division by 10
;
;source: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=37150
; Dirty Math topic
;
; Division by 10, using reciprocal multiplication 
; 
; Call with: 
;   8 bit dividend in r24 
; 
; Returns: 
;   Result in r24 
;   Remainder in r18
;
; uses r0, r1, r19
;
; Uses r1 but resets it to 0 on exit.  This causes a problem when div08uby10
; is used in an interrupt function, since the code being interrupted may
; be using r1 for something else than 0 (e.g. doing a multiply).
; Therefore, if div08uby10 is used in an interrupt function, save r1
; before calling div08uby10, restore afterwards.
;
div08uby10:
	mov   r18,r24			; save original dividend 
	ldi   r19,26			; reciprocal, scaled 56, off a bit 
	mul   r24,r19 
;
; result of multiplication now in r1:r0 
; use only r1, thereby effectively divide by 256 
;
	dec   r1 	    		; if imprecise scaling value influences result, 
							; result will be '+1'. Decrement to avoid neg. 
							; value in later subtraction 
	ldi   r19,10			; re-use r19 for divisor 
	mov   r24,r1			; save result/256 
	mul   r24,r19			; find remainder by multiplication of result by 10d 
;
; result again in r1:r0 
; get remainder and correct result of 'by 10' 
;
	clr		__zero_reg_		;reload after mul. Only useful if not used in ISR.
	sub		r18,r0 
	cpi		r18,10 
	brlo	div08u10done 
	subi	r18,10 
	inc		r24 
div08u10done: 				; result in r24, remainder in r18 
	ret
;
; split an 8-bit binary value into 2 decimal digits (bcd)
;
; argument: r24 binary value
;
; results:	r24 low nibble:  %10 decimal digit
;			r24 high nibble: /10 decimal digit
;
; uses:		r18,r19
;
bintobcd:	//argument in r24. Result in r24 (high & low nibble)
;
; ENHANCEMENT to V3.3 and up: Much faster division by 10
;
	rcall	div08uby10		;divide r24 by 10. Result in r24, Remainder in r18
	cpi		r24,10			;force range limitation (max digit is 9)
	brlt	1f
	ldi		r24,1
1:
	swap	r24				;set high nibble
	or		r24,r18			;merge in low nibble. Result
	ret
;
; Function Generator
;
#if SRAMTABLES
// computed as 1000 / .099341
#define OneKHZ 	10066
#else
// computed as 1000 / .0916994
#define OneKHZ 	10905
#endif

	.global	RunFuncGen

RunFuncGen:
	call	StartMiniDDS		
;
; set sinewave output as default
;
	ldi		r24,'1'			;Sine table
	call	SetWaveTable
	movw	ZL,r24			; setup Z pointer. Always at 0x100 boundary
#if DEBUG
	tst		ZL
1:	brne	1b
#endif

; clear accumulator 

	clr		ACC1				// clear accumulator 
	clr 	ACC0				// clear accumulator 
;
; setup adder registers.
;	
	ldi 	r24,lo8(OneKHZ)		//setup adder value	LSB
	mov		ADDER0,r24
	ldi 	r24,hi8(OneKHZ)		// to 1 kHz			Middle Byte
	mov		ADDER1,r24
	clr 	ADDER2				// 					MSB
;
; initialize DAC to always use DACA
;
	cbi		DACSELECT			;select DACA (active low)
	cbi		DACCTL,DACLDACBit	;update DAC channels continuously
	BEAMON

; wavegen loop
;
;	ACC0,ACC1,r30/r31 (Z) is the phase accumulator
;	r31 (ZH) never changes, except when changing waveform table
;  	ADDER0,ADDER1,ADDER2 is the adder value determining frequency
;	r16,r17 DACCTRL presets
;
; 	add value to accumulator
;	load byte from current table in ROM
;	output byte to DAC
;	repeat 
;
;	NO RETURN
;
	in		r17,DACCTL		;value of DAC Control Port
	mov		r16,r17
	cbr		r16,(1<<4)		;DACWRBit low
wavegen:
	add		ACC0,ADDER0		;1
	adc		ACC1,ADDER1		;1
	adc		ZL,ADDER2		;1
#if SRAMTABLES
	ld		r0,Z			;2
#else
	lpm						;3
#endif
	out		DACLODATA,r0	;1 upper 4 pins are set as inputs
	swap	r0				;1
	out		DACHIDATA,r0	;1 upper 4 pins are set as inputs
	out		DACCTL,r16		;l  latch data: Low High . necessary for AD7302
	out		DACCTL,r17		;1
	rjmp	wavegen			;2 => 12 (SRAMTABLES) or 13 cycles

//
// ulong GetAdder(void)
//
	.global	GetAdder
GetAdder:
	mov		r22,ADDER0
	mov		r23,ADDER1
	mov		r24,ADDER2
	clr		r25
	ret
//
// SetAdder(ulong)
//
	.global	SetAdder
SetAdder:
	mov		ADDER0,r22
	mov		ADDER1,r23
	mov		ADDER2,r24
	ret

;
; Semantics reinterpretation: change FREQUENCY, not ACCUMULATOR
;
	.global	up_one

#if SRAMTABLES
;
; Resolution is 0.099341 => multiplier is 10.066
;
; add 1 to the frequency => add 10.066 to the accumulator
; rounded to 10.
up_one:
		ldi		r24,10
		add		ADDER0,r24
		adc		ADDER1,__zero_reg_
		adc		ADDER2,__zero_reg_
		ret
#else
; Resolution is 0.0916994 => multiplier is 10.905
;
; add 1 to the frequency => add 10.9 to the accumulator
; rounded to 11
up_one:
		ldi		r24,11
		add		ADDER0,r24
		adc		ADDER1,__zero_reg_
		adc		ADDER2,__zero_reg_
;
		ret
#endif
;
#if SRAMTABLES
;
; Resolution is 0.099341 => multiplier is 10.0663
;
; add 10 to the frequency => add 100.66 to the accumulator
; missing: 0.663
up_ten:
		ldi		r24,100
		add		ADDER0,r24
		adc		ADDER1,__zero_reg_
		adc		ADDER2,__zero_reg_
		ret
#else
; add 10 to the frequency => add 109 to the accumulator
; missing: 0.05
;
up_ten:
		ldi		r24,109
		add		ADDER0,r24
		adc		ADDER1,__zero_reg_
		adc		ADDER2,__zero_reg_
		ret
#endif
;
	.global	up_hundred

#if SRAMTABLES
; add 100 to the frequency => add 1006.63 to the accumulator
;		call up_ten 10 times (1000), add an extra 7
; too much added: 0.37
#else
; add 100 to the frequency => add 1090.5 to the accumulator
;		call up_ten 10 times (1090)
; still missing: 0.5
#endif
;
up_hundred:
		ldi		r25, 10
up_hundred_1:
		rcall	up_ten
		dec		r25
		brne	up_hundred_1
#if SRAMTABLES
		ldi		r24,7
		add		ADDER0,r24
		adc		ADDER1,__zero_reg_
		adc		ADDER2,__zero_reg_
#endif
		ret
;
	.global	up_tenthousand

#if SRAMTABLES
; add 10000 to the frequency => add 100663 to the accumulator
;	call up_hundred 100 times (100700) subtract an extra 37
;   experimentation shows that this is too much. Subtract 25.
#else
; add 10000 to the frequency => add 109050 to the accumulator
;	call up_hundred 100 times (109000) add an extra 50
#endif
;
up_tenthousand:
		ldi		r25,100
up_h10:
		push	r25
		rcall	up_hundred
		pop		r25
		dec		r25
		brne	up_h10
#if SRAMTABLES
;
; now make a correction
;
		mov		r24,ADDER0
		subi	r24,25
		mov		ADDER0,r24
		sbc		ADDER1,__zero_reg_
		sbc		ADDER2,__zero_reg_
#else
;
; now add another 50 for the fraction
;
		ldi		r24,50
		add		ADDER0,r24
		adc		ADDER1,__zero_reg_
		adc		ADDER2,__zero_reg_
		ret
#endif
;
	.global	down_one

#if SRAMTABLES
;
; Resolution is 0.099341 => multiplier is 10.066
;
; subtract 1 from the frequency => sub 10.0 to the accumulator
down_one:
		mov		r24,ADDER0
		subi	r24,10
		mov		ADDER0,r24
		sbc		ADDER1,__zero_reg_
		sbc		ADDER2,__zero_reg_
		ret
#else
;
; subtract 1 from the frequency => sub 10.905 to the accumulator
;
down_one:
		mov		r24,ADDER0
		subi	r24,11
		mov		ADDER0,r24
		sbc		ADDER1,__zero_reg_
		sbc		ADDER2,__zero_reg_
		ret
#endif
;
#if SRAMTABLES
;
; Resolution is 0.099341 => multiplier is 10.066
;
; subtract 10 from the frequency => sub 100.66 to the accumulator
down_ten:
		mov		r24,ADDER0
		subi	r24,100
		mov		ADDER0,r24
		sbc		ADDER1,__zero_reg_
		sbc		ADDER2,__zero_reg_
		ret
#else
; subtract 10 from the frequency => sub 109.05 to the accumulator
;
down_ten:
		subi	ADDER0,109
		sbc		ADDER1,__zero_reg_
		sbc		ADDER2,__zero_reg_
		ret
#endif

	.global	down_hundred

;
; subtract 100 from the frequency => call down_ten 10 times
;
down_hundred:
		ldi		r25, 10
down_hundred_1:
		rcall	down_ten
		dec		r25
		brne	down_hundred_1
#if SRAMTABLES
		mov		r24,ADDER0
		subi	r24,7
		mov		ADDER0,r24
		sbc		ADDER1,__zero_reg_
		sbc		ADDER2,__zero_reg_
#endif
		ret

	.global	down_tenthousand

#if SRAMTABLES
; subtract 10,000 from the frequency => subtract 100663 from the accumulator
;	call down_hundred 100 times (100700) add an extra 37
#else
; subtract 10000 from the frequency
#endif
down_tenthousand:
		ldi		r25,100
down_h10:
		push	r25
		rcall	down_hundred
		pop		r25
		dec		r25
		brne	down_h10
#if SRAMTABLES
;
; now make a correction
;
		ldi		r24,37
		add		ADDER0,r24
		adc		ADDER1,__zero_reg_
		adc		ADDER2,__zero_reg_
#else
;
; now subtract another 50 for the fraction
;
		subi	ADDER0,50
		sbc		ADDER1,__zero_reg_
		sbc		ADDER2,__zero_reg_
#endif
		ret

	.balign 2
