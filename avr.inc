#if defined(__GNUC__)
;
; WinAVR
;
#define	XL	r26
#define	XH	r27
#define	YL	r28
#define	YH	r29
#define	ZL	r30
#define	ZH	r31

.macro ldiw arg1 arg2
	.if \arg1 == X
	ldi	XL,lo8(\arg2)
	ldi	XH,hi8(\arg2)
	.elseif \arg1 == Y
	ldi	YL,lo8(\arg2)
	ldi	YH,hi8(\arg2)
	.elseif \arg1 == Z
	ldi	ZL,lo8(\arg2)
	ldi	ZH,hi8(\arg2)
	.elseif \arg1 == A
	ldi	r16,lo8(\arg2)
	ldi	r17,hi8(\arg2)
	.elseif \arg1 == r24
	ldi	r24,lo8(\arg2)
	ldi	r25,hi8(\arg2)
	.elseif \arg1 == r22
	ldi	r22,lo8(\arg2)
	ldi	r23,hi8(\arg2)
	.elseif \arg1 == r18
	ldi	r18,lo8(\arg2)
	ldi	r19,hi8(\arg2)
	.else
		error
	.endif
.endm

.macro	ldsw arg1 arg2
	.if \arg1 == X
	lds	XL,\arg2
	lds	XH,\arg2\()+1
	.elseif \arg1 == Y
	lds	YL,\arg2
	lds	YH,\arg2\()+1
	.elseif \arg1 == Z
	lds	ZL,\arg2
	lds	ZH,\arg2\()+1
	.elseif \arg1 == A
	lds	r16,\arg2
	lds	r17,\arg2\()+1
	.elseif \arg1 == r24
	lds	r24,\arg2
	lds	r25,\arg2\()+1
	.else
		error
	.endif
.endm

.macro	lddw arg1 arg2
	.if \arg1 == X
	ldd	XL,\arg2
	ldd	XH,\arg2\()+1
	.elseif \arg1 == Y
	ldd	YL,\arg2
	ldd	YH,\arg2\()+1
	.elseif \arg1 == Z
	ldd	ZL,\arg2
	ldd	ZH,\arg2\()+1
	.elseif \arg1 == A
	ldd	r16,\arg2
	ldd	r17,\arg2\()+1
	.else
		error
	.endif
.endm


.macro	stsw arg1 arg2
	.if \arg2 == X
	sts	\arg1,XL
	sts	\arg1\()+1,XH
	.elseif \arg2 == Y
	sts	\arg1,YL
	sts	\arg1\()+1,YH
	.elseif \arg2 == Z
	sts	\arg1,ZL
	sts	\arg1\()+1,ZH
	.elseif \arg2 == A
	sts	\arg1,r16
	sts	\arg1\()+1,r17
	.elseif \arg2 == r24
	sts	\arg1,r24
	sts	\arg1\()+1,r25
	.else
		error
	.endif
.endm

.macro	stdw arg1 arg2
	.if \arg2 == X
	std	\arg1,XL
	std	\arg1\()+1,XH
	.elseif \arg2 == Y
	std	\arg1,YL
	std	\arg1\()+1,YH
	.elseif \arg2 == Z
	std	\arg1,ZL
	std	\arg1\()+1,ZH
	.elseif \arg2 == A
	std	\arg1,r16
	std	\arg1\()+1,r17
	.else
		error
	.endif
.endm

.macro	pushw arg1
	.if \arg1 == X
	push	XH
	push	XL
	.elseif \arg1 == Y
	push	YH
	push	YL
	.elseif \arg1 == Z
	push	ZH
	push	ZL
	.elseif \arg1 == A
	push	r17
	push	r16
	.else
		error
	.endif
.endm

.macro	popw arg1
	.if \arg1 == X
	pop		XL
	pop		XH
	.elseif \arg1 == Y
	pop		YL
	pop		YH
	.elseif \arg1 == Z
	pop		ZL
	pop		ZH
	.elseif \arg1 == A
	pop		r16
	pop		r17
	.else
		error
	.endif
.endm

; Store immediate into indirect memory via r24
;
;	sti	Z,imm

.macro	sti arg1 arg2
	ldi	r24,\arg2
	st	\arg1,r24
.endm

;------------------------------------------------;
; add/sub/subc/cp/cpc/lsl/lsr/rol/ror to register pair
;

.macro	addiw arg1 arg2
	.if \arg1 == X
	subi	XL,lo8(-(\arg2))
	sbci	XH,hi8(-(\arg2))
	.elseif \arg1 == Y
	subi	YL,lo8(-(\arg2))
	sbci	YH,hi8(-(\arg2))
	.elseif \arg1 == Z
	subi	ZL,lo8(-(\arg2))
	sbci	ZH,hi8(-(\arg2))
	.elseif \arg1 == A
	subi	r16,lo8(-(\arg2))
	sbci	r17,hi8(-(\arg2))
	.else
		error
	.endif
.endm

.macro	clrw arg1
	.if \arg1 == X
	clr	XL
	clr	XH
	.elseif \arg1 == Y
	clr	YL
	clr	YH
	.elseif \arg1 == Z
	clr	ZL
	clr	ZH
	.elseif \arg1 == A
	clr	r16
	clr	r17
	.else
		error
	.endif
.endm

;------------------------------------------------;
; Add immediate to register

.macro	addi arg1 arg2
	subi	\arg1,-(\arg2)
.endm

;------------------------------------------------;
; Long branch


.macro	rjne arg1
	breq	1f
	rjmp	\arg1
1:
.endm

.macro	rjeq arg1
	brne	1f
	rjmp	\arg1
1:
.endm

.macro	rjcc
	brcs	PC+2
	rjmp	@0
.endm

.macro	rjcs
	brcc	PC+2
	rjmp	@0
.endm

.macro	rjtc
	brts	PC+2
	rjmp	@0
.endm

.macro	rjts
	brtc	PC+2
	rjmp	@0
.endm


.macro	retcc
	brcs	PC+2
	ret
.endm

.macro	retcs
	brcc	PC+2
	ret
.endm

.macro	reteq
	brne	PC+2
	ret
.endm

.macro	retne
	breq	PC+2
	ret
.endm
#endif