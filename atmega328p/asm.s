.global asm_func


.extern ch1 ch2 ch3

asm_func:

	mov r24, SREG ;

    ldi r24, 0x10	;
	sts ch1, r24	;put 0x10 into ch1

	lds r26, ch2	;put ch2 into ch3
	sts ch3, r26	;
    ret


