
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb
.global _start
_start:
	MOVW	r0, #0x60
	MOVW	r1, #0xFF
	MOVW	r2, #0x20
	BL		memset_4
_exit:
	B .
	
	
.global memset_4
.type memset_4, %function
memset_4:
  @ Make a word ready containing 4 bytes of the required byte value
  MOV     r3, r1              @ Move the byte value into r3
  ORR     r3, r3, r3, LSL #8  @ Set byte 2
  ORR     r3, r3, r3, LSL #16 @ Set byte 3 and byte 4

.loop:
  STR     r3, [r0], #4      @ Store word to dest and increment dest by 4
  SUBS    r2, r2, #4        @ Decrement length counter by 4
  BNE     .loop             @ If length is not 0, continue loop

.exit:
  BX      lr                @ Return from function
  .size memset_4, .-memset_4
