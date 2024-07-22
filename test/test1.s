
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
  ORR     r3, r3, #(1 << 6)          @ Set byte 3 and byte 4
  BIC     r3, r3, #(2 << 6)
  BIC     r1, r1, #(0xFF << 24)      @ Clear current priority bits
  TST     r3, #4
.loop:
  STR     r3, [r0], #4      @ Store word to dest and increment dest by 4
  SUBS    r2, r2, #4        @ Decrement length counter by 4
  BNE     .loop             @ If length is not 0, continue loop

.exit:
  BX      lr                @ Return from function
  .size memset_4, .-memset_4

NMI_set_pending:
CPSID   I                @ Disable interrupts
  LDR     r0, =ICSR        @ Load the address of ICSR
  LDR     r1, [r0]         @ Load the current value of ICSR
  ORR     r1, r1, #0x80000000  @ Set the NMI pending bit (bit 31)
  STR     r1, [r0]         @ Write back the modified value to ICSR
  CPSIE   I                @ Enable interrupts
  BX      lr               @ Return from the subroutine


@ ISR_str_actv_num:
@   MRS     r0, IPSR                  @ Read IPSR into r0 (contains active exception number)
@   MOV     r1, r0                    @ Move exception number to r1 (to ensure 8-bit value)
@   LDRB    r0, last_IRQ
@   STRB    r1, r0                    @ Store active exception number in global byte variable
@   BX      lr                        @ Return from the subroutine

.section .bss, "aw", %nobits

last_IRQ: 
  .byte      @ will hold the last IRQ number and written only by ISR_get_active_num
