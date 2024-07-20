
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.bss
.head_ptrs:
  .space 256   @ enough for 32 linked list head nodes (32nodes * 8 (4bytes for data, 4 bytes for next ptr))


.data


.text
_start:




@ brief: Adds new node at the start of the linked list
@ this function puts new data in the head node in .bss section
@ and allocates a new node in the heap with malloc and sets old data in new node 
@ example before calling function do: LDR r0, =head_ptrs then offset it by n bytes
@ n being  multiple of 8
@  r0: Address of head ptr 
@  r1: Value to add to linked list
.global _ll_add_node
.type _ll_add_node, %function

.macro _ll_add_node   malloc_function:req
  PUSH    {r0-r3, lr}          @ Save registers and return address

  LDR     r2, [r0]             @ Load the value of data field from head_ptr
  LDR     r3, [r0, #4]         @ Load the value of next_ptr field from head_ptr

  CMP     r2, #0x0             @ Check if data field is null
  IT      EQ                   @ If-Then condition for EQ (Equal)
  STREQ   r1, [r0]             @ If data field is null, store the new value in head_ptr
  BEQ     1f                   @ If data was null, exit the function

  PUSH    {r0}                 @ Save the address of head ptr
  MOVS    r0, #8               @ Move size 8 to r0 (malloc argument)
  BL      \malloc_function              @ Call malloc to allocate a new node
  CMP     r0, #0               @ Check if malloc returned NULL
  BEQ     1f                   @ If malloc failed, exit the function
  POP     {r4}                 @ Recover the address of head ptr, set it in r4

  STR     r2, [r0]             @ Store old data in the new node's data field
  STR     r3, [r0, #4]         @ Store old next_ptr in the new node's next_ptr field

  STR     r1, [r4]             @ Store new data in head_ptr's data field
  STR     r0, [r4, #4]         @ Store address of new node in head_ptr's next_ptr field
  @ exit
  1:
    POP     {r0-r3, pc}        @ Restore registers and return
.endm






@ brief: Removes node from the linked list that has the value in r1
@ This function iterates over the linked list 
@   - if head ptr is null it exits 
@   - if head ptr is the one to remove and no next node we nullify it
@   - if head ptr is the one to remove and theres a next node we copy next in head and free next from heap
@   - else we iterate and when found we do the same thing copy next in node to remove and free it 
@   - if none found we exit
@ example before calling function do: LDR r0, =head_ptrs then offset it by n bytes n being  multiple of 8
@   r0: Address of head ptr 
@   r1: Value to remove from linked list
.macro _ll_rem_node free_function:req
  PUSH    {r0-r3, lr}           @ Save registers and return address
  
  LDR     r2, [r0]              @ Load the value of data field from head_ptr
  CBZ     r2, 5f                @ Exit if data field of head pointer is null

  CMP     r2, r1                @ Check if node to remove is the head node
  BEQ     2f                    @ If head node is the node to remove

  @ loop
  1:
    MOV     r3, r0                @ Save the address of the current node in r3
    LDR     r0, [r0, #4]          @ Load the value of next_ptr field
    LDR     r2, [r0]              @ Load the data field of the next node
    CMP     r2, r1                @ Check if data field = data to remove
    BEQ     2f                    @ Branch if data matches

    LDR     r2, [r0, #4]          @ Load the value of next_ptr field of the next node
    CBZ     r2, 5f                @ Exit if no more nodes

    B       1f                    @ Continue loop

  @ remove_head
  2:
    LDR     r2, [r0, #4]          @ Load the value of next_ptr field
    CBZ     r2, 4f                @ If there's no next node, nullify head

    @ Free the next node and set head node to the next node
    LDR     r3, [r2]              @ Load the value of data field of next node
    STR     r3, [r0]              @ Store the data field of next node in head node
    LDR     r3, [r2, #4]          @ Load the value of next_ptr field of next node
    STR     r3, [r0, #4]          @ Store the next_ptr field of next node in head node
    MOV     r0, r2                @ Load the address of next node in r0
    BL      \free_function                 @ Free the old head node
    B       5f

  @ remove_node
  3:
    LDR     r1, [r0, #4]          @ Load the value of next_ptr field of node to remove
    STR     r1, [r3, #4]          @ Link previous node to the next node
    MOV     r0, r3                @ Load the address of node to remove in r0
    BL      \free_function        @ Free the node
    B       5f

  @ nullify_head
  4:
    MOVS    r2, #0                @ Set head node data to 0
    STR     r2, [r0]
    STR     r2, [r0, #4]          @ Set head node next_ptr to 0

  @ exit
  5:
    POP     {r0-r3, pc}           @ Restore registers and return
.endm
