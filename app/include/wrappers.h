/**
  * # SystemZ Kernel <PRODUCTION BRANCH>
  * 
  * Copyright (C) 2024 Connexion Nord, Inc. or its affiliates. All Rights Reserved.
  * 
  * SPDX-License-Identifier: MIT
  * 
  * Permission is hereby granted, free of charge, to any person obtaining a copy of
  * this software and associated documentation files (the "Software"), to deal in
  * the Software without restriction, including without limitation the rights to
  * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
  * the Software, and to permit persons to whom the Software is furnished to do so,
  * subject to the following conditions:
  * 
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
  * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
  * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  * 
  * <https://github.com/anomaly1095/systemZ>
  * Author: Youssef Azaiez
*/


#ifndef WRAPPERS_H 1
#include <STM32F401.h>

//----------------------------------------
//----------------------------------------Wrapper for various syscalls that can be used by app developers all of chich use SVC in /src/syscalls.c
//----------------------------------------

/**
 * @brief Enable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
extern uint8_t NVIC_enable_irq(uint8_t IRQ_NUM);

/**
 * @brief Disable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
extern uint8_t NVIC_disable_irq(uint8_t IRQ_NUM);

/**
 * @brief Set an interrupt as pending in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
extern uint8_t NVIC_set_pend_irq(uint8_t IRQ_NUM);

/**
 * @brief Clears an interrupt from pending list in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
extern uint8_t NVIC_clear_pend_irq(uint8_t IRQ_NUM);

/**
 * @brief Check if an interrupt is active in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 1 if active, 0 if idle
 */
extern uint8_t NVIC_check_active_irq(uint8_t IRQ_NUM);

/**
 * @brief Set the priority of an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @param PRIO The priority to assign to the interrupt.
 * @return Returns 0 upon successful execution of the syscall.
 */
extern uint8_t NVIC_set_prio_irq(uint8_t IRQ_NUM, uint8_t PRIO);

/**
 * @brief Get the priority of an interrupt NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns the priority of the interrupt
 */
extern uint8_t NVIC_get_prio_irq(uint8_t IRQ_NUM);

/**
 * @brief Triggers an interrupt of the IRQ specified in IRQ_NUM thru software
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns the priority of the interrupt
 */   
extern uint8_t NVIC_soft_trigger_irq(uint8_t IRQ_NUM);

/**
 * @brief Expand the APP process heap towards the top
 * @param increment Ammount of memory to allocate
 * @return Returns the new pointer address
 * @return returns 0 (NULL) if failed to allocate SRAM
 */   
extern void *sbrk(size_t increment);


/**
 * @brief Expand the APP process heap towards the top
 * @param decrement Ammount of memory to deallocate
 * @return Returns the new pointer address
 * @return returns 0 (NULL) if failed to free SRAM
 */   
extern void *sbrk_free(size_t decrement);


/* @-------System control--------@ */

/**
 * @brief 
 * @return 
 */   
void enable_outoforder_exec();

/**
 * @brief 
 * @return 
 */   
void disable_outoforder_exec();

/**
 * @brief 
 * @return 
 */   
void get_CPUID();

/**
 * @brief 
 * @return 
 */   
void NMI_set_pend();

/**
 * @brief 
 * @return 
 */   
void PendSV_set_pend();

/**
 * @brief 
 * @return 
 */   
void PendSV_clear_pend();

/**
 * @brief 
 * @return 
 */   
void SYSTICK_set_pend();

/**
 * @brief 
 * @return 
 */   
void SYSTICK_clear_pend();

/**
 * @brief 
 * @return 
 */   
void SYSTICK_check_pend();

/**
 * @brief 
 * @return 
 */   
void ISR_check_pend();

/**
 * @brief 
 * @return 
 */   
void prio_set_split16_0();

/**
 * @brief 
 * @return 
 */   
void prio_set_split8_2();

/**
 * @brief 
 * @return 
 */   
void prio_set_split4_4();

/**
 * @brief 
 * @return 
 */   
void prio_set_split2_8();

/**
 * @brief 
 * @return 
 */   
void prio_set_split0_16();

/**
 * @brief 
 * @return 
 */   
void RESET_request();

/**
 * @brief 
 * @return 
 */   
void SEV_on_pend();

/**
 * @brief 
 * @return 
 */   
void sleep_is_sleep_deep();

/**
 * @brief 
 * @return 
 */   
void sleep_on_exit();

/**
 * @brief 
 * @return 
 */   
void Stack_align4bytes();

/**
 * @brief 
 * @return 
 */   
void Stack_align8bytes();

/**
 * @brief 
 * @return 
 */   
void NMI_HardF_dis_fault_handling();

/**
 * @brief 
 * @return 
 */   
void NMI_HardF_en_fault_handling();

/**
 * @brief 
 * @return 
 */   
void DIV0_notrap();

/**
 * @brief 
 * @return 
 */   
void DIV0_trap();

/**
 * @brief 
 * @return 
 */   
void unalign_NTrap();

/**
 * @brief 
 * @return 
 */   
void unalign_Trap();

/**
 * @brief 
 * @return 
 */   
void APP_access_STIR();

/**
 * @brief 
 * @return 
 */   
void exit_nested_IRQs_on_ret();

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_UsageFault_prio(uint8_t prio);

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_MemMan_fault_prio(uint8_t prio);

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_SVC_prio(uint8_t prio);

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_SYSTICK_prio(uint8_t prio);

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_PendSV_prio(uint8_t prio);

/**
 * @brief 
 * @return 
 */   
void en_UsageFault();

/**
 * @brief 
 * @return 
 */   
void en_BusFault();

/**
 * @brief 
 * @return 
 */   
void en_MemMan_fault();

/**
 * @brief 
 * @return 
 */   
void dis_UsageFault();

/**
 * @brief 
 * @return 
 */   
void dis_BusFault();

/**
 * @brief 
 * @return 
 */   
void dis_MemMan_fault();

/**
 * @brief 
 * @return 
 */   
uint8_t is_SVC_pend();


/**
 * @brief 
 * @return 
 */   
uint8_t is_BusFault_pend();


/**
 * @brief 
 * @return 
 */   
uint8_t is_MemMan_fault_pend();


/**
 * @brief 
 * @return 
 */   
uint8_t is_UsageFault_pend();


/**
 * @brief 
 * @return 
 */   
uint8_t is_SYSTICK_actv();


/**
 * @brief 
 * @return 
 */   
uint8_t is_PendSV_actv();


/**
 * @brief 
 * @return 
 */   
uint8_t is_DBGMon_actv();

/**
 * @brief 
 * @return 
 */   
uint8_t is_SVC_actv();


/**
 * @brief 
 * @return 
 */   
uint8_t is_UsageFault_actv();


/**
 * @brief 
 * @return 
 */   
uint8_t is_BusFault_active();


/**
 * @brief 
 * @return 
 */   
uint8_t is_MemMan_fault_active();


/**
 * @brief 
 * @return 
 */   
uint8_t div_by0_UsageFault();


/**
 * @brief 
 * @return 
 */   
uint8_t unalignement_UsageFault();


/**
 * @brief 
 * @return 
 */   
uint8_t coprocessor_UsageFault();


/**
 * @brief 
 * @return 
 */   
uint8_t invPC_UsageFault();


/**
 * @brief 
 * @return 
 */   
uint8_t invEPSR_UsageFault();


/**
 * @brief 
 * @return 
 */   
uint8_t BFAR_valid_addr();


/**
 * @brief 
 * @return 
 */   
uint8_t FP_LazyState_BusFault();


/**
 * @brief 
 * @return 
 */   
uint8_t push_BusFault();


/**
 * @brief 
 * @return 
 */   
uint8_t pop_BusFault();


/**
 * @brief 
 * @return 
 */   
uint8_t imprecise_BusFault();


/**
 * @brief 
 * @return 
 */   
uint8_t precise_DBus_error();


/**
 * @brief 
 * @return 
 */   
uint8_t IBus_error();


/**
 * @brief 
 * @return 
 */   
uint8_t MMAR_valid_addr();


/**
 * @brief 
 * @return 
 */   
uint8_t FP_LazyState_MemMan_fault();


/**
 * @brief
 * @return 
 */   
uint8_t push_MemMan_fault();


/**
 * @brief 
 * @return 
 */   
uint8_t pop_MemMan_fault();


/**
 * @brief 
 * @return 
 */   
uint8_t DataAccess_MemMan_fault();


/**
 * @brief 
 * @return 
 */   
uint8_t ExecNot_section_MemMan_fault();


/**
 * @brief 
 * @return 
 */   
uint8_t forced_HardFault();


/**
 * @brief 
 * @return 
 */   
uint8_t push_MemMan_fault();


/**
 * @brief 
 * @return 
 */   
uint8_t vect_table_HardFault();


/**
 * @brief 
 * @return 
 */   
void *get_MemManFault_addr();

/**
 * @brief 
 * @return 
 */   
void *get_BusFault_addr();


/**
 * @brief 
 * @return 
 */   
void *get_AuxFault_addr();


/**
 * @brief Adds the callback function to the list of functions to be called
 * when that IRQ is triggered
 * @param IRQ_number the IRQ number
 * @param clbk_addr address of the callback function
 */
void IRQ_add_callback(uint8_t IRQ_number, void *clbk_addr);

/**
 * @brief Removes the callback function to the list of functions to be called
 * when that IRQ is triggered
 * @param IRQ_number the IRQ number
 * @param clbk_addr address of the callback function
 */
void IRQ_add_callback(uint8_t IRQ_number, void *clbk_addr);


//----------------------------------------
//----------------------------------------Wrapper for various functions that can be used by app developers
//----------------------------------------

/** 
 * @brief Copies a block of memory from source to destination, handling 4-byte alignment.
 * @param src Pointer to the source memory block.
 * @param dest Pointer to the destination memory block.
 * @param len Length (in bytes) of the memory block to copy (must be 4 bytes aligned).
 */
extern void memcpy_4(void *src, void *dest, size_t len);

/** 
 * @brief Copies a block of memory from source to destination, handling 1-byte alignment.
 * @param src Pointer to the source memory block.
 * @param dest Pointer to the destination memory block.
 * @param len Length (in bytes) of the memory block to copy (can be 1 byte aligned).
 */
extern void memcpy_1(void *src, void *dest, size_t len);

/** 
 * @brief Sets a block of memory to a specified byte value, handling 4-byte alignment.
 * @param dest Pointer to the destination memory block.
 * @param val Value (byte) to set in the memory block.
 * @param len Length (in bytes) of the memory block to set (must be 4 bytes aligned).
 */
extern void memset_4(void *dest, uint8_t val, size_t len);

/** 
 * @brief Sets a block of memory to a specified byte value, handling 1-byte alignment.
 * @param dest Pointer to the destination memory block.
 * @param val Value (byte) to set in the memory block.
 * @param len Length (in bytes) of the memory block to set (can be 1 byte aligned).
 */
extern void memset_1(void *dest, uint8_t val, size_t len);

/** 
 * @brief Clears (zeros out) a block of memory, handling 4-byte alignment.
 * @param dest Pointer to the destination memory block.
 * @param len Length (in bytes) of the memory block to clear (must be 4 bytes aligned).
 */
extern void memzero_4(void *dest, size_t len);

/** 
 * @brief Clears (zeros out) a block of memory, handling 1-byte alignment.
 * @param dest Pointer to the destination memory block.
 * @param len Length (in bytes) of the memory block to clear (can be 1 byte aligned).
 */
extern void memzero_1(void *dest, size_t len);

/** 
 * @brief Compares two memory blocks byte by byte, handling 4-byte alignment.
 * @param start1 Pointer to the first memory block.
 * @param start2 Pointer to the second memory block.
 * @param len Length (in bytes) of the memory blocks to compare (must be 4 bytes aligned).
 * @return 0 if the memory blocks are equal, non-zero otherwise.
 */
extern uint8_t memcmp_4(void *start1, void *start2, size_t len);

/** 
 * @brief Compares two memory blocks byte by byte, handling 1-byte alignment.
 * @param start1 Pointer to the first memory block.
 * @param start2 Pointer to the second memory block.
 * @param len Length (in bytes) of the memory blocks to compare (can be 1 byte aligned).
 * @return 0 if the memory blocks are equal, non-zero otherwise.
 */
extern uint8_t memcmp_1(void *start1, void *start2, size_t len);

#endif // !WRAPPERS_H 1