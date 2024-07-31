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

/* @-------NVIC--------@ */

extern void     NVIC_enable_irq                     (uint8_t IRQ_NUM);
extern void     NVIC_disable_irq                    (uint8_t IRQ_NUM);
extern void     NVIC_set_pend_irq                   (uint8_t IRQ_NUM);
extern void     NVIC_clear_pend_irq                 (uint8_t IRQ_NUM);
extern uint8_t  NVIC_check_active_irq               (uint8_t IRQ_NUM);
extern void     NVIC_set_prio_irq                   (uint8_t IRQ_NUM, uint8_t PRIO);
extern uint8_t  NVIC_get_prio_irq                   (uint8_t IRQ_NUM);
extern void     NVIC_soft_trigger_irq               (uint8_t IRQ_NUM);

/* @-------Memory management--------@ */

extern void *   sbrk                                (size_t increment);
extern void *   sbrk_free                           (size_t decrement);
extern void *   _malloc                             (void);
extern void *   _free                               (void);

/* @-------System control--------@ */

extern void     enable_outoforder_exec              (void);
extern void     disable_outoforder_exec             (void);
extern void     get_CPUID                           (void);
extern void     NMI_set_pend                        (void);
extern void     PendSV_set_pend                     (void);
extern void     PendSV_clear_pend                   (void);
extern void     SYSTICK_set_pend                    (void);
extern void     SYSTICK_clear_pend                  (void);
extern void     SYSTICK_check_pend                  (void);
extern void     ISR_check_pend                      (void);
extern void     prio_set_split16_0                  (void);
extern void     prio_set_split8_2                   (void);
extern void     prio_set_split4_4                   (void);
extern void     prio_set_split2_8                   (void);
extern void     prio_set_split0_16                  (void);
extern void     RESET_request                       (void);
extern void     SEV_on_pend                         (void);
extern void     sleep_is_sleep_deep                 (void);
extern void     sleep_on_exit                       (void);
extern void     Stack_align4bytes                   (void);
extern void     Stack_align8bytes                   (void);
extern void     NMI_HardF_dis_fault_handling        (void);
extern void     NMI_HardF_en_fault_handling         (void);
extern void     DIV0_notrap                         (void);
extern void     DIV0_trap                           (void);
extern void     unalign_NTrap                       (void);
extern void     unalign_Trap                        (void);
extern void     APP_access_STIR                     (void);
extern void     no_enter_thread_mode_on_active_exc  (void);
extern void     Set_UsageFault_prio                 (uint8_t prio);
extern void     Set_MemMan_fault_prio               (uint8_t prio);
extern void     Set_SVC_prio                        (uint8_t prio);
extern void     Set_SYSTICK_prio                    (uint8_t prio);
extern void     Set_PendSV_prio                     (uint8_t prio);
extern void     en_UsageFault                       (void);
extern void     en_BusFault                         (void);
extern void     en_MemMan_fault                     (void);
extern void     dis_UsageFault                      (void);
extern void     dis_BusFault                        (void);
extern void     dis_MemMan_fault                    (void);
extern uint8_t  is_SVC_pend                         (void);
extern uint8_t  is_BusFault_pend                    (void);
extern uint8_t  is_MemMan_fault_pend                (void);
extern uint8_t  is_UsageFault_pend                  (void);
extern uint8_t  is_SYSTICK_actv                     (void);
extern uint8_t  is_PendSV_actv                      (void);
extern uint8_t  is_DBGMon_actv                      (void);
extern uint8_t  is_SVC_actv                         (void);
extern uint8_t  is_UsageFault_actv                  (void);
extern uint8_t  is_BusFault_active                  (void);
extern uint8_t  is_MemMan_fault_active              (void);
extern uint8_t  div_by0_UsageFault                  (void);
extern uint8_t  unalignement_UsageFault             (void);
extern uint8_t  coprocessor_UsageFault              (void);
extern uint8_t  invPC_UsageFault                    (void);
extern uint8_t  invEPSR_UsageFault                  (void);
extern uint8_t  BFAR_valid_addr                     (void);
extern uint8_t  FP_LazyState_BusFault               (void);
extern uint8_t  push_BusFault                       (void);
extern uint8_t  pop_BusFault                        (void);
extern uint8_t  imprecise_BusFault                  (void);
extern uint8_t  precise_DBus_error                  (void);
extern uint8_t  IBus_error                          (void);
extern uint8_t  MMAR_valid_addr                     (void);
extern uint8_t  FP_LazyState_MemMan_fault           (void);
extern uint8_t  push_MemMan_fault                   (void);
extern uint8_t  pop_MemMan_fault                    (void);
extern uint8_t  data_access_violation_MemMan_fault  (void);
extern uint8_t  instr_access_violation_MemMan_fault (void);
extern void *   get_MemMan_fault_address            (void);
extern void *   get_BusFault_address                (void);
extern uint32_t get_precise_DBus_error_address      (void);
extern uint32_t get_IBus_error_address              (void);
extern uint8_t  get_MMFSR                           (void);
extern uint8_t  get_BFSR                            (void);
extern uint16_t get_UFSR                            (void);
extern uint32_t get_APSR                            (void);
extern uint32_t get_IPSR                            (void);
extern uint32_t get_SPPR                            (void);
extern uint32_t get_CONTROL                         (void);
extern void     save_and_switch_context             (void* new_context);
extern void     restore_context                     (void* prev_context);
extern void     switch_to_next_context              (void);
extern void     set_exception_handler               (uint8_t exception_id, void (*handler)());
extern void     clear_exception                     (uint8_t exception_id);


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