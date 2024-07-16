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