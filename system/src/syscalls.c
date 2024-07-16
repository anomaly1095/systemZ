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


#ifndef TYPES_H
  #define TYPES_H     1

  typedef signed char int8_t;
  typedef signed short int int16_t;
  typedef signed int int32_t;
  typedef signed long long int int64_t;
  typedef unsigned char uint8_t;
  typedef unsigned short int uint16_t;
  typedef unsigned int uint32_t;
  typedef unsigned long long int uint64_t;
  typedef float float32_t;
  typedef double float64_t;
  typedef unsigned int size_t;

#endif // !TYPES_H  1

//----------------------------------------
//----------------------------------------SYSCALLS  
//----------------------------------------

/**
 * @file SYSCALLS can be used by application developers
 * @brief
 * According to the ARM Procedure Call Standard (AAPCS)
 * These syscalls should work as so: 
 * 1- User calls the function with args 
 * 2- Compiler sets arg0 in r0 arg1 in r1 arg2 in r2 arg3 in r3
 * 3- SVC call occurs -> CPU does the context switch 
 * 4- r0, r1, r2, r3, r12, lr, pc, xPSR all get pushed on the process stack
 * 5- Switch to privileged handler mode and sp = MSP
 * 6- SVC implementation fetches the immediate value from SVC #imm
 * 7- Service gets called and then returns with the return value or state in r0
 * 8- context switch again to return to unprivileged thread mode and sp = PSP
 * 9- set the return value in "result"
 * 10- return to the local scope of the user code with the return val
*/

// Create the SVC instruction with the given number
#define SVC_NUM(N) "SVC    #" #N "\t\n"

// Define the SVC call macro returns uint8_t
#define SVC_CALL(N, result)             \
  __asm__ volatile (            \
    SVC_NUM(N)                  \
    "MOV    %[result], r0\t\n"  \
    : [result] "=r" (result)    \
    :                           \
    : "memory"                  \
  );

/**
 * @brief Enable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_enable_irq(uint8_t IRQ_NUM)
{
  uint8_t result;
  SVC_CALL(0, result);
  return result;
}

/**
 * @brief Disable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_disable_irq(uint8_t IRQ_NUM)
{
  uint8_t result;
  SVC_CALL(1, result);
  return result;
}

/**
 * @brief Set an interrupt as pending in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_set_pend_irq(uint8_t IRQ_NUM)
{
    uint8_t result;
  SVC_CALL(2, result);
  return result;
}

/**
 * @brief Clears an interrupt from pending list in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_clear_pend_irq(uint8_t IRQ_NUM)
{
  uint8_t result;
  SVC_CALL(3, result);
  return result;
}

/**
 * @brief Check if an interrupt is active in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 1 if active, 0 if idle
 */
uint8_t NVIC_check_active_irq(uint8_t IRQ_NUM)
{
  uint8_t result;
  SVC_CALL(4, result);
  return result;
}

/**
 * @brief Set the priority of an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @param PRIO The priority to assign to the interrupt.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_set_prio_irq(uint8_t IRQ_NUM, uint8_t PRIO)
{
    uint8_t result;
  SVC_CALL(5, result);
  return result;
}

/**
 * @brief Get the priority of an interrupt NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns the priority of the interrupt
 */
uint8_t NVIC_get_prio_irq(uint8_t IRQ_NUM)
{
    uint8_t result;
  SVC_CALL(6, result);
  return result;
}

/**
 * @brief Triggers an interrupt of the IRQ specified in IRQ_NUM thru software
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns the priority of the interrupt
 */   
uint8_t NVIC_soft_trigger_irq(uint8_t IRQ_NUM)
{
  uint8_t result;
  SVC_CALL(7, result);
  return result;
}

/**
 * @brief Expand the APP process heap towards the top
 * @param increment Ammount of memory to allocate
 * @return Returns the new pointer address
 * @return returns 0 ((void*)(0x0)) if failed to allocate SRAM
 */   
void *sbrk(size_t increment)
{
  void *result;
  SVC_CALL(8, result);
  return result;
}

/**
 * @brief Expand the APP process heap towards the top
 * @param decrement Ammount of memory to deallocate
 * @return Returns the new pointer address
 * @return returns 0 ((void*)(0x0)) if failed to free SRAM
 */   
void *sbrk_free(size_t decrement)
{
  void *result;
  SVC_CALL(9, result);
  return result;
}

