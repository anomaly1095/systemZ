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


#include <STM32F401.h>

#ifndef TYPES_H 1

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

// Define the SVC call macro
#define SVC_CALL(N)             \
  uint8_t result;               \
  __asm__ volatile (            \
    SVC_NUM(N)                  \
    "MOV    %[result], r0\t\n"  \
    : [result] "=r" (result)    \
    :                           \
    : "memory"                  \
  );                            \
  return result;

/**
 * @brief Enable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_enable_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(0);
}

/**
 * @brief Disable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_disable_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(1);
}

/**
 * @brief Set an interrupt as pending in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_set_pend_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(2);
}

/**
 * @brief Clears an interrupt from pending list in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_clear_pend_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(3);
}

/**
 * @brief Check if an interrupt is active in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 1 if active, 0 if idle
 */
uint8_t NVIC_check_active_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(4);
}

/**
 * @brief Set the priority of an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @param PRIO The priority to assign to the interrupt.
 * @return Returns 0 upon successful execution of the syscall.
 */
uint8_t NVIC_set_pri_irq(uint8_t IRQ_NUM, uint8_t PRIO)
{
  SVC_CALL(5);
}

/**
 * @brief Get the priority of an interrupt NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns the priority of the interrupt
 */
uint8_t NVIC_get_pri_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(6);
}

/**
 * @brief Triggers an interrupt of the IRQ specified in IRQ_NUM thru software
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns the priority of the interrupt
 */   
uint8_t NVIC_soft_trigger_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(7);
}

