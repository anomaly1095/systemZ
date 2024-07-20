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
#define SVC_CALL_RET(N, result)             \
  __asm__ volatile (            \
    SVC_NUM(N)                  \
    "MOV    %[result], r0\t\n"  \
    : [result] "=r" (result)    \
    :                           \
    : "memory"                  \
  );

#define SVC_CALL(N)             \
  __asm__ volatile (            \
    SVC_NUM(N)                  \
    :                           \
    :                           \
    : "memory"                  \
  );

/* @-------NVIC--------@ */

/**
 * @brief Enable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */
void NVIC_enable_irq(uint8_t IRQ_NUM){ SVC_CALL(0U); }

/**
 * @brief Disable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */
void NVIC_disable_irq(uint8_t IRQ_NUM){ SVC_CALL(1U); }

/**
 * @brief Set an interrupt as pending in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */
void NVIC_set_pend_irq(uint8_t IRQ_NUM){ SVC_CALL(2U); }

/**
 * @brief Clears an interrupt from pending list in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */
void NVIC_clear_pend_irq(uint8_t IRQ_NUM){ SVC_CALL(3U); }

/**
 * @brief Check if an interrupt is active in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 1 if active, 0 if idle
 */
uint8_t NVIC_check_active_irq(uint8_t IRQ_NUM)
{
  uint8_t result;
  SVC_CALL_RET(4U, result);
  return result;
}

/**
 * @brief Set the priority of an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @param PRIO The priority to assign to the interrupt.
 * @return Returns 0 upon successful execution of the syscall.
 */
void NVIC_set_prio_irq(uint8_t IRQ_NUM, uint8_t PRIO){ SVC_CALL(5U); }

/**
 * @brief Get the priority of an interrupt NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns the priority of the interrupt
 */
uint8_t NVIC_get_prio_irq(uint8_t IRQ_NUM)
{
  uint8_t result;
  SVC_CALL_RET(6U, result);
  return result;
}

/**
 * @brief Triggers an interrupt of the IRQ specified in IRQ_NUM thru software
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */   
void NVIC_soft_trigger_irq(uint8_t IRQ_NUM){ SVC_CALL(7U); }

/* @-------Memory management--------@ */

/**
 * @brief Expand the APP process heap towards the top
 * @param increment Ammount of memory to allocate
 * @return Returns the new pointer address
 * @return returns 0 ((void*)(0x0)) if failed to allocate SRAM
 */   
void *sbrk(size_t increment)
{
  void *result;
  SVC_CALL_RET(8U, result);
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
  SVC_CALL_RET(9U, result);
  return result;
}

/**
 * @brief 
 * @param  
 * @return Returns the new pointer 
 * @return returns 0 ((void*)(0x0)) if failed to allocate SRAM
 */   
void *_malloc()
{
  void *result;
  SVC_CALL_RET(10U, result);
  return result;
}

/**
 * @brief  
 * @param 
 * @return 
 * @return 
 */
void *_free()
{
  void *result;
  SVC_CALL_RET(11U, result);
  return result;
}


// /**
//  * @brief 
//  * @param  
//  * @return Returns the new pointer 
//  * @return returns 0 ((void*)(0x0)) if failed to allocate SRAM
//  */   
// void *()
// {
//   void *result;
//   SVC_CALL_RET(10U, result);
//   return result;
// }

// /**
//  * @brief  
//  * @param 
//  * @return 
//  * @return 
//  */
// void *()
// {
//   void *result;
//   SVC_CALL_RET(11U, result);
//   return result;
// }


/* @-------System control--------@ */

/**
 * @brief 
 * @return 
 */   
void enable_outoforder_exec(){ SVC_CALL(14U); }

/**
 * @brief 
 * @return 
 */   
void disable_outoforder_exec(){ SVC_CALL(15U); }

/**
 * @brief 
 * @return 
 */   
void get_CPUID(){ SVC_CALL(16U); }

/**
 * @brief 
 * @return 
 */   
void NMI_set_pend(){ SVC_CALL(17U); }

/**
 * @brief 
 * @return 
 */   
void PendSV_set_pend(){ SVC_CALL(18U); }

/**
 * @brief 
 * @return 
 */   
void PendSV_clear_pend(){ SVC_CALL(19U); }

/**
 * @brief 
 * @return 
 */   
void SYSTICK_set_pend(){ SVC_CALL(20U); }

/**
 * @brief 
 * @return 
 */   
void SYSTICK_clear_pend(){ SVC_CALL(21U); }

/**
 * @brief 
 * @return 
 */   
void SYSTICK_check_pend(){ SVC_CALL(22U); }

/**
 * @brief 
 * @return 
 */   
void ISR_check_pend(){ SVC_CALL(23U); }

/**
 * @brief 
 * @return 
 */   
void prio_set_split16_0(){ SVC_CALL(24U); }

/**
 * @brief 
 * @return 
 */   
void prio_set_split8_2(){ SVC_CALL(25U); }

/**
 * @brief 
 * @return 
 */   
void prio_set_split4_4(){ SVC_CALL(26U); }

/**
 * @brief 
 * @return 
 */   
void prio_set_split2_8(){ SVC_CALL(27U); }

/**
 * @brief 
 * @return 
 */   
void prio_set_split0_16(){ SVC_CALL(28U); }

/**
 * @brief 
 * @return 
 */   
void RESET_request(){ SVC_CALL(29U); }

/**
 * @brief 
 * @return 
 */   
void SEV_on_pend(){ SVC_CALL(30U); }

/**
 * @brief 
 * @return 
 */   
void sleep_is_sleep_deep(){ SVC_CALL(31U); }

/**
 * @brief 
 * @return 
 */   
void sleep_on_exit(){ SVC_CALL(32U); }

/**
 * @brief 
 * @return 
 */   
void Stack_align4bytes(){ SVC_CALL(33U); }

/**
 * @brief 
 * @return 
 */   
void Stack_align8bytes(){ SVC_CALL(34U); }

/**
 * @brief 
 * @return 
 */   
void NMI_HardF_dis_fault_handling(){ SVC_CALL(35U); }

/**
 * @brief 
 * @return 
 */   
void NMI_HardF_en_fault_handling(){ SVC_CALL(36U); }

/**
 * @brief 
 * @return 
 */   
void DIV0_notrap(){ SVC_CALL(37U); }

/**
 * @brief 
 * @return 
 */   
void DIV0_trap(){ SVC_CALL(38U); }

/**
 * @brief 
 * @return 
 */   
void unalign_NTrap(){ SVC_CALL(39U); }

/**
 * @brief 
 * @return 
 */   
void unalign_Trap(){ SVC_CALL(40U); }

/**
 * @brief 
 * @return 
 */   
void APP_access_STIR(){ SVC_CALL(41U); }

/**
 * @brief 
 * @return 
 */   
void exit_nested_IRQs_on_ret(){ SVC_CALL(42U); }

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_UsageFault_prio(uint8_t prio)
{
  register void *r0 __asm__("r0") = prio;
  SVC_CALL(43U);
}

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_MemMan_fault_prio(uint8_t prio)
{
  register void *r0 __asm__("r0") = prio;
  SVC_CALL(44U);
}

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_SVC_prio(uint8_t prio)
{
  register void *r0 __asm__("r0") = prio;
  SVC_CALL(45U);
}

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_SYSTICK_prio(uint8_t prio)
{
  register void *r0 __asm__("r0") = prio;
  SVC_CALL(46U);
}

/**
 * @brief 
 * @param prio 
 * @return 
 */   
void Set_PendSV_prio(uint8_t prio)
{
  register void *r0 __asm__("r0") = prio;
  SVC_CALL(47U);
}

/**
 * @brief 
 * @return 
 */   
void en_UsageFault(){ SVC_CALL(48U); }

/**
 * @brief 
 * @return 
 */   
void en_BusFault(){ SVC_CALL(49U); }

/**
 * @brief 
 * @return 
 */   
void en_MemMan_fault(){ SVC_CALL(50U); }

/**
 * @brief 
 * @return 
 */   
void dis_UsageFault(){ SVC_CALL(51U); }

/**
 * @brief 
 * @return 
 */   
void dis_BusFault(){ SVC_CALL(52U); }

/**
 * @brief 
 * @return 
 */   
void dis_MemMan_fault(){ SVC_CALL(53U); }

/**
 * @brief 
 * @return 
 */   
uint8_t is_SVC_pend()
{
  uint8_t result;
  SVC_CALL_RET(54U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_BusFault_pend()
{
  uint8_t result;
  SVC_CALL_RET(55U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_MemMan_fault_pend()
{
  uint8_t result;
  SVC_CALL_RET(56U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_UsageFault_pend()
{
  uint8_t result;
  SVC_CALL_RET(57U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_SYSTICK_actv()
{
  uint8_t result;
  SVC_CALL_RET(58U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_PendSV_actv()
{
  uint8_t result;
  SVC_CALL_RET(59U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_DBGMon_actv()
{
  uint8_t result;
  SVC_CALL_RET(60U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_SVC_actv()
{
  uint8_t result;
  SVC_CALL_RET(61U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_UsageFault_actv()
{
  uint8_t result;
  SVC_CALL_RET(62U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_BusFault_active()
{
  uint8_t result;
  SVC_CALL_RET(63U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t is_MemMan_fault_active()
{
  uint8_t result;
  SVC_CALL_RET(64U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t div_by0_UsageFault()
{
  uint8_t result;
  SVC_CALL_RET(65U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t unalignement_UsageFault()
{
  uint8_t result;
  SVC_CALL_RET(66U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t coprocessor_UsageFault()
{
  uint8_t result;
  SVC_CALL_RET(67U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t invPC_UsageFault()
{
  uint8_t result;
  SVC_CALL_RET(68U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t invEPSR_UsageFault()
{
  uint8_t result;
  SVC_CALL_RET(69U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t BFAR_valid_addr()
{
  uint8_t result;
  SVC_CALL_RET(70U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t FP_LazyState_BusFault()
{
  uint8_t result;
  SVC_CALL_RET(71U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t push_BusFault()
{
  uint8_t result;
  SVC_CALL_RET(72U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t pop_BusFault()
{
  uint8_t result;
  SVC_CALL_RET(73U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t imprecise_BusFault()
{
  uint8_t result;
  SVC_CALL_RET(74U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t precise_DBus_error()
{
  uint8_t result;
  SVC_CALL_RET(75U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t IBus_error()
{
  uint8_t result;
  SVC_CALL_RET(76U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t MMAR_valid_addr()
{
  uint8_t result;
  SVC_CALL_RET(77U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t FP_LazyState_MemMan_fault()
{
  uint8_t result;
  SVC_CALL_RET(78U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t push_MemMan_fault()
{
  uint8_t result;
  SVC_CALL_RET(79U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t pop_MemMan_fault()
{
  uint8_t result;
  SVC_CALL_RET(80U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t DataAccess_MemMan_fault()
{
  uint8_t result;
  SVC_CALL_RET(81U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t ExecNot_section_MemMan_fault()
{
  uint8_t result;
  SVC_CALL_RET(82U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t forced_HardFault()
{
  uint8_t result;
  SVC_CALL_RET(83U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t push_MemMan_fault()
{
  uint8_t result;
  SVC_CALL_RET(84U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
uint8_t vect_table_HardFault()
{
  uint8_t result;
  SVC_CALL_RET(85U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
void *get_MemManFault_addr()
{
  void *result;
  SVC_CALL_RET(86U, result);
  return (void*)result;
}

/**
 * @brief 
 * @return 
 */   
void *get_BusFault_addr()
{
  void *result;
  SVC_CALL_RET(87U, result);
  return result;
}

/**
 * @brief 
 * @return 
 */   
void *get_AuxFault_addr()
{
  void *result;
  SVC_CALL_RET(88U, result);
  return result;
}

/**
 * @brief Adds the callback function to the list of functions to be called
 * when that IRQ is triggered
 * @param IRQ_number the IRQ number
 * @param clbk_addr address of the callback function
 */
void IRQ_add_callback(uint8_t IRQ_number, void *clbk_addr) {
    register uint8_t r0 __asm__("r0") = IRQ_number;
    register void *r1 __asm__("r1") = clbk_addr;
    SVC_CALL(89U);
}

/**
 * @brief Removes the callback function to the list of functions to be called
 * when that IRQ is triggered
 * @param IRQ_number the IRQ number
 * @param clbk_addr address of the callback function
 */
void IRQ_add_callback(uint8_t IRQ_number, void *clbk_addr)
{
  register uint8_t r0 __asm__("r0") = IRQ_number;
  register void *r1 __asm__("r1") = clbk_addr;
  SVC_CALL(90U);
}
