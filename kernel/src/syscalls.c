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

// Define the SVC call macro for calls with no return value
#define SVC_CALL(N, ...)                            \
  do {                                              \
    uint32_t args[] = { __VA_ARGS__ };              \
    switch (sizeof(args) / sizeof(args[0])) {       \
      case 0:                                       \
        __asm__ volatile (                          \
          SVC_NUM(N)                                \
          :                                         \
          :                                         \
          : "memory"                                \
        );                                          \
        break;                                      \
      case 1:                                       \
        __asm__ volatile (                          \
          SVC_NUM(N)                                \
          :                                         \
          : "r" (args[0])                           \
          : "memory"                                \
        );                                          \
        break;                                      \
      case 2:                                       \
        __asm__ volatile (                          \
          SVC_NUM(N)                                \
          :                                         \
          : "r" (args[0]), "r" (args[1])            \
          : "memory"                                \
        );                                          \
        break;                                      \
    }                                               \
  } while (0)

// Define the SVC call macro for calls with a return value
#define SVC_CALL_RET(N, result, ...)                \
  do {                                              \
    uint32_t args[] = { __VA_ARGS__ };              \
    switch (sizeof(args) / sizeof(args[0])) {       \
      case 0:                                       \
        __asm__ volatile (                          \
          SVC_NUM(N)                                \
          "MOV    %[result], r0\t\n"                \
          : [result] "=r" (result)                  \
          :                                         \
          : "memory"                                \
        );                                          \
        break;                                      \
      case 1:                                       \
        __asm__ volatile (                          \
          SVC_NUM(N)                                \
          "MOV    %[result], r0\t\n"                \
          : [result] "=r" (result)                  \
          : "r" (args[0])                           \
          : "memory"                                \
        );                                          \
        break;                                      \
      case 2:                                       \
        __asm__ volatile (                          \
          SVC_NUM(N)                                \
          "MOV    %[result], r0\t\n"                \
          : [result] "=r" (result)                  \
          : "r" (args[0]), "r" (args[1])            \
          : "memory"                                \
        );                                          \
        break;                                      \
    }                                               \
  } while (0)


/* @-------NVIC--------@ */

/**
 * @brief Enable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */
void NVIC_enable_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(0U, IRQ_NUM);
}
/**
 * @brief Disable an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */
void NVIC_disable_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(1U, IRQ_NUM);
}
/**
 * @brief Set an interrupt as pending in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */
void NVIC_set_pend_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(2U, IRQ_NUM);
}
/**
 * @brief Clears an interrupt from pending list in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */
void NVIC_clear_pend_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(3U, IRQ_NUM);
}

/**
 * @brief Check if an interrupt is active in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns 1 if active, 0 if idle
 */
uint8_t NVIC_check_active_irq(uint8_t IRQ_NUM)
{
  uint8_t result;
  SVC_CALL_RET(4U, result, IRQ_NUM);
  return result;
}

/**
 * @brief Set the priority of an interrupt in NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @param PRIO The priority to assign to the interrupt.
 * @return Returns 0 upon successful execution of the syscall.
 */
void NVIC_set_prio_irq(uint8_t IRQ_NUM, uint8_t PRIO)
{
  SVC_CALL(5U, IRQ_NUM, PRIO);
}

/**
 * @brief Get the priority of an interrupt NVIC using SVC syscall.
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 * @return Returns the priority of the interrupt
 */
uint8_t NVIC_get_prio_irq(uint8_t IRQ_NUM)
{
  uint8_t result;
  SVC_CALL_RET(6U, result, IRQ_NUM);
  return result;
}

/**
 * @brief Triggers an interrupt of the IRQ specified in IRQ_NUM thru software
 * @param IRQ_NUM The number of the IRQ (0..239) to enable.
 */   
void NVIC_soft_trigger_irq(uint8_t IRQ_NUM)
{
  SVC_CALL(7U, IRQ_NUM);
}

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
  SVC_CALL_RET(8U, result, increment);
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
  SVC_CALL_RET(9U, result, decrement);
  return result;
}

/**
 * @brief Allocates memory from SRAM
 * @return Returns the new pointer
 * @return Returns 0 ((void*)(0x0)) if failed to allocate SRAM
 */
void *_malloc()
{
  void *result;
  SVC_CALL_RET(10U, result);
  return result;
}

/**
 * @brief Frees allocated memory
 * @return Returns the pointer to the freed memory
 * @return Returns 0 ((void*)(0x0)) if failed to free memory
 */
void *_free()
{
  void *result;
  SVC_CALL_RET(11U, result);
  return result;
}

/* @-------System control--------@ */

/**
 * @brief Enables out-of-order execution
 */
void enable_outoforder_exec(){ SVC_CALL(14U); }

/**
 * @brief Disables out-of-order execution
 */
void disable_outoforder_exec(){ SVC_CALL(15U); }

/**
 * @brief Retrieves the CPUID
 */
void get_CPUID(){ SVC_CALL(16U); }

/**
 * @brief Sets the NMI interrupt pending
 */
void NMI_set_pend(){ SVC_CALL(17U); }

/**
 * @brief Sets the PendSV interrupt pending
 */
void PendSV_set_pend(){ SVC_CALL(18U); }

/**
 * @brief Clears the PendSV interrupt pending
 */
void PendSV_clear_pend(){ SVC_CALL(19U); }

/**
 * @brief Sets the SysTick interrupt pending
 */
void SYSTICK_set_pend(){ SVC_CALL(20U); }

/**
 * @brief Clears the SysTick interrupt pending
 */
void SYSTICK_clear_pend(){ SVC_CALL(21U); }

/**
 * @brief Checks if the SysTick interrupt is pending
 */
void SYSTICK_check_pend(){ SVC_CALL(22U); }

/**
 * @brief Checks if any interrupt is pending
 */
void ISR_check_pend(){ SVC_CALL(23U); }

/**
 * @brief Sets priority grouping to 16 pre-emption priority bits, 0 subpriority bits
 */
void prio_set_split16_0(){ SVC_CALL(24U); }

/**
 * @brief Sets priority grouping to 8 pre-emption priority bits, 2 subpriority bits
 */
void prio_set_split8_2(){ SVC_CALL(25U); }

/**
 * @brief Sets priority grouping to 4 pre-emption priority bits, 4 subpriority bits
 */
void prio_set_split4_4(){ SVC_CALL(26U); }

/**
 * @brief Sets priority grouping to 2 pre-emption priority bits, 8 subpriority bits
 */
void prio_set_split2_8(){ SVC_CALL(27U); }

/**
 * @brief Sets priority grouping to 0 pre-emption priority bits, 16 subpriority bits
 */
void prio_set_split0_16(){ SVC_CALL(28U); }

/**
 * @brief Requests a system reset
 */
void RESET_request(){ SVC_CALL(29U); }

/**
 * @brief Sets the SEVONPEND bit
 */
void SEV_on_pend(){ SVC_CALL(30U); }

/**
 * @brief Sets the SLEEPDEEP bit
 */
void sleep_is_sleep_deep(){ SVC_CALL(31U); }

/**
 * @brief Sets the SLEEPONEXIT bit
 */
void sleep_on_exit(){ SVC_CALL(32U); }

/**
 * @brief Sets the stack alignment to 4 bytes
 */
void Stack_align4bytes(){ SVC_CALL(33U); }

/**
 * @brief Sets the stack alignment to 8 bytes
 */
void Stack_align8bytes(){ SVC_CALL(34U); }

/**
 * @brief Disables fault handling for NMI and HardFault
 */
void NMI_HardF_dis_fault_handling(){ SVC_CALL(35U); }

/**
 * @brief Enables fault handling for NMI and HardFault
 */
void NMI_HardF_en_fault_handling(){ SVC_CALL(36U); }

/**
 * @brief Disables trapping of division by zero
 */
void DIV0_notrap(){ SVC_CALL(37U); }

/**
 * @brief Enables trapping of division by zero
 */
void DIV0_trap(){ SVC_CALL(38U); }

/**
 * @brief Disables trapping of unaligned accesses
 */
void unalign_NTrap(){ SVC_CALL(39U); }

/**
 * @brief Enables trapping of unaligned accesses
 */
void unalign_Trap(){ SVC_CALL(40U); }

/**
 * @brief Allows application to access the STIR register
 */
void APP_access_STIR(){ SVC_CALL(41U); }

/**
 * @brief Prevents entry into Thread mode on return from an active exception
 */
void no_enter_thread_mode_on_active_exc(){ SVC_CALL(42U); }

/**
 * @brief Prevents entry into Thread mode on return from an active exception (duplicate function)
 */
void no_enter_thread_mode_on_active_exc(){ SVC_CALL(43U); }

/**
 * @brief Sets the priority of the Usage Fault exception
 * @param prio Priority value
 */
void Set_UsageFault_prio(uint8_t prio)
{
  SVC_CALL(44U, prio);
}

/**
 * @brief Sets the priority of the Memory Management Fault
 * @param prio Priority value
 */
void Set_MemMan_fault_prio(uint8_t prio) {
  SVC_CALL(45U, prio);
}

/**
 * @brief Sets the priority of the Supervisor Call (SVC)
 * @param prio Priority value
 */
void Set_SVC_prio(uint8_t prio) {
  SVC_CALL(46U, prio);
}

/**
 * @brief Sets the priority of the SysTick
 * @param prio Priority value
 */
void Set_SYSTICK_prio(uint8_t prio) {
  SVC_CALL(47U, prio);
}

/**
 * @brief Sets the priority of the PendSV
 * @param prio Priority value
 */
void Set_PendSV_prio(uint8_t prio) {
  SVC_CALL(48U, prio);
}

/**
 * @brief Enables the Usage Fault
 */
void en_UsageFault() { SVC_CALL(49U); }

/**
 * @brief Enables the Bus Fault
 */
void en_BusFault() { SVC_CALL(50U); }

/**
 * @brief Enables the Memory Management Fault
 */
void en_MemMan_fault() { SVC_CALL(51U); }

/**
 * @brief Disables the Usage Fault
 */
void dis_UsageFault() { SVC_CALL(52U); }

/**
 * @brief Disables the Bus Fault
 */
void dis_BusFault() { SVC_CALL(53U); }

/**
 * @brief Disables the Memory Management Fault
 */
void dis_MemMan_fault() { SVC_CALL(54U); }

/**
 * @brief Checks if SVC is pending
 * @return 1 if pending, 0 otherwise
 */
uint8_t is_SVC_pend() {
  uint8_t result;
  SVC_CALL_RET(55U, result);
  return result;
}

/**
 * @brief Checks if Bus Fault is pending
 * @return 1 if pending, 0 otherwise
 */
uint8_t is_BusFault_pend() {
  uint8_t result;
  SVC_CALL_RET(56U, result);
  return result;
}

/**
 * @brief Checks if Memory Management Fault is pending
 * @return 1 if pending, 0 otherwise
 */
uint8_t is_MemMan_fault_pend() {
  uint8_t result;
  SVC_CALL_RET(57U, result);
  return result;
}

/**
 * @brief Checks if Usage Fault is pending
 * @return 1 if pending, 0 otherwise
 */
uint8_t is_UsageFault_pend() {
  uint8_t result;
  SVC_CALL_RET(58U, result);
  return result;
}

/**
 * @brief Checks if SysTick is active
 * @return 1 if active, 0 otherwise
 */
uint8_t is_SYSTICK_actv() {
  uint8_t result;
  SVC_CALL_RET(59U, result);
  return result;
}

/**
 * @brief Checks if PendSV is active
 * @return 1 if active, 0 otherwise
 */
uint8_t is_PendSV_actv() {
  uint8_t result;
  SVC_CALL_RET(60U, result);
  return result;
}

/**
 * @brief Checks if Debug Monitor is active
 * @return 1 if active, 0 otherwise
 */
uint8_t is_DBGMon_actv() {
  uint8_t result;
  SVC_CALL_RET(61U, result);
  return result;
}

/**
 * @brief Checks if SVC is active
 * @return 1 if active, 0 otherwise
 */
uint8_t is_SVC_actv() {
  uint8_t result;
  SVC_CALL_RET(62U, result);
  return result;
}

/**
 * @brief Checks if Usage Fault is active
 * @return 1 if active, 0 otherwise
 */
uint8_t is_UsageFault_actv() {
  uint8_t result;
  SVC_CALL_RET(63U, result);
  return result;
}

/**
 * @brief Checks if Bus Fault is active
 * @return 1 if active, 0 otherwise
 */
uint8_t is_BusFault_active() {
  uint8_t result;
  SVC_CALL_RET(64U, result);
  return result;
}

/**
 * @brief Checks if Memory Management Fault is active
 * @return 1 if active, 0 otherwise
 */
uint8_t is_MemMan_fault_active() {
  uint8_t result;
  SVC_CALL_RET(65U, result);
  return result;
}

/**
 * @brief Checks if division by zero caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t div_by0_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(66U, result);
  return result;
}

/**
 * @brief Checks if unaligned access caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t unalignement_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(67U, result);
  return result;
}

/**
 * @brief Checks if coprocessor access caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t coprocessor_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(68U, result);
  return result;
}

/**
 * @brief Checks if an invalid PC value caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t invPC_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(69U, result);
  return result;
}

/**
 * @brief Checks if an invalid EPSR value caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t invEPSR_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(70U, result);
  return result;
}

/**
 * @brief Checks if BFAR contains a valid address
 * @return 1 if true, 0 otherwise
 */
uint8_t BFAR_valid_addr() {
  uint8_t result;
  SVC_CALL_RET(71U, result);
  return result;
}

/**
 * @brief Checks if lazy state preservation caused a Bus Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t FP_LazyState_BusFault() {
  uint8_t result;
  SVC_CALL_RET(72U, result);
  return result;
}

/**
 * @brief Checks if a push operation caused a Bus Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t push_BusFault() {
  uint8_t result;
  SVC_CALL_RET(73U, result);
  return result;
}

/**
 * @brief Checks if a pop operation caused a Bus Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t pop_BusFault() {
  uint8_t result;
  SVC_CALL_RET(74U, result);
  return result;
}

/**
 * @brief Checks if an imprecise Bus Fault occurred
 * @return 1 if true, 0 otherwise
 */
uint8_t imprecise_BusFault() {
  uint8_t result;
  SVC_CALL_RET(75U, result);
  return result;
}

/**
 * @brief Checks if a precise data bus error occurred
 * @return 1 if true, 0 otherwise
 */
uint8_t precise_DBus_error() {
  uint8_t result;
  SVC_CALL_RET(76U, result);
  return result;
}

/**
 * @brief Checks if an instruction bus error occurred
 * @return 1 if true, 0 otherwise
 */
uint8_t IBus_error() {
  uint8_t result;
  SVC_CALL_RET(77U, result);
  return result;
}

/**
 * @brief Checks if MMAR contains a valid address
 * @return 1 if true, 0 otherwise
 */
uint8_t MMAR_valid_addr() {
  uint8_t result;
  SVC_CALL_RET(78U, result);
  return result;
}

/**
 * @brief Checks if lazy state preservation caused a Memory Management Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t FP_LazyState_MemMan_fault() {
  uint8_t result;
  SVC_CALL_RET(79U, result);
  return result;
}

/**
 * @brief Checks if a push operation caused a Memory Management Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t push_MemMan_fault() {
  uint8_t result;
  SVC_CALL_RET(80U, result);
  return result;
}

/**
 * @brief Checks if a pop operation caused a Memory Management Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t pop_MemMan_fault() {
  uint8_t result;
  SVC_CALL_RET(81U, result);
  return result;
}

/**
 * @brief Checks if division by zero caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t div_by0_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(66U, result);
  return result;
}

/**
 * @brief Checks if unaligned access caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t unalignement_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(67U, result);
  return result;
}

/**
 * @brief Checks if coprocessor access caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t coprocessor_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(68U, result);
  return result;
}

/**
 * @brief Checks if an invalid PC value caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t invPC_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(69U, result);
  return result;
}

/**
 * @brief Checks if an invalid EPSR value caused a Usage Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t invEPSR_UsageFault() {
  uint8_t result;
  SVC_CALL_RET(70U, result);
  return result;
}

/**
 * @brief Checks if BFAR contains a valid address
 * @return 1 if true, 0 otherwise
 */
uint8_t BFAR_valid_addr() {
  uint8_t result;
  SVC_CALL_RET(71U, result);
  return result;
}

/**
 * @brief Checks if lazy state preservation caused a Bus Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t FP_LazyState_BusFault() {
  uint8_t result;
  SVC_CALL_RET(72U, result);
  return result;
}

/**
 * @brief Checks if a push operation caused a Bus Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t push_BusFault() {
  uint8_t result;
  SVC_CALL_RET(73U, result);
  return result;
}

/**
 * @brief Checks if a pop operation caused a Bus Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t pop_BusFault() {
  uint8_t result;
  SVC_CALL_RET(74U, result);
  return result;
}

/**
 * @brief Checks if an imprecise Bus Fault occurred
 * @return 1 if true, 0 otherwise
 */
uint8_t imprecise_BusFault() {
  uint8_t result;
  SVC_CALL_RET(75U, result);
  return result;
}

/**
 * @brief Checks if a precise data bus error occurred
 * @return 1 if true, 0 otherwise
 */
uint8_t precise_DBus_error() {
  uint8_t result;
  SVC_CALL_RET(76U, result);
  return result;
}

/**
 * @brief Checks if an instruction bus error occurred
 * @return 1 if true, 0 otherwise
 */
uint8_t IBus_error() {
  uint8_t result;
  SVC_CALL_RET(77U, result);
  return result;
}

/**
 * @brief Checks if MMAR contains a valid address
 * @return 1 if true, 0 otherwise
 */
uint8_t MMAR_valid_addr() {
  uint8_t result;
  SVC_CALL_RET(78U, result);
  return result;
}

/**
 * @brief Checks if lazy state preservation caused a Memory Management Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t FP_LazyState_MemMan_fault() {
  uint8_t result;
  SVC_CALL_RET(79U, result);
  return result;
}

/**
 * @brief Checks if a push operation caused a Memory Management Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t push_MemMan_fault() {
  uint8_t result;
  SVC_CALL_RET(80U, result);
  return result;
}

/**
 * @brief Checks if a pop operation caused a Memory Management Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t pop_MemMan_fault() {
  uint8_t result;
  SVC_CALL_RET(81U, result);
  return result;
}

/**
 * @brief Checks if a data access violation caused a Memory Management Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t data_access_violation_MemMan_fault() {
  uint8_t result;
  SVC_CALL_RET(82U, result);
  return result;
}

/**
 * @brief Checks if an instruction access violation caused a Memory Management Fault
 * @return 1 if true, 0 otherwise
 */
uint8_t instr_access_violation_MemMan_fault() {
  uint8_t result;
  SVC_CALL_RET(83U, result);
  return result;
}


/**
 * @brief Retrieves the address that caused the Memory Management Fault
 * @return Address that caused the fault
 */
void *get_MemMan_fault_address() {
  void *result;
  SVC_CALL_RET(84U, result);
  return result;
}

/**
 * @brief Retrieves the address that caused the Bus Fault
 * @return Address that caused the fault
 */
void *get_BusFault_address() {
  void *result;
  SVC_CALL_RET(85U, result);
  return result;
}

/**
 * @brief Retrieves the address that caused the precise data bus error
 * @return Address that caused the error
 */
uint32_t get_precise_DBus_error_address() {
  uint32_t result;
  SVC_CALL_RET(86U, result);
  return result;
}

/**
 * @brief Retrieves the address that caused the instruction bus error
 * @return Address that caused the error
 */
uint32_t get_IBus_error_address() {
  uint32_t result;
  SVC_CALL_RET(87U, result);
  return result;
}

/**
 * @brief Retrieves the Memory Management Fault Status Register (MMFSR) value
 * @return MMFSR value
 */
uint8_t get_MMFSR() {
  uint8_t result;
  SVC_CALL_RET(88U, result);
  return result;
}

/**
 * @brief Retrieves the Bus Fault Status Register (BFSR) value
 * @return BFSR value
 */
uint8_t get_BFSR() {
  uint8_t result;
  SVC_CALL_RET(89U, result);
  return result;
}

/**
 * @brief Retrieves the Usage Fault Status Register (UFSR) value
 * @return UFSR value
 */
uint16_t get_UFSR() {
  uint16_t result;
  SVC_CALL_RET(90U, result);
  return result;
}

/**
 * @brief Retrieves the Application Program Status Register (APSR) value
 * @return APSR value
 */
uint32_t get_APSR() {
  uint32_t result;
  SVC_CALL_RET(93U, result);
  return result;
}

/**
 * @brief Retrieves the Interrupt Program Status Register (IPSR) value
 * @return IPSR value
 */
uint32_t get_IPSR() {
  uint32_t result;
  SVC_CALL_RET(94U, result);
  return result;
}

/**
 * @brief Retrieves the Special Purpose Program Status Register (SPPR) value
 * @return SPPR value
 */
uint32_t get_SPPR() {
  uint32_t result;
  SVC_CALL_RET(95U, result);
  return result;
}

/**
 * @brief Retrieves the Control Register (CONTROL) value
 * @return CONTROL value
 */
uint32_t get_CONTROL() {
  uint32_t result;
  SVC_CALL_RET(96U, result);
  return result;
}

/**
 * @brief Saves the current context and switches to a new context
 * @param new_context Pointer to the new context
 */
void save_and_switch_context(void* new_context) {
  SVC_CALL(97U, new_context);
}

/**
 * @brief Restores the previous context
 * @param prev_context Pointer to the previous context
 */
void restore_context(void* prev_context) {
  SVC_CALL(98U, prev_context);
}

/**
 * @brief Switches to the next context
 */
void switch_to_next_context() {
  SVC_CALL(99U);
}

/**
 * @brief Sets a handler for the specified system exception
 * @param exception_id ID of the system exception
 * @param handler Pointer to the handler function
 */
void set_exception_handler(uint8_t exception_id, void (*handler)()) {
  SVC_CALL(100U, exception_id, handler);
}

/**
 * @brief Clears the specified system exception
 * @param exception_id ID of the system exception
 */
void clear_exception(uint8_t exception_id) {
  SVC_CALL(101U, exception_id);
}
