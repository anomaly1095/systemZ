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

#include <stdint.h>

/*---------------------------------------------------------------*/
/*-------------------Macros to manipulate bits-------------------*/
/*---------------------------------------------------------------*/
#ifndef BIT_MANIP 1
/* Clear the bits with MASK and set new vbits with VALUE in the register REG*/
#define MODIFY_REG(REG, MASK, VALUE) ((REG) = ((REG) & ~(MASK)) | (VALUE))

/*sets the BIT_NUM bit in the register*/
#define SET_BIT(REG, BIT_NUM)   ((REG) |= (0b1U << BIT_NUM))

/*sets the bits in the register that are set in MASK*/
#define SET_BITS(REG, MASK)   ((REG) |= (MASK))

/*reads the bit from the register and return boolean value*/
#define READ_BIT(REG, BIT_NUM)  (((REG) & (1U << (BIT_NUM))) != 0U)

/// @brief Reads the bits from the register and returns true if all MASK bits are set.
#define READ_BITS(REG, MASK)  (((REG) & (MASK)) == (MASK))

/*clears the BIT_NUM bit in the register*/
#define CLEAR_BIT(REG, BIT_NUM) ((REG) &= ~(1U << (BIT_NUM)))

/*clears the bits in the register based off the set bits in MASK*/
#define CLEAR_BITS(REG, MASK) ((REG) &= ~(MASK)))

/*rotates the bits in the register by N rotations*/
#define ROTATE_RIGHT(REG, N) \
  __asm__ volatile ( \
    "ROR %0, %0, %1" \
    : "+r" (REG) \
    : "I" (N) \
  )

#endif // !BIT_MANIP 1

/*---------------------------------------------------------------*/
/*----------Nested vector interrupt controller macros------------*/
/*---------------------------------------------------------------*/
#ifndef NVIC_H  1

typedef enum IRQ_num_t {
  /* External Interrupts */
  WWDG_IRQ              = 0U,
  PVD_IRQ               = 1U,
  TAMP_STAMP_IRQ        = 2U,
  RTC_WKUP_IRQ          = 3U,
  FLASH_IRQ             = 4U,
  RCC_IRQ               = 5U,
  EXTI0_IRQ             = 6U,
  EXTI1_IRQ             = 7U,
  EXTI2_IRQ             = 8U,
  EXTI3_IRQ             = 9U,
  EXTI4_IRQ             = 10U,
  DMA1_Stream0_IRQ      = 11U,
  DMA1_Stream1_IRQ      = 12U,
  DMA1_Stream2_IRQ      = 13U,
  DMA1_Stream3_IRQ      = 14U,
  DMA1_Stream4_IRQ      = 15U,
  DMA1_Stream5_IRQ      = 16U,
  DMA1_Stream6_IRQ      = 17U,
  ADC_IRQ               = 18U,
  EXTI9_5_IRQ           = 23U,
  TIM1_BRK_TIM9_IRQ     = 24U,
  TIM1_UP_TIM10_IRQ     = 25U,
  TIM1_TRG_COM_TIM11_IRQ= 26U,
  TIM1_CC_IRQ           = 27U,
  TIM2_IRQ              = 28U,
  TIM3_IRQ              = 29U,
  TIM4_IRQ              = 30U,
  I2C1_EV_IRQ           = 31U,
  I2C1_ER_IRQ           = 32U,
  I2C2_EV_IRQ           = 33U,
  I2C2_ER_IRQ           = 34U,
  SPI1_IRQ              = 35U,
  SPI2_IRQ              = 36U,
  USART1_IRQ            = 37U,
  USART2_IRQ            = 38U,
  EXTI15_10_IRQ         = 40U,
  RTC_Alarm_IRQ         = 41U,
  OTG_FS_WKUP_IRQ       = 42U,
  DMA1_Stream7_IRQ      = 55U,
  SDIO_IRQ              = 56U,
  TIM5_IRQ              = 50U,
  SPI3_IRQ              = 51U,
  DMA2_Stream0_IRQ      = 56U,
  DMA2_Stream1_IRQ      = 57U,
  DMA2_Stream2_IRQ      = 58U,
  DMA2_Stream3_IRQ      = 59U,
  DMA2_Stream4_IRQ      = 60U,
  OTG_FS_IRQ            = 67U,
  DMA2_Stream5_IRQ      = 68U,
  DMA2_Stream6_IRQ      = 69U,
  DMA2_Stream7_IRQ      = 70U,
  USART6_IRQ            = 71U,
  I2C3_EV_IRQ           = 72U,
  I2C3_ER_IRQ           = 73U,
  FPU_IRQ               = 81U,
  SPI4_IRQ              = 84U
} IRQ_num_t;

/// @brief Used by apps to trigger an interrupt on the specified interrupt in IRQ_num
/// @brief This function is only accessible from unprevileged apps if
/// @brief SCR reg is set so in page 230 of the stm32-cortex-M4 Referance Manual
/// @param IRQ_num irq number as specified in the enum
extern void _NVIC_soft_trigger_irq(IRQ_num_t IRQ_num);


#endif // !NVIC_H

/*=============================================================================*/

/*---------------------------------------------------------------*/
/*---------------Macros for Reset and clock control--------------*/
/*---------------------------------------------------------------*/
#ifndef RCC_HELPERS             1

#define RCC_BASE                (0x40023800U)
#define RCC_CR_OFFSET           (0x0U)
#define RCC_PLLCFGR_OFFSET      (0x04U)
#define RCC_CFGR_OFFSET         (0x08U)
#define RCC_CIR_OFFSET          (0x0CU)
#define RCC_AHB1RSTR_OFFSET     (0x10U)
#define RCC_AHB2RSTR_OFFSET     (0x14U)
#define RCC_APB1RSTR_OFFSET     (0x20U)
#define RCC_APB2RSTR_OFFSET     (0x24U)
#define RCC_AHB1ENR_OFFSET      (0x30U)
#define RCC_AHB2ENR_OFFSET      (0x34U)
#define RCC_APB1ENR_OFFSET      (0x40U)
#define RCC_APB2ENR_OFFSET      (0x44U)
#define RCC_AHB1LPENR_OFFSET    (0x50U)
#define RCC_AHB2LPENR_OFFSET    (0x54U)
#define RCC_APB1LPENR_OFFSET    (0x60U)
#define RCC_APB2LPENR_OFFSET    (0x64U)
#define RCC_BDCR_OFFSET         (0x70U)
#define RCC_CSR_OFFSET          (0x74U)
#define RCC_SSCGR_OFFSET        (0x80U)
#define RCC_PLLI2SCFGR_OFFSET   (0x84U)
#define RCC_DCKCFGR_OFFSET      (0x8CU)


/*RCC control register*/
#define RCC_CR         (*(volatile uint32_t *)(RCC_BASE + RCC_CR_OFFSET))
#define RCC_PLLCFGR    (*(volatile uint32_t *)(RCC_BASE + RCC_PLLCFGR_OFFSET))
#define RCC_CFGR       (*(volatile uint32_t *)(RCC_BASE + RCC_CFGR_OFFSET))
#define RCC_CIR        (*(volatile uint32_t *)(RCC_BASE + RCC_CIR_OFFSET))
#define RCC_AHB1RSTR   (*(volatile uint32_t *)(RCC_BASE + RCC_AHB1RSTR_OFFSET))
#define RCC_AHB2RSTR   (*(volatile uint32_t *)(RCC_BASE + RCC_AHB2RSTR_OFFSET))
#define RCC_APB1RSTR   (*(volatile uint32_t *)(RCC_BASE + RCC_APB1RSTR_OFFSET))
#define RCC_APB2RSTR   (*(volatile uint32_t *)(RCC_BASE + RCC_APB2RSTR_OFFSET))
#define RCC_AHB1ENR    (*(volatile uint32_t *)(RCC_BASE + RCC_AHB1ENR_OFFSET))
#define RCC_AHB2ENR    (*(volatile uint32_t *)(RCC_BASE + RCC_AHB2ENR_OFFSET))
#define RCC_APB1ENR    (*(volatile uint32_t *)(RCC_BASE + RCC_APB1ENR_OFFSET))
#define RCC_APB2ENR    (*(volatile uint32_t *)(RCC_BASE + RCC_APB2ENR_OFFSET))
#define RCC_AHB1LPENR  (*(volatile uint32_t *)(RCC_BASE + RCC_AHB1LPENR_OFFSET))
#define RCC_AHB2LPENR  (*(volatile uint32_t *)(RCC_BASE + RCC_AHB2LPENR_OFFSET))
#define RCC_APB1LPENR  (*(volatile uint32_t *)(RCC_BASE + RCC_APB1LPENR_OFFSET))
#define RCC_APB2LPENR  (*(volatile uint32_t *)(RCC_BASE + RCC_APB2LPENR_OFFSET))
#define RCC_BDCR       (*(volatile uint32_t *)(RCC_BASE + RCC_BDCR_OFFSET))
#define RCC_CSR        (*(volatile uint32_t *)(RCC_BASE + RCC_CSR_OFFSET))
#define RCC_SSCGR      (*(volatile uint32_t *)(RCC_BASE + RCC_SSCGR_OFFSET))
#define RCC_PLLI2SCFGR (*(volatile uint32_t *)(RCC_BASE + RCC_PLLI2SCFGR_OFFSET))
#define RCC_DCKCFGR    (*(volatile uint32_t *)(RCC_BASE + RCC_DCKCFGR_OFFSET))


/*--------RCC_CR---------*/
/*--------RCC_CR---------*/

/*Enable the Phase locked loop for the inter integrated sound system*/
#define RCC_PLLI2S_ENABLE()   (SET_BIT  (RCC_CR, 26U))
/*Disable the PLLI2S*/
#define RCC_PLLI2S_DISABLE()  (CLEAR_BIT(RCC_CR, 26U))
/*Check if the PLLI2S is ready*/
#define RCC_PLLI2S_READY()    (READ_BIT (RCC_CR, 27U))

/*Enable the Phase locked loop*/
#define RCC_PLL_ENABLE()      (SET_BIT  (RCC_CR, 24U))
/*Disable the PLL*/
#define RCC_PLL_DISABLE()     (CLEAR_BIT(RCC_CR, 24U))
/*Check if the PLL is ready*/
#define RCC_PLL_READY()       (READ_BIT (RCC_CR, 25U))

/*Enable the clock security system*/
#define RCC_CSS_ENABLE()      (SET_BIT  (RCC_CR, 19U))
/*Disable the CSS*/
#define RCC_CSS_DISABLE()     (CLEAR_BIT(RCC_CR, 19U))

/*Enable the High speed external oscillator (not provided in the nucleo)*/
#define RCC_HSE_ENABLE()      (SET_BIT  (RCC_CR, 16U))
/*Disable the HSE*/
#define RCC_HSE_DISABLE()     (CLEAR_BIT(RCC_CR, 16U))
/*Check if the HSE is ready*/
#define RCC_HSE_READY()       (READ_BIT (RCC_CR, 17U))
/*Bypass the HSE with an externally provided oscillator (crystal or ceramic)*/
#define RCC_HSE_BYPASS()      (SET_BIT  (RCC_CR, 18U))
/*No Bypass the HSE with an externally provided piezoelectric oscillator*/
#define RCC_HSE_NOBYPASS()    (CLEAR_BIT(RCC_CR, 18U))

/*Enable the High speed internal RC feedback oscillator*/
#define RCC_HSI_ENABLE()      (SET_BIT  (RCC_CR, 0U))
/*Disable the HSI*/
#define RCC_HSI_DISABLE()     (CLEAR_BIT(RCC_CR, 0U))
/*Check if the HSI is ready*/
#define RCC_HSI_READY()       (READ_BIT (RCC_CR, 1U))


/*--------RCC_PLLCFGR---------*/
/*--------RCC_PLLCFGR---------*/

/*Set the PLL frequency at 84Mhz*/
/* PLLSRC = HSI, PLLN = 336, PLLP = 4, PLLM = 16 */ 
#define RCC_PLL_84MHz() \
  MODIFY_REG(RCC_PLLCFGR, \
  0x1FFCFFFFU,    /* Reset PLLN and PLLP bits */ \
  (1U << 22U) | (336U << 6U) | (4U << 16U) | (16U << 0U));

/*Set the PLL frequency at 72Mhz*/
/* PLLSRC = HSI, PLLN = 432, PLLP = 6, PLLM = 16 */
#define RCC_PLL_72MHz() \
  MODIFY_REG(RCC_PLLCFGR, \
  0x1FFCFFFFU,    /* Reset PLLN and PLLP bits */ \
  (1U << 22U) | (432U << 6U) | (6U << 16U) | (16U << 0U));

/*Set the PLL frequency at 64Mhz*/
/* PLLSRC = HSI, PLLN = 256, PLLP = 4, PLLM = 16 */
#define RCC_PLL_64MHz() \
  MODIFY_REG(RCC_PLLCFGR, \
  0x1FFCFFFFU,    /* Reset PLLN and PLLP bits */ \
  (1U << 22U) | (256U << 6U) | (4U << 16U) | (16U << 0U));

/*Set the PLL frequency at 42Mhz*/
/* PLLSRC = HSI, PLLN = 336, PLLP = 8, PLLM = 16 */
#define RCC_PLL_42MHz() \
  MODIFY_REG(RCC_PLLCFGR, \
    0x1FFCFFFFU,    /* Reset PLLN and PLLP bits */ \
    (1U << 22U) | (336U << 6U) | (8U << 16U) | (16U << 0U));


/*--------RCC_CFGR---------*/
/*--------RCC_CFGR---------*/

/// @param SOURCE Is the source of output in Microcontroller clock output 2
/// 0b00: SYSCLK / 0b01: PLLI2S / 0b10: HSE / 0b11: PLL
#define RCC_MCO2_SRC(SOURCE) \
  CLEAR_BITS(RCC_CFGR, 0b11 << 30U) \
  SET_BITS(RCC_CFGR, SOURCE << 30U)

/// @param PRE is the prescaler to apply on the signal of Microcontroller clock output 2
/// 0xx: no division / 0b100: division by 2 / 0b101: division by 3 /
// 0xx: no division
// 100: division by 2
// 101: division by 3
// 110: division by 4
// 111: division by 5
#define RCC_MCO2_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b111U << 27U) \
  SET_BITS(RCC_CFGR, PRE << 27U)

/// @param SOURCE Is the source of output in Microcontroller clock output 1
// 00: HSI clock selected
// 01: LSE oscillator selected
// 10: HSE oscillator clock selected
// 11: PLL clock selected
#define RCC_MCO1_SRC(SOURCE) \
  CLEAR_BITS(RCC_CFGR, 0b11U << 21U) \
  SET_BITS(RCC_CFGR, SOURCE << 21U)

/// @param SOURCE Is the source of inter integrated sound clock
// 0: PLLI2S clock used as I2S clock source
// 1: External clock mapped on the I2S_CKIN pin used as I2S clock source
#define RCC_MCO1_SRC(SOURCE) \
  CLEAR_BITS(RCC_CFGR, 0b1U << 23U) \
  SET_BITS(RCC_CFGR, SOURCE << 23U)

/// @param PRE is the prescaler to apply on the signal of Microcontroller clock output 1
/// 0xx: no division / 0b100: division by 2 / 0b101: division by 3 /
// 0xx: no division
// 100: division by 2
// 101: division by 3
// 110: division by 4
// 111: division by 5
#define RCC_MCO1_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b111U << 24U) \
  SET_BITS(RCC_CFGR, PRE << 24U)

/// @param PRE is the prescaler to apply on the signal of the real time clock
/// 00000: no clock
/// 00001: no clock
/// 00010: HSE/2
/// 00011: HSE/3
/// 00100: HSE/4
/// ...
/// 11110: HSE/30
/// 11111: HSE/31
#define RCC_RTC_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b11111U << 16U) \
  SET_BITS(RCC_CFGR, PRE << 16U)

/// @param PRE is the prescaler to apply on the APB2 
// 0xx: AHB clock not divided
// 100: AHB clock divided by 2
// 101: AHB clock divided by 4
// 110: AHB clock divided by 8
// 111: AHB clock divided by 16
#define RCC_APB2_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b111U << 13U) \
  SET_BITS(RCC_CFGR, PRE << 13U)

/// @param PRE is the prescaler to apply on the APB1
// 0xx: AHB clock not divided
// 100: AHB clock divided by 2
// 101: AHB clock divided by 4
// 110: AHB clock divided by 8
// 111: AHB clock divided by 16
#define RCC_APB1_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b111U << 10U) \
  SET_BITS(RCC_CFGR, PRE << 10U)

/// @param PRE is the prescaler to apply on the AHB1
// 0xxx: system clock not divided
// 1000: system clock divided by 2
// 1001: system clock divided by 4
// 1010: system clock divided by 8
// 1011: system clock divided by 16
// 1100: system clock divided by 64
// 1101: system clock divided by 128
// 1110: system clock divided by 256
// 1111: system clock divided by 512
#define RCC_AHB1_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b1111U << 4U) \
  SET_BITS(RCC_CFGR, PRE << 4U)


/// @param STAT Is the mask to compare SWS bits to
// 00: HSI oscillator selected as system clock
// 01: HSE oscillator selected as system clock
// 10: PLL selected as system clock
// 11: not allowed
#define RCC_SYSCLK_STAT(STAT) \
  READ_BITS(RCC_CFGR, STAT)


/// @param SOURCE Is the source of the SYSCLK
// 00: HSI oscillator selected as system clock
// 01: HSE oscillator selected as system clock
// 10: PLL selected as system clock
// 11: not allowed
#define RCC_SYSCLK_SRC(SOURCE) \
  CLEAR_BITS(RCC_CFGR, 0b11U) \
  SET_BITS(RCC_CFGR, SOURCE)


/*--------RCC_CIR---------*/
/*--------RCC_CIR---------*/

/*Clear CSS interrupt flag*/
#define RCC_CSS_IC() SET_BIT(RCC_CIR, 23U)
/*Clear PLLI2S ready interrupt flag*/
#define RCC_PLLI2SRDY_IC() SET_BIT(RCC_CIR, 21U)
/*Clear PLL ready interrupt flag*/
#define RCC_PLLRDY_IC() SET_BIT(RCC_CIR, 20U)
/*Clear HSE ready interrupt flag*/
#define RCC_HSERDY_IC() SET_BIT(RCC_CIR, 19U)
/*Clear HSI ready interrupt flag*/
#define RCC_HSIRDY_IC() SET_BIT(RCC_CIR, 18U)
/*Clear LSE ready interrupt flag*/
#define RCC_LSERDY_IC() SET_BIT(RCC_CIR, 17U)
/*Clear LSI ready interrupt flag*/
#define RCC_LSIRDY_IC() SET_BIT(RCC_CIR, 16U)

/*Enable PLLI2S ready interrupt */
#define RCC_PLLI2SRDY_IE() SET_BIT(RCC_CIR, 13U)
/*Enable PLL ready interrupt */
#define RCC_PLLRDY_IE() SET_BIT(RCC_CIR, 12U)
/*Enable HSE ready interrupt */
#define RCC_HSERDY_IE() SET_BIT(RCC_CIR, 11U)
/*Enable HSI ready interrupt */
#define RCC_HSIRDY_IE() SET_BIT(RCC_CIR, 10U)
/*Enable LSE ready interrupt */
#define RCC_LSERDY_IE() SET_BIT(RCC_CIR, 9U)
/*Enable LSI ready interrupt */
#define RCC_LSIRDY_IE() SET_BIT(RCC_CIR, 8U)

/*Disable PLLI2S ready interrupt */
#define RCC_PLLI2SRDY_ID() CLEAR_BIT(RCC_CIR, 13U)
/*Disable PLL ready interrupt */
#define RCC_PLLRDY_ID() CLEAR_BIT(RCC_CIR, 12U)
/*Disable HSE ready interrupt */
#define RCC_HSERDY_ID() CLEAR_BIT(RCC_CIR, 11U)
/*Disable HSI ready interrupt */
#define RCC_HSIRDY_ID() CLEAR_BIT(RCC_CIR, 10U)
/*Disable LSE ready interrupt */
#define RCC_LSERDY_ID() CLEAR_BIT(RCC_CIR, 9U)
/*Disable LSI ready interrupt */
#define RCC_LSIRDY_ID() CLEAR_BIT(RCC_CIR, 8U)

/*Check if PLLI2S ready interrupt flag is set*/
/// @return 1 if interrupt flag is set 0 if not
#define RCC_PLLI2SRDY_IE() READ_BIT(RCC_CIR, 7U)
/*Check if PLLI2S ready interrupt flag is set*/
/// @return 1 if interrupt flag is set 0 if not
#define RCC_PLLI2SRDY_IF() READ_BIT(RCC_CIR, 5U)
/*Check if PLL ready interrupt flag is set*/
/// @return 1 if interrupt flag is set 0 if not
#define RCC_PLLRDY_IF() READ_BIT(RCC_CIR, 4U)
/*Check if HSE ready interrupt flag is set*/
/// @return 1 if interrupt flag is set 0 if not
#define RCC_HSERDY_IF() READ_BIT(RCC_CIR, 3U)
/*Check if HSI ready interrupt flag is set*/
/// @return 1 if interrupt flag is set 0 if not
#define RCC_HSIRDY_IF() READ_BIT(RCC_CIR, 2U)
/*Check if LSE ready interrupt flag is set*/
/// @return 1 if interrupt flag is set 0 if not
#define RCC_LSERDY_IF() READ_BIT(RCC_CIR, 1U)
/*Check if LSI ready interrupt flag is set*/
/// @return 1 if interrupt flag is set 0 if not
#define RCC_LSIRDY_IF() READ_BIT(RCC_CIR, 0U)


/*--------RCC_AHB1RSTR---------*/
/*--------RCC_AHB1RSTR---------*/
#define RCC_RES_DMA2()  SET_BIT(RCC_AHB1RSTR, 22U)
#define RCC_RES_DMA1()  SET_BIT(RCC_AHB1RSTR, 21U)
#define RCC_RES_CRC()   SET_BIT(RCC_AHB1RSTR, 12U)
#define RCC_RES_GPIOH() SET_BIT(RCC_AHB1RSTR, 7U)
#define RCC_RES_GPIOE() SET_BIT(RCC_AHB1RSTR, 4U)
#define RCC_RES_GPIOD() SET_BIT(RCC_AHB1RSTR, 3U)
#define RCC_RES_GPIOC() SET_BIT(RCC_AHB1RSTR, 2U)
#define RCC_RES_GPIOB() SET_BIT(RCC_AHB1RSTR, 1U)
#define RCC_RES_GPIOA() SET_BIT(RCC_AHB1RSTR, 0U)

/*--------RCC_AHB2RSTR---------*/
/*--------RCC_AHB2RSTR---------*/
#define RCC_RES_USB_OTG SET_BIT(RCC_AHB2RSTR, 7U)

/*--------RCC_APB1RSTR---------*/
/*--------RCC_APB1RSTR---------*/
#define RCC_RES_PWR()    SET_BIT(RCC_APB1RSTR, 28U)
#define RCC_RES_I2C3()   SET_BIT(RCC_APB1RSTR, 23U)
#define RCC_RES_I2C2()   SET_BIT(RCC_APB1RSTR, 22U)
#define RCC_RES_I2C1()   SET_BIT(RCC_APB1RSTR, 21U)
#define RCC_RES_USART2() SET_BIT(RCC_APB1RSTR, 17U)
#define RCC_RES_SPI3()   SET_BIT(RCC_APB1RSTR, 15U)
#define RCC_RES_SPI2()   SET_BIT(RCC_APB1RSTR, 14U)
#define RCC_RES_WWDG()   SET_BIT(RCC_APB1RSTR, 11U)
#define RCC_RES_TIM5()   SET_BIT(RCC_APB1RSTR, 3U)
#define RCC_RES_TIM4()   SET_BIT(RCC_APB1RSTR, 2U)
#define RCC_RES_TIM3()   SET_BIT(RCC_APB1RSTR, 1U)
#define RCC_RES_TIM2()   SET_BIT(RCC_APB1RSTR, 0U)

/*--------RCC_APB2RSTR---------*/
/*--------RCC_APB2RSTR---------*/
#define RCC_RES_TIM11()  SET_BIT(RCC_APB2RSTR, 18U)
#define RCC_RES_TIM10()  SET_BIT(RCC_APB2RSTR, 17U)
#define RCC_RES_TIM9()   SET_BIT(RCC_APB2RSTR, 16U)
#define RCC_RES_SYSGFG() SET_BIT(RCC_APB2RSTR, 14U)
#define RCC_RES_SPI4()   SET_BIT(RCC_APB2RSTR, 13U)
#define RCC_RES_SPI1()   SET_BIT(RCC_APB2RSTR, 12U)
#define RCC_RES_SDIO()   SET_BIT(RCC_APB2RSTR, 11U)
#define RCC_RES_ADC1()   SET_BIT(RCC_APB2RSTR, 8U)
#define RCC_RES_USART6() SET_BIT(RCC_APB2RSTR, 5U)
#define RCC_RES_USART1() SET_BIT(RCC_APB2RSTR, 4U)
#define RCC_RES_TIM1()   SET_BIT(RCC_APB2RSTR, 0U)


/*--------RCC_AHB1ENR---------*/
/*--------RCC_AHB1ENR---------*/
#define RCC_EN_DMA2()   SET_BIT(RCC_AHB1ENR, 22U)
#define RCC_EN_DMA1()   SET_BIT(RCC_AHB1ENR, 21U)
#define RCC_EN_CRC()    SET_BIT(RCC_AHB1ENR, 12U)
#define RCC_EN_GPIOH()  SET_BIT(RCC_AHB1ENR, 7U)
#define RCC_EN_GPIOE()  SET_BIT(RCC_AHB1ENR, 4U)
#define RCC_EN_GPIOD()  SET_BIT(RCC_AHB1ENR, 3U)
#define RCC_EN_GPIOC()  SET_BIT(RCC_AHB1ENR, 2U)
#define RCC_EN_GPIOB()  SET_BIT(RCC_AHB1ENR, 1U)
#define RCC_EN_GPIOA()  SET_BIT(RCC_AHB1ENR, 0U)
#define RCC_DIS_DMA2()  CLEAR_BIT(RCC_AHB1ENR, 22U)
#define RCC_DIS_DMA1()  CLEAR_BIT(RCC_AHB1ENR, 21U)
#define RCC_DIS_CRC()   CLEAR_BIT(RCC_AHB1ENR, 12U)
#define RCC_DIS_GPIOH() CLEAR_BIT(RCC_AHB1ENR, 7U)
#define RCC_DIS_GPIOE() CLEAR_BIT(RCC_AHB1ENR, 4U)
#define RCC_DIS_GPIOD() CLEAR_BIT(RCC_AHB1ENR, 3U)
#define RCC_DIS_GPIOC() CLEAR_BIT(RCC_AHB1ENR, 2U)
#define RCC_DIS_GPIOB() CLEAR_BIT(RCC_AHB1ENR, 1U)
#define RCC_DIS_GPIOA() CLEAR_BIT(RCC_AHB1ENR, 0U)

/*--------RCC_AHB2ENR---------*/
/*--------RCC_AHB2ENR---------*/
#define RCC_EN_USB_OTG SET_BIT(RCC_AHB2ENR, 7U)
#define RCC_DIS_USB_OTG CLEAR_BIT(RCC_AHB2ENR, 7U)

/*--------RCC_APB1ENR---------*/
/*--------RCC_APB1ENR---------*/
#define RCC_EN_PWR()     SET_BIT(RCC_APB1ENR, 28U)
#define RCC_EN_I2C3()    SET_BIT(RCC_APB1ENR, 23U)
#define RCC_EN_I2C2()    SET_BIT(RCC_APB1ENR, 22U)
#define RCC_EN_I2C1()    SET_BIT(RCC_APB1ENR, 21U)
#define RCC_EN_USART2()  SET_BIT(RCC_APB1ENR, 17U)
#define RCC_EN_SPI3()    SET_BIT(RCC_APB1ENR, 15U)
#define RCC_EN_SPI2()    SET_BIT(RCC_APB1ENR, 14U)
#define RCC_EN_WWDG()    SET_BIT(RCC_APB1ENR, 11U)
#define RCC_EN_TIM5()    SET_BIT(RCC_APB1ENR, 3U)
#define RCC_EN_TIM4()    SET_BIT(RCC_APB1ENR, 2U)
#define RCC_EN_TIM3()    SET_BIT(RCC_APB1ENR, 1U)
#define RCC_EN_TIM2()    SET_BIT(RCC_APB1ENR, 0U)
#define RCC_DIS_PWR()    CLEAR_BIT(RCC_APB1ENR, 28U)
#define RCC_DIS_I2C3()   CLEAR_BIT(RCC_APB1ENR, 23U)
#define RCC_DIS_I2C2()   CLEAR_BIT(RCC_APB1ENR, 22U)
#define RCC_DIS_I2C1()   CLEAR_BIT(RCC_APB1ENR, 21U)
#define RCC_DIS_USART2() CLEAR_BIT(RCC_APB1ENR, 17U)
#define RCC_DIS_SPI3()   CLEAR_BIT(RCC_APB1ENR, 15U)
#define RCC_DIS_SPI2()   CLEAR_BIT(RCC_APB1ENR, 14U)
#define RCC_DIS_WWDG()   CLEAR_BIT(RCC_APB1ENR, 11U)
#define RCC_DIS_TIM5()   CLEAR_BIT(RCC_APB1ENR, 3U)
#define RCC_DIS_TIM4()   CLEAR_BIT(RCC_APB1ENR, 2U)
#define RCC_DIS_TIM3()   CLEAR_BIT(RCC_APB1ENR, 1U)
#define RCC_DIS_TIM2()   CLEAR_BIT(RCC_APB1ENR, 0U)

/*--------RCC_APB2ENR---------*/
/*--------RCC_APB2ENR---------*/
#define RCC_EN_TIM11()   SET_BIT(RCC_APB2ENR, 18U)
#define RCC_EN_TIM10()   SET_BIT(RCC_APB2ENR, 17U)
#define RCC_EN_TIM9()    SET_BIT(RCC_APB2ENR, 16U)
#define RCC_EN_SYSGFG()  SET_BIT(RCC_APB2ENR, 14U)
#define RCC_EN_SPI4()    SET_BIT(RCC_APB2ENR, 13U)
#define RCC_EN_SPI1()    SET_BIT(RCC_APB2ENR, 12U)
#define RCC_EN_SDIO()    SET_BIT(RCC_APB2ENR, 11U)
#define RCC_EN_ADC1()    SET_BIT(RCC_APB2ENR, 8U)
#define RCC_EN_USART6()  SET_BIT(RCC_APB2ENR, 5U)
#define RCC_EN_USART1()  SET_BIT(RCC_APB2ENR, 4U)
#define RCC_EN_TIM1()    SET_BIT(RCC_APB2ENR, 0U)
#define RCC_DIS_TIM11()  CLEAR_BIT(RCC_APB2ENR, 18U)
#define RCC_DIS_TIM10()  CLEAR_BIT(RCC_APB2ENR, 17U)
#define RCC_DIS_TIM9()   CLEAR_BIT(RCC_APB2ENR, 16U)
#define RCC_DIS_SYSGFG() CLEAR_BIT(RCC_APB2ENR, 14U)
#define RCC_DIS_SPI4()   CLEAR_BIT(RCC_APB2ENR, 13U)
#define RCC_DIS_SPI1()   CLEAR_BIT(RCC_APB2ENR, 12U)
#define RCC_DIS_SDIO()   CLEAR_BIT(RCC_APB2ENR, 11U)
#define RCC_DIS_ADC1()   CLEAR_BIT(RCC_APB2ENR, 8U)
#define RCC_DIS_USART6() CLEAR_BIT(RCC_APB2ENR, 5U)
#define RCC_DIS_USART1() CLEAR_BIT(RCC_APB2ENR, 4U)
#define RCC_DIS_TIM1()   CLEAR_BIT(RCC_APB2ENR, 0U)

/*--------RCC_AHB1LPENR---------*/
/*--------RCC_AHB1LPENR---------*/
#define RCC_LPEN_DMA2()  SET_BIT(RCC_AHB1LPENR, 22U)
#define RCC_LPEN_DMA1()  SET_BIT(RCC_AHB1LPENR, 21U)
#define RCC_LPEN_SRAM1() SET_BIT(RCC_AHB1LPENR, 16U)
#define RCC_LPEN_FLITF() SET_BIT(RCC_AHB1LPENR, 15U)
#define RCC_LPEN_CRC()   SET_BIT(RCC_AHB1LPENR, 12U)
#define RCC_LPEN_GPIOH() SET_BIT(RCC_AHB1LPENR, 7U)
#define RCC_LPEN_GPIOE() SET_BIT(RCC_AHB1LPENR, 4U)
#define RCC_LPEN_GPIOD() SET_BIT(RCC_AHB1LPENR, 3U)
#define RCC_LPEN_GPIOC() SET_BIT(RCC_AHB1LPENR, 2U)
#define RCC_LPEN_GPIOB() SET_BIT(RCC_AHB1LPENR, 1U)
#define RCC_LPEN_GPIOA() SET_BIT(RCC_AHB1LPENR, 0U)
#define RCC_LPDIS_DMA2()  CLEAR_BIT(RCC_AHB1LPENR, 22U)
#define RCC_LPDIS_DMA1()  CLEAR_BIT(RCC_AHB1LPENR, 21U)
#define RCC_LPDIS_SRAM1() CLEAR_BIT(RCC_AHB1LPENR, 16U)
#define RCC_LPDIS_FLITF() CLEAR_BIT(RCC_AHB1LPENR, 15U)
#define RCC_LPDIS_CRC()   CLEAR_BIT(RCC_AHB1LPENR, 12U)
#define RCC_LPDIS_GPIOH() CLEAR_BIT(RCC_AHB1LPENR, 7U)
#define RCC_LPDIS_GPIOE() CLEAR_BIT(RCC_AHB1LPENR, 4U)
#define RCC_LPDIS_GPIOD() CLEAR_BIT(RCC_AHB1LPENR, 3U)
#define RCC_LPDIS_GPIOC() CLEAR_BIT(RCC_AHB1LPENR, 2U)
#define RCC_LPDIS_GPIOB() CLEAR_BIT(RCC_AHB1LPENR, 1U)
#define RCC_LPDIS_GPIOA() CLEAR_BIT(RCC_AHB1LPENR, 0U)

/*--------RCC_AHB2LPENR---------*/
/*--------RCC_AHB2LPENR---------*/
#define RCC_EN_USB_OTG    SET_BIT(RCC_AHB2LPENR, 7U)
#define RCC_DIS_USB_OTG   CLEAR_BIT(RCC_AHB2LPENR, 7U)

/*--------RCC_APB1LPENR---------*/
/*--------RCC_APB1LPENR---------*/
#define RCC_LPEN_PWR()    SET_BIT(RCC_APB1LPENR, 28U)
#define RCC_LPEN_I2C3()   SET_BIT(RCC_APB1LPENR, 23U)
#define RCC_LPEN_I2C2()   SET_BIT(RCC_APB1LPENR, 22U)
#define RCC_LPEN_I2C1()   SET_BIT(RCC_APB1LPENR, 21U)
#define RCC_LPEN_USART2() SET_BIT(RCC_APB1LPENR, 17U)
#define RCC_LPEN_SPI3()   SET_BIT(RCC_APB1LPENR, 15U)
#define RCC_LPEN_SPI2()   SET_BIT(RCC_APB1LPENR, 14U)
#define RCC_LPEN_WWDG()   SET_BIT(RCC_APB1LPENR, 11U)
#define RCC_LPEN_TIM5()   SET_BIT(RCC_APB1LPENR, 3U)
#define RCC_LPEN_TIM4()   SET_BIT(RCC_APB1LPENR, 2U)
#define RCC_LPEN_TIM3()   SET_BIT(RCC_APB1LPENR, 1U)
#define RCC_LPEN_TIM2()   SET_BIT(RCC_APB1LPENR, 0U)
#define RCC_LPDIS_PWR()    CLEAR_BIT(RCC_APB1LPENR, 28U)
#define RCC_LPDIS_I2C3()   CLEAR_BIT(RCC_APB1LPENR, 23U)
#define RCC_LPDIS_I2C2()   CLEAR_BIT(RCC_APB1LPENR, 22U)
#define RCC_LPDIS_I2C1()   CLEAR_BIT(RCC_APB1LPENR, 21U)
#define RCC_LPDIS_USART2() CLEAR_BIT(RCC_APB1LPENR, 17U)
#define RCC_LPDIS_SPI3()   CLEAR_BIT(RCC_APB1LPENR, 15U)
#define RCC_LPDIS_SPI2()   CLEAR_BIT(RCC_APB1LPENR, 14U)
#define RCC_LPDIS_WWDG()   CLEAR_BIT(RCC_APB1LPENR, 11U)
#define RCC_LPDIS_TIM5()   CLEAR_BIT(RCC_APB1LPENR, 3U)
#define RCC_LPDIS_TIM4()   CLEAR_BIT(RCC_APB1LPENR, 2U)
#define RCC_LPDIS_TIM3()   CLEAR_BIT(RCC_APB1LPENR, 1U)
#define RCC_LPDIS_TIM2()   CLEAR_BIT(RCC_APB1LPENR, 0U)

/*--------RCC_APB2LPENR---------*/
/*--------RCC_APB2LPENR---------*/
#define RCC_LPEN_TIM11()   SET_BIT(RCC_APB2LPENR, 18U)
#define RCC_LPEN_TIM10()   SET_BIT(RCC_APB2LPENR, 17U)
#define RCC_LPEN_TIM9()    SET_BIT(RCC_APB2LPENR, 16U)
#define RCC_LPEN_SYSGFG()  SET_BIT(RCC_APB2LPENR, 14U)
#define RCC_LPEN_SPI4()    SET_BIT(RCC_APB2LPENR, 13U)
#define RCC_LPEN_SPI1()    SET_BIT(RCC_APB2LPENR, 12U)
#define RCC_LPEN_SDIO()    SET_BIT(RCC_APB2LPENR, 11U)
#define RCC_LPEN_ADC1()    SET_BIT(RCC_APB2LPENR, 8U)
#define RCC_LPEN_USART6()  SET_BIT(RCC_APB2LPENR, 5U)
#define RCC_LPEN_USART1()  SET_BIT(RCC_APB2LPENR, 4U)
#define RCC_LPEN_TIM1()    SET_BIT(RCC_APB2LPENR, 0U)
#define RCC_LPDIS_TIM11()  CLEAR_BIT(RCC_APB2LPENR, 18U)
#define RCC_LPDIS_TIM10()  CLEAR_BIT(RCC_APB2LPENR, 17U)
#define RCC_LPDIS_TIM9()   CLEAR_BIT(RCC_APB2LPENR, 16U)
#define RCC_LPDIS_SYSGFG() CLEAR_BIT(RCC_APB2LPENR, 14U)
#define RCC_LPDIS_SPI4()   CLEAR_BIT(RCC_APB2LPENR, 13U)
#define RCC_LPDIS_SPI1()   CLEAR_BIT(RCC_APB2LPENR, 12U)
#define RCC_LPDIS_SDIO()   CLEAR_BIT(RCC_APB2LPENR, 11U)
#define RCC_LPDIS_ADC1()   CLEAR_BIT(RCC_APB2LPENR, 8U)
#define RCC_LPDIS_USART6() CLEAR_BIT(RCC_APB2LPENR, 5U)
#define RCC_LPDIS_USART1() CLEAR_BIT(RCC_APB2LPENR, 4U)
#define RCC_LPDIS_TIM1()   CLEAR_BIT(RCC_APB2LPENR, 0U)

/*--------RCC_BDCR---------*/
/*--------RCC_BDCR---------*/
#define RCC_RES_BD()      SET_BIT(RCC_BDCR, 16U)
#define RCC_EN_RTC_CLK()  SET_BIT(RCC_BDCR, 15U)
#define RCC_DIS_RTC_CLK() CLEAR_BIT(RCC_BDCR, 15U)
#define RCC_SEL_RTC_SRC(SOURCE) \
  CLEAR_BITS(RCC_BDCR, (0b11U << 8U)) \
  SET_BITS(RCC_BDCR, (SOURCE << 8U))

#define RCC_BYP_LSE()   SET_BIT(RCC_BDCR, 2U)
#define RCC_NOBYP_LSE() CLEAR_BIT(RCC_BDCR, 2U)
#define RCC_RDY_LSE()   READ_BIT(RCC_BDCR, 1U)
#define RCC_EN_LSE()    SET_BIT(RCC_BDCR, 0U)
#define RCC_DIS_LSE()   CLEAR_BIT(RCC_BDCR, 0U)

/*--------RCC_CSR---------*/
/*--------RCC_CSR---------*/

/*get LOW POWER reset flag*/
#define RCC_LPRES_FLAG()  READ_BIT(RCC_CSR, 31U)
/*get WWDG reset flag*/
#define RCC_WWDGRES_FLAG() READ_BIT(RCC_CSR, 30U)
/*get IWDG reset flag*/
#define RCC_IWDGRES_FLAG() READ_BIT(RCC_CSR, 29U)
/*get SOFTWARE reset flag*/
#define RCC_SFTRES_FLAG() READ_BIT(RCC_CSR, 28U)
/*get POR/PDR reset flag*/
#define RCC_PORRES_FLAG() READ_BIT(RCC_CSR, 27U)
/*get PIN reset flag*/
#define RCC_PINRES_FLAG() READ_BIT(RCC_CSR, 26U)
/*get BOR reset flag*/
#define RCC_BORRES_FLAG() READ_BIT(RCC_CSR, 25U)
/*Clear all reset flags*/
#define RCC_RMRES_FLAGS() SET_BIT(RCC_CSR, 24U)
#define RCC_RDY_HSI()     READ_BIT(RCC_CSR, 1U)
#define RCC_EN_HSI()      SET_BIT(RCC_CSR, 0U)
#define RCC_DIS_HSI()     CLEAR_BIT(RCC_CSR, 0U)


/*--------RCC_SSCGR---------*/
/*--------RCC_SSCGR---------*/
/*
The spread spectrum clock generation is available only for the main PLL
*/
/*Spread spectrum modulation enable*/
#define RCC_EN_SSM() SET_BIT(RCC_SSCGR, 31U)
/*Spread spectrum modulation disable*/
#define RCC_DIS_SSM() CLEAR_BIT(RCC_SSCGR, 31U)
/*Select Center spread*/
#define RCC_CENTER_SPREAD() CLEAR_BIT(RCC_SSCGR, 30U)
/*Select Down spread*/
#define RCC_DOWN_SPREAD() SET_BIT(RCC_SSCGR, 30U)

/*Set Incrementation step, Configuration input for modulation profile amplitude.*/
#define RCC_SET_INCSTEP(INCSTEP) \
  CLEAR_BITS(RCC_SSCGR, 0x7FFFU << 13U) \
  SET_BITS(RCC_SSCGR, INCSTEP << 13U)

/*Modulation period, Configuration input for modulation profile period.*/
#define RCC_SET_MODPER(MODPER) \
  CLEAR_BITS(RCC_SSCGR, 0x1FFFU) \
  SET_BITS(RCC_SSCGR, MODPER)
s
/*--------RCC_PLLI2SCFGR---------*/
/*--------RCC_PLLI2SCFGR---------*/

// f(VCO clock) = f(PLLI2S clock input) × (PLLI2SN / PLLM)
// f(PLL I2S clock output) = f(VCO clock) / PLLI2SR

// I2S clock frequency = VCO frequency / PLLR with 2 ≤ PLLR ≤ 7
#define RCC_SET_PLLI2SR(PRE) \
  CLEAR_BITS(RCC_PLLI2SCFGR, (7U << 28U)) \
  SET_BITS(RCC_PLLI2SCFGR, (PRE << 28U))

// VCO output frequency = VCO input frequency × PLLI2SN with 192 ≤ PLLI2SN ≤ 432
#define RCC_SET_PLLI2SN(PRE) \
  CLEAR_BITS(RCC_PLLI2SCFGR, (511U << 6U)) \
  SET_BITS(RCC_PLLI2SCFGR, (PRE << 6U))


/*--------RCC_DCKCFGR---------*/
/*--------RCC_DCKCFGR---------*/
#define RCC_SET_TIMERPRE()    SET_BIT(RCC_DCKCFGR, 24U) 
#define RCC_CLEAR_TIMERPRE()  CLEAR_BIT(RCC_DCKCFGR, 24U) 

#endif // !RCC_HELPERS

/*=============================================================================*/
/*=============================================================================*/

#ifndef FLASH_HELPERS 1


#endif // !FLASH_HELPERS 1

/*=============================================================================*/
/*=============================================================================*/

#ifndef PWR_HELPERS 1


#endif // !PWR_HELPERS 1

/*=============================================================================*/
/*=============================================================================*/

#ifndef GPIO_HELPERS 1


#endif // !GPIO_HELPERS 1

/*=============================================================================*/
/*=============================================================================*/
