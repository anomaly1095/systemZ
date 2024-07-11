#include <stdint.h>

#ifndef BIT_MANIP 1

/*Macros to manipulate bits*/

/* Clear the bits with MASK and set new vbits with VALUE in the register REG*/
#define MODIFY_REG(REG, MASK, VALUE) ((REG) = ((REG) & ~(MASK)) | (VALUE))

/*sets the BIT_NUM bit in the register*/
#define SET_BIT(REG, BIT_NUM)   ((REG) |= (0b1 << BIT_NUM))

/*sets the bits in the register that are set in MASK*/
#define SET_BITS(REG, MASK)   ((REG) |= (MASK))

/*reads the bit from the register and return boolean value*/
#define READ_BIT(REG, BIT_NUM)  (((REG) & (1U << (BIT_NUM))) != 0)

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

/*RCC registers*/
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

/* Macros for RCC register manipulation */
#define RCC_PLLI2S_ENABLE()   (SET_BIT  (RCC_CR, 26U))
#define RCC_PLLI2S_DISABLE()  (CLEAR_BIT(RCC_CR, 26U))
#define RCC_PLLI2S_READY()    (READ_BIT (RCC_CR, 27U))

#define RCC_PLL_ENABLE()      (SET_BIT  (RCC_CR, 24U))
#define RCC_PLL_DISABLE()     (CLEAR_BIT(RCC_CR, 24U))
#define RCC_PLL_READY()       (READ_BIT (RCC_CR, 25U))

#define RCC_CSS_ENABLE()      (SET_BIT  (RCC_CR, 19U))
#define RCC_CSS_DISABLE()     (CLEAR_BIT(RCC_CR, 19U))

#define RCC_HSE_ENABLE()      (SET_BIT  (RCC_CR, 16U))
#define RCC_HSE_DISABLE()     (CLEAR_BIT(RCC_CR, 16U))
#define RCC_HSE_READY()       (READ_BIT (RCC_CR, 17U))
#define RCC_HSE_BYPASS()      (SET_BIT  (RCC_CR, 18U))
#define RCC_HSE_NOBYPASS()    (CLEAR_BIT(RCC_CR, 18U))

#define RCC_HSI_ENABLE()      (SET_BIT  (RCC_CR, 0U))
#define RCC_HSI_DISABLE()     (CLEAR_BIT(RCC_CR, 0U))
#define RCC_HSI_READY()       (READ_BIT (RCC_CR, 1U))

/* PLLSRC = HSI, PLLN = 336, PLLP = 4, PLLM = 16 */ 
#define RCC_PLL_84MHz() \
  MODIFY_REG(RCC_PLLCFGR, \
    0x1FFCFFFFU,    /* Reset PLLN and PLLP bits */ \
    (1U << 22U) | (336U << 6U) | (4U << 16U) | (16U << 0U)); \

/* PLLSRC = HSI, PLLN = 336, PLLP = 8, PLLM = 16 */
#define RCC_PLL_42MHz() \
  MODIFY_REG(RCC_PLLCFGR, \
    0x1FFCFFFFU,    /* Reset PLLN and PLLP bits */ \
    (1U << 22U) | (336U << 6U) | (8U << 16U) | (16U << 0U)); \

/* PLLSRC = HSI, PLLN = 256, PLLP = 4, PLLM = 16 */
#define RCC_PLL_64MHz() \
  MODIFY_REG(RCC_PLLCFGR, \
    0x1FFCFFFFU,    /* Reset PLLN and PLLP bits */ \
    (1U << 22U) | (256U << 6U) | (4U << 16U) | (16U << 0U)); \

/* PLLSRC = HSI, PLLN = 432, PLLP = 6, PLLM = 16 */
#define RCC_PLL_72MHz() \
  MODIFY_REG(RCC_PLLCFGR, \
    0x1FFCFFFFU,    /* Reset PLLN and PLLP bits */ \
    (1U << 22U) | (432U << 6U) | (6U << 16U) | (16U << 0U)); \

/// @param SOURCE Is the source of output in Microcontroller clock output 2
/// 0b00: SYSCLK / 0b01: PLLI2S / 0b10: HSE / 0b11: PLL
#define RCC_MCO2_SRC(SOURCE) \
  CLEAR_BITS(RCC_CFGR, 0b11 << 30) \ 
  SET_BITS(RCC_CFGR, SOURCE << 30)

/// @param PRE is the prescaler to apply on the signal of Microcontroller clock output 2
/// 0xx: no division / 0b100: division by 2 / 0b101: division by 3 /
// 0xx: no division
// 100: division by 2
// 101: division by 3
// 110: division by 4
// 111: division by 5
#define RCC_MCO2_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b111 << 27) \ 
  SET_BITS(RCC_CFGR, PRE << 27)

/// @param SOURCE Is the source of output in Microcontroller clock output 1
// 00: HSI clock selected
// 01: LSE oscillator selected
// 10: HSE oscillator clock selected
// 11: PLL clock selected
#define RCC_MCO1_SRC(SOURCE) \
  CLEAR_BITS(RCC_CFGR, 0b11 << 21) \ 
  SET_BITS(RCC_CFGR, SOURCE << 21)

/// @param SOURCE Is the source of inter integrated sound clock
// 0: PLLI2S clock used as I2S clock source
// 1: External clock mapped on the I2S_CKIN pin used as I2S clock source
#define RCC_MCO1_SRC(SOURCE) \
  CLEAR_BITS(RCC_CFGR, 0b1 << 23) \ 
  SET_BITS(RCC_CFGR, SOURCE << 23)

/// @param PRE is the prescaler to apply on the signal of Microcontroller clock output 1
/// 0xx: no division / 0b100: division by 2 / 0b101: division by 3 /
// 0xx: no division
// 100: division by 2
// 101: division by 3
// 110: division by 4
// 111: division by 5
#define RCC_MCO1_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b111 << 24) \ 
  SET_BITS(RCC_CFGR, PRE << 24)

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
  CLEAR_BITS(RCC_CFGR, 0b11111 << 16) \ 
  SET_BITS(RCC_CFGR, PRE << 16)

/// @param PRE is the prescaler to apply on the APB2 
// 0xx: AHB clock not divided
// 100: AHB clock divided by 2
// 101: AHB clock divided by 4
// 110: AHB clock divided by 8
// 111: AHB clock divided by 16
#define RCC_APB2_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b111 << 13) \ 
  SET_BITS(RCC_CFGR, PRE << 13)

/// @param PRE is the prescaler to apply on the APB1
// 0xx: AHB clock not divided
// 100: AHB clock divided by 2
// 101: AHB clock divided by 4
// 110: AHB clock divided by 8
// 111: AHB clock divided by 16
#define RCC_APB1_PRE(PRE) \
  CLEAR_BITS(RCC_CFGR, 0b111 << 10) \ 
  SET_BITS(RCC_CFGR, PRE << 10)

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
  CLEAR_BITS(RCC_CFGR, 0b1111 << 4) \ 
  SET_BITS(RCC_CFGR, PRE << 4)


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
  CLEAR_BITS(RCC_CFGR, 0b11) \ 
  SET_BITS(RCC_CFGR, SOURCE)

// write the RCC_CIR 
// write RCC_AHB1RSTR RCC_AHB2RSTR RCC_APB1RSTR RCC_APB2RSTR
// write RCC_AHB1ENR RCC_AHB2ENR RCC_APB1ENR RCC_APB2ENR 
// write RCC_AHB1LPENR RCC_AHB2LPENR RCC_APB1LPENR RCC_APB2LPENR
// write RCC_BDCR
#endif // !RCC_HELPERS

/*=============================================================================*/
