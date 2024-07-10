
#ifndef WRAPPERS_H  1
  #include <stdint.h>
typedef enum IRQ_num_t {
  /* External Interrupts */
  WWDG_IRQ = 0,
  PVD_IRQ = 1,
  TAMP_STAMP_IRQ = 2,
  RTC_WKUP_IRQ = 3,
  FLASH_IRQ = 4,
  RCC_IRQ = 5,
  EXTI0_IRQ = 6,
  EXTI1_IRQ = 7,
  EXTI2_IRQ = 8,
  EXTI3_IRQ = 9,
  EXTI4_IRQ = 10,
  DMA1_Stream0_IRQ = 11,
  DMA1_Stream1_IRQ = 12,
  DMA1_Stream2_IRQ = 13,
  DMA1_Stream3_IRQ = 14,
  DMA1_Stream4_IRQ = 15,
  DMA1_Stream5_IRQ = 16,
  DMA1_Stream6_IRQ = 17,
  ADC_IRQ = 18,
  EXTI9_5_IRQ = 23,
  TIM1_BRK_TIM9_IRQ = 24,
  TIM1_UP_TIM10_IRQ = 25,
  TIM1_TRG_COM_TIM11_IRQ = 26,
  TIM1_CC_IRQ = 27,
  TIM2_IRQ = 28,
  TIM3_IRQ = 29,
  TIM4_IRQ = 30,
  I2C1_EV_IRQ = 31,
  I2C1_ER_IRQ = 32,
  I2C2_EV_IRQ = 33,
  I2C2_ER_IRQ = 34,
  SPI1_IRQ = 35,
  SPI2_IRQ = 36,
  USART1_IRQ = 37,
  USART2_IRQ = 38,
  EXTI15_10_IRQ = 40,
  RTC_Alarm_IRQ = 41,
  OTG_FS_WKUP_IRQ = 42,
  DMA1_Stream7_IRQ = 55,
  SDIO_IRQ = 56,
  TIM5_IRQ = 50,
  SPI3_IRQ = 51,
  DMA2_Stream0_IRQ = 56,
  DMA2_Stream1_IRQ = 57,
  DMA2_Stream2_IRQ = 58,
  DMA2_Stream3_IRQ = 59,
  DMA2_Stream4_IRQ = 60,
  OTG_FS_IRQ = 67,
  DMA2_Stream5_IRQ = 68,
  DMA2_Stream6_IRQ = 69,
  DMA2_Stream7_IRQ = 70,
  USART6_IRQ = 71,
  I2C3_EV_IRQ = 72,
  I2C3_ER_IRQ = 73,
  FPU_IRQ = 81,
  SPI4_IRQ = 84
} IRQ_num_t;

/// @brief Used by apps to trigger an interrupt on the specified interrupt in IRQ_num
/// @brief This function is only accessible from unprevileged apps if
/// @brief SCR reg is set so in page 230 of the stm32-cortex-M4 Referance Manual
/// @param IRQ_num irq number as specified in the enum
extern void _NVIC_soft_trigger_irq(IRQ_num_t IRQ_num);


#endif // !WRAPPERS_H