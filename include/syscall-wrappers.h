
#ifndef SYSCALL_WRAPPERS_H 1
#include "STM32F401.h"

/*---------------------------------------------------------------*/
/*---IRQ numbers as provided by the STM32F401 reference manual---*/
/*---------------------------------------------------------------*/

#ifndef NVIC_H  1

/*List of IRQ numbers should be used ad IRQ_NUM argument in the macros below*/
#define WWDG_IRQ                ((uint8_t)0U)
#define PVD_IRQ                 ((uint8_t)1U)
#define TAMP_STAMP_IRQ          ((uint8_t)2U)
#define RTC_WKUP_IRQ            ((uint8_t)3U)
#define FLASH_IRQ               ((uint8_t)4U)
#define RCC_IRQ                 ((uint8_t)5U)
#define EXTI0_IRQ               ((uint8_t)6U)
#define EXTI1_IRQ               ((uint8_t)7U)
#define EXTI2_IRQ               ((uint8_t)8U)
#define EXTI3_IRQ               ((uint8_t)9U)
#define EXTI4_IRQ               ((uint8_t)10U)
#define DMA1_Stream0_IRQ        ((uint8_t)11U)
#define DMA1_Stream1_IRQ        ((uint8_t)12U)
#define DMA1_Stream2_IRQ        ((uint8_t)13U)
#define DMA1_Stream3_IRQ        ((uint8_t)14U)
#define DMA1_Stream4_IRQ        ((uint8_t)15U)
#define DMA1_Stream5_IRQ        ((uint8_t)16U)
#define DMA1_Stream6_IRQ        ((uint8_t)17U)
#define ADC_IRQ                 ((uint8_t)18U)
#define EXTI9_5_IRQ             ((uint8_t)23U)
#define TIM1_BRK_TIM9_IRQ       ((uint8_t)24U)
#define TIM1_UP_TIM10_IRQ       ((uint8_t)25U)
#define TIM1_TRG_COM_TIM11_IRQ  ((uint8_t)26U)
#define TIM1_CC_IRQ             ((uint8_t)27U)
#define TIM2_IRQ                ((uint8_t)28U)
#define TIM3_IRQ                ((uint8_t)29U)
#define TIM4_IRQ                ((uint8_t)30U)
#define I2C1_EV_IRQ             ((uint8_t)31U)
#define I2C1_ER_IRQ             ((uint8_t)32U)
#define I2C2_EV_IRQ             ((uint8_t)33U)
#define I2C2_ER_IRQ             ((uint8_t)34U)
#define SPI1_IRQ                ((uint8_t)35U)
#define SPI2_IRQ                ((uint8_t)36U)
#define USART1_IRQ              ((uint8_t)37U)
#define USART2_IRQ              ((uint8_t)38U)
#define EXTI15_10_IRQ           ((uint8_t)40U)
#define RTC_Alarm_IRQ           ((uint8_t)41U)
#define OTG_FS_WKUP_IRQ         ((uint8_t)42U)
#define DMA1_Stream7_IRQ        ((uint8_t)55U)
#define SDIO_IRQ                ((uint8_t)56U)
#define TIM5_IRQ                ((uint8_t)50U)
#define SPI3_IRQ                ((uint8_t)51U)
#define DMA2_Stream0_IRQ        ((uint8_t)56U)
#define DMA2_Stream1_IRQ        ((uint8_t)57U)
#define DMA2_Stream2_IRQ        ((uint8_t)58U)
#define DMA2_Stream3_IRQ        ((uint8_t)59U)
#define DMA2_Stream4_IRQ        ((uint8_t)60U)
#define OTG_FS_IRQ              ((uint8_t)67U)
#define DMA2_Stream5_IRQ        ((uint8_t)68U)
#define DMA2_Stream6_IRQ        ((uint8_t)69U)
#define DMA2_Stream7_IRQ        ((uint8_t)70U)
#define USART6_IRQ              ((uint8_t)71U)
#define I2C3_EV_IRQ             ((uint8_t)72U)
#define I2C3_ER_IRQ             ((uint8_t)73U)
#define FPU_IRQ                 ((uint8_t)81U)
#define SPI4_IRQ                ((uint8_t)84U)

#endif // !NVIC_H

uint32_t SVC0(uint8_t IRQ_NUM){__asm__ volatile ("SVC  #0");}
#define NVIC_enable_irq(IRQ_NUM) SVC0(IRQ_NUM)

uint32_t SVC1(uint8_t IRQ_NUM){__asm__ volatile ("SVC  #1");}
#define NVIC_disable_irq(IRQ_NUM) SVC1(IRQ_NUM)

uint32_t SVC2(uint8_t IRQ_NUM){__asm__ volatile ("SVC  #2");}
#define NVIC_set_pend_irq(IRQ_NUM) SVC2(IRQ_NUM)

uint32_t SVC3(uint8_t IRQ_NUM){__asm__ volatile ("SVC  #3");}
#define NVIC_clear_pend_irq(IRQ_NUM) SVC3(IRQ_NUM)

uint32_t SVC4(uint8_t IRQ_NUM){__asm__ volatile ("SVC  #4");}
#define NVIC_check_active_irq(IRQ_NUM) SVC4(IRQ_NUM)

uint32_t SVC5(uint8_t IRQ_NUM){__asm__ volatile ("SVC  #5");}
#define NVIC_set_pri_irq(IRQ_NUM) SVC5(IRQ_NUM)

uint32_t SVC6(uint8_t IRQ_NUM){__asm__ volatile ("SVC  #6");}
#define NVIC_get_pri_irq(IRQ_NUM) SVC6(IRQ_NUM)

uint32_t SVC7(uint8_t IRQ_NUM){__asm__ volatile ("SVC  #7");}
#define NVIC_soft_trigger_irq(IRQ_NUM) SVC7(IRQ_NUM)

#endif // !SYSCALL_WRAPPERS_H 1