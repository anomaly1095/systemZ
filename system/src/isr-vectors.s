
@ # SystemZ Kernel <PRODUCTION BRANCH>

@ Copyright (C) 2024 Connexion Nord, Inc. or its affiliates. All Rights Reserved.

@ SPDX-License-Identifier: MIT

@ Permission is hereby granted, free of charge, to any person obtaining a copy of
@ this software and associated documentation files (the "Software"), to deal in
@ the Software without restriction, including without limitation the rights to
@ use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
@ the Software, and to permit persons to whom the Software is furnished to do so,
@ subject to the following conditions:

@ The above copyright notice and this permission notice shall be included in all
@ copies or substantial portions of the Software.

@ THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
@ IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
@ FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
@ COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
@ IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
@ CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

@ <https://github.com/anomaly1095/systemZ>
@ Author: Youssef Azaiez

.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

@----------------------------------------------
// system interrupt vector table for stm32F401xx
  .section .isr_vectors, "a", %progbits
  .type  g_pfnVectors, %object
  .extern _reset_handler
  .extern _default_handler
@----------------------------------------------
 
g_pfnVectors:
  .word  _ekstack     @ end of kernel stack (lowest address)  
  .word  _reset_handler @ initial address of PC
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  
  /* External Interrupts */
  .word     WWDG_IRQHandler
  .word     PVD_IRQHandler
  .word     TAMP_STAMP_IRQHandler
  .word     RTC_WKUP_IRQHandler
  .word     FLASH_IRQHandler                  
  .word     RCC_IRQHandler                    
  .word     EXTI0_IRQHandler
  .word     EXTI1_IRQHandler 
  .word     EXTI2_IRQHandler 
  .word     EXTI3_IRQHandler 
  .word     EXTI4_IRQHandler 
  .word     DMA1_Stream0_IRQHandler 
  .word     DMA1_Stream1_IRQHandler 
  .word     DMA1_Stream2_IRQHandler 
  .word     DMA1_Stream3_IRQHandler 
  .word     DMA1_Stream4_IRQHandler 
  .word     DMA1_Stream5_IRQHandler 
  .word     DMA1_Stream6_IRQHandler 
  .word     ADC_IRQHandler
  .word     0               				  
  .word     0              					  
  .word     0                                 
  .word     0                                 
  .word     EXTI9_5_IRQHandler
  .word     TIM1_BRK_TIM9_IRQHandler
  .word     TIM1_UP_TIM10_IRQHandler
  .word     TIM1_TRG_COM_TIM11_IRQHandler
  .word     TIM1_CC_IRQHandler
  .word     TIM2_IRQHandler
  .word     TIM3_IRQHandler
  .word     TIM4_IRQHandler
  .word     I2C1_EV_IRQHandler
  .word     I2C1_ER_IRQHandler
  .word     I2C2_EV_IRQHandler
  .word     I2C2_ER_IRQHandler
  .word     SPI1_IRQHandler
  .word     SPI2_IRQHandler
  .word     USART1_IRQHandler
  .word     USART2_IRQHandler
  .word     0               				  
  .word     EXTI15_10_IRQHandler
  .word     RTC_Alarm_IRQHandler
  .word     OTG_FS_WKUP_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     DMA1_Stream7_IRQHandler
  .word     0           
  .word     SDIO_IRQHandler
  .word     TIM5_IRQHandler
  .word     SPI3_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     DMA2_Stream0_IRQHandler
  .word     DMA2_Stream1_IRQHandler
  .word     DMA2_Stream2_IRQHandler
  .word     DMA2_Stream3_IRQHandler
  .word     DMA2_Stream4_IRQHandler
  .word     0		  
  .word     0	  
  .word     0	  
  .word     0	  
  .word     0	  
  .word     0	  
  .word     OTG_FS_IRQHandler
  .word     DMA2_Stream5_IRQHandler 
  .word     DMA2_Stream6_IRQHandler 
  .word     DMA2_Stream7_IRQHandler 
  .word     USART6_IRQHandler
  .word     I2C3_EV_IRQHandler
  .word     I2C3_ER_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     0
  .word     0
  .word     0
  .word     FPU_IRQHandler
  .word     0
  .word     0
  .word     SPI4_IRQHandler

  .size  g_pfnVectors, .-g_pfnVectors

@  isr vectors will be overrided by _default_handler or by a stronger definition of the isr 

  .weak      NMI_Handler

  .weak      HardFault_Handler

  .weak      MemManage_Handler

  .weak      BusFault_Handler

  .weak      UsageFault_Handler

  .weak      SVC_Handler

  .weak      DebugMon_Handler

  .weak      PendSV_Handler

  .weak      SysTick_Handler

  .weak      WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler, _default_handler
                
  .weak      PVD_IRQHandler
  .thumb_set PVD_IRQHandler, _default_handler
            
  .weak      TAMP_STAMP_IRQHandler
  .thumb_set TAMP_STAMP_IRQHandler, _default_handler
          
  .weak      RTC_WKUP_IRQHandler
  .thumb_set RTC_WKUP_IRQHandler, _default_handler
          
  .weak      FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler, _default_handler
                
  .weak      RCC_IRQHandler
  .thumb_set RCC_IRQHandler, _default_handler
                
  .weak      EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler, _default_handler
                
  .weak      EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler, _default_handler
                  
  .weak      EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler, _default_handler
              
  .weak      EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler, _default_handler
                      
  .weak      EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler, _default_handler
                
  .weak      DMA1_Stream0_IRQHandler
  .thumb_set DMA1_Stream0_IRQHandler, _default_handler
      
  .weak      DMA1_Stream1_IRQHandler
  .thumb_set DMA1_Stream1_IRQHandler, _default_handler
                
  .weak      DMA1_Stream2_IRQHandler
  .thumb_set DMA1_Stream2_IRQHandler, _default_handler
                
  .weak      DMA1_Stream3_IRQHandler
  .thumb_set DMA1_Stream3_IRQHandler, _default_handler
              
  .weak      DMA1_Stream4_IRQHandler
  .thumb_set DMA1_Stream4_IRQHandler, _default_handler
                
  .weak      DMA1_Stream5_IRQHandler
  .thumb_set DMA1_Stream5_IRQHandler, _default_handler
                
  .weak      DMA1_Stream6_IRQHandler
  .thumb_set DMA1_Stream6_IRQHandler, _default_handler
                
  .weak      ADC_IRQHandler
  .thumb_set ADC_IRQHandler, _default_handler
          
  .weak      EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler, _default_handler
          
  .weak      TIM1_BRK_TIM9_IRQHandler
  .thumb_set TIM1_BRK_TIM9_IRQHandler, _default_handler
          
  .weak      TIM1_UP_TIM10_IRQHandler
  .thumb_set TIM1_UP_TIM10_IRQHandler, _default_handler
    
  .weak      TIM1_TRG_COM_TIM11_IRQHandler
  .thumb_set TIM1_TRG_COM_TIM11_IRQHandler, _default_handler
    
  .weak      TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler, _default_handler
                
  .weak      TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler, _default_handler
                
  .weak      TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler, _default_handler
                
  .weak      TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler, _default_handler
                
  .weak      I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler, _default_handler
                  
  .weak      I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler, _default_handler
                  
  .weak      I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler, _default_handler
                
  .weak      I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler, _default_handler
                        
  .weak      SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler, _default_handler
                      
  .weak      SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler, _default_handler
                
  .weak      USART1_IRQHandler
  .thumb_set USART1_IRQHandler, _default_handler
                  
  .weak      USART2_IRQHandler
  .thumb_set USART2_IRQHandler, _default_handler

  .weak      EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler, _default_handler

  .weak      RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler, _default_handler
          
  .weak      OTG_FS_WKUP_IRQHandler
  .thumb_set OTG_FS_WKUP_IRQHandler, _default_handler
          
  .weak      DMA1_Stream7_IRQHandler
  .thumb_set DMA1_Stream7_IRQHandler, _default_handler
                  
  .weak      SDIO_IRQHandler
  .thumb_set SDIO_IRQHandler, _default_handler
                  
  .weak      TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler, _default_handler
                  
  .weak      SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler, _default_handler
                  
  .weak      DMA2_Stream0_IRQHandler
  .thumb_set DMA2_Stream0_IRQHandler, _default_handler
            
  .weak      DMA2_Stream1_IRQHandler
  .thumb_set DMA2_Stream1_IRQHandler, _default_handler
                
  .weak      DMA2_Stream2_IRQHandler
  .thumb_set DMA2_Stream2_IRQHandler, _default_handler
          
  .weak      DMA2_Stream3_IRQHandler
  .thumb_set DMA2_Stream3_IRQHandler, _default_handler
          
  .weak      DMA2_Stream4_IRQHandler
  .thumb_set DMA2_Stream4_IRQHandler, _default_handler
          
  .weak      OTG_FS_IRQHandler
  .thumb_set OTG_FS_IRQHandler, _default_handler
                  
  .weak      DMA2_Stream5_IRQHandler
  .thumb_set DMA2_Stream5_IRQHandler, _default_handler
                
  .weak      DMA2_Stream6_IRQHandler
  .thumb_set DMA2_Stream6_IRQHandler, _default_handler
                
  .weak      DMA2_Stream7_IRQHandler
  .thumb_set DMA2_Stream7_IRQHandler, _default_handler
                
  .weak      USART6_IRQHandler
  .thumb_set USART6_IRQHandler, _default_handler

  .weak      I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler, _default_handler

  .weak      I2C3_ER_IRQHandler   
  .thumb_set I2C3_ER_IRQHandler, _default_handler

  .weak      FPU_IRQHandler
  .thumb_set FPU_IRQHandler, _default_handler

  .weak      SPI4_IRQHandler
  .thumb_set SPI4_IRQHandler, _default_handler