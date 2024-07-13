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


#ifndef BIT_MANIP
#define BIT_MANIP     1
/*---------------------------------------------------------------*/
/*-------------------Macros to manipulate bits-------------------*/
/*---------------------------------------------------------------*/

  /* Clear the bits with MASK and set new vbits with VALUE in the register REG*/
  #define MODIFY_REG(REG, MASK, VALUE) ((REG) = ((REG) & ~(MASK)) | (VALUE))

  /*sets the BIT_NUM bit in the register*/
  #define SET_BIT(REG, BIT_NUM)   ((REG) |= (0b1U << BIT_NUM))

  /*sets the bits in the register that are set in MASK*/
  #define SET_BITS(REG, MASK)     ((REG) |= (MASK))

  /*reads the bit from the register and return boolean value*/
  #define READ_BIT(REG, BIT_NUM)  (((REG) & (1U << (BIT_NUM))) != 0U)

  /// @brief Reads the bits from the register and returns true if all MASK bits are set.
  #define READ_BITS(REG, MASK)    (((REG) & (MASK)) == (MASK))

  /*clears the BIT_NUM bit in the register*/
  #define CLEAR_BIT(REG, BIT_NUM) ((REG) &= ~(1U << (BIT_NUM)))

  /*clears the bits in the register based off the set bits in MASK*/
  #define CLEAR_BITS(REG, MASK)   ((REG) &= ~(MASK)))

  /*rotates the bits in the register by N rotations*/
  #define ROTATE_RIGHT(REG, N) \
    __asm__ volatile ( \
      "ROR %0, %0, %1" \
      : "+r" (REG) \
      : "I" (N) \
    )

#endif // !BIT_MANIP 1



#ifndef NVIC_H 
#define NVIC_H      1
/*---------------------------------------------------------------*/
/*---IRQ numbers as provided by the STM32F401 reference manual---*/
/*---------------------------------------------------------------*/

/*List of IRQ numbers should be used as IRQ_NUM argument in the NVIC syscalls*/

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

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/


#ifndef FLASH_HELPERS
  #define FLASH_HELPERS   1
  #define FLASH_BASE               (0x40023C00U)
  #define FLASH_ACR_OFFSET         (0x00U)
  #define FLASH_KEYR_OFFSET        (0x04U)
  #define FLASH_OPTKEYR_OFFSET     (0x08U)
  #define FLASH_SR_OFFSET          (0x0CU)
  #define FLASH_CR_OFFSET          (0x10U)
  #define FLASH_OPTCR_OFFSET       (0x14U)

  #define FLASH_KEY1               (0x45670123U)
  #define FLASH_KEY2               (0xCDEF89ABU)
  #define FLASH_OPTKEY1            (0x08192A3BU)
  #define FLASH_OPTKEY2            (0x4C5D6E7FU)

  /*FLASH interface registers*/
  /*
  Each memory mapped regitser pointed to is dereferenced to avoid dereferencing in each op
  */
  #define FLASH_ACR         (*(volatile uint32_t *)(FLASH_BASE + FLASH_ACR_OFFSET))
  #define FLASH_KEYR        (*(volatile uint32_t *)(FLASH_BASE + FLASH_KEYR_OFFSET))
  #define FLASH_OPTKEYR     (*(volatile uint32_t *)(FLASH_BASE + FLASH_OPTKEYR_OFFSET))
  #define FLASH_SR          (*(volatile uint32_t *)(FLASH_BASE + FLASH_SR_OFFSET))
  #define FLASH_CR          (*(volatile uint32_t *)(FLASH_BASE + FLASH_CR_OFFSET))
  #define FLASH_OPTCR       (*(volatile uint32_t *)(FLASH_BASE + FLASH_OPTCR_OFFSET))


  /*--------FLASH_ACR---------*/
  /*--------FLASH_ACR---------*/

  /* Enable the data cache */
  #define FLASH_EN_DCACHE()       SET_BIT(FLASH_ACR, 10U)
  /* Disable the data cache */
  #define FLASH_DIS_DCACHE()      CLEAR_BIT(FLASH_ACR, 10U)
  /* Reset the data cache */
  #define FLASH_RES_DCACHE()      SET_BIT(FLASH_ACR, 12U)
  /* Enable the instruction cache */
  #define FLASH_EN_ICACHE()       SET_BIT(FLASH_ACR, 9U)
  /* Disable the instruction cache */
  #define FLASH_DIS_ICACHE()      CLEAR_BIT(FLASH_ACR, 9U)
  /* Reset the instruction cache */
  #define FLASH_RES_ICACHE()      SET_BIT(FLASH_ACR, 11U)
  /* Enable the prefetch buffer */
  #define FLASH_EN_PREFETCH()     SET_BIT(FLASH_ACR, 8U)
  /* Disable the prefetch buffer */
  #define FLASH_DIS_PREFETCH()    CLEAR_BIT(FLASH_ACR, 8U)

  /// @param MASK:
  // 0000: Zero wait state
  // 0001: One wait state
  // 0010: Two wait states
  // ....
  // ....
  // 1110: Fourteen wait states
  // 1111: Fifteen wait states
  #define FLASH_SET_LATENCY(MASK) \
    do { \
    CLEAR_BITS(FLASH_ACR, 0b1111U); \
    SET_BITS(FLASH_ACR, MASK); \
    } while (0)

  /*--------FLASH_KEYR---------*/
  /*--------FLASH_KEYR---------*/

  /*Unlock write access to the FLASH_CR register*/
  #define FLASH_UNLOCK_CR() \
    SET_BITS(FLASH_KEYR, FLASH_KEY1) \
    SET_BITS(FLASH_KEYR, FLASH_KEY2)

  /*--------FLASH_OPTKEYR---------*/
  /*--------FLASH_OPTKEYR---------*/

  /*Unlock write access to the FLASH_OPTCR register to alter OPTION BYTES*/
  #define FLASH_UNLOCK_OPTCR() \
    SET_BITS(FLASH_OPTKEYR, FLASH_OPTKEY1) \
    SET_BITS(FLASH_OPTKEYR, FLASH_OPTKEY2)

  /*--------FLASH_SR---------*/
  /*--------FLASH_SR---------*/

  /* Check if the FLASH is busy */
  #define FLASH_CHECK_BUSY()            READ_BIT(FLASH_SR, 31U)
  /* Check for a protection error */
  #define FLASH_CHECK_PROTERROR()       READ_BIT(FLASH_SR, 8U)
  /* Check for a programming sequence error */
  #define FLASH_CHECK_PROG_SEQERR()     READ_BIT(FLASH_SR, 7U)
  /* Check for a programming parallelism error */
  #define FLASH_CHECK_PROG_PARALLERR()  READ_BIT(FLASH_SR, 6U)
  /* Check for a programming alignment error */
  #define FLASH_CHECK_PROG_ALIGNERR()   READ_BIT(FLASH_SR, 5U)
  /* Check for a write protection error */
  #define FLASH_CHECK_WRITE_PROTERR()   READ_BIT(FLASH_SR, 4U)
  /* Check for an operation error */
  #define FLASH_CHECK_OPERR()           READ_BIT(FLASH_SR, 1U)
  /* Check if the end of operation flag is set */
  #define FLASH_CHECK_END_OP()          READ_BIT(FLASH_SR, 0U)
  /* Clear the protection error flag */
  #define FLASH_CLEAR_PROTERROR()       SET_BIT(FLASH_SR, 8U)
  /* Clear the programming sequence error flag */
  #define FLASH_CLEAR_PROG_SEQERR()     SET_BIT(FLASH_SR, 7U)
  /* Clear the programming parallelism error flag */
  #define FLASH_CLEAR_PROG_PARALLERR()  SET_BIT(FLASH_SR, 6U)
  /* Clear the programming alignment error flag */
  #define FLASH_CLEAR_PROG_ALIGNERR()   SET_BIT(FLASH_SR, 5U)
  /* Clear the write protection error flag */
  #define FLASH_CLEAR_WRITE_PROTERR()   SET_BIT(FLASH_SR, 4U)
  /* Clear the operation error flag */
  #define FLASH_CLEAR_OPERR()           SET_BIT(FLASH_SR, 1U)
  /* Clear the end of operation flag */
  #define FLASH_CLEAR_END_OP()          SET_BIT(FLASH_SR, 0U)


  /*--------FLASH_CR---------*/
  /*--------FLASH_CR---------*/
  #define FLASH_LOCK_CR()       SET_BIT(FLASH_CR, 31U)
  /*FLASH error interrupt enable*/
  #define FLASH_ERR_IE()        SET_BIT(FLASH_CR, 25U)
  /*FLASH error interrupt disable*/
  #define FLASH_ERR_ID()        CLEAR_BIT(FLASH_CR, 25U)
  /*FLASH end of operation interrupt enable*/
  #define FLASH_EOP_IE()        SET_BIT(FLASH_CR, 24U)
  /*FLASH end of operation interrupt disable*/
  #define FLASH_EOP_ID()        CLEAR_BIT(FLASH_CR, 24U)
  #define FLASH_START_OP()      SET_BIT(FLASH_CR, 16U)
  /// @param SIZE: These bits select the program parallelism.
  // 00 program x8
  // 01 program x16
  // 10 program x32
  // 11 program x64
  #define FLASH_SET_PROG_SIZE(SIZE) \
    do { \
      CLEAR_BITS(FLASH_CR, 0b11U << 8U); \
      SET_BITS(FLASH_CR, SIZE << 8U); \
    } while (0)

  /// @param NUM: These bits select the sector to erase.
  // 0000 sector 0
  // 0001 sector 1
  // ...
  // 0101 sector 5
  // 0110 sector 6 (STM32F401xD/E devices only)
  // 0111 sector 7 (STM32F401xE devices only)
  #define FLASH_SEL_SECTOR_NUM(NUM) \
    do { \
      CLEAR_BITS(FLASH_CR, 0b1111U << 3); \
      SET_BITS(FLASH_CR, NUM << 3); \
    } while (0)
  
  #define FLASH_MASS_ERASE()  SET_BIT(FLASH_CR, 2U)
  #define FLASH_SECTOR_ERASE()  SET_BIT(FLASH_CR, 1U)
  #define FLASH_PROGRAMMING() SET_BIT(FLASH_CR, 0U)

  /*--------FLASH_OPTCR---------*/
  /*--------FLASH_OPTCR---------*/

  /* Reset SPRMOD bit */
  #define FLASH_WPRMODE_RESET()     CLEAR_BIT(FLASH_OPTCR, 31U)
  /* SPRMOD: Selection of protection mode of WPR bits. */
  #define FLASH_WPRMODE_SELECT()    SET_BIT(FLASH_OPTCR, 31U)
  /* nWRP sector 7 */
  #define FLASH_WRP_SECTOR_7()      SET_BIT(FLASH_OPTCR, 23U)
  /* nWRP sector 6 */
  #define FLASH_WRP_SECTOR_6()      SET_BIT(FLASH_OPTCR, 22U)
  /* nWRP sector 5 */
  #define FLASH_WRP_SECTOR_5()      SET_BIT(FLASH_OPTCR, 21U)
  /* nWRP sector 4 */
  #define FLASH_WRP_SECTOR_4()      SET_BIT(FLASH_OPTCR, 20U)
  /* nWRP sector 3 */
  #define FLASH_WRP_SECTOR_3()      SET_BIT(FLASH_OPTCR, 19U)
  /* nWRP sector 2 */
  #define FLASH_WRP_SECTOR_2()      SET_BIT(FLASH_OPTCR, 18U)
  /* nWRP sector 1 */
  #define FLASH_WRP_SECTOR_1()      SET_BIT(FLASH_OPTCR, 17U)
  /* nWRP sector 0 */
  #define FLASH_WRP_SECTOR_0()      SET_BIT(FLASH_OPTCR, 16U)
  /* @param SECTORS: These bits define the write protection sectors. */
  // Bit 16: nWRP[0]
  // Bit 17: nWRP[1]
  // ...
  // Bit 23: nWRP[7]
  #define FLASH_SET_WRP_SECTORS(SECTORS) \
    do { \
      CLEAR_BITS(FLASH_OPTCR, 0xFFU << 16U); \
      SET_BITS(FLASH_OPTCR, SECTORS << 16U); \
    } while (0)
  /* @param LEVEL: These bits define the read protection level. */
  // 0xAA: Level 0
  // 0x55: Level 1
  // 0xCC: Level 2
  #define FLASH_SET_RDP_LEVEL(LEVEL) \
    do { \
      CLEAR_BITS(FLASH_OPTCR, 0xFFU << 8U); \
      SET_BITS(FLASH_OPTCR, LEVEL << 8U); \
    } while (0)

  /* Read protection level 1 */
  #define FLASH_RDP_LVL1()          CLEAR_BIT(FLASH_OPTCR, 8U)
  /* Read protection level 2 */
  #define FLASH_RDP_LVL2()          SET_BIT(FLASH_OPTCR, 8U)
  /* Option lock */
  #define FLASH_LOCK_OPTCR()        SET_BIT(FLASH_OPTCR, 0U)


#endif // !FLASH_HELPERS 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/


#ifndef CRC_HELPERS
#define CRC_HELPERS   1

  /*The CRC calculation unit Base address*/ 
  #define CRC_BASE            (0x40023000U)

  #define CRC_DR_OFFSET       (0x00U)
  #define CRC_IDR_OFFSET      (0x04U)
  #define CRC_CR_OFFSET       (0x08U)

  /* The CRC Data register */
  #define CRC_DR        (*(volatile uint32_t *)(CRC_BASE + CRC_DR_OFFSET))
  /* The CRC Independent Data register */
  #define CRC_IDR       (*(volatile uint32_t *)(CRC_BASE + CRC_IDR_OFFSET))
  /* The CRC Control register */
  #define CRC_CR        (*(volatile uint32_t *)(CRC_BASE + CRC_CR_OFFSET))


  /*--------CRC_DR---------*/
  /*--------CRC_DR---------*/

  /* Read the CRC Data register */
  #define READ_CRC_DR()            (CRC_DR)

  /* Write to the CRC Data register */
  #define WRITE_CRC_DR(VAL)      (CRC_DR = (VAL))

  /*--------CRC_IDR---------*/
  /*--------CRC_IDR---------*/

  /* Read the CRC Independent Data register */
  #define READ_CRC_IDR()           (CRC_IDR)

  /* Write to the CRC Independent Data register */
  #define WRITE_CRC_IDR(VAL)     (CRC_IDR = (VAL))

  /*--------CRC_CR---------*/
  /*--------CRC_CR---------*/

  /* Reset the CRC calculation unit */
  #define CRC_RESET()              SET_BIT(CRC_CR, 0U)

  /* Check if CRC unit is reset */
  #define CRC_IS_RESET()           READ_BIT(CRC_CR, 0U)

#endif // !CRC_HELPERS 1


/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/


#ifndef PWR_HELPERS
  #define PWR_HELPERS   1

  #define PWR_BASE          (0x40007000U)
  #define PWR_CR_OFFSET     (0x00U)
  #define PWR_CSR_OFFSET    (0x04U)

  #define PWR_CR            (*(volatile uint32_t *)(PWR_BASE + PWR_CR_OFFSET))
  #define PWR_CSR           (*(volatile uint32_t *)(PWR_BASE + PWR_CSR_OFFSET))

  /*--------PWR_CR---------*/
  /*--------PWR_CR---------*/

  // Sets Regulator voltage scaling output selection
  // 01: Scale 3 mode 
  // 10: Scale 2 mode 
  #define PWR_REGULATOR_VOS2() \
    do { \
      CLEAR_BITS(PWR_CR, 0b11U << 14U); \
      SET_BIT(PWR_CR, 15U); \
    } while (0)
  #define PWR_REGULATOR_VOS3() \
    do { \
      CLEAR_BITS(PWR_CR, 0b11U << 14U); \
      SET_BIT(PWR_CR, 14U); \
    } while (0)

  // 0: Main regulator in Voltage scale 3 when the device is in Stop mode.
  // 1: Main regulator in Low Voltage and Flash memory in Deep Sleep mode when the device is 
  // in Stop mode.
  #define PWR_MRLVDS(BITVAL) \
    do { \
      CLEAR_BIT(PWR_CR, 11U); \
      ((PWR_CR) |= (BITVAL << 11U)); \ 
    } while (0)
  // 0: Low-power regulator on if LPDS bit is set when the device is in Stop mode.
  // 1: Low-power regulator in Low Voltage and Flash memory in Deep Sleep mode if LPDS bit is 
  // set when device is in Stop mode.
  #define PWR_MRLVDS(BITVAL) \
    do { \
      CLEAR_BIT(PWR_CR, 10U); \
      ((PWR_CR) |= (BITVAL << 9U)); \
    } while (0)
  // 0: Flash memory not in power-down when the device is in Stop mode
  // 1: Flash memory in power-down when the device is in Stop mode
  #define PWR_FPDS(BITVAL) \
    do { \
      CLEAR_BIT(PWR_CR, 9U); \
      ((PWR_CR) |= (BITVAL << 9U)); \
    } while (0)

  #define PWR_EN_RTC_ACCESS() SET_BIT(PWR_CR, 8U)
  #define PWR_DIS_RTC_ACCESS() CLEAR_BIT(PWR_CR, 8U)


  #define PWR_PVD_LEVEL(MASK) \
    do { \
      CLEAR_BITS(PWR_CR, 0b111U << 5U); \
      SET_BITS(PWR_CR, MASK << 5U); \
    } while (0)

  /*Set the Power voltage detector threshhold at 2.2V*/
  #define PWR_PVD_LEVEL2_2() PWR_PVD_LEVEL(0b000U)
  /*Set the Power voltage detector threshhold at 2.3V*/
  #define PWR_PVD_LEVEL2_3() PWR_PVD_LEVEL(0b001U)
  /*Set the Power voltage detector threshhold at 2.4V*/
  #define PWR_PVD_LEVEL2_4() PWR_PVD_LEVEL(0b010U)
  /*Set the Power voltage detector threshhold at 2.5V*/
  #define PWR_PVD_LEVEL2_5() PWR_PVD_LEVEL(0b011U)
  /*Set the Power voltage detector threshhold at 2.6V*/
  #define PWR_PVD_LEVEL2_6() PWR_PVD_LEVEL(0b100U)
  /*Set the Power voltage detector threshhold at 2.7V*/
  #define PWR_PVD_LEVEL2_7() PWR_PVD_LEVEL(0b101U)
  /*Set the Power voltage detector threshhold at 2.8V*/
  #define PWR_PVD_LEVEL2_8() PWR_PVD_LEVEL(0b110U)
  /*Set the Power voltage detector threshhold at 2.9V*/
  #define PWR_PVD_LEVEL2_9() PWR_PVD_LEVEL(0b111U)

  /*Enable the power voltage detector*/
  #define PWR_EN_PVD() SET_BIT(PWR_CR, 4U)
  /*Disable the power voltage detector*/
  #define PWR_DIS_PVD() CLEAR_BIT(PWR_CR, 4U)

  #define PWR_CLEAR_STDBY_FLAG()  SET_BIT(PWR_CR, 3U)
  #define PWR_CLEAR_WKUP_FLAG()   SET_BIT(PWR_CR, 2U)
  // 0: Enter Stop mode when the CPU enters deepsleep. 
  // The regulator status depends on the LPDS bit set by the  function.
  #define PWR_PDWN_DEEPSLEEP() CLEAR_BIT(PWR_CR, 1U)
  // 1: Enter Standby mode when the CPU enters deepsleep. 
  #define PWR_STDBY_DEEPSLEEP() SET_BIT(PWR_CR, 1U)
  // 0: Voltage regulator on during Stop mode.
  #define PWR_NO_LOWP_DEEPSLEEP() CLEAR_BIT(PWR_CR, 0U)
  // 1: Low-power Voltage regulator on during Stop mode
  #define PWR_LOWP_DEEPSLEEP() SET_BIT(PWR_CR, 0U)

  /*--------PWR_CSR---------*/
  /*--------PWR_CSR---------*/

  // Clear the Wakeup flag
  #define PWR_CLEAR_WU_FLAG() SET_BIT(PWR_CSR, 0U)
  // Clear the Standby flag
  #define PWR_CLEAR_STDBY_FLAG() SET_BIT(PWR_CSR, 1U)

  // Enable the wakeup pin
  #define PWR_EN_WAKEUP_PIN() SET_BIT(PWR_CSR, 8U)
  // Disable the wakeup pin
  #define PWR_DIS_WAKEUP_PIN() CLEAR_BIT(PWR_CSR, 8U)

  // PVD output
  #define PWR_PVD_OUTPUT() (READ_BIT(PWR_CSR, 2U))

  // Backup regulator ready
  #define PWR_BKP_REG_RDY() (READ_BIT(PWR_CSR, 3U))

  // Regulator voltage scaling output selection ready bit
  #define PWR_REGULATOR_VOS_RDY() (READ_BIT(PWR_CSR, 4U))

  // Check if Wakeup flag is set
  #define PWR_GET_WU_FLAG() (READ_BIT(PWR_CSR, 0U))
  // Check if Standby flag is set
  #define PWR_GET_STDBY_FLAG() (READ_BIT(PWR_CSR, 1U))

  // Check if regulator voltage scaling is ready
  #define PWR_GET_VOS_RDY() (READ_BIT(PWR_CSR, 4U))

  // Check if backup regulator is ready
  #define PWR_GET_BKP_REG_RDY() (READ_BIT(PWR_CSR, 3U))

  // Check if PVD output is high
  #define PWR_GET_PVD_OUTPUT() (READ_BIT(PWR_CSR, 2U))

  // Enable backup regulator
  #define PWR_EN_BKP_REG() SET_BIT(PWR_CSR, 9U)
  // Disable backup regulator
  #define PWR_DIS_BKP_REG() CLEAR_BIT(PWR_CSR, 9U)

  // Use WKUP and STDBY for flags
  #define PWR_CLEAR_WKUP_FLAG() SET_BIT(PWR_CSR, 0U)
  #define PWR_CLEAR_STDBY_FLAG() SET_BIT(PWR_CSR, 1U)
  #define PWR_GET_WKUP_FLAG() (READ_BIT(PWR_CSR, 0U))
  #define PWR_GET_STDBY_FLAG() (READ_BIT(PWR_CSR, 1U))

#endif // !PWR_HELPERS 1


/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/
/*---------------------------------------------------------------*/
/*---------------Macros for Reset and clock control--------------*/
/*---------------------------------------------------------------*/
#ifndef RCC_HELPERS
  #define RCC_HELPERS     1

  #define RCC_BASE                (0x40023800U)
  #define RCC_CR_OFFSET           (0x00U)
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


  /*RCC interface register*/
  /*
  Each memory mapped regitser pointed to is dereferenced to avoid dereferencing in each op
  */
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

  /* Set PLL frequency with configurable parameters */
  #define RCC_PLL_CONFIG(N, P, M) \
    MODIFY_REG(RCC_PLLCFGR, 0x1FFCFFFFU, \
      (1U << 22U) | ((N) << 6U) | ((P) << 16U) | ((M) << 0U))

  /* Set the PLL frequency at 84MHz */
  #define RCC_PLL_84MHz() RCC_PLL_CONFIG(336U, 4U, 16U)

  /* Set the PLL frequency at 72MHz */
  #define RCC_PLL_72MHz() RCC_PLL_CONFIG(432U, 6U, 16U)

  /* Set the PLL frequency at 64MHz */
  #define RCC_PLL_64MHz() RCC_PLL_CONFIG(256U, 4U, 16U)

  /* Set the PLL frequency at 42MHz */
  #define RCC_PLL_42MHz() RCC_PLL_CONFIG(336U, 8U, 16U)



  /*--------RCC_CFGR---------*/
  /*--------RCC_CFGR---------*/

  /// @param SOURCE Is the source of output in Microcontroller clock output 2
  /// 0b00: SYSCLK / 0b01: PLLI2S / 0b10: HSE / 0b11: PLL
  #define RCC_MCO2_SRC(SOURCE) \
    do { \
      CLEAR_BITS(RCC_CFGR, 0b11 << 30U); \
      SET_BITS(RCC_CFGR, SOURCE << 30U); \
    } while (0)

  /// @param PRE is the prescaler to apply on the signal of Microcontroller clock output 2
  /// 0xx: no division / 0b100: division by 2 / 0b101: division by 3 /
  // 0xx: no division
  // 100: division by 2
  // 101: division by 3
  // 110: division by 4
  // 111: division by 5
  #define RCC_MCO2_PRE(PRE) \
    do { \
      CLEAR_BITS(RCC_CFGR, 0b111U << 27U); \
      SET_BITS(RCC_CFGR, PRE << 27U); \
    } while (0)

  /// @param SOURCE Is the source of output in Microcontroller clock output 1
  // 00: HSI clock selected
  // 01: LSE oscillator selected
  // 10: HSE oscillator clock selected
  // 11: PLL clock selected
  #define RCC_MCO1_SRC(SOURCE) \
    do { \
      CLEAR_BITS(RCC_CFGR, 0b11U << 21U); \
      SET_BITS(RCC_CFGR, SOURCE << 21U); \
    } while (0)

  /// @param SOURCE Is the source of inter integrated sound clock
  // 0: PLLI2S clock used as I2S clock source
  // 1: External clock mapped on the I2S_CKIN pin used as I2S clock source
  #define RCC_MCO1_SRC(SOURCE) \
    do { \
      CLEAR_BITS(RCC_CFGR, 0b1U << 23U); \
      SET_BITS(RCC_CFGR, SOURCE << 23U); \
    } while (0)


  /// @param PRE is the prescaler to apply on the signal of Microcontroller clock output 1
  /// 0xx: no division / 0b100: division by 2 / 0b101: division by 3 /
  // 0xx: no division
  // 100: division by 2
  // 101: division by 3
  // 110: division by 4
  // 111: division by 5
  #define RCC_MCO1_PRE(PRE) \
    do { \
      CLEAR_BITS(RCC_CFGR, 0b111U << 24U); \
      SET_BITS(RCC_CFGR, PRE << 24U); \
    } while (0)

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
    do { \
      CLEAR_BITS(RCC_CFGR, 0b11111U << 16U); \
      SET_BITS(RCC_CFGR, PRE << 16U); \
    } while (0)

  /// @param PRE is the prescaler to apply on the APB2 
  // 0xx: AHB clock not divided
  // 100: AHB clock divided by 2
  // 101: AHB clock divided by 4
  // 110: AHB clock divided by 8
  // 111: AHB clock divided by 16
  #define RCC_APB2_PRE(PRE) \
    do { \
      CLEAR_BITS(RCC_CFGR, 0b111U << 13U); \
      SET_BITS(RCC_CFGR, PRE << 13U); \
    } while (0)

  /// @param PRE is the prescaler to apply on the APB1
  // 0xx: AHB clock not divided
  // 100: AHB clock divided by 2
  // 101: AHB clock divided by 4
  // 110: AHB clock divided by 8
  // 111: AHB clock divided by 16
  #define RCC_APB1_PRE(PRE) \
    do { \
      CLEAR_BITS(RCC_CFGR, 0b111U << 10U); \
      SET_BITS(RCC_CFGR, PRE << 10U); \
    } while (0)

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
    do { \
      CLEAR_BITS(RCC_CFGR, 0b1111U << 4U); \
      SET_BITS(RCC_CFGR, PRE << 4U); \
    } while (0)


  /// @param STAT Is the mask to compare SWS bits to
  // 00: HSI oscillator selected as system clock
  // 01: HSE oscillator selected as system clock
  // 10: PLL selected as system clock
  // 11: not allowed
  #define RCC_SYSCLK_STAT(STAT) READ_BITS(RCC_CFGR, STAT)


  /// @param SOURCE Is the source of the SYSCLK
  // 00: HSI oscillator selected as system clock
  // 01: HSE oscillator selected as system clock
  // 10: PLL selected as system clock
  // 11: not allowed
  #define RCC_SYSCLK_SRC(SOURCE) \
    do { \
      CLEAR_BITS(RCC_CFGR, 0b11U); \
      SET_BITS(RCC_CFGR, SOURCE); \
    } while (0)


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
    do { \
      CLEAR_BITS(RCC_BDCR, (0b11U << 8U)); \
      SET_BITS(RCC_BDCR, (SOURCE << 8U)); \
    } while (0)

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
    do { \
      CLEAR_BITS(RCC_SSCGR, 0x7FFFU << 13U); \
      SET_BITS(RCC_SSCGR, INCSTEP << 13U); \
    } while (0)
  /*Modulation period, Configuration input for modulation profile period.*/
  #define RCC_SET_MODPER(MODPER) \
    do { \
      CLEAR_BITS(RCC_SSCGR, 0x1FFFU); \
      SET_BITS(RCC_SSCGR, MODPER); \
    } while (0)
  /*--------RCC_PLLI2SCFGR---------*/
  /*--------RCC_PLLI2SCFGR---------*/

  // f(VCO clock) = f(PLLI2S clock input) × (PLLI2SN / PLLM)
  // f(PLL I2S clock output) = f(VCO clock) / PLLI2SR

  // I2S clock frequency = VCO frequency / PLLR with 2 ≤ PLLR ≤ 7
  #define RCC_SET_PLLI2SR(PRE) \
    do { \
      CLEAR_BITS(RCC_PLLI2SCFGR, (7U << 28U)); \
      SET_BITS(RCC_PLLI2SCFGR, (PRE << 28U)); \
    } while (0)


  // VCO output frequency = VCO input frequency × PLLI2SN with 192 ≤ PLLI2SN ≤ 432
  #define RCC_SET_PLLI2SN(PRE) \
    do { \
      CLEAR_BITS(RCC_PLLI2SCFGR, (511U << 6U)); \
      SET_BITS(RCC_PLLI2SCFGR, (PRE << 6U)); \
    } while (0)



  /*--------RCC_DCKCFGR---------*/
  /*--------RCC_DCKCFGR---------*/
  #define RCC_SET_TIMERPRE()    SET_BIT(RCC_DCKCFGR, 24U) 
  #define RCC_CLEAR_TIMERPRE()  CLEAR_BIT(RCC_DCKCFGR, 24U) 

#endif // !RCC_HELPERS


/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef GPIO_HELPERS
  #define GPIO_HELPERS  1

  #define GPIOA_BASE        (0x40020000U)
  #define GPIOB_BASE        (0x40020400U)
  #define GPIOC_BASE        (0x40020800U)
  #define GPIOD_BASE        (0x40020C00U)
  #define GPIOE_BASE        (0x40021000U)
  #define GPIOH_BASE        (0x40021C00U)

  #define GPIO_MODER_OFFSET   (0x00U)
  #define GPIO_OTYPER_OFFSET  (0x04U)
  #define GPIO_OSPEEDR_OFFSET (0x08U)
  #define GPIO_PUPDR_OFFSET   (0x0CU)
  #define GPIO_IDR_OFFSET     (0x10U)
  #define GPIO_ODR_OFFSET     (0x14U)
  #define GPIO_BSRR_OFFSET    (0x18U)
  #define GPIO_LCKR_OFFSET    (0x1CU)
  #define GPIO_AFRL_OFFSET    (0x20U)
  #define GPIO_AFRH_OFFSET    (0x24U)


  /*--------GPIOx_MODER---------*/
  /*--------GPIOx_MODER---------*/

  #define GPIOx_SET_MODE(GPIOx_BASE, PIN_N, MODE) \
    do { \
      CLEAR_BITS((GPIOx_BASE) + GPIO_MODER_OFFSET, 0b11U << ((PIN_N) / 2U)); \
      SET_BITS((GPIOx_BASE) + GPIO_MODER_OFFSET, (MODE) << ((PIN_N) / 2U)); \
    } while (0)


  /// @param PIN_N is the pin number of the port (0..15)
  /// @param MODE 
  // 00: Input (reset state) /
  // 01: General purpose output mode /
  // 10: Alternate function mode /
  // 11: Analog mode
  #define GPIOA_SET_MODE(PIN_N, MODE) GPIOx_SET_MODE(GPIOA_BASE, PIN_N, MODE)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param MODE 
  // 00: Input (reset state) /
  // 01: General purpose output mode /
  // 10: Alternate function mode /
  // 11: Analog mode
  #define GPIOB_SET_MODE(PIN_N, MODE) GPIOx_SET_MODE(GPIOB_BASE, PIN_N, MODE)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param MODE 
  // 00: Input (reset state) /
  // 01: General purpose output mode /
  // 10: Alternate function mode /
  // 11: Analog mode
  #define GPIOC_SET_MODE(PIN_N, MODE) GPIOx_SET_MODE(GPIOC_BASE, PIN_N, MODE)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param MODE 
  // 00: Input (reset state) /
  // 01: General purpose output mode /
  // 10: Alternate function mode /
  // 11: Analog mode
  #define GPIOD_SET_MODE(PIN_N, MODE) GPIOx_SET_MODE(GPIOD_BASE, PIN_N, MODE)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param MODE 
  // 00: Input (reset state) /
  // 01: General purpose output mode /
  // 10: Alternate function mode /
  // 11: Analog mode
  #define GPIOE_SET_MODE(PIN_N, MODE) GPIOx_SET_MODE(GPIOE_BASE, PIN_N, MODE)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param MODE 
  // 00: Input (reset state) /
  // 01: General purpose output mode /
  // 10: Alternate function mode /
  // 11: Analog mode
  #define GPIOH_SET_MODE(PIN_N, MODE) GPIOx_SET_MODE(GPIOH_BASE, PIN_N, MODE)


  /*--------GPIOx_OTYPER---------*/
  /*--------GPIOx_OTYPER---------*/

  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOA_SET_OPENDRAIN(PIN_N)    SET_BIT(GPIOA_BASE + GPIO_OTYPER_OFFSET, PIN_N)
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOA_SET_PUSHPULL(PIN_N)     CLEAR_BIT(GPIOA_BASE + GPIO_OTYPER_OFFSET, PIN_N)

  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOB_SET_OPENDRAIN(PIN_N)    SET_BIT(GPIOB_BASE + GPIO_OTYPER_OFFSET, PIN_N)
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOB_SET_PUSHPULL(PIN_N)     CLEAR_BIT(GPIOB_BASE + GPIO_OTYPER_OFFSET, PIN_N)

  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOC_SET_OPENDRAIN(PIN_N)    SET_BIT(GPIOC_BASE + GPIO_OTYPER_OFFSET, PIN_N)
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOC_SET_PUSHPULL(PIN_N)     CLEAR_BIT(GPIOC_BASE + GPIO_OTYPER_OFFSET, PIN_N)

  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOD_SET_OPENDRAIN(PIN_N)    SET_BIT(GPIOD_BASE + GPIO_OTYPER_OFFSET, PIN_N)
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOD_SET_PUSHPULL(PIN_N)     CLEAR_BIT(GPIOD_BASE + GPIO_OTYPER_OFFSET, PIN_N)

  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOE_SET_OPENDRAIN(PIN_N)    SET_BIT(GPIOE_BASE + GPIO_OTYPER_OFFSET, PIN_N)
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOE_SET_PUSHPULL(PIN_N)     CLEAR_BIT(GPIOE_BASE + GPIO_OTYPER_OFFSET, PIN_N)

  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOH_SET_OPENDRAIN(PIN_N)    SET_BIT(GPIOH_BASE + GPIO_OTYPER_OFFSET, PIN_N)
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOH_SET_PUSHPULL(PIN_N)     CLEAR_BIT(GPIOH_BASE + GPIO_OTYPER_OFFSET, PIN_N)


  /*--------GPIOx_OSPEEDR---------*/
  /*--------GPIOx_OSPEEDR---------*/

  #define GPIOx_SET_OSPEED(GPIOx_BASE, PIN_N, OSPEED) \
    do { \
      CLEAR_BITS((GPIOx_BASE) + GPIO_OSPEEDR_OFFSET, 0b11U << ((PIN_N) / 2U)); \
      SET_BITS((GPIOx_BASE) + GPIO_OSPEEDR_OFFSET, (OSPEED) << ((PIN_N) / 2U)); \
    } while (0)


  /// @param PIN_N is the pin number of the port (0..15)
  /// @param OSPEED is the signal output speed to apply to that pin: 
  // 00: Low speed /
  // 01: Medium speed /
  // 10: High speed /
  // 11: Very high speed /
  #define GPIOA_SET_OSPEED(PIN_N, OSPEED) GPIOx_SET_OSPEED(GPIOA_BASE, PIN_N, OSPEED)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param OSPEED is the signal output speed to apply to that pin: 
  // 00: Low speed /
  // 01: Medium speed /
  // 10: High speed /
  // 11: Very high speed /
  #define GPIOB_SET_OSPEED(PIN_N, OSPEED) GPIOx_SET_OSPEED(GPIOB_BASE, PIN_N, OSPEED)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param OSPEED is the signal output speed to apply to that pin: 
  // 00: Low speed /
  // 01: Medium speed /
  // 10: High speed /
  // 11: Very high speed /
  #define GPIOC_SET_OSPEED(PIN_N, OSPEED) GPIOx_SET_OSPEED(GPIOC_BASE, PIN_N, OSPEED)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param OSPEED is the signal output speed to apply to that pin: 
  // 00: Low speed /
  // 01: Medium speed /
  // 10: High speed /
  // 11: Very high speed /
  #define GPIOD_SET_OSPEED(PIN_N, OSPEED) GPIOx_SET_OSPEED(GPIOD_BASE, PIN_N, OSPEED)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param OSPEED is the signal output speed to apply to that pin: 
  // 00: Low speed /
  // 01: Medium speed /
  // 10: High speed /
  // 11: Very high speed /
  #define GPIOE_SET_OSPEED(PIN_N, OSPEED) GPIOx_SET_OSPEED(GPIOE_BASE, PIN_N, OSPEED)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param OSPEED is the signal output speed to apply to that pin: 
  // 00: Low speed /
  // 01: Medium speed /
  // 10: High speed /
  // 11: Very high speed /
  #define GPIOH_SET_OSPEED(PIN_N, OSPEED) GPIOx_SET_OSPEED(GPIOH_BASE, PIN_N, OSPEED)

  /*--------GPIOx_PUPDR---------*/
  /*--------GPIOx_PUPDR---------*/

  /// @param GPIO_BASE is the base address of the GPIO port
  /// @param PIN_N is the pin number of the port (0..15)
  /// @param STATE Dictates the pull-up pull-down state of the pin
  // 00: No pull-up, pull-down
  // 01: Pull-up
  // 10: Pull-down
  // 11: Reserved
  #define GPIO_SET_PUPD(GPIO_BASE, PIN_N, STATE) \
    do { \
      CLEAR_BITS((GPIO_BASE) + GPIO_PUPDR_OFFSET, 0b11U << ((PIN_N) * 2U)); \
      SET_BITS((GPIO_BASE) + GPIO_PUPDR_OFFSET, (STATE) << ((PIN_N) * 2U)); \
    } while (0)

  /// Macros for GPIOA
  #define GPIOA_SET_PUPD(PIN_N, STATE) GPIO_SET_PUPD(GPIOA_BASE, PIN_N, STATE)

  /// Macros for GPIOB
  #define GPIOB_SET_PUPD(PIN_N, STATE) GPIO_SET_PUPD(GPIOB_BASE, PIN_N, STATE)

  /// Macros for GPIOC
  #define GPIOC_SET_PUPD(PIN_N, STATE) GPIO_SET_PUPD(GPIOC_BASE, PIN_N, STATE)

  /// Macros for GPIOD
  #define GPIOD_SET_PUPD(PIN_N, STATE) GPIO_SET_PUPD(GPIOD_BASE, PIN_N, STATE)

  /// Macros for GPIOE
  #define GPIOE_SET_PUPD(PIN_N, STATE) GPIO_SET_PUPD(GPIOE_BASE, PIN_N, STATE)

  /// Macros for GPIOH
  #define GPIOH_SET_PUPD(PIN_N, STATE) GPIO_SET_PUPD(GPIOH_BASE, PIN_N, STATE)


  /*--------GPIOx_IDR---------*/
  /*--------GPIOx_IDR---------*/

  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOA_PINREAD(PIN_N) READ_BIT(GPIOA_BASE + GPIO_IDR_OFFSET, (PIN_N))
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOB_PINREAD(PIN_N) READ_BIT(GPIOB_BASE + GPIO_IDR_OFFSET, (PIN_N))
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOC_PINREAD(PIN_N) READ_BIT(GPIOC_BASE + GPIO_IDR_OFFSET, (PIN_N))
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOD_PINREAD(PIN_N) READ_BIT(GPIOD_BASE + GPIO_IDR_OFFSET, (PIN_N))
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOE_PINREAD(PIN_N) READ_BIT(GPIOE_BASE + GPIO_IDR_OFFSET, (PIN_N))
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOH_PINREAD(PIN_N) READ_BIT(GPIOH_BASE + GPIO_IDR_OFFSET, (PIN_N))


  /*--------GPIOx_BSSR---------*/
  /*--------GPIOx_BSSR---------*/

  /// @param GPIO_BASE is the base address of the GPIO port
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIO_PIN_HIGH(GPIO_BASE, PIN_N) SET_BIT((GPIO_BASE) + GPIO_BSRR_OFFSET, (PIN_N))
  #define GPIO_PIN_LOW(GPIO_BASE, PIN_N) SET_BIT((GPIO_BASE) + GPIO_BSRR_OFFSET, (PIN_N) + 16U)

  /// Macros for GPIOA
  #define GPIOA_PIN_HIGH(PIN_N) GPIO_PIN_HIGH(GPIOA_BASE, PIN_N)
  #define GPIOA_PIN_LOW(PIN_N) GPIO_PIN_LOW(GPIOA_BASE, PIN_N)

  /// Macros for GPIOB
  #define GPIOB_PIN_HIGH(PIN_N) GPIO_PIN_HIGH(GPIOB_BASE, PIN_N)
  #define GPIOB_PIN_LOW(PIN_N) GPIO_PIN_LOW(GPIOB_BASE, PIN_N)

  /// Macros for GPIOC
  #define GPIOC_PIN_HIGH(PIN_N) GPIO_PIN_HIGH(GPIOC_BASE, PIN_N)
  #define GPIOC_PIN_LOW(PIN_N) GPIO_PIN_LOW(GPIOC_BASE, PIN_N)

  /// Macros for GPIOD
  #define GPIOD_PIN_HIGH(PIN_N) GPIO_PIN_HIGH(GPIOD_BASE, PIN_N)
  #define GPIOD_PIN_LOW(PIN_N) GPIO_PIN_LOW(GPIOD_BASE, PIN_N)

  /// Macros for GPIOE
  #define GPIOE_PIN_HIGH(PIN_N) GPIO_PIN_HIGH(GPIOE_BASE, PIN_N)
  #define GPIOE_PIN_LOW(PIN_N) GPIO_PIN_LOW(GPIOE_BASE, PIN_N)

  /// Macros for GPIOH
  #define GPIOH_PIN_HIGH(PIN_N) GPIO_PIN_HIGH(GPIOH_BASE, PIN_N)
  #define GPIOH_PIN_LOW(PIN_N) GPIO_PIN_LOW(GPIOH_BASE, PIN_N)


  /*--------GPIOx_AFRL/AFRH---------*/
  /*--------GPIOx_AFRL/AFRH---------*/

  #define GPIO_SET_AF(GPIOx_BASE, PIN_N, AF_NUM) \
    do { \
      CLEAR_BITS(GPIOx_BASE + ((PIN_N) < 8U ? GPIO_AFRL_OFFSET : GPIO_AFRH_OFFSET), \
        0xFU << (((PIN_N) - ((PIN_N) < 8U ? 0U : 8U)) / 4U)); \
      SET_BITS(GPIOx_BASE + ((PIN_N) < 8U ? GPIO_AFRL_OFFSET : GPIO_AFRH_OFFSET), \
        (AF_NUM) << (((PIN_N) - ((PIN_N) < 8U ? 0U : 8U)) / 4U)); \
    } while (0)


  /// @param PIN_N is the pin number of the port (0..15)
  /// @param AF_NUM is the alternate function number (0..15)
  #define GPIOA_SET_AF(PIN_N, AF_NUM) GPIO_SET_AF(GPIOA_BASE, PIN_N, AF_NUM)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param AF_NUM is the alternate function number (0..15)
  #define GPIOB_SET_AF(PIN_N, AF_NUM) GPIO_SET_AF(GPIOB_BASE, PIN_N, AF_NUM)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param AF_NUM is the alternate function number (0..15)
  #define GPIOC_SET_AF(PIN_N, AF_NUM) GPIO_SET_AF(GPIOC_BASE, PIN_N, AF_NUM)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param AF_NUM is the alternate function number (0..15)
  #define GPIOD_SET_AF(PIN_N, AF_NUM) GPIO_SET_AF(GPIOD_BASE, PIN_N, AF_NUM)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param AF_NUM is the alternate function number (0..15)
  #define GPIOE_SET_AF(PIN_N, AF_NUM) GPIO_SET_AF(GPIOE_BASE, PIN_N, AF_NUM)

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param AF_NUM is the alternate function number (0..15)
  #define GPIOH_SET_AF(PIN_N, AF_NUM) GPIO_SET_AF(GPIOH_BASE, PIN_N, AF_NUM)

#endif // !GPIO_HELPERS 1


/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/


#ifndef SYSCFG_HELPERS
  #define SYSCFG_HELPERS  1

  #define SYSCFG_BASE             (0x4001800U)
  #define SYSCFG_MEMRMP_OFFSET    (0x00U)
  #define SYSCFG_PMC_OFFSET       (0x04U)
  #define SYSCFG_EXTICR1_OFFSET   (0x08U)
  #define SYSCFG_EXTICR2_OFFSET   (0x0CU)
  #define SYSCFG_EXTICR3_OFFSET   (0x10U)
  #define SYSCFG_EXTICR4_OFFSET   (0x14U)
  #define SYSCFG_CMPCR_OFFSET     (0x20U)

  #define SYSCFG_MEMRMP           (*(volatile uint32_t *)(SYSCFG_BASE + SYSCFG_MEMRMP_OFFSET))
  #define SYSCFG_PMC              (*(volatile uint32_t *)(SYSCFG_BASE + SYSCFG_PMC_OFFSET))
  #define SYSCFG_EXTICR1          (*(volatile uint32_t *)(SYSCFG_BASE + SYSCFG_EXTICR1_OFFSET))
  #define SYSCFG_EXTICR2          (*(volatile uint32_t *)(SYSCFG_BASE + SYSCFG_EXTICR2_OFFSET))
  #define SYSCFG_EXTICR3          (*(volatile uint32_t *)(SYSCFG_BASE + SYSCFG_EXTICR3_OFFSET))
  #define SYSCFG_EXTICR4          (*(volatile uint32_t *)(SYSCFG_BASE + SYSCFG_EXTICR4_OFFSET))
  #define SYSCFG_CMPCR            (*(volatile uint32_t *)(SYSCFG_BASE + SYSCFG_CMPCR_OFFSET))

  /*--------SYSCFG_MEMRMP---------*/
  /*--------SYSCFG_MEMRMP---------*/

  /// @param NEW_MAP
  // 00: Main Flash memory mapped at 0x0000 0000
  // 01: System Flash memory mapped at 0x0000 0000
  // 11: Embedded SRAM mapped at 0x0000 0000
  #define SYSCFG_MEM_REMAP(NEW_MAP) \
      do { \
          CLEAR_BITS(SYSCFG_MEMRMP, 0b11U); \
          SET_BITS(SYSCFG_MEMRMP, (NEW_MAP)); \
      } while (0)

  /*--------SYSCFG_EXTICR---------*/
  /*--------SYSCFG_EXTICR---------*/

  /// @param PORT 
  // 0000: PA[x] pin
  // 0001: PB[x] pin
  // 0010: PC[x] pin
  // 0011: PD[x] pin
  // 0100: PE[x] pin
  // 0101: Reserved
  // 0110: Reserved
  // 0111: PH[x] pin
  #define EXTI_SEL_PORT(REG, PORT, BITS_OFFSET) \
    do { \
      CLEAR_BITS(REG, 0b1111U << BITS_OFFSET); \
      SET_BITS(REG, (PORT) << BITS_OFFSET); \
    } while (0)

  /// @brief Select the port on which EXTI0 should listen
  #define EXTI0_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR1, PORT, 0U)
  /// @brief Select the port on which EXTI1 should listen
  #define EXTI1_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR1, PORT, 4U)
  /// @brief Select the port on which EXTI2 should listen
  #define EXTI2_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR1, PORT, 8U)
  /// @brief Select the port on which EXTI3 should listen
  #define EXTI3_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR1, PORT, 12U)
  /// @brief Select the port on which EXTI4 should listen
  #define EXTI4_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR2, PORT, 0U)
  /// @brief Select the port on which EXTI5 should listen
  #define EXTI5_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR2, PORT, 4U)
  /// @brief Select the port on which EXTI6 should listen
  #define EXTI6_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR2, PORT, 8U)
  /// @brief Select the port on which EXTI7 should listen
  #define EXTI7_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR2, PORT, 12U)
  /// @brief Select the port on which EXTI8 should listen
  #define EXTI8_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR3, PORT, 0U)
  /// @brief Select the port on which EXTI9 should listen
  #define EXTI9_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR3, PORT, 4U)
  /// @brief Select the port on which EXTI10 should listen
  #define EXTI10_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR3, PORT, 8U)
  /// @brief Select the port on which EXTI11 should listen
  #define EXTI11_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR3, PORT, 12U)
  /// @brief Select the port on which EXTI12 should listen
  #define EXTI12_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR4, PORT, 0U)
  /// @brief Select the port on which EXTI13 should listen
  #define EXTI13_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR4, PORT, 4U)
  /// @brief Select the port on which EXTI14 should listen
  #define EXTI14_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR4, PORT, 8U)
  /// @brief Select the port on which EXTI15 should listen
  #define EXTI15_SEL_PORT(PORT) EXTI_SEL_PORT(SYSCFG_EXTICR4, PORT, 12U)

  /*--------SYSCFG_CMPCR---------*/
  /*--------SYSCFG_CMPCR---------*/

  /// @brief Enable the compensation cell
  #define SYSCFG_COMP_CELL_EN() SET_BIT(SYSCFG_CMPCR, 0U)
  /// @brief Disable the compensation cell
  #define SYSCFG_COMP_CELL_DIS() CLEAR_BIT(SYSCFG_CMPCR, 0U)
  /// @brief Check if compensation cell is enabled
  #define SYSCFG_COMP_CELL_RDY() READ_BIT(SYSCFG_CMPCR, 8U)

#endif /* SYSCFG_HELPERS */

