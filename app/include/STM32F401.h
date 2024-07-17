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



#ifndef NVIC_HAL 
#define NVIC_HAL      1
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

#endif // !NVIC_HAL

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/


#ifndef FLASH_HAL
  #define FLASH_HAL   1

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

#endif // !CRC_HAL 1


/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/


#ifndef PWR_HAL
  #define PWR_HAL   1

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

#endif // !PWR_HAL 1


/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/
/*---------------------------------------------------------------*/
/*---------------Macros for Reset and clock control--------------*/
/*---------------------------------------------------------------*/
#ifndef RCC_HAL
  #define RCC_HAL     1

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
  #define RCC_LPEN_ADC1()    SET_BIT(RCC_APB2LPENELPERSR, 8U)
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

#endif // !RCC_HAL


/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/


#ifndef SYSCFG_HAL
  #define SYSCFG_HAL  1

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

#endif /* SYSCFG_HAL */

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef GPIO_HEAL
  #define GPIO_HAL  1

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

  #define GPIOA_MODER   (*(volatile uint32_t *)(GPIOA_BASE + GPIO_MODER_OFFSET))
  #define GPIOA_OTYPER  (*(volatile uint32_t *)(GPIOA_BASE + GPIO_OTYPER_OFFSET))
  #define GPIOA_OSPEEDR (*(volatile uint32_t *)(GPIOA_BASE + GPIO_OSPEEDR_OFFSET))
  #define GPIOA_PUPDR   (*(volatile uint32_t *)(GPIOA_BASE + GPIO_PUPDR_OFFSET))
  #define GPIOA_IDR     (*(volatile uint32_t *)(GPIOA_BASE + GPIO_IDR_OFFSET))
  #define GPIOA_ODR     (*(volatile uint32_t *)(GPIOA_BASE + GPIO_ODR_OFFSET))
  #define GPIOA_BSRR    (*(volatile uint32_t *)(GPIOA_BASE + GPIO_BSRR_OFFSET))
  #define GPIOA_LCKR    (*(volatile uint32_t *)(GPIOA_BASE + GPIO_LCKR_OFFSET))
  #define GPIOA_AFRL    (*(volatile uint32_t *)(GPIOA_BASE + GPIO_AFRL_OFFSET))
  #define GPIOA_AFRH    (*(volatile uint32_t *)(GPIOA_BASE + GPIO_AFRH_OFFSET))

  #define GPIOB_MODER   (*(volatile uint32_t *)(GPIOB_BASE + GPIO_MODER_OFFSET))
  #define GPIOB_OTYPER  (*(volatile uint32_t *)(GPIOB_BASE + GPIO_OTYPER_OFFSET))
  #define GPIOB_OSPEEDR (*(volatile uint32_t *)(GPIOB_BASE + GPIO_OSPEEDR_OFFSET))
  #define GPIOB_PUPDR   (*(volatile uint32_t *)(GPIOB_BASE + GPIO_PUPDR_OFFSET))
  #define GPIOB_IDR     (*(volatile uint32_t *)(GPIOB_BASE + GPIO_IDR_OFFSET))
  #define GPIOB_ODR     (*(volatile uint32_t *)(GPIOB_BASE + GPIO_ODR_OFFSET))
  #define GPIOB_BSRR    (*(volatile uint32_t *)(GPIOB_BASE + GPIO_BSRR_OFFSET))
  #define GPIOB_LCKR    (*(volatile uint32_t *)(GPIOB_BASE + GPIO_LCKR_OFFSET))
  #define GPIOB_AFRL    (*(volatile uint32_t *)(GPIOB_BASE + GPIO_AFRL_OFFSET))
  #define GPIOB_AFRH    (*(volatile uint32_t *)(GPIOB_BASE + GPIO_AFRH_OFFSET))

  #define GPIOC_MODER   (*(volatile uint32_t *)(GPIOC_BASE + GPIO_MODER_OFFSET))
  #define GPIOC_OTYPER  (*(volatile uint32_t *)(GPIOC_BASE + GPIO_OTYPER_OFFSET))
  #define GPIOC_OSPEEDR (*(volatile uint32_t *)(GPIOC_BASE + GPIO_OSPEEDR_OFFSET))
  #define GPIOC_PUPDR   (*(volatile uint32_t *)(GPIOC_BASE + GPIO_PUPDR_OFFSET))
  #define GPIOC_IDR     (*(volatile uint32_t *)(GPIOC_BASE + GPIO_IDR_OFFSET))
  #define GPIOC_ODR     (*(volatile uint32_t *)(GPIOC_BASE + GPIO_ODR_OFFSET))
  #define GPIOC_BSRR    (*(volatile uint32_t *)(GPIOC_BASE + GPIO_BSRR_OFFSET))
  #define GPIOC_LCKR    (*(volatile uint32_t *)(GPIOC_BASE + GPIO_LCKR_OFFSET))
  #define GPIOC_AFRL    (*(volatile uint32_t *)(GPIOC_BASE + GPIO_AFRL_OFFSET))
  #define GPIOC_AFRH    (*(volatile uint32_t *)(GPIOC_BASE + GPIO_AFRH_OFFSET))

  #define GPIOD_MODER   (*(volatile uint32_t *)(GPIOD_BASE + GPIO_MODER_OFFSET))
  #define GPIOD_OTYPER  (*(volatile uint32_t *)(GPIOD_BASE + GPIO_OTYPER_OFFSET))
  #define GPIOD_OSPEEDR (*(volatile uint32_t *)(GPIOD_BASE + GPIO_OSPEEDR_OFFSET))
  #define GPIOD_PUPDR   (*(volatile uint32_t *)(GPIOD_BASE + GPIO_PUPDR_OFFSET))
  #define GPIOD_IDR     (*(volatile uint32_t *)(GPIOD_BASE + GPIO_IDR_OFFSET))
  #define GPIOD_ODR     (*(volatile uint32_t *)(GPIOD_BASE + GPIO_ODR_OFFSET))
  #define GPIOD_BSRR    (*(volatile uint32_t *)(GPIOD_BASE + GPIO_BSRR_OFFSET))
  #define GPIOD_LCKR    (*(volatile uint32_t *)(GPIOD_BASE + GPIO_LCKR_OFFSET))
  #define GPIOD_AFRL    (*(volatile uint32_t *)(GPIOD_BASE + GPIO_AFRL_OFFSET))
  #define GPIOD_AFRH    (*(volatile uint32_t *)(GPIOD_BASE + GPIO_AFRH_OFFSET))

  #define GPIOE_MODER   (*(volatile uint32_t *)(GPIOE_BASE + GPIO_MODER_OFFSET))
  #define GPIOE_OTYPER  (*(volatile uint32_t *)(GPIOE_BASE + GPIO_OTYPER_OFFSET))
  #define GPIOE_OSPEEDR (*(volatile uint32_t *)(GPIOE_BASE + GPIO_OSPEEDR_OFFSET))
  #define GPIOE_PUPDR   (*(volatile uint32_t *)(GPIOE_BASE + GPIO_PUPDR_OFFSET))
  #define GPIOE_IDR     (*(volatile uint32_t *)(GPIOE_BASE + GPIO_IDR_OFFSET))
  #define GPIOE_ODR     (*(volatile uint32_t *)(GPIOE_BASE + GPIO_ODR_OFFSET))
  #define GPIOE_BSRR    (*(volatile uint32_t *)(GPIOE_BASE + GPIO_BSRR_OFFSET))
  #define GPIOE_LCKR    (*(volatile uint32_t *)(GPIOE_BASE + GPIO_LCKR_OFFSET))
  #define GPIOE_AFRL    (*(volatile uint32_t *)(GPIOE_BASE + GPIO_AFRL_OFFSET))
  #define GPIOE_AFRH    (*(volatile uint32_t *)(GPIOE_BASE + GPIO_AFRH_OFFSET))

  #define GPIOH_MODER   (*(volatile uint32_t *)(GPIOE_BASE + GPIO_MODER_OFFSET))
  #define GPIOH_OTYPER  (*(volatile uint32_t *)(GPIOE_BASE + GPIO_OTYPER_OFFSET))
  #define GPIOH_OSPEEDR (*(volatile uint32_t *)(GPIOE_BASE + GPIO_OSPEEDR_OFFSET))
  #define GPIOH_PUPDR   (*(volatile uint32_t *)(GPIOE_BASE + GPIO_PUPDR_OFFSET))
  #define GPIOH_IDR     (*(volatile uint32_t *)(GPIOE_BASE + GPIO_IDR_OFFSET))
  #define GPIOH_ODR     (*(volatile uint32_t *)(GPIOE_BASE + GPIO_ODR_OFFSET))
  #define GPIOH_BSRR    (*(volatile uint32_t *)(GPIOE_BASE + GPIO_BSRR_OFFSET))
  #define GPIOH_LCKR    (*(volatile uint32_t *)(GPIOE_BASE + GPIO_LCKR_OFFSET))
  #define GPIOH_AFRL    (*(volatile uint32_t *)(GPIOE_BASE + GPIO_AFRL_OFFSET))
  #define GPIOH_AFRH    (*(volatile uint32_t *)(GPIOE_BASE + GPIO_AFRH_OFFSET))

  #define GPIOx_BASE(GPIO_PORT)     GPIO##GPIO_PORT##_BASE
  #define GPIOx_MODER(GPIO_PORT)    GPIO##GPIO_PORT##_MODER
  #define GPIOx_OTYPER(GPIO_PORT)   GPIO##GPIO_PORT##_OTYPER
  #define GPIOx_OSPEEDR(GPIO_PORT)  GPIO##GPIO_PORT##_OSPEEDR
  #define GPIOx_PUPDR(GPIO_PORT)    GPIO##GPIO_PORT##_PUPDR
  #define GPIOx_IDR(GPIO_PORT)      GPIO##GPIO_PORT##_IDR
  #define GPIOx_ODR(GPIO_PORT)      GPIO##GPIO_PORT##_ODR
  #define GPIOx_BSRR(GPIO_PORT)     GPIO##GPIO_PORT##_BSRR
  #define GPIOx_LCKR(GPIO_PORT)     GPIO##GPIO_PORT##_LCKR


  /*--------GPIOx_MODER---------*/
  /*--------GPIOx_MODER---------*/

  /// @param PIN_N is the pin number of the port (0..15)
  /// @param MODE 
  // 00: Input (reset state) /
  // 01: General purpose output mode /
  // 10: Alternate function mode /
  // 11: Analog mode
  #define GPIOx_SET_MODE(GPIO_PORT, PIN_N, MODE) \
    do { \
      CLEAR_BITS(GPIOx_MODER(GPIO_PORT), 0b11U << ((PIN_N) / 2U)); \
      SET_BITS(GPIOx_MODER(GPIO_PORT), (MODE) << ((PIN_N) / 2U)); \
    } while (0)



  /*--------GPIOx_OTYPER---------*/
  /*--------GPIOx_OTYPER---------*/

  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOx_SET_OPENDRAIN(GPIO_PORT, PIN_N) \
    SET_BIT(GPIOx_OTYPER(GPIO_PORT), PIN_N)
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOx_SET_PUSHPULL(GPIO_PORT, PIN_N) \
    CLEAR_BIT(GPIOx_OTYPER(GPIO_PORT), PIN_N)


  /*--------GPIOx_OSPEEDR---------*/
  /*--------GPIOx_OSPEEDR---------*/

  #define GPIOx_SET_OSPEED(GPIO_PORT, PIN_N, OSPEED) \
    do { \
      CLEAR_BITS(GPIOx_OSPEEDR(GPIO_PORT), 0b11U << ((PIN_N) / 2U)); \
      SET_BITS(GPIOx_OSPEEDR(GPIO_PORT), (OSPEED) << ((PIN_N) / 2U)); \
    } while (0)


  /*--------GPIOx_PUPDR---------*/
  /*--------GPIOx_PUPDR---------*/

  /// @param GPIO_BASE is the base address of the GPIO port
  /// @param PIN_N is the pin number of the port (0..15)
  /// @param STATE Dictates the pull-up pull-down state of the pin
  // 00: No pull-up, pull-down
  // 01: Pull-up
  // 10: Pull-down
  // 11: Reserved
  #define GPIO_SET_PUPD(GPIO_PORT, PIN_N, STATE) \
    do { \
      CLEAR_BITS(GPIOx_PUPDR(GPIO_PORT), 0b11U << ((PIN_N) * 2U)); \
      SET_BITS(GPIOx_PUPDR(GPIO_PORT), (STATE) << ((PIN_N) * 2U)); \
    } while (0)


  /*--------GPIOx_IDR---------*/
  /*--------GPIOx_IDR---------*/

  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOx_PINREAD(GPIO_PORT, PIN_N) READ_BIT(GPIOx_IDR(GPIO_PORT), (PIN_N))

  /*--------GPIOx_BSSR---------*/
  /*--------GPIOx_BSSR---------*/

  /// @param GPIO_BASE is the base address of the GPIO port
  /// @param PIN_N is the pin number of the port (0..15)
  #define GPIOx_PIN_HIGH(GPIO_PORT, PIN_N) \
    SET_BIT(GPIOx_BSSR(GPIO_PORT), PIN_N)
  #define GPIOx_PIN_LOW(GPIO_PORT, PIN_N) \
    SET_BIT(GPIOx_BSSR(GPIO_PORT), PIN_N + 16U)


  /*--------GPIOx_AFRL/AFRH---------*/
  /*--------GPIOx_AFRL/AFRH---------*/

  /**
  * @brief  Set the alternate function for a GPIO pin.
  * @note   This macro modifies the GPIO alternate function register (AFRL/AFRH) for a given pin.
  * @param  GPIO_PORT The GPIO port (A, B, C, D, E, H) where the pin belongs.
  * @param  PIN_N     The pin number within the GPIO port (0 to 15).
  * @param  AF_NUM    The alternate function number to set (0 to 15).
  * @retval None
  */
  #define GPIO_SET_AF(GPIO_PORT, PIN_N, AF_NUM) \
    do { \
        /* Calculate the base address of GPIOx */ \
        uint32_t gpio_base = GPIOx_BASE(GPIO_PORT); \
        /* Calculate the offset of AFRL or AFRH based on PIN_N */ \
        uint32_t af_offset = ((PIN_N) < 8U) ? GPIO_AFRL_OFFSET : GPIO_AFRH_OFFSET; \
        /* Calculate the bit shift position for the AF register */ \
        uint32_t af_shift = (((PIN_N) & 0x7U) * 4U); \
        /* Mask and set the alternate function bits */ \
        MODIFY_REG(gpio_base + af_offset, 0xFU << af_shift, (AF_NUM) << af_shift); \
    } while (0)


  /// Example usage:
  /// Set alternate function AF_NUM for pin PIN_N on GPIO_PORT
  #define GPIO_SET_AF_BY_PORT(GPIO_PORT, PIN_N, AF_NUM) \
    GPIO_SET_AF(GPIO_PORT, PIN_N, AF_NUM)


#endif // !GPIO_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef DMA_HAL
  #define DMA_HAL 1

#endif // !DMA_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef EXTI_HAL
  #define EXTI_HAL 1

  #define EXTI_BASE         (0x40013C00U)
  #define EXTI_IMR_OFFSET   (0x00U)
  #define EXTI_EMR_OFFSET   (0x04U)
  #define EXTI_RSTR_OFFSET  (0x08U)
  #define EXTI_FTSR_OFFSET  (0x0CU)
  #define EXTI_SWIER_OFFSET (0x10U)
  #define EXTI_PR_OFFSET    (0x14U)

  #define EXTI_IMR   (*(volatile uint32_t *)(EXTI_BASE + EXTI_IMR_OFFSET)))
  #define EXTI_EMR   (*(volatile uint32_t *)(EXTI_BASE + EXTI_EMR_OFFSET)))
  #define EXTI_RTSR  (*(volatile uint32_t *)(EXTI_BASE + EXTI_RSTR_OFFSET)))
  #define EXTI_FTSR  (*(volatile uint32_t *)(EXTI_BASE + EXTI_FTSR_OFFSET)))
  #define EXTI_SWIER (*(volatile uint32_t *)(EXTI_BASE + EXTI_SWIER_OFFSET)))
  #define EXTI_PR    (*(volatile uint32_t *)(EXTI_BASE + EXTI_PR_OFFSET)))

/*--------EXTI_IMR---------*/
/*--------EXTI_IMR---------*/

  /** @brief Masks interrupts on the LINE
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_IMASK(LINE) CLEAR_BIT(EXTI_IMR, LINE)

  /** @brief Do not mask interrupts on LINE
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_INOMASK(LINE) SET_BIT(EXTI_IMR, LINE)

/*--------EXTI_EMR---------*/
/*--------EXTI_EMR---------*/

  /** @brief Masks events on the LINE
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_EMASK(LINE) CLEAR_BIT(EXTI_EMR, LINE)

  /** @brief Do not mask events on LINE
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_ENOMASK(LINE) SET_BIT(EXTI_EMR, LINE)

/*--------EXTI_RTSR---------*/
/*--------EXTI_RTSR---------*/

  /** @brief Configure interrupt or event generation on rising edge on LINE input
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_EN_RISING_TRIG(LINE) SET_BIT(EXTI_RTSR, LINE)

  /** @brief No interrupt or event on rising edge on LINE  
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_DIS_RISING_TRIG(LINE) CLEAR_BIT(EXTI_RTSR, LINE)

/*--------EXTI_FTSR---------*/
/*--------EXTI_FTSR---------*/

  /** @brief Configure interrupt or event generation on falling edge on LINE input
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_EN_FALLING_TRIG(LINE) SET_BIT(EXTI_FTSR, LINE)

  /** @brief No interrupt or event on falling edge on LINE input
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_DIS_FALLING_TRIG(LINE) CLEAR_BIT(EXTI_FTSR, LINE)

/*--------EXTI_SWIER---------*/
/*--------EXTI_SWIER---------*/

  /** @brief Triggers a swi on LINE
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_SWI_TRIGGER(LINE) SET_BIT(EXTI_SWIER, LINE) 

  /** @brief checks for a swi on LINE
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_SWI_CHECK(LINE) READ_BIT(EXTI_SWIER, LINE) 

/*--------EXTI_PR---------*/
/*--------EXTI_PR---------*/

  /** @brief checks if LINE has an event or interrupt pending
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_CHECK_PEND(LINE) READ_BIT(EXTI_PR, LINE)

  /** @brief remove pending event or interrupt on LINE  
   *  @param LINE (0..18, 21, 22)
   */
  #define EXTI_CLEAR_PEND(LINE) SET_BIT(EXTI_PR, LINE)

#endif // !EXTI_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef ADC_HAL
  #define ADC_HAL 1



#endif // !ADC_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef TIM1_HAL
  #define TIM1_HAL 1



#endif // !TIM1_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef TIM2_5_HAL
  #define TIM2_5_HAL 1



#endif // !TIM2_5_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef TIM9_11_HAL
  #define TIM9_11_HAL 1



#endif // !TIM9_11_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef IWDG_HAL
  #define IWDG_HAL 1



#endif // !IWDG_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef WWDG_HAL
  #define WWDG_HAL 1



#endif // !WWDG_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef RTC_HAL
  #define RTC_HAL 1



#endif // !RTC_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef I2C_HAL
  #define I2C_HAL 1



#endif // !I2C_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef USART_HAL
  #define USART_HAL 1

  #define USART6_BASE 0x40011400U
  #define USART1_BASE 0x40011000U
  #define USART2_BASE 0x40004400U

  #define USART_SR_OFFSET     (0x00U)
  #define USART_DR_OFFSET     (0x04U)
  #define USART_BRR_OFFSET    (0x08U)
  #define USART_CR1_OFFSET    (0x0CU)
  #define USART_CR2_OFFSET    (0x10U)
  #define USART_CR3_OFFSET    (0x14U)
  #define USART_GTPR_OFFSET   (0x18U)

  #define USART6_SR     (*(volatile uint32_t *)(USART6_BASE + USART_SR_OFFSET))
  #define USART6_DR     (*(volatile uint32_t *)(USART6_BASE + USART_DR_OFFSET))
  #define USART6_BRR    (*(volatile uint32_t *)(USART6_BASE + USART_BRR_OFFSET))
  #define USART6_CR1    (*(volatile uint32_t *)(USART6_BASE + USART_CR1_OFFSET))
  #define USART6_CR2    (*(volatile uint32_t *)(USART6_BASE + USART_CR2_OFFSET))
  #define USART6_CR3    (*(volatile uint32_t *)(USART6_BASE + USART_CR3_OFFSET))
  #define USART6_GTPR   (*(volatile uint32_t *)(USART6_BASE + USART_GTPR_OFFSET))


  #define USART1_SR     (*(volatile uint32_t *)(USART1_BASE + USART_SR_OFFSET))
  #define USART1_DR     (*(volatile uint32_t *)(USART1_BASE + USART_DR_OFFSET))
  #define USART1_BRR    (*(volatile uint32_t *)(USART1_BASE + USART_BRR_OFFSET))
  #define USART1_CR1    (*(volatile uint32_t *)(USART1_BASE + USART_CR1_OFFSET))
  #define USART1_CR2    (*(volatile uint32_t *)(USART1_BASE + USART_CR2_OFFSET))
  #define USART1_CR3    (*(volatile uint32_t *)(USART1_BASE + USART_CR3_OFFSET))
  #define USART1_GTPR   (*(volatile uint32_t *)(USART1_BASE + USART_GTPR_OFFSET))

  #define USART2_SR     (*(volatile uint32_t *)(USART2_BASE + USART_SR_OFFSET))
  #define USART2_DR     (*(volatile uint32_t *)(USART2_BASE + USART_DR_OFFSET))
  #define USART2_BRR    (*(volatile uint32_t *)(USART2_BASE + USART_BRR_OFFSET))
  #define USART2_CR1    (*(volatile uint32_t *)(USART2_BASE + USART_CR1_OFFSET))
  #define USART2_CR2    (*(volatile uint32_t *)(USART2_BASE + USART_CR2_OFFSET))
  #define USART2_CR3    (*(volatile uint32_t *)(USART2_BASE + USART_CR3_OFFSET))
  #define USART2_GTPR   (*(volatile uint32_t *)(USART2_BASE + USART_GTPR_OFFSET))

  #define USARTx_SR(USART_NUM) USART##USART_NUM##_SR
  #define USARTx_DR(USART_NUM) USART##USART_NUM##_DR
  #define USARTx_BRR(USART_NUM) USART##USART_NUM##_BRR
  #define USARTx_CR1(USART_NUM) USART##USART_NUM##_CR1
  #define USARTx_CR2(USART_NUM) USART##USART_NUM##_CR2
  #define USARTx_CR3(USART_NUM) USART##USART_NUM##_CR3
  #define USARTx_GTPR(USART_NUM) USART##USART_NUM##_GTPR

/*--------USARTx_SR---------*/
/*--------USARTx_SR---------*/

  #define USARTx_CHECK_CTS_FLAG(USART_NUM)  READ_BIT(USARTx_SR(USART_NUM), 9U)
  #define USARTx_CLEAR_CTS_FLAG(USART_NUM)  CLEAR_BIT(USARTx_SR(USART_NUM), 9U)
  #define USARTx_CHECK_LBD_FLAG(USART_NUM)  READ_BIT(USARTx_SR(USART_NUM), 8U)
  #define USARTx_CLEAR_LBD_FLAG(USART_NUM)  CLEAR_BIT(USARTx_SR(USART_NUM), 8U)
  #define USARTx_CLEAR_TXE_FLAG(USART_NUM)  READ_BIT(USARTx_SR(USART_NUM), 7U)  
  #define USARTx_CHECK_TC_FLAG(USART_NUM)   READ_BIT(USARTx_SR(USART_NUM), 6U)
  #define USARTx_CLEAR_TC_FLAG(USART_NUM)   CLEAR_BIT(USARTx_SR(USART_NUM), 6U)
  #define USARTx_CHECK_RXNE_FLAG(USART_NUM) READ_BIT(USARTx_SR(USART_NUM), 5U)
  #define USARTx_CLEAR_RXNE_FLAG(USART_NUM) CLEAR_BIT(USARTx_SR(USART_NUM), 5U)
  #define USARTx_CHECK_IDLE_FLAG(USART_NUM) READ_BIT(USARTx_SR(USART_NUM), 5U)
  #define USARTx_CHECK_ORE_FLAG(USART_NUM)  READ_BIT(USARTx_SR(USART_NUM), 5U)
  #define USARTx_CHECK_NF_FLAG(USART_NUM)   READ_BIT(USARTx_SR(USART_NUM), 5U)
  #define USARTx_CHECK_FE_FLAG(USART_NUM)   READ_BIT(USARTx_SR(USART_NUM), 5U)
  #define USARTx_CHECK_PE_FLAG(USART_NUM)   READ_BIT(USARTx_SR(USART_NUM), 5U)

/*--------USARTx_DR---------*/
/*--------USARTx_DR---------*/

  #define USARTx_READ_DATA(USART_NUM) (USARTx_DR(USART_NUM))

  #define USARTx_WRITE_DATA(USART_NUM, DATA) \
    do { \
      USARTx_DR(USART_NUM) = (DATA); \  
    } while(0)

/*--------USARTx_BRR---------*/
/*--------USARTx_BRR---------*/

  #define USARTx_SET_BAUD_RATE(USART_NUM, BAUD_RATE, PCLK) \
    do { \
      USARTx_BRR(USART_NUM) = ((PCLK) / (BAUD_RATE)); \
    } while(0)

/*--------USARTx_CR1---------*/
/*--------USARTx_CR1---------*/

  #define USARTx_SET_OVER16(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 15U)
  #define USARTx_SET_OVER8(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 15U)

  #define USARTx_DIS(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 13U)
  #define USARTx_EN(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 13U)

  #define USARTx_SET_WORDLEN8(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 12U)
  #define USARTx_SET_WORDLEN9(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 12U)

  #define USARTx_IDLE_WKUP(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 11U)
  #define USARTx_ADDR_MARK_WKUP(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 11U)

  #define USARTx_PARITY_DIS(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 10U)
  #define USARTx_PARITY_EN(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 10U)

  #define USARTx_SET_EVEN_PARITY(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 9U)
  #define USARTx_SET_ODD_PARITY(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 9U)

  #define USARTx_PEI_DIS(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 8U)
  #define USARTx_PEI_EN(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 8U)

  #define USARTx_TXEI_DIS(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 7U)
  #define USARTx_TXEI_EN(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 7U)

  #define USARTx_TCI_DIS(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 6U)
  #define USARTx_SET_EN(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 6U)

  #define USARTx_RXN_DIS(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 5U)
  #define USARTx_RXN_EN(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 5U)

  #define USARTx_IDLEI_DIS(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 4U)
  #define USARTx_IDLEI_EN(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 4U)

  #define USARTx_TRANS_DIS(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 3U)
  #define USARTx_TRANS_EN(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 3U)

  #define USARTx_RECV_DIS(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 2U)
  #define USARTx_RECV_EN(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 2U)

  #define USARTx_RCV_ACTIVE(USART_NUM) CLEAR_BIT(USARTx_CR1(USART_NUM), 1U)
  #define USARTx_RCV_MUTE(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 1U)

  #define USARTx_SEND_BREAK(USART_NUM) SET_BIT(USARTx_CR1(USART_NUM), 0U)

/*--------USARTx_CR2---------*/
/*--------USARTx_CR2---------*/

  #define USARTx_LINEN_EN(USART_NUM)        SET_BIT(USARTx_CR2(USART_NUM), 14U)
  #define USARTx_LINEN_DIS(USART_NUM)       CLEAR_BIT(USARTx_CR2(USART_NUM), 14U)

  #define USARTx_STOPBITS(USART_NUM, BIT_MASK) \
    do { \
      CLEAR_BITS(USARTx_CR2(USART_NUM), 0b11U << 12U); \
      SET_BIT(USARTx_CR2(USART_NUM), BIT_MASK << 12U); \
    } while(0)

  #define USARTx_STOPBITS_1(USART_NUM)      USARTx_STOPBITS(USART_NUM, 0b00U)
  #define USARTx_STOPBITS_0_5(USART_NUM)    USARTx_STOPBITS(USART_NUM, 0b01U)
  #define USARTx_STOPBITS_2(USART_NUM)      USARTx_STOPBITS(USART_NUM, 0b10U)
  #define USARTx_STOPBITS_1_5(USART_NUM)    USARTx_STOPBITS(USART_NUM, 0b11U)

  #define USARTx_CLKEN(USART_NUM)           SET_BIT(USARTx_CR2(USART_NUM), 11U)
  #define USARTx_CLKDIS(USART_NUM)          CLEAR_BIT(USARTx_CR2(USART_NUM), 11U)

  #define USARTx_CPOL_LOW(USART_NUM)        CLEAR_BIT(USARTx_CR2(USART_NUM), 10U)
  #define USARTx_CPOL_HIGH(USART_NUM)       SET_BIT(USARTx_CR2(USART_NUM), 10U)

  #define USARTx_CPHA_FIRST(USART_NUM)      CLEAR_BIT(USARTx_CR2(USART_NUM), 9U)
  #define USARTx_CPHA_SECOND(USART_NUM)     SET_BIT(USARTx_CR2(USART_NUM), 9U)

  #define USARTx_LBCL_NORMAL(USART_NUM)     CLEAR_BIT(USARTx_CR2(USART_NUM), 8U)
  #define USARTx_LBCL_CLOCK(USART_NUM)      SET_BIT(USARTx_CR2(USART_NUM), 8U)

  #define USARTx_LBDIE_EN(USART_NUM)        SET_BIT(USARTx_CR2(USART_NUM), 6U)
  #define USARTx_LBDIE_DIS(USART_NUM)       CLEAR_BIT(USARTx_CR2(USART_NUM), 6U)

  #define USARTx_LBDL_10BIT(USART_NUM)      CLEAR_BIT(USARTx_CR2(USART_NUM), 5U)
  #define USARTx_LBDL_11BIT(USART_NUM)      SET_BIT(USARTx_CR2(USART_NUM), 5U)


  /*--------USARTx_CR3---------*/
  /*--------USARTx_CR3---------*/

  #define USARTx_ONEBIT_EN(USART_NUM)       SET_BIT(USARTx_CR3(USART_NUM), 11U)
  #define USARTx_ONEBIT_DIS(USART_NUM)      CLEAR_BIT(USARTx_CR3(USART_NUM), 11U)

  #define USARTx_CTSIE_EN(USART_NUM)        SET_BIT(USARTx_CR3(USART_NUM), 10U)
  #define USARTx_CTSIE_DIS(USART_NUM)       CLEAR_BIT(USARTx_CR3(USART_NUM), 10U)

  #define USARTx_CTSE_EN(USART_NUM)         SET_BIT(USARTx_CR3(USART_NUM), 9U)
  #define USARTx_CTSE_DIS(USART_NUM)        CLEAR_BIT(USARTx_CR3(USART_NUM), 9U)

  #define USARTx_RTSE_EN(USART_NUM)         SET_BIT(USARTx_CR3(USART_NUM), 8U)
  #define USARTx_RTSE_DIS(USART_NUM)        CLEAR_BIT(USARTx_CR3(USART_NUM), 8U)

  #define USARTx_DMAT_EN(USART_NUM)         SET_BIT(USARTx_CR3(USART_NUM), 7U)
  #define USARTx_DMAT_DIS(USART_NUM)        CLEAR_BIT(USARTx_CR3(USART_NUM), 7U)

  #define USARTx_DMAR_EN(USART_NUM)         SET_BIT(USARTx_CR3(USART_NUM), 6U)
  #define USARTx_DMAR_DIS(USART_NUM)        CLEAR_BIT(USARTx_CR3(USART_NUM), 6U)

  #define USARTx_SCEN_EN(USART_NUM)         SET_BIT(USARTx_CR3(USART_NUM), 5U)
  #define USARTx_SCEN_DIS(USART_NUM)        CLEAR_BIT(USARTx_CR3(USART_NUM), 5U)

  #define USARTx_NACK_EN(USART_NUM)         SET_BIT(USARTx_CR3(USART_NUM), 4U)
  #define USARTx_NACK_DIS(USART_NUM)        CLEAR_BIT(USARTx_CR3(USART_NUM), 4U)

  #define USARTx_HDSEL_EN(USART_NUM)        SET_BIT(USARTx_CR3(USART_NUM), 3U)
  #define USARTx_HDSEL_DIS(USART_NUM)       CLEAR_BIT(USARTx_CR3(USART_NUM), 3U)

  #define USARTx_IRLP_EN(USART_NUM)         SET_BIT(USARTx_CR3(USART_NUM), 2U)
  #define USARTx_IRLP_DIS(USART_NUM)        CLEAR_BIT(USARTx_CR3(USART_NUM), 2U)

  #define USARTx_IREN_EN(USART_NUM)         SET_BIT(USARTx_CR3(USART_NUM), 1U)
  #define USARTx_IREN_DIS(USART_NUM)        CLEAR_BIT(USARTx_CR3(USART_NUM), 1U)

  #define USARTx_EIE_EN(USART_NUM)          SET_BIT(USARTx_CR3(USART_NUM), 0U)
  #define USARTx_EIE_DIS(USART_NUM)         CLEAR_BIT(USARTx_CR3(USART_NUM), 0U)

  /*--------USARTx_GTPR---------*/
  /*--------USARTx_GTPR---------*/

  #define USARTx_GT(USART_NUM, GT) \
    do { \
      USARTx_GTPR(USART_NUM) &= ~(0xFFU << 8U); \
      USARTx_GTPR(USART_NUM) |= ((GT) & 0xFFU) << 8U; \
    } while(0)

  #define USARTx_PSC(USART_NUM, PSC) \
    do { \
      USARTx_GTPR(USART_NUM) &= ~0xFFU; \
      USARTx_GTPR(USART_NUM) |= (PSC) & 0xFFU; \
    } while(0)


#endif // !USART_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef SDIO_HAL
  #define SDIO_HAL 1



#endif // !SDIO_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/

#ifndef OTG_FS_HAL
  #define OTG_FS_HAL 1



#endif // !OTG_FS_HAL 1

/*=============================================================================*/
/*=============================================================================*/
/*=============================================================================*/
