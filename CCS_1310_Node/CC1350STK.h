/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       CC1350STK.h
 *
 *  @brief      CC1350STK Board Specific header file.
 *
 *  The CC1350STK header file should be included in an application as
 *  follows:
 *  @code
 *  #include "CC1350STK.h"
 *  @endcode
 *  ============================================================================
 */
#ifndef __CC1350STK_BOARD_H__
#define __CC1350STK_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <ti/drivers/PIN.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>


/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Defines */
#define CC1350STK

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>            <pin mapping>
 */

/* Analog Capable DIOs */
#define CC1350STK_ANALOG0               IOID_30  // 0: solaire
#define CC1350STK_ANALOG3               IOID_27  // 1: batterie
#define CC1350STK_ANALOG4               IOID_26  //

// 3 analogueZLAL0

//  -> 28-AL0
// 29(batte)  -> AL1
// 30(solaire)-> AL2

// 2 analogue :
// 29(batt)->AL0
/* 30(solaire)-> AL1

apres inversion
30(solaire) -> AL0

*/
/* Audio */
#define CC1350STK_MIC_POWER             IOID_13
#define CC1350STK_MIC_POWER_ON          1
#define CC1350STK_MIC_POWER_OFF         0
#define CC1350STK_AUDIO_DI              IOID_2
#define CC1350STK_AUDIO_CLK             IOID_3

/* Buzzer & PWM */
#ifdef C10_CAMERA_PIR   // Pinout Buzzer et PWM
    #define CC1350STK_BUZZER                IOID_11  // Buzzer CC1310
    #define CC1310STK_PIN_PWM1              IOID_12  // LED Flash IR
#else
#ifdef CC1310_carte  // Spot-Radar-Telecommande
    #define CC1350STK_BUZZER                IOID_2  // Buzzer CC1310
    #define CC1310STK_PIN_PWM1              IOID_3  // Spot  CC1310
#else
    #define CC1350STK_BUZZER                IOID_21 // Buzzer CC1350
    #define CC1310STK_PIN_PWM1              IOID_26  // non utilise sur cc1350
#endif
#endif

#define CC1350STK_BUZZER_ON             1
#define CC1350STK_BUZZER_OFF            0

/* DevPack */
#define CC1350STK_AUDIOFS_TDO           IOID_16
#define CC1350STK_AUDIODO               IOID_22
#define CC1350STK_DP2                   IOID_23
#define CC1350STK_DP1                   IOID_24
#define CC1350STK_DP0                   IOID_25
#define CC1350STK_DP3                   IOID_27  // si modif => 80uA
#define CC1350STK_DP4_UARTRX            IOID_28
#define CC1350STK_DP5_UARTTX            IOID_29
#define CC1350STK_DEVPK_ID              IOID_30  // ex 30
#define CC1350STK_SPI_DEVPK_CS          IOID_20

/* Discrete Outputs TOR */
#define Board_GPIO_S0                   IOID_10  // Sortie 0    LED0
#ifdef C10_CAMERA_PIR   // Pinout Sorties
    #define Board_GPIO_S1                   IOID_9  // Sortie 1    Enable TP4056(charge)
    #define Board_GPIO_S2                   IOID_8  // Sortie 2    Enable ESP
#else
    #ifdef C10_sonde_HUmSol
        #define Board_GPIO_S1                   IOID_9  // Sortie 1    Highdrive      Sonde humidite
    #else
    #ifdef C10_Chien
        #define Board_GPIO_S1                   IOID_10  // Sortie 1    Alarme chien
    #else
        #define Board_GPIO_S1                   IOID_9  // Sortie 1    LED1                   EV1 ouverture, chien
    #endif
    #endif
        #define Board_GPIO_S2                   IOID_8  // Sortie 2    Enable TP4056(charge)  EV1 extinction
        #define Board_GPIO_S3                   IOID_7  // Sortie 3    Enable ESP  -  EV2 ouverture
        #define Board_GPIO_S4                   IOID_20  // Sortie 4    EV2 extinction
        #define Board_GPIO_S5                   IOID_21  // Sortie 5    EV3 ouverture
        #define Board_GPIO_S6                   IOID_25  // Sortie 6    EV3 extinction
#endif


#define CC1350STK_LED_ON                1
#define CC1350STK_LED_OFF               0



/* Discrete Inputs */
#ifdef C10_CAMERA_PIR  // Pinout Entrees
    #define CC1350STK_KEY_LEFT              IOID_23  // Entree 0 : PIR
    #define CC1350STK_KEY_RIGHT             IOID_4
#else
    #define CC1350STK_KEY_LEFT              IOID_15
    #define CC1350STK_KEY_RIGHT             IOID_4
    #define CC1350STK_RELAY                 IOID_1
#endif

/* GPIO */
#define CC1350STK_GPIO_LED_ON           1
#define CC1350STK_GPIO_LED_OFF          0

/* I2C */
#define CC1350STK_I2C0_SDA0             IOID_5
#define CC1350STK_I2C0_SCL0             IOID_6
#define CC1350STK_I2C0_SDA1             IOID_8
#define CC1350STK_I2C0_SCL1             IOID_9

/* I2S */
#define CC1350STK_I2S_ADO               IOID_25
#define CC1350STK_I2S_ADI               IOID_26
#define CC1350STK_I2S_BCLK              IOID_27
#define CC1350STK_I2S_MCLK              PIN_UNASSIGNED
#define CC1350STK_I2S_WCLK              IOID_28

/* LED-Audio DevPack */
#define CC1350STK_DEVPK_LIGHT_BLUE      IOID_23
#define CC1350STK_DEVPK_LIGHT_GREEN     IOID_24
#define CC1350STK_DEVPK_LIGHT_WHITE     IOID_25
#define CC1350STK_DEVPK_LIGHT_RED       IOID_27

/* Power */
#define CC1350STK_MPU_POWER             IOID_12
#define CC1350STK_MPU_POWER_ON          1
#define CC1350STK_MPU_POWER_OFF         0

/* PWM */
#define CC1350STK_PWMPIN0               CC1350STK_BUZZER
#define CC1350STK_PWMPIN1               CC1310STK_PIN_PWM1
/*#define CC1350STK_PWMPIN2               PIN_UNASSIGNED
#define CC1350STK_PWMPIN3               PIN_UNASSIGNED
#define CC1350STK_PWMPIN4               PIN_UNASSIGNED
#define CC1350STK_PWMPIN5               PIN_UNASSIGNED
#define CC1350STK_PWMPIN6               PIN_UNASSIGNED
#define CC1350STK_PWMPIN7               PIN_UNASSIGNED*/

/* Sensors */
#define CC1350STK_MPU_INT               IOID_7
#define CC1350STK_TMP_RDY               IOID_11

/* SPI */
#define CC1350STK_SPI_FLASH_CS          IOID_14
#define CC1350STK_FLASH_CS_ON           0
#define CC1350STK_FLASH_CS_OFF          1

/* SPI Board */
#define CC1350STK_SPI0_MISO             IOID_18
#define CC1350STK_SPI0_MOSI             IOID_19
#define CC1350STK_SPI0_CLK              IOID_17
#define CC1350STK_SPI0_CSN              PIN_UNASSIGNED
#define CC1350STK_SPI1_MISO             PIN_UNASSIGNED
#define CC1350STK_SPI1_MOSI             PIN_UNASSIGNED
#define CC1350STK_SPI1_CLK              PIN_UNASSIGNED
#define CC1350STK_SPI1_CSN              PIN_UNASSIGNED

/* UART */
#define CC1350STK_UART_TX               CC1350STK_DP5_UARTTX
#define CC1350STK_UART_RX               CC1350STK_DP4_UARTRX

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void CC1350STK_initGeneral(void);

/*!
 *  @brief  Turn off the external flash on LaunchPads
 *
 */
void CC1350STK_shutDownExtFlash(void);

/*!
 *  @brief  Wake up the external flash present on the board files
 *
 *  This function toggles the chip select for the amount of time needed
 *  to wake the chip up.
 */
void CC1350STK_wakeUpExtFlash(void);

/*!
 *  @def    CC1350_STK_ADCBufName
 *  @brief  Enum of ADCBufs
 */
typedef enum CC1350STK_ADCBufName {
    CC1350STK_ADCBUF0 = 0,

    CC1350STK_ADCBUFCOUNT
} CC1350STK_ADCBufName;

/*!
 *  @def    CC1350STK_ADCBuf0SourceName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum CC1350STK_ADCBuf0ChannelName {
    CC1350STK_ADCBUF0CHANNEL0 = 0,
    CC1350STK_ADCBUF0CHANNEL1,
    #if (NB_ANALOG >2)
      CC1350STK_ADCBUF0CHANNEL2,
    #endif

/*    CC1350_LAUNCHXL_ADCBUF0CHANNEL3,
    CC1350_LAUNCHXL_ADCBUF0CHANNEL4,
    CC1350_LAUNCHXL_ADCBUF0CHANNEL5,
    CC1350_LAUNCHXL_ADCBUF0CHANNEL6,
    CC1350_LAUNCHXL_ADCBUF0CHANNEL7, */
    CC1350STK_ADCBUF0CHANNELVDDS,
    CC1350STK_ADCBUF0CHANNELDCOUPL,
    CC1350STK_ADCBUF0CHANNELVSS,

    CC1350STK_ADCBUF0CHANNELCOUNT
} CC1350STK_ADCBuf0ChannelName;

/*!
 *  @def    CC1350_LAUNCHXL_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC1350STK_ADCName {
    CC1350STK_ADC0 = 0,
    CC1350STK_ADC1,
    #if (NB_ANALOG >2)
    CC1350STK_ADC2,
    #endif
    /*   CC1350_LAUNCHXL_ADC3,
    CC1350_LAUNCHXL_ADC4,
    CC1350_LAUNCHXL_ADC5,
    CC1350_LAUNCHXL_ADC6,
    CC1350_LAUNCHXL_ADC7,*/
    CC1350STK_ADCDCOUPL,
    CC1350STK_ADCVSS,
    CC1350STK_ADCVDDS,

    CC1350STK_ADCCOUNT
} CC1350STK_ADCName;

/*!
 *  @def    CC1350STK_CryptoName
 *  @brief  Enum of Crypto names
 */
typedef enum CC1350STK_CryptoName {
    CC1350STK_CRYPTO0 = 0,

    CC1350STK_CRYPTOCOUNT
} CC1350STK_CryptoName;

/*!
 *  @def    CC1350STK_AESCCMName
 *  @brief  Enum of AESCCM names
 */
typedef enum CC1350STK_AESCCMName {
    CC1350STK_AESCCM0 = 0,

    CC1350STK_AESCCMCOUNT
} CC1350STK_AESCCMName;

/*!
 *  @def    CC1350STK_AESGCMName
 *  @brief  Enum of AESGCM names
 */
typedef enum CC1350STK_AESGCMName {
    CC1350STK_AESGCM0 = 0,

    CC1350STK_AESGCMCOUNT
} CC1350STK_AESGCMName;

/*!
 *  @def    CC1350STK_AESCBCName
 *  @brief  Enum of AESCBC names
 */
typedef enum CC1350STK_AESCBCName {
    CC1350STK_AESCBC0 = 0,

    CC1350STK_AESCBCCOUNT
} CC1350STK_AESCBCName;

/*!
 *  @def    CC1350STK_AESCTRName
 *  @brief  Enum of AESCTR names
 */
typedef enum CC1350STK_AESCTRName {
    CC1350STK_AESCTR0 = 0,

    CC1350STK_AESCTRCOUNT
} CC1350STK_AESCTRName;

/*!
 *  @def    CC1350STK_AESECBName
 *  @brief  Enum of AESECB names
 */
typedef enum CC1350STK_AESECBName {
    CC1350STK_AESECB0 = 0,

    CC1350STK_AESECBCOUNT
} CC1350STK_AESECBName;

/*!
 *  @def    CC1350STK_AESCTRDRBGName
 *  @brief  Enum of AESCTRDRBG names
 */
typedef enum CC1350STK_AESCTRDRBGName {
    CC1350STK_AESCTRDRBG0 = 0,

    CC1350STK_AESCTRDRBGCOUNT
} CC1350STK_AESCTRDRBGName;

/*!
 *  @def    CC1350STK_TRNGName
 *  @brief  Enum of TRNG names
 */
typedef enum CC1350STK_TRNGName {
    CC1350STK_TRNG0 = 0,

    CC1350STK_TRNGCOUNT
} CC1350STK_TRNGName;

/*!
 *  @def    CC1350STK_GPIOName
 *  @brief  Enum of GPIO names
 */
typedef enum CC1350STK_GPIOName {
    CC1350STK_GPIO_S1 = 0,
    CC1350STK_GPIO_S2,
    CC1350STK_GPIO_S3,
    CC1350STK_GPIO_LED0,
    CC1350STK_GPIO_SPI_FLASH_CS,
    CC1350STK_GPIO_LCD_CS,
    CC1350STK_GPIO_LCD_ENABLE,

    CC1350STK_GPIOCOUNT
} CC1350STK_GPIOName;

/*!
 *  @def    CC1350STK_GPTimerName
 *  @brief  Enum of GPTimers parts
 */
typedef enum CC1350STK_GPTimerName {
    CC1350STK_GPTIMER0A = 0,
    CC1350STK_GPTIMER0B,
/*    CC1350STK_GPTIMER1A,
    CC1350STK_GPTIMER1B,
    CC1350STK_GPTIMER2A,
    CC1350STK_GPTIMER2B,
    CC1350STK_GPTIMER3A,
    CC1350STK_GPTIMER3B,*/

    CC1350STK_GPTIMERPARTSCOUNT
} CC1350STK_GPTimerName;

/*!
 *  @def    CC1350STK_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC1350STK_GPTimers {
    CC1350STK_GPTIMER0 = 0,
/*    CC1350STK_GPTIMER1,
    CC1350STK_GPTIMER2,
    CC1350STK_GPTIMER3,*/

    CC1350STK_GPTIMERCOUNT
} CC1350STK_GPTimers;

/*!
 *  @def    CC1350STK_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum CC1350STK_I2CName {
    CC1350STK_I2C0 = 0,

    CC1350STK_I2CCOUNT
} CC1350STK_I2CName;

/*!
 *  @def    CC1350STK_I2SName
 *  @brief  Enum of I2S names
 */
typedef enum CC1350STK_I2SName {
    CC1350STK_I2S0 = 0,

    CC1350STK_I2SCOUNT
} CC1350STK_I2SName;

/*!
 *  @def    CC1350STK_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum CC1350STK_NVSName {
    CC1350STK_NVSCC26XX0 = 0,
    CC1350STK_NVSSPI25X0,

    CC1350STK_NVSCOUNT
} CC1350STK_NVSName;

/*!
 *  @def    CC1350STK_PdmName
 *  @brief  Enum of PDM names
 */
typedef enum CC1350STK_PDMName {
    CC1350STK_PDM0 = 0,

    CC1350STK_PDMCOUNT
} CC1350STK_PDMName;

/*!
 *  @def    CC1350STK_PWM
 *  @brief  Enum of PWM outputs
 */
typedef enum CC1350STK_PWMName {   // 2 pwm suffisent pour cette appli
    CC1350STK_PWM0 = 0,
    #if (NB_PWM>1)
       CC1350STK_PWM1,
    #endif
/*    CC1350STK_PWM2,
    CC1350STK_PWM3,
    CC1350STK_PWM4,
    CC1350STK_PWM5,
    CC1350STK_PWM6,
    CC1350STK_PWM7,*/

    CC1350STK_PWMCOUNT
} CC1350STK_PWMName;

/*!
 *  @def    CC1350STK_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum CC1350STK_SPIName {
    CC1350STK_SPI0 = 0,
    CC1350STK_SPI1,

    CC1350STK_SPICOUNT
} CC1350STK_SPIName;

/*!
 *  @def    CC1350STK_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CC1350STK_UARTName {
    CC1350STK_UART0 = 0,

    CC1350STK_UARTCOUNT
} CC1350STK_UARTName;

/*!
 *  @def    CC1350STK_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC1350STK_UDMAName {
    CC1350STK_UDMA0 = 0,

    CC1350STK_UDMACOUNT
} CC1350STK_UDMAName;

/*!
 *  @def    CC1350STK_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum CC1350STK_WatchdogName {
    CC1350STK_WATCHDOG0 = 0,

    CC1350STK_WATCHDOGCOUNT
} CC1350STK_WatchdogName;

#ifdef __cplusplus
}
#endif

#endif /* __CC1350STK_BOARD_H__ */
