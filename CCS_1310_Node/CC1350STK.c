/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
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

/*
 *  ====================== CC1350STK.c =========================================
 *  This file is responsible for setting up the board specific items for the
 *  CC1350STK board.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <ti/devices/cc13x0/driverlib/udma.h>
#include <ti/devices/cc13x0/inc/hw_ints.h>
#include <ti/devices/cc13x0/inc/hw_memmap.h>

#include "board.h"
#include "CC1350STK.h"

#include "utilitaires.h"

/*
 *  =============================== ADCBuf ===============================
 */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufCC26XX.h>

ADCBufCC26XX_Object adcBufCC26XXobjects[CC1350STK_ADCBUFCOUNT];

/*
 *  This table converts a virtual adc channel into a dio and internal analogue
 *  input signal. This table is necessary for the functioning of the adcBuf
 *  driver. Comment out unused entries to save flash. Dio and internal signal
 *  pairs are hardwired. Do not remap them in the table. You may reorder entire
 *  entries. The mapping of dio and internal signals is package dependent.
 */
const ADCBufCC26XX_AdcChannelLutEntry ADCBufCC26XX_adcChannelLut[CC1350STK_ADCBUF0CHANNELCOUNT] = {
/*    {CC1350STK_DIO23_ANALOG, ADC_COMPB_IN_AUXIO7},
    {CC1350STK_DIO24_ANALOG, ADC_COMPB_IN_AUXIO6},
    {CC1350STK_DIO25_ANALOG, ADC_COMPB_IN_AUXIO5},
    {CC1350STK_DIO26_ANALOG, ADC_COMPB_IN_AUXIO4},
    {CC1350STK_DIO27_ANALOG, ADC_COMPB_IN_AUXIO3},*/
    #if (NB_ANALOG >2)
      {CC1350STK_ANALOG4, ADC_COMPB_IN_AUXIO4},
    #endif
    {CC1350STK_ANALOG3, ADC_COMPB_IN_AUXIO3},
    {CC1350STK_ANALOG0, ADC_COMPB_IN_AUXIO0},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VDDS},
    {PIN_UNASSIGNED, ADC_COMPB_IN_DCOUPL},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VSS},
};

const ADCBufCC26XX_HWAttrs adcBufCC26XXHWAttrs[CC1350STK_ADCBUFCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
        .adcChannelLut     = ADCBufCC26XX_adcChannelLut,
    }
};

const ADCBuf_Config ADCBuf_config[CC1350STK_ADCBUFCOUNT] = {
    {
        &ADCBufCC26XX_fxnTable,
        &adcBufCC26XXobjects[CC1350STK_ADCBUF0],
        &adcBufCC26XXHWAttrs[CC1350STK_ADCBUF0]
    },
};

const uint_least8_t ADCBuf_count = CC1350STK_ADCBUFCOUNT;

/*
 *  =============================== ADC ===============================
 */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>

ADCCC26XX_Object adcCC26xxObjects[CC1350STK_ADCCOUNT];

const ADCCC26XX_HWAttrs adcCC26xxHWAttrs[CC1350STK_ADCCOUNT] = {
    {
     .adcDIO              = CC1350STK_ANALOG0,  // Ana0 : PIN 30
     .adcCompBInput       = ADC_COMPB_IN_AUXIO0,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_10P9_MS,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
     .adcDIO              = CC1350STK_ANALOG3,     // Ana1 : PIN 27
     .adcCompBInput       = ADC_COMPB_IN_AUXIO3,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_10P9_MS,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    #if (NB_ANALOG >2)
    {
     .adcDIO              = CC1350STK_ANALOG4,      // PIN 26
     .adcCompBInput       = ADC_COMPB_IN_AUXIO4,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_10P9_MS,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    #endif
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_DCOUPL,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_VSS,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_VDDS,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    }
};

const ADC_Config ADC_config[CC1350STK_ADCCOUNT] = {
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADC0], &adcCC26xxHWAttrs[CC1350STK_ADC0]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADC1], &adcCC26xxHWAttrs[CC1350STK_ADC1]},
    #if (NB_ANALOG >2)
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADC2], &adcCC26xxHWAttrs[CC1350STK_ADC2]},
    #endif
    /*    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADC3], &adcCC26xxHWAttrs[CC1350STK_ADC3]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADC4], &adcCC26xxHWAttrs[CC1350STK_ADC4]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADC5], &adcCC26xxHWAttrs[CC1350STK_ADC5]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADC6], &adcCC26xxHWAttrs[CC1350STK_ADC6]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADC7], &adcCC26xxHWAttrs[CC1350STK_ADC7]}, */
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADCDCOUPL], &adcCC26xxHWAttrs[CC1350STK_ADCDCOUPL]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADCVSS], &adcCC26xxHWAttrs[CC1350STK_ADCVSS]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1350STK_ADCVDDS], &adcCC26xxHWAttrs[CC1350STK_ADCVDDS]},
};

const uint_least8_t ADC_count = CC1350STK_ADCCOUNT;


/*
 *  =============================== Crypto ===============================
 */
#include <ti/drivers/crypto/CryptoCC26XX.h>

CryptoCC26XX_Object cryptoCC26XXObjects[CC1350STK_CRYPTOCOUNT];

const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC1350STK_CRYPTOCOUNT] = {
    {
        .baseAddr       = CRYPTO_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_CRYPTO,
        .intNum         = INT_CRYPTO_RESULT_AVAIL_IRQ,
        .intPriority    = ~0,
    }
};

const CryptoCC26XX_Config CryptoCC26XX_config[CC1350STK_CRYPTOCOUNT] = {
    {
         .object  = &cryptoCC26XXObjects[CC1350STK_CRYPTO0],
         .hwAttrs = &cryptoCC26XXHWAttrs[CC1350STK_CRYPTO0]
    }
};

/*
 *  =============================== AESCCM ===============================
 */
#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMCC26XX.h>

AESCCMCC26XX_Object aesccmCC26XXObjects[CC1350STK_AESCCMCOUNT];

const AESCCMCC26XX_HWAttrs aesccmCC26XXHWAttrs[CC1350STK_AESCCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCCM_Config AESCCM_config[CC1350STK_AESCCMCOUNT] = {
    {
         .object  = &aesccmCC26XXObjects[CC1350STK_AESCCM0],
         .hwAttrs = &aesccmCC26XXHWAttrs[CC1350STK_AESCCM0]
    },
};

const uint_least8_t AESCCM_count = CC1350STK_AESCCMCOUNT;


/*
 *  =============================== AESGCM ===============================
 */
#include <ti/drivers/AESGCM.h>
#include <ti/drivers/aesgcm/AESGCMCC26XX.h>

AESGCMCC26XX_Object aesgcmCC26XXObjects[CC1350STK_AESGCMCOUNT];

const AESGCMCC26XX_HWAttrs aesgcmCC26XXHWAttrs[CC1350STK_AESGCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESGCM_Config AESGCM_config[CC1350STK_AESGCMCOUNT] = {
    {
         .object  = &aesgcmCC26XXObjects[CC1350STK_AESGCM0],
         .hwAttrs = &aesgcmCC26XXHWAttrs[CC1350STK_AESGCM0]
    },
};

const uint_least8_t AESGCM_count = CC1350STK_AESGCMCOUNT;

/*
 *  =============================== AESCBC ===============================
 */
#include <ti/drivers/AESCBC.h>
#include <ti/drivers/aescbc/AESCBCCC26XX.h>

AESCBCCC26XX_Object aescbcCC26XXObjects[CC1350STK_AESCBCCOUNT];

const AESCBCCC26XX_HWAttrs aescbcCC26XXHWAttrs[CC1350STK_AESCBCCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCBC_Config AESCBC_config[CC1350STK_AESCBCCOUNT] = {
    {
         .object  = &aescbcCC26XXObjects[CC1350STK_AESCBC0],
         .hwAttrs = &aescbcCC26XXHWAttrs[CC1350STK_AESCBC0]
    },
};

const uint_least8_t AESCBC_count = CC1350STK_AESCBCCOUNT;

/*
 *  =============================== AESCTR ===============================
 */
#include <ti/drivers/AESCTR.h>
#include <ti/drivers/aesctr/AESCTRCC26XX.h>

AESCTRCC26XX_Object aesctrCC26XXObjects[CC1350STK_AESCTRCOUNT];

const AESCTRCC26XX_HWAttrs aesctrCC26XXHWAttrs[CC1350STK_AESCTRCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCTR_Config AESCTR_config[CC1350STK_AESCTRCOUNT] = {
    {
         .object  = &aesctrCC26XXObjects[CC1350STK_AESCTR0],
         .hwAttrs = &aesctrCC26XXHWAttrs[CC1350STK_AESCTR0]
    },
};

const uint_least8_t AESCTR_count = CC1350STK_AESCTRCOUNT;

/*
 *  =============================== AESECB ===============================
 */
#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBCC26XX.h>

AESECBCC26XX_Object aesecbCC26XXObjects[CC1350STK_AESECBCOUNT];

const AESECBCC26XX_HWAttrs aesecbCC26XXHWAttrs[CC1350STK_AESECBCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESECB_Config AESECB_config[CC1350STK_AESECBCOUNT] = {
    {
         .object  = &aesecbCC26XXObjects[CC1350STK_AESECB0],
         .hwAttrs = &aesecbCC26XXHWAttrs[CC1350STK_AESECB0]
    },
};

const uint_least8_t AESECB_count = CC1350STK_AESECBCOUNT;

/*
 *  =============================== AESCTRDRBG ===============================
 */
#include <ti/drivers/AESCTRDRBG.h>
#include <ti/drivers/aesctrdrbg/AESCTRDRBGXX.h>

AESCTRDRBGXX_Object aesctrdrbgXXObjects[CC1350STK_AESCTRDRBGCOUNT];

const AESCTRDRBGXX_HWAttrs aesctrdrbgXXHWAttrs[CC1350STK_AESCTRDRBGCOUNT] = {
    {
        .aesctrIndex       = CC1350STK_AESCTR0,
    }
};

const AESCTRDRBG_Config AESCTRDRBG_config[CC1350STK_AESCTRDRBGCOUNT] = {
    {
         .object  = &aesctrdrbgXXObjects[CC1350STK_AESCTRDRBG0],
         .hwAttrs = &aesctrdrbgXXHWAttrs[CC1350STK_AESCTRDRBG0]
    },
};

const uint_least8_t AESCTRDRBG_count = CC1350STK_AESCTRDRBGCOUNT;

/*
 *  =============================== TRNG ===============================
 */
#include <ti/drivers/TRNG.h>
#include <ti/drivers/trng/TRNGCC26XX.h>

TRNGCC26XX_Object trngCC26XXObjects[CC1350STK_TRNGCOUNT];

const TRNGCC26XX_HWAttrs trngCC26X2HWAttrs[CC1350STK_TRNGCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
        .samplesPerCycle   = 240000,
    }
};

const TRNG_Config TRNG_config[CC1350STK_TRNGCOUNT] = {
    {
         .object  = &trngCC26XXObjects[CC1350STK_TRNG0],
         .hwAttrs = &trngCC26X2HWAttrs[CC1350STK_TRNG0]
    },
};

const uint_least8_t TRNG_count = CC1350STK_TRNGCOUNT;


/*
 *  =============================== Display ===============================
 */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/display/DisplaySharp.h>

#ifndef CC1350STK_DISPLAY_UART_STRBUF_SIZE
#define CC1350STK_DISPLAY_UART_STRBUF_SIZE    128
#endif

/* This value can be changed to 96 for use with the 430BOOST-SHARP96 BoosterPack. */
#define CC1350STK_DISPLAY_SHARP_SIZE    128

DisplayUart_Object     displayUartObject;
DisplaySharp_Object    displaySharpObject;

static char uartStringBuf[CC1350STK_DISPLAY_UART_STRBUF_SIZE];
static uint_least8_t sharpDisplayBuf[CC1350STK_DISPLAY_SHARP_SIZE * CC1350STK_DISPLAY_SHARP_SIZE / 8];

const DisplayUart_HWAttrs displayUartHWAttrs = {
    .uartIdx      = CC1350STK_UART0,
    .baudRate     = 57600,
    .mutexTimeout = (unsigned int)(-1),
    .strBuf       = uartStringBuf,
    .strBufLen    = CC1350STK_DISPLAY_UART_STRBUF_SIZE,
};

const DisplaySharp_HWAttrsV1 displaySharpHWattrs = {
    .spiIndex    = CC1350STK_SPI0,
    .csPin       = CC1350STK_GPIO_LCD_CS,
    .powerPin    = (uint32_t)(-1),  /* no need to apply power for this LCD */
    .enablePin   = CC1350STK_GPIO_LCD_ENABLE,
    .pixelWidth  = CC1350STK_DISPLAY_SHARP_SIZE,
    .pixelHeight = CC1350STK_DISPLAY_SHARP_SIZE,
    .displayBuf  = sharpDisplayBuf,
};

/*
 * The pins for the UART and Watch Devpack conflict, prefer UART by default
 */
#ifndef BOARD_DISPLAY_USE_UART
#define BOARD_DISPLAY_USE_UART 1
#endif
#ifndef BOARD_DISPLAY_USE_UART_ANSI
#define BOARD_DISPLAY_USE_UART_ANSI 0
#endif
#ifndef BOARD_DISPLAY_USE_LCD
#define BOARD_DISPLAY_USE_LCD 0
#endif

/*
 * This #if/#else is needed to workaround a problem with the
 * IAR compiler. The IAR compiler doesn't like the empty array
 * initialization. (IAR Error[Pe1345])
 */
#if (BOARD_DISPLAY_USE_UART || BOARD_DISPLAY_USE_LCD)

const Display_Config Display_config[] = {
#if (BOARD_DISPLAY_USE_UART)
    {
#  if (BOARD_DISPLAY_USE_UART_ANSI)
        .fxnTablePtr = &DisplayUartAnsi_fxnTable,
#  else /* Default to minimal UART with no cursor placement */
        .fxnTablePtr = &DisplayUartMin_fxnTable,
#  endif
        .fxnTablePtr = &DisplayUart_fxnTable,
        .object      = &displayUartObject,
        .hwAttrs     = &displayUartHWAttrs,
    },
#endif
#if (BOARD_DISPLAY_USE_LCD)
    {
        .fxnTablePtr = &DisplaySharp_fxnTable,
        .object      = &displaySharpObject,
        .hwAttrs     = &displaySharpHWattrs
    },
#endif
};

const uint_least8_t Display_count = sizeof(Display_config) / sizeof(Display_Config);

#else

const Display_Config *Display_config = NULL;
const uint_least8_t Display_count = 0;

#endif /* (BOARD_DISPLAY_USE_UART || BOARD_DISPLAY_USE_LCD) */

/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC26XX.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC1350STK.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array. Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* Input pins */
    GPIOCC26XX_DIO_15 | GPIO_DO_NOT_CONFIG,  /* Button 0 */
    GPIOCC26XX_DIO_04 | GPIO_DO_NOT_CONFIG,  /* Button 1 */

    /* Output pins */
    GPIOCC26XX_DIO_10 | GPIO_DO_NOT_CONFIG,  /* LED */

    /* SPI Flash CSN */
    GPIOCC26XX_DIO_14 | GPIO_DO_NOT_CONFIG,

    /* Sharp Display - GPIO configurations will be done in the Display files */
    GPIOCC26XX_DIO_20 | GPIO_DO_NOT_CONFIG,    /* LCD CS */
    GPIOCC26XX_DIO_29 | GPIO_DO_NOT_CONFIG,    /* LCD Enable */
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC1350STK.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,  /*  Button 0 */
    NULL,  /*  Button 1 */
};

const GPIOCC26XX_Config GPIOCC26XX_config = {
    .pinConfigs         = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks          = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = CC1350STK_GPIOCOUNT,
    .numberOfCallbacks  = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority        = (~0)
};

/*
 *  =============================== GPTimer ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/timer/GPTimerCC26XX.h>

GPTimerCC26XX_Object gptimerCC26XXObjects[CC1350STK_GPTIMERCOUNT];

const GPTimerCC26XX_HWAttrs gptimerCC26xxHWAttrs[CC1350STK_GPTIMERPARTSCOUNT] = {
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0A, },
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0B, },
/*    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1A, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1B, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2A, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2B, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3A, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3B, },*/
};

const GPTimerCC26XX_Config GPTimerCC26XX_config[CC1350STK_GPTIMERPARTSCOUNT] = {
    { &gptimerCC26XXObjects[CC1350STK_GPTIMER0], &gptimerCC26xxHWAttrs[CC1350STK_GPTIMER0A], GPT_A },
    { &gptimerCC26XXObjects[CC1350STK_GPTIMER0], &gptimerCC26xxHWAttrs[CC1350STK_GPTIMER0B], GPT_B },
/*    { &gptimerCC26XXObjects[CC1350STK_GPTIMER1], &gptimerCC26xxHWAttrs[CC1350STK_GPTIMER1A], GPT_A },
    { &gptimerCC26XXObjects[CC1350STK_GPTIMER1], &gptimerCC26xxHWAttrs[CC1350STK_GPTIMER1B], GPT_B },
    { &gptimerCC26XXObjects[CC1350STK_GPTIMER2], &gptimerCC26xxHWAttrs[CC1350STK_GPTIMER2A], GPT_A },
    { &gptimerCC26XXObjects[CC1350STK_GPTIMER2], &gptimerCC26xxHWAttrs[CC1350STK_GPTIMER2B], GPT_B },
    { &gptimerCC26XXObjects[CC1350STK_GPTIMER3], &gptimerCC26xxHWAttrs[CC1350STK_GPTIMER3A], GPT_A },
    { &gptimerCC26XXObjects[CC1350STK_GPTIMER3], &gptimerCC26xxHWAttrs[CC1350STK_GPTIMER3B], GPT_B },*/
};

/*
 *  =============================== I2C ===============================
*/
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

I2CCC26XX_Object i2cCC26xxObjects[CC1350STK_I2CCOUNT];

const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CC1350STK_I2CCOUNT] = {
    {
        .baseAddr    = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum      = INT_I2C_IRQ,
        .intPriority = ~0,
        .swiPriority = 0,
        .sdaPin      = CC1350STK_I2C0_SDA0,
        .sclPin      = CC1350STK_I2C0_SCL0,
    }
};

const I2C_Config I2C_config[CC1350STK_I2CCOUNT] = {
    {
        .fxnTablePtr = &I2CCC26XX_fxnTable,
        .object      = &i2cCC26xxObjects[CC1350STK_I2C0],
        .hwAttrs     = &i2cCC26xxHWAttrs[CC1350STK_I2C0]
    },
};

const uint_least8_t I2C_count = CC1350STK_I2CCOUNT;

/*
 *  =============================== I2S ===============================
*/
#include <ti/drivers/I2S.h>
#include <ti/drivers/i2s/I2SCC26XX.h>

I2SCC26XX_Object i2sCC26XXObjects[CC1350STK_I2SCOUNT];

const I2SCC26XX_HWAttrs i2sCC26XXHWAttrs[CC1350STK_I2SCOUNT] = {
    {
        .pinSD1      =  CC1350STK_I2S_ADI,
        .pinSD0      =  CC1350STK_I2S_ADO,
        .pinSCK      =  CC1350STK_I2S_BCLK,
        .pinMCLK     =  CC1350STK_I2S_MCLK,
        .pinWS       =  CC1350STK_I2S_WCLK,
        .intPriority = ~0,
    }
};

const I2S_Config I2S_config[CC1350STK_I2SCOUNT] = {
    {
        .object      = &i2sCC26XXObjects[CC1350STK_I2S0],
        .hwAttrs     = &i2sCC26XXHWAttrs[CC1350STK_I2S0]
    },
};

const uint_least8_t I2S_count = CC1350STK_I2SCOUNT;

/*
 *  =============================== NVS ===============================
 */
#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSSPI25X.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

#define NVS_REGIONS_BASE 0x1A000
#define SECTORSIZE       0x1000
#define REGIONSIZE       (SECTORSIZE * 4)
#define SPIREGIONSIZE    (SECTORSIZE * 32)
#define VERIFYBUFSIZE    64

static uint8_t verifyBuf[VERIFYBUFSIZE];

/*
 * Reserve flash sectors for NVS driver use by placing an uninitialized byte
 * array at the desired flash address.
 */
#if defined(__TI_COMPILER_VERSION__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
#pragma LOCATION(flashBuf, NVS_REGIONS_BASE);
#pragma NOINIT(flashBuf);
static char flashBuf[REGIONSIZE];

#elif defined(__IAR_SYSTEMS_ICC__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
static __no_init char flashBuf[REGIONSIZE] @ NVS_REGIONS_BASE;

#elif defined(__GNUC__)

/*
 * Place the flash buffers in the .nvs section created in the gcc linker file.
 * The .nvs section enforces alignment on a sector boundary but may
 * be placed anywhere in flash memory.  If desired the .nvs section can be set
 * to a fixed address by changing the following in the gcc linker file:
 *
 * .nvs (FIXED_FLASH_ADDR) (NOLOAD) : AT (FIXED_FLASH_ADDR) {
 *      *(.nvs)
 * } > REGION_TEXT
 */
__attribute__ ((section (".nvs")))
static char flashBuf[REGIONSIZE];

#endif

/* Allocate objects for NVS and NVS SPI */
NVSCC26XX_Object nvsCC26xxObjects[1];
NVSSPI25X_Object nvsSPI25XObjects[1];

/* Hardware attributes for NVS */
const NVSCC26XX_HWAttrs nvsCC26xxHWAttrs[1] = {
    {
        .regionBase = (void *)flashBuf,
        .regionSize = REGIONSIZE,
    },
};

/* Hardware attributes for NVS SPI */
const NVSSPI25X_HWAttrs nvsSPI25XHWAttrs[1] = {
    {
        .regionBaseOffset = 0,
        .regionSize = SPIREGIONSIZE,
        .sectorSize = SECTORSIZE,
        .verifyBuf = verifyBuf,
        .verifyBufSize = VERIFYBUFSIZE,
        .spiHandle = NULL,
        .spiIndex = 0,
        .spiBitRate = 4000000,
        .spiCsnGpioIndex = CC1350STK_GPIO_SPI_FLASH_CS,
        .statusPollDelayUs = 100,
    },
};

/* NVS Region index 0 and 1 refer to NVS and NVS SPI respectively */
const NVS_Config NVS_config[CC1350STK_NVSCOUNT] = {
    {
        .fxnTablePtr = &NVSCC26XX_fxnTable,
        .object = &nvsCC26xxObjects[0],
        .hwAttrs = &nvsCC26xxHWAttrs[0],
    },
    {
        .fxnTablePtr = &NVSSPI25X_fxnTable,
        .object = &nvsSPI25XObjects[0],
        .hwAttrs = &nvsSPI25XHWAttrs[0],
    },
};

const uint_least8_t NVS_count = CC1350STK_NVSCOUNT;

/*
 *  =============================== PDM ===============================
*/
#include <ti/drivers/pdm/PDMCC26XX.h>
#include <ti/drivers/pdm/PDMCC26XX_util.h>

PDMCC26XX_Object        pdmCC26XXObjects[CC1350STK_PDMCOUNT];
PDMCC26XX_I2S_Object    pdmCC26XXI2SObjects[CC1350STK_PDMCOUNT];

const PDMCC26XX_HWAttrs pdmCC26XXHWAttrs[CC1350STK_PDMCOUNT] = {
    {
        .micPower     = CC1350STK_MIC_POWER,
        .taskPriority = 2
    }
};

const PDMCC26XX_Config PDMCC26XX_config[CC1350STK_PDMCOUNT] = {
    {
        .object  = &pdmCC26XXObjects[CC1350STK_PDM0],
        .hwAttrs = &pdmCC26XXHWAttrs[CC1350STK_PDM0]
    }
};

const PDMCC26XX_I2S_HWAttrs pdmC26XXI2SHWAttrs[CC1350STK_PDMCOUNT] = {
    {
        .baseAddr       = I2S0_BASE,
        .intNum         = INT_I2S_IRQ,
        .powerMngrId    = PowerCC26XX_PERIPH_I2S,
        .intPriority    = ~0,
        .mclkPin        = PIN_UNASSIGNED,
        .bclkPin        = CC1350STK_AUDIO_CLK,
        .wclkPin        = PIN_UNASSIGNED,
        .ad0Pin         = CC1350STK_AUDIO_DI,
    }
};

const PDMCC26XX_I2S_Config PDMCC26XX_I2S_config[CC1350STK_PDMCOUNT] = {
    {
        .object  = &pdmCC26XXI2SObjects[CC1350STK_PDM0],
        .hwAttrs = &pdmC26XXI2SHWAttrs[CC1350STK_PDM0]
    }
};

/*
 *  =============================== PIN ===============================
 */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

// If you have any unused pins in your design make sure to either not initialize them (leave them floating with input+output buffer disabled) or enable pull up/down to ensure a defined logical input level

#ifdef CC1310_carte
#ifdef C10_CAMERA_PIR // Pinout E/S
const PIN_Config BoardGpioInitTable[] = {

    Board_GPIO_S0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,           /* LED initially off */
    CC1350STK_KEY_LEFT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,          /* Button 0 is active low  IOID_15 */
    //CC1350STK_KEY_RIGHT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,         /* Button 1 is active low  IOID_4 */
    //CC1350STK_RELAY | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS | PIN_INPUT_DIS,                   /*  Button 2 is active high */
    //CC1350STK_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,         /* Buzzer initially off */
    CC1350STK_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,  /* External flash chip select */
    CC1350STK_UART_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,                         /* DevPack */
    CC1350STK_UART_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                              /* DevPack */

    PIN_TERMINATE
};

#else
const PIN_Config BoardGpioInitTable[] = {

    //Board_GPIO_S0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,           /* LED initially off */
    CC1350STK_KEY_LEFT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,          /* Button 0 is active low  IOID_15 */
    CC1350STK_KEY_RIGHT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,         /* Button 1 is active low  IOID_4 */
    //CC1350STK_RELAY | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS | PIN_INPUT_DIS,                   /*  Button 2 is active high */
    //CC1350STK_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,         /* Buzzer initially off */
    //CC1350STK_UART_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,                         /* DevPack */
    //CC1350STK_UART_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                              /* DevPack */

    PIN_TERMINATE
};
#endif // CC1310

#else  // CC1350

const PIN_Config BoardGpioInitTable[] = {

    Board_GPIO_S0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,           /* LED initially off */
    CC1350STK_KEY_LEFT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,          /* Button is active low */
    CC1350STK_KEY_RIGHT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,         /* Button is active low */
    //CC1350STK_RELAY | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,           /* version 1.4.5  Relay is active high */
    CC1350STK_RELAY | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS | PIN_INPUT_DIS,           /*  version 1.5.1  Relay is active high */
    CC1350STK_MPU_INT | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_NEGEDGE | PIN_HYSTERESIS,           /* MPU_INT is active low */
    CC1350STK_TMP_RDY | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,                               /* TMP_RDY is active high */
    CC1350STK_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,         /* Buzzer initially off */
    CC1350STK_MPU_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* MPU initially on */
    CC1350STK_MIC_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,      /* MIC initially off */
    CC1350STK_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,  /* External flash chip select */
    CC1350STK_SPI_DEVPK_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,   /* DevPack chip select */
    CC1350STK_AUDIO_DI | PIN_INPUT_EN | PIN_PULLDOWN,                                             /* Audio DI */
    CC1350STK_AUDIODO | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,       /* Audio data out */
    CC1350STK_AUDIO_CLK | PIN_INPUT_EN | PIN_PULLDOWN,                                            /* DevPack */
    CC1350STK_DP2 | PIN_INPUT_EN | PIN_PULLDOWN,                                                  /* DevPack */
    CC1350STK_DP1 | PIN_INPUT_EN | PIN_PULLDOWN,                                                  /* DevPack */
    CC1350STK_DP0 | PIN_INPUT_EN | PIN_PULLDOWN,                                                  /* DevPack */
    CC1350STK_DP3 | PIN_INPUT_EN | PIN_PULLDOWN,                                                  /* DevPack */
#ifdef Uart2
    CC1350STK_UART_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,                         /* DevPack */
    CC1350STK_UART_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                              /* DevPack */
#endif
    CC1350STK_DEVPK_ID | PIN_INPUT_EN | PIN_NOPULL,                                               /* Device pack ID - external PU */
    CC1350STK_SPI0_MOSI | PIN_INPUT_EN | PIN_PULLDOWN,                                            /* SPI master out - slave in */
    CC1350STK_SPI0_MISO | PIN_INPUT_EN | PIN_PULLDOWN,                                            /* SPI master in - slave out */
    CC1350STK_SPI0_CLK | PIN_INPUT_EN | PIN_PULLDOWN,                                             /* SPI clock */

    PIN_TERMINATE
};
#endif

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = ~0,
    .swiPriority = 0
};

/*
 *  =============================== Power ===============================
 */
const PowerCC26XX_Config PowerCC26XX_config = {
    .policyInitFxn      = NULL,
    .policyFxn          = &PowerCC26XX_standbyPolicy,
    .calibrateFxn       = &PowerCC26XX_calibrate,
    .enablePolicy       = true,
    .calibrateRCOSC_LF  = true,
    .calibrateRCOSC_HF  = true,
};

/*
 *  =============================== PWM ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC26XX.h>

PWMTimerCC26XX_Object pwmtimerCC26xxObjects[CC1350STK_PWMCOUNT];

const PWMTimerCC26XX_HwAttrs pwmtimerCC26xxHWAttrs[CC1350STK_PWMCOUNT] = {
    { .pwmPin = CC1350STK_PWMPIN0, .gpTimerUnit = CC1350STK_GPTIMER0A },
   #if (NB_PWM>1)
     { .pwmPin = CC1350STK_PWMPIN1, .gpTimerUnit = CC1350STK_GPTIMER0B },
   #endif
 /*   { .pwmPin = CC1350STK_PWMPIN2, .gpTimerUnit = CC1350STK_GPTIMER1A },
    { .pwmPin = CC1350STK_PWMPIN3, .gpTimerUnit = CC1350STK_GPTIMER1B },
    { .pwmPin = CC1350STK_PWMPIN4, .gpTimerUnit = CC1350STK_GPTIMER2A },
    { .pwmPin = CC1350STK_PWMPIN5, .gpTimerUnit = CC1350STK_GPTIMER2B },
    { .pwmPin = CC1350STK_PWMPIN6, .gpTimerUnit = CC1350STK_GPTIMER3A },
    { .pwmPin = CC1350STK_PWMPIN7, .gpTimerUnit = CC1350STK_GPTIMER3B },*/
};

const PWM_Config PWM_config[CC1350STK_PWMCOUNT] = {
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1350STK_PWM0], &pwmtimerCC26xxHWAttrs[CC1350STK_PWM0] },
    #if (NB_PWM>1)
     { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1350STK_PWM1], &pwmtimerCC26xxHWAttrs[CC1350STK_PWM1] },
    #endif
   /*    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1350STK_PWM2], &pwmtimerCC26xxHWAttrs[CC1350STK_PWM2] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1350STK_PWM3], &pwmtimerCC26xxHWAttrs[CC1350STK_PWM3] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1350STK_PWM4], &pwmtimerCC26xxHWAttrs[CC1350STK_PWM4] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1350STK_PWM5], &pwmtimerCC26xxHWAttrs[CC1350STK_PWM5] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1350STK_PWM6], &pwmtimerCC26xxHWAttrs[CC1350STK_PWM6] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1350STK_PWM7], &pwmtimerCC26xxHWAttrs[CC1350STK_PWM7] },*/
};

const uint_least8_t PWM_count = CC1350STK_PWMCOUNT;

/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = ~0,       /* Lowest HWI priority */
    .swiPriority        = 0,        /* Lowest SWI priority */
    .xoscHfAlwaysNeeded = true,     /* Keep XOSC dependency while in standby */
    .globalCallback     = NULL,     /* No board specific callback */
    .globalEventMask    = 0         /* No events subscribed to */
};

/*
 *  =============================== SPI DMA ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

SPICC26XXDMA_Object spiCC26XXDMAObjects[CC1350STK_SPICOUNT];

const SPICC26XXDMA_HWAttrsV1 spiCC26XXDMAHWAttrs[CC1350STK_SPICOUNT] = {
    {
        .baseAddr           = SSI0_BASE,
        .intNum             = INT_SSI0_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI0,
        .defaultTxBufValue  = 0,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI0_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI0_TX,
        .mosiPin            = CC1350STK_SPI0_MOSI,
        .misoPin            = CC1350STK_SPI0_MISO,
        .clkPin             = CC1350STK_SPI0_CLK,
        .csnPin             = CC1350STK_SPI0_CSN,
        .minDmaTransferSize = 10
    },
    {
        .baseAddr           = SSI1_BASE,
        .intNum             = INT_SSI1_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI1,
        .defaultTxBufValue  = 0,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI1_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI1_TX,
        .mosiPin            = CC1350STK_SPI1_MOSI,
        .misoPin            = CC1350STK_SPI1_MISO,
        .clkPin             = CC1350STK_SPI1_CLK,
        .csnPin             = CC1350STK_SPI1_CSN,
        .minDmaTransferSize = 10
    }
};

const SPI_Config SPI_config[CC1350STK_SPICOUNT] = {
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[CC1350STK_SPI0],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[CC1350STK_SPI0]
    },
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[CC1350STK_SPI1],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[CC1350STK_SPI1]
    },
};

const uint_least8_t SPI_count = CC1350STK_SPICOUNT;

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

UARTCC26XX_Object uartCC26XXObjects[CC1350STK_UARTCOUNT];

uint8_t uartCC26XXRingBuffer[CC1350STK_UARTCOUNT][32];

const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs[CC1350STK_UARTCOUNT] = {
    {
        .baseAddr       = UART0_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_UART0,
        .intNum         = INT_UART0_COMB,
        .intPriority    = ~0,
        .swiPriority    = 0,
        .txPin          = CC1350STK_UART_TX,
        .rxPin          = CC1350STK_UART_RX,
        .ctsPin         = PIN_UNASSIGNED,
        .rtsPin         = PIN_UNASSIGNED,
        .ringBufPtr     = uartCC26XXRingBuffer[CC1350STK_UART0],
        .ringBufSize    = sizeof(uartCC26XXRingBuffer[CC1350STK_UART0]),
        .txIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_1_8,
        .rxIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_4_8,
        .errorFxn       = NULL
    }
};

const UART_Config UART_config[CC1350STK_UARTCOUNT] = {
    {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects[CC1350STK_UART0],
        .hwAttrs     = &uartCC26XXHWAttrs[CC1350STK_UART0]
    },
};

const uint_least8_t UART_count = CC1350STK_UARTCOUNT;

/*
 *  =============================== UDMA ===============================
 */
#include <ti/drivers/dma/UDMACC26XX.h>

UDMACC26XX_Object udmaObjects[CC1350STK_UDMACOUNT];

const UDMACC26XX_HWAttrs udmaHWAttrs[CC1350STK_UDMACOUNT] = {
    {
        .baseAddr    = UDMA0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_UDMA,
        .intNum      = INT_DMA_ERR,
        .intPriority = ~0
    }
};

const UDMACC26XX_Config UDMACC26XX_config[CC1350STK_UDMACOUNT] = {
    {
         .object  = &udmaObjects[CC1350STK_UDMA0],
         .hwAttrs = &udmaHWAttrs[CC1350STK_UDMA0]
    },
};

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>

WatchdogCC26XX_Object watchdogCC26XXObjects[CC1350STK_WATCHDOGCOUNT];

const WatchdogCC26XX_HWAttrs watchdogCC26XXHWAttrs[CC1350STK_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WDT_BASE,
        .reloadValue = 10000 /* Reload value in milliseconds */
    },
};

const Watchdog_Config Watchdog_config[CC1350STK_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogCC26XX_fxnTable,
        .object      = &watchdogCC26XXObjects[CC1350STK_WATCHDOG0],
        .hwAttrs     = &watchdogCC26XXHWAttrs[CC1350STK_WATCHDOG0]
    },
/*    {
    .fxnTablePtr = NULL,
    .object = NULL,
    .hwAttrs = NULL,
    }*/
};

const uint_least8_t Watchdog_count = CC1350STK_WATCHDOGCOUNT;

/*
 *  Board-specific initialization function to disable external flash.
 *  This function is defined in the file CC1350STK_fxns.c
 */
extern void Board_initHook(void);

/*
 *  ======== CC1350STK_initGeneral ========
 */
void CC1350STK_initGeneral(void)
{
    Power_init();

//#ifdef CAMERA_SOM
//    if ( PIN_init(BoardGpioInitTable1310CAM) != PIN_SUCCESS)  ret=1 ;  //Error with PIN_init
//#else
    if ( PIN_init(BoardGpioInitTable) != PIN_SUCCESS) {
        /* Error with PIN_init */
        while (1);
    }
//#endif

    /* Perform board-specific initialization */
    Board_initHook();

    // Map Serial Wire Viewer to JTAG TDO pin
    IOCPortConfigureSet(IOID_16, IOC_PORT_MCU_SWV, IOC_STD_OUTPUT);
}
