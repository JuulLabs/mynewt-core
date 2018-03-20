#ifndef NRFX_CONFIG_H__
#define NRFX_CONFIG_H__

#include "syscfg/syscfg.h"

#define NRFX_CLOCK_ENABLED 0
#define NRFX_COMP_ENABLED 0
#define NRFX_GPIOTE_ENABLED 0
#define NRFX_I2S_ENABLED 0
#define NRFX_LPCOMP_ENABLED 0
#define NRFX_PDM_ENABLED 0
#define NRFX_POWER_ENABLED 0
#define NRFX_PPI_ENABLED 0
#define NRFX_PRS_ENABLDE 0
#define NRFX_QDEC_ENABLED 0
#define NRFX_RNG_ENABLED 0
#define NRFX_RTC_ENABLED 0
#define NRFX_SPIM_ENABLED 0
#define NRFX_SPIS_ENABLED 0
#define NRFX_SPI_ENABLED 0
#define NRFX_SWI_ENABLED 0
#define NRFX_SYSTICK_ENABLED 0
#define NRFX_TIMER_ENABLED 0
#define NRFX_TWIM_ENABLED 0
#define NRFX_TWIS_ENABLED 0
#define NRFX_TWI_ENABLED 0
#define NRFX_UARTE_ENABLED 0
#define NRFX_UART_ENABLED 0
#define NRFX_WDT_ENABLED 0

#if MYNEWT_VAL(ADC_0)
#define NRFX_SAADC_ENABLED 1
#define NRFX_SAADC_CONFIG_RESOLUTION 1
#define NRFX_SAADC_CONFIG_OVERSAMPLE 0
#define NRFX_SAADC_CONFIG_LP_MODE 0
#define NRFX_SAADC_CONFIG_IRQ_PRIORITY 7
#endif

#if (MYNEWT_VAL(PWM_0) || MYNEWT_VAL(PWM_1) || MYNEWT_VAL(PWM_2) || MYNEWT_VAL(PWM_3))
#define NRFX_PWM_ENABLED 1
#define NRFX_PWM_DEFAULT_CONFIG_OUT0_PIN 31
#define NRFX_PWM_DEFAULT_CONFIG_OUT1_PIN 31
#define NRFX_PWM_DEFAULT_CONFIG_OUT2_PIN 31
#define NRFX_PWM_DEFAULT_CONFIG_OUT3_PIN 31
#define NRFX_PWM_DEFAULT_CONFIG_BASE_CLOCK 4
#define NRFX_PWM_DEFAULT_CONFIG_COUNT_MODE 0
#define NRFX_PWM_DEFAULT_CONFIG_TOP_VALUE 1000
#define NRFX_PWM_DEFAULT_CONFIG_LOAD_MODE 0
#define NRFX_PWM_DEFAULT_CONFIG_STEP_MODE 0
#define NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY 7
#endif

#if MYNEWT_VAL(PWM_0)
#define NRFX_PWM0_ENABLED 1
#endif

#if MYNEWT_VAL(PWM_1)
#define NRFX_PWM1_ENABLED 1
#endif

#if MYNEWT_VAL(PWM_2)
#define NRFX_PWM2_ENABLED 1
#endif

#if MYNEWT_VAL(PWM_3)
#define NRFX_PWM3_ENABLED 1
#endif

#endif // NRFX_CONFIG_H__
