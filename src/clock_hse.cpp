// System clock from the TCXO (HSE32) instead of the core's default MSI 48 MHz.
//
// Why: MSI is an on-chip RC oscillator. On the E77 it ran ~1.9% slow, so micros() and every
// timed symbol were 1.9% slow too (Horus 4FSK went out at ~98 baud and could not be decoded).
// LoRa itself is unaffected because the radio uses its own clock.
//
// This overrides the core's weak SystemClock_Config(). The STM32LowPower library calls it again
// after waking from Stop2, so the TCXO clock is restored on every wake-up. If HSE does not start
// (crystal-only module, no TCXO), it falls back to the core's original MSI setup.
#include "config.h"

#if HSE_SYSCLK
#include <Arduino.h>

static bool setupHse() {
    RCC_OscInitTypeDef osc = {};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_BYPASS_PWR;     // MCU powers the TCXO (VDDTCXO) and uses it as HSE
    osc.HSEDiv         = RCC_HSE_DIV1;
    osc.PLL.PLLState   = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;

    RCC_ClkInitTypeDef clk = {};
    clk.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSE;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    clk.AHBCLK3Divider = RCC_SYSCLK_DIV1;
    return HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) == HAL_OK;   // also re-scales SysTick
}

static void setupMsiFallback() {                 // identical to the core's default
    RCC_OscInitTypeDef osc = {};
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState            = RCC_MSI_ON;
    osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    osc.MSIClockRange       = RCC_MSIRANGE_11;
    osc.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    RCC_ClkInitTypeDef clk = {};
    clk.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    clk.AHBCLK3Divider = RCC_SYSCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

extern "C" void SystemClock_Config(void) {
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    if (!setupHse()) setupMsiFallback();
}
#endif
