/**
 * @brief   This file provides the extensions to complement the functions in timer HAL.
*/
#include "stm32f7xx_hal.h"

#ifndef __STM32F7xx_TIMER_EXTENSION_H
#define __STM32F7xx_TIMER_EXTENSION_H

#ifdef __cplusplus
 extern "C" {
#endif

HAL_StatusTypeDef TIM_OC_Start_DMA_Double_Buffer(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint16_t DataLength);

#ifdef __cplusplus
}
#endif

#endif  /* __STM32F7xx_TIMER_EXTENSION_H */