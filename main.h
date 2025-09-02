#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <time.h>

// UART2 pins: PA2=TX, PA3=RX
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA

// LDR in PA0 (ADC1_IN0)
#define LDR_Pin GPIO_PIN_0
#define LDR_GPIO_Port GPIOA

    void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
