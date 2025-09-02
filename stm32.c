#include "main.h"
#include "string.h"
#include "stdio.h"
#include "time.h"

UART_HandleTypeDef huart2;

// Función para enviar por UART
void uart_send(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

// Generar token simple
unsigned int generate_token(time_t t)
{
    return (unsigned int)(t / 30) % 1000000;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    char rx_buffer[64];
    char msg[64];
    time_t stm_time = 0;

    uart_send("Esperando sincronización...\r\n");

    // Recibir hora desde PC
    HAL_UART_Receive(&huart2, (uint8_t *)rx_buffer, sizeof(rx_buffer), HAL_MAX_DELAY);
    stm_time = atol(rx_buffer);

    snprintf(msg, sizeof(msg), "STM32 sincronizado con tiempo: %ld\r\n", stm_time);
    uart_send(msg);

    // Generar token
    unsigned int token = generate_token(stm_time);
    snprintf(msg, sizeof(msg), "TOKEN STM32: %06u\r\n", token);
    uart_send(msg);

    while (1)
    {
        HAL_Delay(1000); // Esperar, no hace nada más
    }
}
