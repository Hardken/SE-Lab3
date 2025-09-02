#include "main.h"
#include "string.h"
#include "stdio.h"
#include "time.h"

UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;

// --- UART send helper ---
void uart_send(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

// --- Token generator (igual al PC) ---
unsigned int generate_token(time_t t)
{
    return (unsigned int)(t / 30) % 1000000;
}

// --- Init ADC1 (PA0 como entrada analógica) ---
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    sConfig.Channel = ADC_CHANNEL_0; // PA0
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

// --- Init UART2 (PA2=TX, PA3=RX) ---
void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();

    char rx_buffer[64];
    char msg[128];
    time_t stm_time = 0;

    uart_send("Esperando sincronización desde PC...\r\n");

    // --- Recibir hora desde PC (como string de segundos UNIX) ---
    HAL_UART_Receive(&huart2, (uint8_t *)rx_buffer, sizeof(rx_buffer), HAL_MAX_DELAY);
    stm_time = atol(rx_buffer);

    snprintf(msg, sizeof(msg), "STM32 sincronizado con tiempo: %ld\r\n", stm_time);
    uart_send(msg);

    // --- Generar token ---
    unsigned int token = generate_token(stm_time);
    snprintf(msg, sizeof(msg), "TOKEN STM32: %06u\r\n", token);
    uart_send(msg);

    while (1)
    {
        // Leer fotorresistencia
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t ldr_value = HAL_ADC_GetValue(&hadc1);

        // Imprimir valor leído junto con hora y token
        time_t now = stm_time + HAL_GetTick() / 1000; // tiempo aproximado
        snprintf(msg, sizeof(msg), "Hora: %ld | Token: %06u | LDR: %lu\r\n", now, generate_token(now), ldr_value);
        uart_send(msg);

        HAL_Delay(1000); // 1 segundo
    }
}
