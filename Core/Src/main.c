/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 步进电机S型加减速控制主程序
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stepper.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void UART_Send(const char *s);
/* USER CODE END PFP */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    Stepper_Init();
    UART_Send("[OK] Ready\r\n");
    /* USER CODE END 2 */

    char rx_buf[128];
    uint16_t rx_idx = 0;
    uint8_t rx_byte;

    while (1)
    {
        /* 处理往返逻辑 */
        if (motor.need_return && !motor.running)
        {
            motor.need_return = 0;
            if (motor.return_count > 0 && motor.return_cur < motor.return_count)
            {
                motor.return_cur++;
                motor.return_dir = !motor.return_dir;
                Stepper_Start(motor.total_steps, motor.return_dir);
                char rsp[48];
                snprintf(rsp, sizeof(rsp), "RETURN %d/%d\r\n", motor.return_cur, motor.return_count);
                UART_Send(rsp);
            }
            else
            {
                motor.return_count = 0;
                motor.return_cur = 0;
                UART_Send("RETURN DONE\r\n");
            }
        }

        /* 串口接收 */
        if (HAL_UART_Receive(&huart1, &rx_byte, 1, 10) == HAL_OK)
        {
            if (rx_byte == '\r' || rx_byte == '\n')
            {
                if (rx_idx > 0)
                {
                    rx_buf[rx_idx] = '\0';
                    UART_Send("RX:");
                    UART_Send(rx_buf);
                    UART_Send("\r\n");

                    char buf[128];
                    strcpy(buf, rx_buf);
                    char *c1 = strtok(buf, ",");
                    char *c2 = strtok(NULL, ",");
                    char *c3 = strtok(NULL, ",");
                    char *c4 = strtok(NULL, ",");

                    if (c1 == NULL) goto NEXT;

                    if (strcmp(c1, "$STATUS") == 0)
                    {
                        const char *s;
                        switch (motor.state) {
                            case STATE_IDLE:  s="IDLE";  break;
                            case STATE_ACCEL: s="ACCEL"; break;
                            case STATE_CRUISE:s="CRUISE";break;
                            case STATE_DECEL: s="DECEL"; break;
                            case STATE_DONE:  s="DONE";  break;
                            case STATE_ESTOP: s="ESTOP"; break;
                            default:          s="???";   break;
                        }
                        char rsp[80];
                        snprintf(rsp, sizeof(rsp), "STATUS,POS=%lu,STATE=%s,RET=%d/%d\r\n",
                                 motor.cur_step, s, motor.return_cur, motor.return_count);
                        UART_Send(rsp);
                    }
                    else if (strcmp(c1, "$SET") == 0 && c2 && c3)
                    {
                        uint32_t v = (uint32_t)atol(c3);
                        if      (strcmp(c2, "PPR") == 0) motor.pulse_per_rev = v;
                        else if (strcmp(c2, "PPM") == 0) motor.pulse_per_mm  = v;
                        else if (strcmp(c2, "MAXFREQ") == 0) motor.max_freq = (v > 50000) ? 50000 : v;  // ← 改成50000
                        else if (strcmp(c2, "ACCEL") == 0) motor.accel_steps = v;
                        else if (strcmp(c2, "DECEL") == 0) motor.decel_steps = v;
                        char rsp[32];
                        snprintf(rsp, sizeof(rsp), "OK:%s=%lu\r\n", c2, v);
                        UART_Send(rsp);
                    }
                    else if (strcmp(c1, "$MOVE") == 0 && c2 && c3)
                    {
                        uint32_t val = (uint32_t)atol(c3);
                        uint8_t dir = c4 ? (uint8_t)atoi(c4) : 0;
                        uint32_t steps = 0;
                        if      (strcmp(c2, "STEPS") == 0) steps = val;
                        else if (strcmp(c2, "MM") == 0)    steps = Stepper_MmToSteps(val);
                        else if (strcmp(c2, "REV") == 0)   steps = Stepper_RevToSteps(val);
                        if (steps == 0) {
                            UART_Send("ERR:ZERO\r\n");
                        } else {
                            uint8_t ret = Stepper_Start(steps, dir);
                            char rsp[48];
                            if (ret == 0) snprintf(rsp, sizeof(rsp), "OK:MOVE %lu\r\n", steps);
                            else if (ret == 1) snprintf(rsp, sizeof(rsp), "ERR:BUSY\r\n");
                            else snprintf(rsp, sizeof(rsp), "ERR:PARAM\r\n");
                            UART_Send(rsp);
                        }
                    }
                    else if (strcmp(c1, "$STOP") == 0)
                    {
                        motor.return_count = 0;
                        Stepper_Stop();
                        UART_Send("OK:STOP\r\n");
                    }
                    else if (strcmp(c1, "$ESTOP") == 0)
                    {
                        motor.return_count = 0;
                        Stepper_EStop();
                        UART_Send("OK:ESTOP\r\n");
                    }
                    else if (strcmp(c1, "$RESET") == 0)
                    {
                        Stepper_Init();
                        UART_Send("OK:RESET\r\n");
                    }
                    else if (strcmp(c1, "$RETURN") == 0 && c2)
                    {
                        motor.return_count = (uint16_t)atoi(c2);
                        motor.return_cur = 0;
                        motor.return_dir = 0;
                        char rsp[32];
                        snprintf(rsp, sizeof(rsp), "OK:RETURN=%d\r\n", motor.return_count);
                        UART_Send(rsp);
                    }
                    else
                    {
                        UART_Send("ERR:UNKNOWN\r\n");
                    }

NEXT:
                    rx_idx = 0;
                }
            }
            else
            {
                if (rx_idx < sizeof(rx_buf) - 1)
                    rx_buf[rx_idx++] = rx_byte;
            }
        }
    }
}

static void UART_Send(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), 100);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) Stepper_OnTimer();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
