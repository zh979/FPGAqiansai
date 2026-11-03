#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

#include "Emm_V5.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define RX_BUF_SIZE 32
char    rx_buffer[RX_BUF_SIZE];
char    process_buffer[RX_BUF_SIZE];
uint8_t rx_index = 0;
uint8_t rx_byte;
volatile uint8_t rx_complete = 0;

uint16_t target_x = 0;
uint16_t target_y = 0;
uint8_t  target_valid = 0;

int32_t pos_x = 0, pos_y = 0;
int32_t pos_sent_x = 0, pos_sent_y = 0;
uint8_t pos_valid = 0;

#define POS_VEL   60
#define POS_ACC   20

uint32_t last_rx_time = 0;
uint8_t  motors_stopped = 1;

#define CENTER_X 160
#define CENTER_Y 120
#define DEADZONE 25

float Kp_vel_x = 0.1f;
float Kp_vel_y = 0.05f;

#define MAX_RPM 150
#define ACCEL   15

#define CONTROL_INTERVAL_MS 10
uint32_t last_control_time = 0;

#define LASER_PORT        GPIOB
#define LASER_PIN         GPIO_PIN_0

#define LASER_ACTIVE_HIGH 1
#define LASER_MAX_ON_MS   3000

#define LASER_LED_PORT        GPIOC
#define LASER_LED_PIN         GPIO_PIN_13
#define LASER_LED_ACTIVE_LOW  1

uint8_t  laser_req      = 0;
uint8_t  laser_on       = 0;
uint8_t  laser_latched  = 0;
uint32_t laser_on_since = 0;
uint32_t laser_fire_cnt = 0;

static void laser_set(uint8_t on)
{
    if (on == laser_on)
        return;
#if LASER_ACTIVE_HIGH
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
    laser_on = on;
    if (on) {
        laser_on_since = HAL_GetTick();
        laser_fire_cnt++;
    }

#if LASER_LED_ACTIVE_LOW
    HAL_GPIO_WritePin(LASER_LED_PORT, LASER_LED_PIN,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(LASER_LED_PORT, LASER_LED_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

static void laser_gpio_init(void)
{
    GPIO_InitTypeDef gi = {0};

#if LASER_ACTIVE_HIGH
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
#endif
    gi.Pin   = LASER_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LASER_PORT, &gi);

    __HAL_RCC_GPIOC_CLK_ENABLE();
#if LASER_LED_ACTIVE_LOW
    HAL_GPIO_WritePin(LASER_LED_PORT, LASER_LED_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(LASER_LED_PORT, LASER_LED_PIN, GPIO_PIN_RESET);
#endif
    gi.Pin = LASER_LED_PIN;
    HAL_GPIO_Init(LASER_LED_PORT, &gi);

    laser_on = 0;
}

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();

  laser_gpio_init();

  for (int i = 0; i < 3; i++) {
#if LASER_LED_ACTIVE_LOW
      HAL_GPIO_WritePin(LASER_LED_PORT, LASER_LED_PIN, GPIO_PIN_RESET);
      HAL_Delay(120);
      HAL_GPIO_WritePin(LASER_LED_PORT, LASER_LED_PIN, GPIO_PIN_SET);
#else
      HAL_GPIO_WritePin(LASER_LED_PORT, LASER_LED_PIN, GPIO_PIN_SET);
      HAL_Delay(120);
      HAL_GPIO_WritePin(LASER_LED_PORT, LASER_LED_PIN, GPIO_PIN_RESET);
#endif
      HAL_Delay(120);
  }

  USER_CAN1_Filter_Init();
  if (HAL_CAN_Start(&hcan) != HAL_OK)                                             { Error_Handler(); }

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  HAL_Delay(500);

  while (1)
  {
    if (rx_complete == 1)
    {
        rx_complete = 0;

        if (process_buffer[0] == 'X')
        {
            char *y_ptr = strchr(process_buffer, 'Y');
            if (y_ptr != NULL)
            {
                char *l_ptr = strchr(y_ptr, 'L');

                *y_ptr = '\0';
                target_x = atoi(&process_buffer[1]);
                target_y = atoi(y_ptr + 1);

                laser_req = (l_ptr != NULL && l_ptr[1] == '1') ? 1 : 0;

                if (!laser_req)
                    laser_latched = 0;

                target_valid   = 1;
                last_rx_time   = HAL_GetTick();
                motors_stopped = 0;
            }
        }

        else if (process_buffer[0] == 'A')
        {
            char *b_ptr = strchr(process_buffer, 'B');
            if (b_ptr != NULL)
            {
                char *l_ptr = strchr(b_ptr, 'L');

                *b_ptr = '\0';
                pos_x = atoi(&process_buffer[1]);
                pos_y = atoi(b_ptr + 1);

                laser_req = (l_ptr != NULL && l_ptr[1] == '1') ? 1 : 0;
                if (!laser_req)
                    laser_latched = 0;

                pos_valid    = 1;
                last_rx_time = HAL_GetTick();
            }
        }

        else if (process_buffer[0] == 'Z')
        {
            Emm_V5_Reset_CurPos_To_Zero(1);
            HAL_Delay(2);
            Emm_V5_Reset_CurPos_To_Zero(2);
            pos_x = pos_y = 0;
            pos_sent_x = pos_sent_y = 0;
            last_rx_time = HAL_GetTick();
        }
    }

    uint32_t current_time = HAL_GetTick();

    uint8_t comm_lost = (current_time - last_rx_time > 200);

    if (laser_on && (current_time - laser_on_since > LASER_MAX_ON_MS))
    {
        laser_latched = 1;
    }

    laser_set((laser_req && !comm_lost && !laser_latched) ? 1 : 0);

    if (pos_valid && (pos_x != pos_sent_x || pos_y != pos_sent_y))
    {
        int32_t dx = pos_x, dy = pos_y;
        Emm_V5_Pos_Control(1, (dx < 0) ? 1 : 0, POS_VEL, POS_ACC,
                           (uint32_t)(dx < 0 ? -dx : dx), true, 0);
        HAL_Delay(1);
        Emm_V5_Pos_Control(2, (dy < 0) ? 1 : 0, POS_VEL, POS_ACC,
                           (uint32_t)(dy < 0 ? -dy : dy), true, 0);
        pos_sent_x = pos_x;
        pos_sent_y = pos_y;
    }

    if ((current_time - last_control_time >= CONTROL_INTERVAL_MS) && (target_valid == 1))
    {
        last_control_time = current_time;
        target_valid = 0;

        int16_t error_x = target_x - CENTER_X;
        int16_t error_y = target_y - CENTER_Y;

        uint16_t rpm_x = 0;
        uint8_t  dir_x = 0;
        if (abs(error_x) > DEADZONE) {
            rpm_x = (uint16_t)(abs(error_x) * Kp_vel_x);
            if (rpm_x > MAX_RPM) rpm_x = MAX_RPM;
            dir_x = (error_x > 0) ? 1 : 0;
        }

        uint16_t rpm_y = 0;
        uint8_t  dir_y = 0;
        if (abs(error_y) > DEADZONE) {
            rpm_y = (uint16_t)(abs(error_y) * Kp_vel_y);
            if (rpm_y > 60) rpm_y = 60;
            dir_y = (error_y > 0) ? 1 : 0;
        }
        Emm_V5_Vel_Control(1, dir_x, rpm_x, ACCEL, 0);
        HAL_Delay(1);
        Emm_V5_Vel_Control(2, dir_y, rpm_y, ACCEL, 0);
    }

    if (comm_lost && (motors_stopped == 0) && !pos_valid)
    {
        Emm_V5_Vel_Control(1, 0, 0, ACCEL, 0);
        HAL_Delay(1);
        Emm_V5_Vel_Control(2, 0, 0, ACCEL, 0);
        motors_stopped = 1;
    }
  }
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_byte == '\n')
        {
            rx_buffer[rx_index] = '\0';
            strcpy(process_buffer, rx_buffer);
            rx_complete = 1;
            rx_index = 0;
        }
        else if (rx_byte != '\r')
        {
            if (rx_index < RX_BUF_SIZE - 1)
            {
                rx_buffer[rx_index++] = rx_byte;
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
