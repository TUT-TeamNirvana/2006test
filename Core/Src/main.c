/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Allheader.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
M2006_t motors[2];
int8_t dir[2] = { +1, -1 };
__attribute__((used)) volatile uint32_t hss_m1_actual_rpm  = 0.0f;
__attribute__((used)) volatile uint32_t hss_m1_pid_error  = 0.0f;

// BMI088数据发送缓冲区
static char uart_buffer[256];

/**
 * @brief 通过UART发送BMI088数据
 * @param huart UART句柄
 */
void Send_BMI088_Data(UART_HandleTypeDef *huart)
{
    // 格式化输出BMI088数据
    int len = sprintf(uart_buffer,
        "BMI088 Data:\r\n"
        "Accel: X=%.3f, Y=%.3f, Z=%.3f m/s^2\r\n"
        "Gyro:  X=%.3f, Y=%.3f, Z=%.3f rad/s\r\n"
        "Temp:  %.2f C\r\n"
        "--------------------------------\r\n",
        BMI088.Accel[0], BMI088.Accel[1], BMI088.Accel[2],
        BMI088.Gyro[0], BMI088.Gyro[1], BMI088.Gyro[2],
        BMI088.Temperature
    );
    
    // 通过UART发送
    HAL_UART_Transmit(huart, (uint8_t*)uart_buffer, len, 1000);
}

/**
 * @brief 将浮点数拆分为整数部分和两位小数部分（用于不支持%f的printf）
 * @param value 输入的浮点数
 * @param int_part 输出的整数部分指针
 * @param frac_part 输出的小数部分指针（总为正数，范围0-99）
 */
static void float_to_parts(float value, int32_t *int_part, int32_t *frac_part) {
  // 1. 将浮点数放大100倍并转换为整数（实现四舍五入）
  int32_t scaled = (int32_t)(value * 100.0f + (value < 0 ? -0.5f : 0.5f));

  // 2. 分离整数和小数部分
  *int_part = scaled / 100;
  *frac_part = (scaled < 0 ? -scaled : scaled) % 100; // 小数部分始终取正
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  SEGGER_RTT_Init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // 初始化DWT，STM32F407通常是168MHz（根据你的实际配置调整）
  DWT_Init(168);  // 如果你的CPU频率是其他值，请修改这里（单位：MHz）
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  M2006_InitAll(motors, &hcan1);
  User_Uart_Init(&huart6);
  
  // 初始化BMI088（0表示不使用在线标定，使用离线参数）
  uint8_t bmi088_error = BMI088Init(&hspi1, 0);
  if (bmi088_error != BMI088_NO_ERROR)
  {
    // 初始化失败，通过UART发送错误信息
    char error_msg[64];
    int len = sprintf(error_msg, "BMI088 Init Failed! Error: 0x%02X\r\n", bmi088_error);
    HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, len, 1000);
  }
  else
  {
    // 初始化成功
    HAL_UART_Transmit(&huart1, (uint8_t*)"BMI088 Init Success!\r\n", 22, 1000);
  }
  
  HAL_Delay(500);  // 等待一段时间让传感器稳定
  
  demo_motor_init_lowpos();
  HAL_Delay(1000);
  
  M2006_SetTarget(&motors[0], dir[0] * 3000);
  M2006_SetTarget(&motors[1], dir[1] * 3000);
  HAL_Delay(5000);
  // ===== 修复后的PID参数打印（使用整数技巧）=====
  int32_t kp_int, kp_frac, ki_int, ki_frac, kd_int, kd_frac, max_int, max_frac;
  float_to_parts(motors[0].controller.inner_loop.Kp, &kp_int, &kp_frac);
  float_to_parts(motors[0].controller.inner_loop.Ki, &ki_int, &ki_frac);
  float_to_parts(motors[0].controller.inner_loop.Kd, &kd_int, &kd_frac);
  float_to_parts(motors[0].controller.inner_loop.output_max, &max_int, &max_frac);

  int32_t target_int, target_frac;
  float_to_parts(motors[0].target, &target_int, &target_frac);

  SEGGER_RTT_printf(0, "\n===== PID Speed Loop Debug =====\n");
  SEGGER_RTT_printf(0, "Motor 1 PID Params: Kp=%s%d.%02d, Ki=%s%d.%02d, Kd=%s%d.%02d, MaxOut=%s%d.%02d\n",
                    (motors[0].controller.inner_loop.Kp < 0 ? "-" : ""), (kp_int < 0 ? -kp_int : kp_int), kp_frac,
                    (motors[0].controller.inner_loop.Ki < 0 ? "-" : ""), (ki_int < 0 ? -ki_int : ki_int), ki_frac,
                    (motors[0].controller.inner_loop.Kd < 0 ? "-" : ""), (kd_int < 0 ? -kd_int : kd_int), kd_frac,
                    (motors[0].controller.inner_loop.output_max < 0 ? "-" : ""), (max_int < 0 ? -max_int : max_int), max_frac);
  SEGGER_RTT_printf(0, "Initial Target: %s%d.%02d RPM\n",
                    (motors[0].target < 0 ? "-" : ""),
                    (target_int < 0 ? -target_int : target_int),
                    target_frac);
  SEGGER_RTT_printf(0, "-------------------------------\n");
  static uint32_t loop_counter = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    M2006_UpdateAll(motors, 2);
    hss_m1_actual_rpm = motors[0].feedback.speed_filtered;
    hss_m1_pid_error  = motors[0].controller.inner_loop.last_error;

    // ===== 每100次循环打印一次反馈频率（每100ms） =====
    if (loop_counter % 100 == 0) {
      uint32_t feedback_freq = M2006_GetFeedbackFrequency(0);
      SEGGER_RTT_printf(0, "[Feedback Freq] Motor 1 CAN Feedback: %lu Hz\n", feedback_freq);
    }
    
    // ===== 每10次循环打印一行紧凑数据 =====
    if (loop_counter % 10 == 0) {
      // 1. 将当前所有关键数据转换为整数部分和小数部分
      int32_t t_int, t_frac, a_int, a_frac, e_int, e_frac, o_int, o_frac, i_int, i_frac;
      float_to_parts(motors[0].target, &t_int, &t_frac);
      float_to_parts(motors[0].feedback.speed_filtered, &a_int, &a_frac);
      float_to_parts(motors[0].controller.inner_loop.last_error, &e_int, &e_frac);
      float_to_parts(motors[0].controller.inner_loop.output, &o_int, &o_frac);
      float_to_parts(motors[0].controller.inner_loop.Ki * motors[0].controller.inner_loop.integral, &i_int, &i_frac);

      // 2. 打印
      SEGGER_RTT_printf(0,
          "[%05lu] Target:%s%4d.%02d | Actual:%s%5d.%02d | Error:%s%6d.%02d | PID_Out:%s%7d.%02d | I-Term:%s%9d.%02d\n",
          loop_counter,
          (motors[0].target < 0 ? "-" : " "), (t_int < 0 ? -t_int : t_int), t_frac,
          (motors[0].feedback.speed_rpm < 0 ? "-" : " "), (a_int < 0 ? -a_int : a_int), a_frac,
          (motors[0].controller.inner_loop.last_error < 0 ? "-" : " "), (e_int < 0 ? -e_int : e_int), e_frac,
          (motors[0].controller.inner_loop.output < 0 ? "-" : " "), (o_int < 0 ? -o_int : o_int), o_frac,
          (motors[0].controller.inner_loop.integral < 0 ? "-" : " "), (i_int < 0 ? -i_int : i_int), i_frac
      );
    }
    loop_counter++;

    HAL_Delay(1);
    /*// 读取BMI088数据
    BMI088_Read(&BMI088);
    
    // 发送数据并通过UART1输出（每100ms发送一次，10Hz频率）
    Send_BMI088_Data(&huart1);
    
    // 延时100ms，控制发送频率为10Hz
    HAL_Delay(100);*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
