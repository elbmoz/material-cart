/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor_Move.h"
#include "serial.h"
#include "HWT101.h"
#include "GC_Chassis_Control.h"
#include "delay.h"
#include "servo.h"
#include "QRcode.h"
#include "bluetooth.h"
#include "LobotServoController.h"
#include "action.h"
#include "huaner_servo.h"
#include "ServoMotorControl.h"
#include "encoder_f407.h"
#include "GC_task.h"
#include "WS2812.h"
#include "laser.h"
#include "screen.h"
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static volatile uint8_t main_qrcode_task1 = 0U;
static volatile uint8_t main_qrcode_task2 = 0U;
static volatile uint8_t main_qrcode_task3 = 0U;
static volatile uint8_t main_qrcode_task4 = 0U;
static volatile uint8_t main_qrcode_task5 = 0U;
static volatile uint8_t main_qrcode_task6 = 0U;
static volatile uint8_t main_qrcode_ready = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_UART5_Init();
	
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
  DWT_Delay_Init();
  WS2812_Init();
  Encoder_init();
  Motor_Init(&huart1);
  Enable();
  /* QR code reception is started in GC_saoma by GC_Task_ReadQRCodeTasks(). */

  
  /* Vision serial reception starts after QR code scanning in GC_task. */
  HWT101_Init(&huart2);
  LobotServo_Init();
  GC_Task_Init();
	fangche1();
	
  WS2812_SetRGB(0U, 20U, 0U);
  WS2812_Rainbow(WS2812_LED_COUNT, 0U, 255U);
	//	HAL_UART_Transmit(&huart5, (uint8_t *)"hi", 2, HAL_MAX_DELAY);
	//servo13_Home(75);
	
#if 1
  if (ZAxis_Home(1000, 200, 5000) != HAL_OK) {
    Error_Handler();
  }
  delay_ms(10);
  ZAxis_MoveRelative(-1000, 200, 1000);//Ҫ����-3300���ܷţ�������-2000��Ϊ�˿������Բ��?
delay_ms(1000);
#endif
  
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Error_Handler();
  }
	
fang();
//zhua();
  
  
  //kan();
//			delay_ms(500);
//  delay_ms(2000);
//	
  
  // ????,????????
	
	
//*******************************************************************************  cjh_6_7
//                  /*有用的代码，误删*/
//  //扫二维码
//  if (QRcode_WaitTaskGroupsAndStop(&main_qrcode_task1,
//                                   &main_qrcode_task2,
//                                   &main_qrcode_task3,
//                                   &main_qrcode_task4,
//                                   &main_qrcode_task5,
//                                   &main_qrcode_task6,
//                                   &main_qrcode_ready) != HAL_OK) {
//    Error_Handler();
//  }



//	fang();
//	delay_ms(1000);
 //Serial_Init(&huart5);
//                     /*第一圈*/
//  //抓物块
//  Action_RunQRCodeTasks_Pos(1,
//                        2,
//                       3);



//  //视觉矫正
//  VisionPrecision_Calibrate();
////  //抓车上的物块到地面上
//  Action_PlaceLoadedQRCodeTasks1(1,
//                              2,
//                              3);

//  //放物块回车上
//  Action_RecoverGroundQRCodeTasks((uint8_t)main_qrcode_task1,
//                                (uint8_t)main_qrcode_task2,
//                                (uint8_t)main_qrcode_task3);
//  //抓车上的物块到地面上
//  Action_PlaceLoadedQRCodeTasks1((uint8_t)main_qrcode_task1,
//                              (uint8_t)main_qrcode_task2,
//                              (uint8_t)main_qrcode_task3);
//				/*第一圈结束*/

//				/*第二圈*/
//  //抓物块
//  Action_RunQRCodeTasks((uint8_t)main_qrcode_task4,
//                        (uint8_t)main_qrcode_task5,
//                        (uint8_t)main_qrcode_task6);
//  //视觉矫正
//  VisionPrecision_Calibrate();
//  //抓车上的物块到地面上
//  Action_PlaceLoadedQRCodeTasks1((uint8_t)main_qrcode_task4,
//                              (uint8_t)main_qrcode_task5,
//                              (uint8_t)main_qrcode_task6);
//  //放物块回车上
//  Action_RecoverGroundQRCodeTasks((uint8_t)main_qrcode_task4,
//                                (uint8_t)main_qrcode_task5,
//                                (uint8_t)main_qrcode_task6);
//  //视觉矫正
//Serial_Init(&huart5);
//  VisionPrecision_Calibrate2();
//  //抓车上的物块叠放到地面上
// Action_PlaceLoadedQRCodeTasks2(1,
//                              2,
//                              3);
//                        /*截止*/
//********************************************************************************


//		delay_ms(100);
//zhuafang3();
//	Pos_zuoxie(200,2000,6000);
////	delay_ms(1000);
//	Pos_RunStraight(150,200,6000);
////	Pos_RunStraight(150,200,6000);
////	Pos_RunStraight(150,200,6000);
////	Pos_RunStraight(150,200,20000);
//	
////	delay_ms(1000);
////	InPlaceTurn_Block(&in_place_turn, -2.0f);
////	delay_ms(100);
////	//saoma
////	
////	//liaopan
////	Pos_RunStraight(150,200,6000);
////	Pos_RunStraight(100,1000,1800);
			//Pos_RunStraight(150,2000,1000);
//	
//	
//	//delay_ms(1000);
//	//CarStraight_Block(&car_straight, 1000.0f, 0.0f, (float)global_angle); // 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_bt_tick = HAL_GetTick();
//HAL_UART_Transmit(&huart5, (uint8_t *)"hi", 2, HAL_MAX_DELAY);

//FieldPos_Block(&field_pos, 200.0f, 200.0f, -90.0f);

		//Pos_RunRight(200,2000,2000);
		//Pos_RunLeft(200,2000,1000);
		//Pos_RunRight(200,2000,1000);
		//Pos_RunLeft(200,2000,1200);
	//Laser_On();
//	Screen_TestSend();
//	Screen_TestSend();
	
	//zhua();
	//fang();
//	delay_ms(2000);
//	
//	Serial_Init(&huart5);
//	
//	VisionPrecision_Calibrate();
//	Action_PlaceLoadedQRCodeTasks1(1U,2U,3U);
                             
	//Pos_Run(R,200,2000,1200);
	//WS2812_SetRGB(0U, 20U, 0U);
	
  while (1)
  {
    /* USER CODE END WHILE */
    pit_control();
    /* USER CODE BEGIN 3 */
    if ((GC_Task_GetState() != GC_WAIT_BTN) &&
        (GC_Task_GetState() != GC_DONE) &&
        (GC_Task_GetState() != GC_FAIL)) {
      WS2812_BluePurpleFlow(40U, 255U);
    }
    //GC_Task_Run();
		
		GC_Task_Run_Pos();
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (uint32_t)(HAL_RCC_GetPCLK1Freq() / 1000000U) - 1U;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000U - 1U;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    pit_1ms_pending++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  Motor_UART_RxCpltCallback(huart);
  serial_Handler(huart);
  QRcode_UART_RxCpltCallback(huart);

  hwt_Handler(huart);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
    Bluetooth_TxCplt();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Motor_UART_ErrorCallback(huart);
    serial_ErrorHandler(huart);
    QRcode_UART_ErrorCallback(huart);
    Bluetooth_TxCplt();
}

#ifdef  USE_FULL_ASSERT
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
