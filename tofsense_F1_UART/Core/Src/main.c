/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define TOF_UART	huart1

uint8_t rx_temp;		//临时接收缓存
uint8_t u_rx_buf[16];	//缓存数组
float final_data = 0;	//最终输出

//重定向printf
#include "stdio.h"

int fputc(int ch, FILE *f) //重定向printf函数
{
    HAL_UART_Transmit(&huart2, (void*)&ch, 1, 1000);
    return ch;
}

//串口数据处理、解析
#include "nlink_tofsense_frame0.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t c = 0;
	
	static float old_data = 0;
	
	if (c == 0)
	{
		if (rx_temp == 0x57) 	//判断帧头
		{
			u_rx_buf[c++] = rx_temp;
		}
		else
		{
			c = 0;
		}
	}
	else if (c == 1)
	{
		if (rx_temp == 0x00)		//判断关键字
		{
			u_rx_buf[c++] = rx_temp;
		}
		else
		{
			c = 0;
		}
	}
	else
	{
		u_rx_buf[c++] = rx_temp;
		if (c >= 16)	//数据接够了 开始解析
		{
			if (g_nts_frame0.UnpackData(u_rx_buf, sizeof(u_rx_buf)/sizeof(u_rx_buf[0])))
			{
				//这里做了一个简单的过滤与数据处理
				//不同型号的TOFsense过滤参数是不一样的需要根据数据手册来处理
				if (
					g_nts_frame0.result.dis <= 0		  ||	//量程过滤
					g_nts_frame0.result.dis >= 5		  ||
					g_nts_frame0.result.dis_status == 14  ||	//距离状态指示过滤
					g_nts_frame0.result.dis_status == 255 ||
					g_nts_frame0.result.signal_strength == 0	//信号强度过滤
				)
				{
					final_data = old_data;	//对异常数据取上一正常数据的处理
				}
				else
				{
					final_data = g_nts_frame0.result.dis;		//更新当前数据
					old_data = g_nts_frame0.result.dis;		//更新上一数据
				}
				
				printf("接收的数据为:%f\r\n",final_data);		//打印到串口
			}
			c = 0;	//清空索引
		}

	}
	HAL_UART_Receive_IT(&TOF_UART, &rx_temp, 1);
}

//发送查询命令获取数据
void Inquire_data(uint8_t id)
{
	static uint8_t u_tx_buf[8];
	
	u_tx_buf[0] = 0x57;	//帧头
	u_tx_buf[1] = 0x10;	//关键字
	
	u_tx_buf[2] = 0xFF;
	u_tx_buf[3] = 0xFF;
	
	u_tx_buf[4] = id;	//所需查询模块的ID
	
	u_tx_buf[5] = 0xFF;
	u_tx_buf[6] = 0xFF;
	
	u_tx_buf[7] = 0;	//清空校验和
	
	for (int i = 0; i < 7; i++)
	{
		u_tx_buf[7] += u_tx_buf[i];
	}
	HAL_UART_Transmit_DMA(&TOF_UART, u_tx_buf, sizeof(u_tx_buf));
}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&TOF_UART, &rx_temp, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  Inquire_data(0);	//查询模式获取数据，主动输出模式注释掉
	  //查询之间最好延迟一段时间，一般为 delay = 1/刷新频率，如果超过这个速度会得到重复的数据
//	  HAL_Delay(100);	
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
