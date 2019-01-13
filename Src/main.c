/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "MAX30102/MAX30102.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_BRIGHTNESS 255
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t IrBuffer[500]; //IR LED sensor data
int32_t IrBufferLength;    //data length
uint32_t RedBuffer[500];    //Red LED sensor data
int32_t Sp02Value; //SPO2 value
int8_t Sp02IsValid;   //indicator to show if the SP02 calculation is valid
int32_t HeartRate;   //heart rate value
int8_t  IsHrValid;    //indicator to show if the heart rate calculation is valid
uint8_t Dummy;
char UartBuffer[32];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART2_SendString(char* s)
{
 HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  UART2_SendString("START\n\r");
  HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, 1);
  HAL_Delay(200);
  HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, 0);
  HAL_Delay(200);
  HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, 1);
  HAL_Delay(200);
  HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, 0);
  uint32_t MinValue, MaxValue, PreviousData;  //variables to calculate the on-board LED brightness that reflects the heartbeats
      int i;
      int32_t Brightness;
      float tmp;

      Max30102_Init(&hi2c1);  //initializes the MAX30102

      Brightness=0;
      MinValue=0x3FFFF;
      MaxValue=0;


      IrBufferLength=500; //buffer length of 100 stores 5 seconds of samples running at 100sps

      //read the first 500 samples, and determine the signal range
      for(i=0;i<IrBufferLength;i++)
      {
          while(HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin)==1);   //wait until the interrupt pin asserts
          Max30102_ReadFifo((RedBuffer+i), (IrBuffer+i));  //read from MAX30102 FIFO

          if(MinValue>RedBuffer[i])
              MinValue=RedBuffer[i];    //update signal min
          if(MaxValue<RedBuffer[i])
              MaxValue=RedBuffer[i];    //update signal max
          UART2_SendString("red=");
          sprintf(UartBuffer, "%i", RedBuffer[i]);
          UART2_SendString(UartBuffer);
          UART2_SendString(", ir=");
          sprintf(UartBuffer, "%i\n\r", IrBuffer[i]);
          UART2_SendString(UartBuffer);
      }
      PreviousData=RedBuffer[i];


      //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
      maxim_heart_rate_and_oxygen_saturation(IrBuffer, IrBufferLength, RedBuffer, &Sp02Value, &Sp02IsValid, &HeartRate, &IsHrValid);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   i=0;
	        MinValue=0x3FFFF;
	        MaxValue=0;

	        //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
	        for(i=100;i<500;i++)
	        {
	            RedBuffer[i-100]=RedBuffer[i];
	            IrBuffer[i-100]=IrBuffer[i];

	            //update the signal min and max
	            if(MinValue>RedBuffer[i])
	            MinValue=RedBuffer[i];
	            if(MaxValue<RedBuffer[i])
	            MaxValue=RedBuffer[i];
	        }

	        //take 100 sets of samples before calculating the heart rate.
	        for(i=400;i<500;i++)
	        {
	            PreviousData=RedBuffer[i-1];
	            while(HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin)==1);
	            Max30102_ReadFifo((RedBuffer+i), (IrBuffer+i));

	            if(RedBuffer[i]>PreviousData)
	            {
	                tmp=RedBuffer[i]-PreviousData;
	                tmp/=(MaxValue-MinValue);
	                tmp*=MAX_BRIGHTNESS;
	                Brightness-=(int)tmp;
	                if(Brightness<0)
	                    Brightness=0;
	            }
	            else
	            {
	                tmp=PreviousData-RedBuffer[i];
	                tmp/=(MaxValue-MinValue);
	                tmp*=MAX_BRIGHTNESS;
	                Brightness+=(int)tmp;
	                if(Brightness>MAX_BRIGHTNESS)
	                    Brightness=MAX_BRIGHTNESS;
	            }
	            //send samples and calculation result to terminal program through UART
//	            UART2_SendString("red=");
	            sprintf(UartBuffer, "%i\n\r", RedBuffer[i]);
	            UART2_SendString(UartBuffer);
//	            UART2_SendString(", ir=");
//	            sprintf(UartBuffer, "%i", IrBuffer[i]);
//	            UART2_SendString(UartBuffer);
//	            sprintf(UartBuffer, ", HR=%i, ", HeartRate);
//	            UART2_SendString(UartBuffer);
//	            sprintf(UartBuffer, "HRvalid=%i, ", IsHrValid);
//	            UART2_SendString(UartBuffer);
//	            sprintf(UartBuffer, "SpO2=%i, ", Sp02Value);
//	            UART2_SendString(UartBuffer);
//	            sprintf(UartBuffer, "SPO2Valid=%i\n\r", Sp02IsValid);
//	            UART2_SendString(UartBuffer);
	        }
	        maxim_heart_rate_and_oxygen_saturation(IrBuffer, IrBufferLength, RedBuffer, &Sp02Value, &Sp02IsValid, &HeartRate, &IsHrValid);

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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
