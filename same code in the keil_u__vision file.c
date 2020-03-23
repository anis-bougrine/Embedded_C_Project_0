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
  * COPYRIGHT(c) 2020 STMicroelectronics
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt_delay.h"

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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t	x[8]={0,0,0,0,0,0,0,0};  		//where we will save the sensor ROM code (read from sensor and sended to stm with one-wire protocol) 
GPIO_InitTypeDef	GPIO_InitStruct;	//configuration classique I/O

void gpio_set_input(void)						//..
{
	GPIO_InitStruct.Pin=GPIO_PIN_7;
	GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
}

void gpio_set_output(void)					//..
{
	GPIO_InitStruct.Pin=GPIO_PIN_7;
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull=GPIO_NOPULL;
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
}

uint8_t ds18b20_init (void)					//initialisation of the communication (1-wire)
{
    gpio_set_output ();   // set the pin as output
    HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, 0);  // pull the pin low
    DWT_Delay(480);   // delay according to datasheet
    gpio_set_input ();    // set the pin as input
    DWT_Delay(80);    // delay according to datasheet
    if (!(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_7)))    // if the pin is low i.e the presence pulse is there
    {
        DWT_Delay(400); // wait for 400 us
        return 0;
    }
    else
    {
        DWT_Delay(400);  // wait for 400 us
        return 1;
    }
}

void write (uint8_t data)							//Function allows the stm write something on the wire to configure the sensor mode
{
    gpio_set_output ();   // set as output
    for (int i=0; i<8; i++)
    {
        if ((data & (1<<i))!=0)  // if the bit is high
        {
            // write 1
            gpio_set_output ();  // set as output
            HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, 0);  // pull the pin LOW
            DWT_Delay(1);  // wait for  us
            gpio_set_input ();  // set as input
            DWT_Delay(60);  // wait for 60 us
        }
        else  // if the bit is low
        {
            // write 0
            gpio_set_output ();
            HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, 0);  // pull the pin LOW
            DWT_Delay(60);  // wait for 60 us
            gpio_set_input ();
        }
    }
}

uint8_t read (void)											//Function allows the stm to recieve a byte from the sensor
{
	uint8_t value=0;
	gpio_set_input ();

	for (int i=0;i<8;i++)
	{
		gpio_set_output ();   // set as output

		HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, 0);  // pull the data pin LOW
		DWT_Delay(2);  // wait for 2 us

		gpio_set_input ();  // set as input
		if (HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_7))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		DWT_Delay(60);  // wait for 60 us
	}
	return value;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)														//main begins
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)													//infinitive loop
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		ds18b20_init();									//initialisation of the communication (1-wire)
		write(0x33);										//configure the sensor on ROM mode
		for(int i=7;i>0;i--)						//loop to reed the 8 bytes of the sensor ROM (byte/byte)
		{
					x[i]=read();
		}

			for(int i=0;i<8;i++)					//decode the code recieved from the sensor (decode algorithm is in the specification)
			{
			if((x[i]&1111)==0xA)
				x[i]=((x[i]&11110000)|1);
			if((x[i]&11110000)==0xA0)
				x[i]=((x[i]&1111)|10000);
			if((x[i]&1111)==0xB)
				x[i]=((x[i]&11110000)|10);
			if((x[i]&11110000)==0xB0)
				x[i]=((x[i]&1111)|100000);
			if((x[i]&1111)==0xC)
				x[i]=((x[i]&11110000)|11);
			if((x[i]&11110000)==0xC0)
				x[i]=((x[i]&1111)|110000);			
			if((x[i]&1111)==0xD)
				x[i]=((x[i]&11110000)|100);
			if((x[i]&11110000)==0xD0)
				x[i]=((x[i]&1111)|1000000);
			if((x[i]&1111)==0xE)
				x[i]=((x[i]&11110000)|101);
			if((x[i]&11110000)==0xE0)
				x[i]=((x[i]&1111)|1010000);
			if((x[i]&1111)==0xF)
				x[i]=((x[i]&11110000)|110);
			if((x[i]&11110000)==0xF0)
				x[i]=((x[i]&1111)|1100000);	
			}

				//we will send the code saved in X (4 bytes/4 bytes) because the spi of esp support only 4bytes per trame.
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);  //activate the ESP chipselect
        HAL_SPI_Transmit(&hspi1,x,4, HAL_MAX_DELAY);					 //send the first 4 bytes of X to the esp 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);		 //desactivate the ESP chipselect
        HAL_Delay(10);
				for(int i=4;i<8;i++)
				{
					x[i-4]=x[i];
				}
		
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);		//activate the ESP chipselect
        HAL_SPI_Transmit(&hspi1,x,4, HAL_MAX_DELAY);						//send the last 4 bytes  of X to the esp 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);     //desactivate the ESP chipselect
        HAL_Delay(10);    
	HAL_Delay(1000); //wait 1 s to let the ESP execute her code before we send another time
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

