#include "main.h"

I2C_HandleTypeDef hi2c1;

#define slaveaddress 0xD0  // Address of MPU6050 sensor
int16_t ax,ay,az;          // To save the 16 bit data from sensor
float xaccel,yaccel,zaccel;// Which will contain the final output values

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

uint8_t i2cBuf[8]; //Array which is used in reading and writing of values to sensor using I2C

int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
	
	// Loop to check if the sensor is connected to the board and to verify its address 
	for(uint8_t i=0;i<255;i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1,i,1,10)==HAL_OK)
		{
 			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);// Inboard LED toggles when sensor is detected
			break;
		}
	}
	// Powering up the sensor 
	i2cBuf[0]=0x6B;//Corresponds to power configuration register
	i2cBuf[1]=0x00;//Enabling internal 8MHz clock of sensor
	HAL_I2C_Master_Transmit(&hi2c1,slaveaddress,i2cBuf,2,10);
	i2cBuf[0]=28;// Corresponds to accelerometer configuration register
	i2cBuf[1]=0x08;// Selecting full-scale range of +/- 4g
	HAL_I2C_Master_Transmit(&hi2c1,slaveaddress,i2cBuf,2,10);
	
	// Following lines can be used to verify if the read and write operations are working fine
	/*i2cBuf[0]=28;
	HAL_I2C_Master_Transmit(&hi2c1,slaveaddress,i2cBuf,1,10);
	i2cBuf[1]=0x00;
	// If i2cBuf[1] has 0x08 after read operation, the read and write operation works fine
	HAL_I2C_Master_Receive(&hi2c1,slaveaddress,&i2cBuf[1],1,10);*/
	
  /* Infinite loop */
  while (1)
  {
	  i2cBuf[0]=0x3B;// Corresponds to register containing values of x axis
		// Request to read from 3B register
	  HAL_I2C_Master_Transmit(&hi2c1,slaveaddress,i2cBuf,1,1);
		// The values of x,y and z axis are in 2 bytes (high and low) separately in the sensor from register 3B
		// So we have to read 6 bytes starting from register 3B to 40
	  HAL_I2C_Master_Receive(&hi2c1,slaveaddress,&i2cBuf[1],6,1);
		// Combining the 2 bytes and saving it in a 16 bit number
		ax=(i2cBuf[1]<<8|i2cBuf[2]);
		ay=(i2cBuf[3]<<8|i2cBuf[4]);
		az=(i2cBuf[5]<<8|i2cBuf[6]);
		// Sensitivity of accelerometer in +/- 4g is 8192/g
		// Converting the raw data in terms of acceleration in g
		xaccel=ax/8192.0;
		yaccel=ay/8192.0;
		zaccel=az/8192.0;
  }
}


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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


static void MX_I2C1_Init(void)
{


  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }


}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}



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

/********************END OF FILE****/
