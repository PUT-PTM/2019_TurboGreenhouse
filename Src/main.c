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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
#define DISP_1_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define DISP_1_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define DISP_2_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define DISP_2_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define DISP_3_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define DISP_3_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define DISP_4_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define DISP_4_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)

#define DISP_VAL_NULL HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                               GPIO_PIN_2 | GPIO_PIN_3 | \
                                               GPIO_PIN_4 | GPIO_PIN_5 | \
                                               GPIO_PIN_6 | GPIO_PIN_7 , \
                                               GPIO_PIN_SET)

#define DISP_VAL_0	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 | GPIO_PIN_3 | \
                                              GPIO_PIN_4 | GPIO_PIN_5 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_1	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 | GPIO_PIN_2 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_2	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_3 | GPIO_PIN_4 | \
                                              GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_3	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 | GPIO_PIN_3 | \
                                              GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_4	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 | GPIO_PIN_2 | \
    	                                       GPIO_PIN_5 | GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_5	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_2 | \
                                              GPIO_PIN_3 | GPIO_PIN_5 | \
                                              GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_6	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_2 | \
                                              GPIO_PIN_3 | GPIO_PIN_4 | \
                                              GPIO_PIN_5 | GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_7	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_8	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 | GPIO_PIN_3 | \
                                              GPIO_PIN_4 | GPIO_PIN_5 | \
                                              GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_9	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 | GPIO_PIN_3 | \
                                              GPIO_PIN_5 | GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)

#define DISP_DOT  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 , \
                                              GPIO_PIN_RESET)

int segmentIndex = 0;
int dispVal1 = 2;
int dispVal2 = 1;
int dispVal3 = 3;
int dispVal4 = 7;
int segmentsCount = 4;

int counterToDisplayValue = -1;

long liquid_level; //PA4 ADC1
uint32_t soil_moisture; //PA6 ADC2
int pom=0;

#define LIGTH_SENSOR_ADDRESS_WRITE 0x39<<1
#define LIGTH_SENSOR_ADDRESS_READ	 0x73

uint32_t lightIntensity;

int secs = 0;
int mins = 0;
int hours = 0;

static const int WATERING_TIME = 5; // seconds
static const int ABSORPTION_TIME = 60; //seconds
int wateringCounter = 0;
int absorptionCounter = 0;
int waterFlag = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PRZEPELINENIE TIMERA CO SEKUNDE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM4)
       {
            secs++;
            if(secs >=60 ){
            	secs=0;
            	mins++;
            }
            if(mins>=60){
            	mins=0;
            	hours++;
            }
            if(hours==24){
            	secs = 0;
            	mins = 0;
            	hours = 0;
            }

			wateringCounter++;
			absorptionCounter++;
       }

	// WYSWIETLANIE WARTOŒCI
	if(htim->Instance == TIM2)
	   	{
		if(segmentIndex==0){
				DISP_2_OFF;
				DISP_3_OFF;
				DISP_4_OFF;
				DISP_VAL_NULL;

				DISP_1_ON;
				if(dispVal1==0) DISP_VAL_0;
				if(dispVal1==1) DISP_VAL_1;
				if(dispVal1==2) DISP_VAL_2;
				if(dispVal1==3) DISP_VAL_3;
				if(dispVal1==4) DISP_VAL_4;
				if(dispVal1==5) DISP_VAL_5;
				if(dispVal1==6) DISP_VAL_6;
				if(dispVal1==7) DISP_VAL_7;
				if(dispVal1==8) DISP_VAL_8;
				if(dispVal1==9) DISP_VAL_9;
				DISP_DOT;
				segmentIndex=1;
			}
			else if(segmentIndex==1 && segmentsCount > 1){
				DISP_1_OFF;
				DISP_3_OFF;
				DISP_4_OFF;
				DISP_VAL_NULL;

				DISP_2_ON;
				if(dispVal2==0) DISP_VAL_0;
				if(dispVal2==1) DISP_VAL_1;
				if(dispVal2==2) DISP_VAL_2;
				if(dispVal2==3) DISP_VAL_3;
				if(dispVal2==4) DISP_VAL_4;
				if(dispVal2==5) DISP_VAL_5;
				if(dispVal2==6) DISP_VAL_6;
				if(dispVal2==7) DISP_VAL_7;
				if(dispVal2==8) DISP_VAL_8;
				if(dispVal2==9) DISP_VAL_9;

				segmentIndex=2;
			}
			else if(segmentIndex==2 && segmentsCount > 2){
				DISP_1_OFF;
				DISP_2_OFF;
				DISP_4_OFF;
				DISP_VAL_NULL;

				DISP_3_ON;
				if(dispVal3==0) DISP_VAL_0;
				if(dispVal3==1) DISP_VAL_1;
				if(dispVal3==2) DISP_VAL_2;
				if(dispVal3==3) DISP_VAL_3;
				if(dispVal3==4) DISP_VAL_4;
				if(dispVal3==5) DISP_VAL_5;
				if(dispVal3==6) DISP_VAL_6;
				if(dispVal3==7) DISP_VAL_7;
				if(dispVal3==8) DISP_VAL_8;
				if(dispVal3==9) DISP_VAL_9;

				segmentIndex=3;
			}
			else if(segmentIndex==3 && segmentsCount > 3){
				DISP_1_OFF;
				DISP_2_OFF;
				DISP_3_OFF;
				DISP_VAL_NULL;

				DISP_4_ON;
				if(dispVal4==0) DISP_VAL_0;
				if(dispVal4==1) DISP_VAL_1;
				if(dispVal4==2) DISP_VAL_2;
				if(dispVal4==3) DISP_VAL_3;
				if(dispVal4==4) DISP_VAL_4;
				if(dispVal4==5) DISP_VAL_5;
				if(dispVal4==6) DISP_VAL_6;
				if(dispVal4==7) DISP_VAL_7;
				if(dispVal4==8) DISP_VAL_8;
				if(dispVal4==9) DISP_VAL_9;

				segmentIndex=0;
			}
	   	}

       //CZAS PODLEWANIA PRZEZ POMPE
       if(htim->Instance == TIM3)
        {
             /*watering_timer++;
             if(watering_timer >= 10){
            	 HAL_TIM_Base_Stop_IT(&htim3); //PO 10 SEKUNDACH PODLEWANIA POMPA SIE WYLACZA
            	 HAL_TIM_Base_Start_IT(&htim4); //ODPALAMY TIMER ODLICZAJACY CZAS
             }*/
        }
}

void I2C_DEV_Write(uint16_t I2C_Addr,uint16_t Register_Addr,uint8_t Register_Data)
{
	HAL_I2C_Mem_Write(&hi2c3, I2C_Addr, Register_Addr, I2C_MEMADD_SIZE_8BIT, &Register_Data, 1, 1000);
}

uint8_t I2C_DEV_Read(uint16_t Register_Addr)
{
	uint8_t ReadBuffer[1];
	HAL_I2C_Mem_Read(&hi2c3, LIGTH_SENSOR_ADDRESS_READ, Register_Addr, I2C_MEMADD_SIZE_8BIT,ReadBuffer,1, 1000);
	return ReadBuffer[0];
}

uint32_t getLightIntensityValue(){
	uint32_t DataLow = I2C_DEV_Read(0x80 | 0x40 | 0X14);
	uint32_t DataHigh = I2C_DEV_Read(0x80 | 0x40 | 0X15);

	return (256 * DataHigh + DataLow);
}

void lightSensorInit()
{
	// Power ON
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x00,0x01);
	HAL_Delay(2000);

	// Timing 400ms
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x01, 0x6C);

	// ADC Interrupt
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x00, 0X02 | 0x01);
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0X02, 0x18);

	// Gain 16x (1x -> 0x00; 8x -> 0x01; 16x ->0x02; 111x -> 0x03)
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x07, 0x02);
}

void Reload_register(void)
{
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x60 | 0x01, 0x18);
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x00, 0X02 | 0x01);
}

void SET_Interrupt_Threshold(uint16_t min,uint16_t max)
{
	uint8_t DataLLow,DataLHigh,DataHLow,DataHHigh;
	DataLLow = min % 256;
	DataLHigh = min / 256;
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x03, DataLLow);
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x04, DataLHigh);

	DataHLow = max % 256;
	DataHHigh = max / 256;
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x05, DataHLow);
	I2C_DEV_Write(LIGTH_SENSOR_ADDRESS_WRITE,0x80 | 0x06, DataHHigh);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  lightSensorInit();
 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_Start(&hadc2);
	  if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	  {
	      liquid_level = HAL_ADC_GetValue(&hadc1);
	  }

	  if(HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
	  {
	      soil_moisture = HAL_ADC_GetValue(&hadc2);
	      soil_moisture = 4095 - soil_moisture;
	  }

	  if(soil_moisture <= 700 && absorptionCounter > ABSORPTION_TIME)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // ZACZYNA DZIALAC POMPA
		  absorptionCounter = 0;
		  wateringCounter = 0;
	  }

	  if(wateringCounter > WATERING_TIME){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // PRZESTAJE DZIALAC POMPA
		  wateringCounter = 0;
	  }

	  if(liquid_level < 2000)
	  {
		  //ALARM JAKIS
	  }

	  //CONFIRMED
	  if(lightIntensity < 5000)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	  }
	  if(lightIntensity > 5000)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	  }

	  //Odczyt natê¿enia œwiat³a
	  SET_Interrupt_Threshold(2000,50000);
	  lightIntensity = getLightIntensityValue();
	  HAL_Delay(500);

	  if(counterToDisplayValue != mins){
		  double dispVal = liquid_level/4095.1*1000.0;
		  dispVal1 = 0;
		  dispVal2 = (int)(dispVal/100);
		  dispVal3 = (int)((dispVal - dispVal2*100)/10);
		  dispVal4 = (int)(dispVal - dispVal2*100 - dispVal3*10);
	  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
