/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_interface.h"
#include <stdbool.h>
#include <stdio.h>

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
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t device_id = 0x50; //unique device address for CAN
FDCAN_RxHeaderTypeDef rxHeader; //CAN Bus Receive Header
FDCAN_TxHeaderTypeDef txHeader; //CAN Bus Transmit Header
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t csend[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

//uint32_t canMailbox; //CAN Bus Mail box variable

struct can_interface canbus;

//Laser controller
void delay_us (uint32_t us){

	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the "us" input in the parameter

}

//Initialize structure to organize code in while loop
struct laser_line
	{
	uint32_t exp_time;
	uint32_t intensity;
	volatile uint32_t *reg_ref; // format for assigning pointers --> volatile uint32_t* a= l1.reg_ref;
	uint32_t delay;
	};

//define piezo parameters for sequence
uint32_t piezo_start; //in units of micrometer
uint32_t piezo_steps; // in units of steps
uint32_t piezo_step_size; // in units of micrometer

//define piezo parameters for solo use
uint32_t piezo_position = 1; //positioning of piezo for solo use
bool solo_piezo = false; //changes piezo independently

	//define camara exposure variables
	uint8_t cam_expo = false; // logical to delay code if camara exposur is "true"

	void expose(struct laser_line laser) {

		*laser.reg_ref = laser.intensity; // *laser.reg_ref is storing the laser intensity in the corresponding output pin register
		HAL_Delay(1); //additional delay to make sure laser is on. HAL_Delay is in units of millisecond.
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); //turns TTL camtriger ON
		delay_us(laser.exp_time*1000); // Implements exposure time. This delay is in units MICRO-seconds (1000 times smaller that the units used for HAL delay).
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); //turns TTL camtriger OFF
		//additional delay to make sure adquisition is completed by rolling shutter
		do{
			HAL_Delay(1); //minimum delay to make sure the rolling shutter finishes its operation
			cam_expo = HAL_GPIO_ReadPin(Cam_exp_PB5_GPIO_Port, Cam_exp_PB5_Pin); // read status of GPIO input pin PB5 for camara exposure
		}	while (cam_expo == true); //delay continues for as long as shutter cam_expo is "true"
		//stop illumination
		*laser.reg_ref = 4; // 4/4096 bytes for a 12bit register produces a minimal duty cycle that creates a voltage very close to 0V.
		HAL_Delay(1); //delay to ramp down the voltage
		*laser.reg_ref = 0; // iddle mode for about 65 milli-seconds
		delay_us(laser.delay*1000); // This delay is in units MICRO-seconds (1000 times smaller that the units used for the exposure time).

}

void piezo(uint32_t piezo_start,uint32_t steps, uint32_t displacement){

//steps = 1;	//steps input not allowed to proceed. Capped at a single step. Comment out to modify
double piezo_out = (steps*displacement+piezo_start)/100.; //Make into a double to create fraction output

	if (piezo_out==0) {

		TIM3->CCR1 = 4;
		HAL_Delay(1); //delay to ramp down the voltage
		TIM3->CCR1 = 0;

	}
	else {
		TIM3->CCR1 = 4095*(piezo_out); //Assign piezo movement to lase line 4. Defined by TIM3->CCR1
		HAL_Delay(30); //30 ms delay to allow piezo to stabilize before imaging each plane

	}

} // end of piezo function

struct laser_line laser_arr[4];

// define a boolean trigger (trig) to start, and stop, the laser adquisition sequence
bool trig = false;
// boolean to start solo imaging; that is, whitout a sequence
bool solo_imaging = false;

// define laser sequence
uint32_t laser_seq = 2; //tells the program how many lasers will get activated in an imaging sequence. Numbers from 1 to 4.

//void set_startimaging(bool data){trig = data;}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

void set_pressure(uint8_t, uint8_t, uint32_t); //function prototype of set_pressure
void set_exposure(uint8_t, uint8_t, uint32_t); //function prototype of set_exposure
void set_intensity(uint8_t, uint8_t, uint32_t); //function prototype of set_intensity
void set_delay(uint8_t, uint8_t, uint32_t); //function prototype of set_delay
void set_startimaging(uint8_t, uint8_t, uint32_t); //function prototype of set_startimaging
void set_piezo(uint8_t, uint8_t, uint32_t); //function prototype of set_piezo
void set_solopiezo(uint8_t, uint8_t, uint32_t); //function prototype to set piezo independently
void set_soloimaging(uint8_t, uint8_t, uint32_t); //function prototype to set imaging independently
void can_write(int, uint8_t*);

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

	canbus.transmit_function = can_write; // add wrapper for transmit

	//Add functions to canbus interface
	canbus.interface_functions [MSG_WRITE][SET_EXPOSURE] = set_exposure;
	canbus.interface_functions [MSG_WRITE][SET_INTENSITY] = set_intensity;
	canbus.interface_functions [MSG_WRITE][SET_DELAY] = set_delay;
	canbus.interface_functions [MSG_WRITE][SET_STARTIMAGING] = set_startimaging;
	canbus.interface_functions [MSG_WRITE][SET_PIEZO] = set_piezo;
	canbus.interface_functions [MSG_WRITE][SET_SOLOPIEZO] = set_solopiezo;
	canbus.interface_functions [MSG_WRITE][SET_SOLOIMAGING] = set_soloimaging;

	FDCAN_FilterTypeDef canfil; //CAN Bus Filter

	canfil.IdType = FDCAN_STANDARD_ID; //code from https://programmer.ink/think/stm32-canfd-basics.html
	canfil.FilterIndex = 0;
	canfil.FilterType = FDCAN_FILTER_MASK;
	canfil.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	canfil.FilterID1 = device_id; // for laser piezo controller, it's 0x50
	canfil.FilterID2 = 0x7FF; // 0x7FF blocks everzthing, except for FilterID1
	//canfil.FilterID2 = 0x0; // 0x0 lets msgs all through

if (HAL_FDCAN_ConfigFilter(&hfdcan1, &canfil) != HAL_OK)
{
  Error_Handler();
}

HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

	// FDCAN code

if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
{
				Error_Handler();
}

if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
{
        Error_Handler();
}

if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
{
        Error_Handler();
}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

// Laser controller code

HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //Laser 1 -- D3  -> PB0 output code --> TIM3_CH3
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //Laser 2 -- D6 -> PB6 --> TIM4_CH1
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Laser 3 -- D9 -> PA8 --> TIM1_CH1
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Laser 4 -- A5 -> PA6 --> TIM3_CH1
HAL_TIM_Base_Start(&htim2); // Timer 2, used to initialize micro-second delay

	//Register assignment
	//saved address of the register in  pointer defined by laser.reg_ref
	// laser_arr[i].reg_ref was activated as a pointer in reg.ref function definition. therefore, here we assign the address of the register &TIMx->CCRy to laser_arr[i].reg_ref
		laser_arr[0].reg_ref = &TIM3->CCR3; 	//Brightfield
		laser_arr[1].reg_ref = &TIM4->CCR1;		//Laser 1
		laser_arr[2].reg_ref = &TIM1->CCR1;		//Laser 2
		laser_arr[3].reg_ref = &TIM3->CCR1;		//Laser 3

//CAN fuction interfaces for testing

// IR
set_exposure(0,0,100); // brightfield exposure
//set_intensity(0, 0,50);
set_intensity(0, 0,25); // set the illumination of the IR
set_delay (0,0,1);

// Laser 1
set_exposure(1,0,20); // first input is the laser line to be activated, and third input is the exposure in milliseconds
//set_intensity(1, 0,60); //intensity is fed as percentage of the max. acceptable output for the controller
set_intensity(1, 0,43); // hologram illumination requirement 43%
set_delay (1, 0,1); // delay before next laser gets activated in units of miliseconds

// Laser 2
set_exposure(2,0,350); //405 nm laser w/ 6310 controller at 500 mA
//set_intensity(2, 0,90); // For max output, exposure = 500 ms and intensity = 72
set_intensity(2, 0,72);
set_delay (2, 0,1);


//Piezo
//set_piezo(0, 10, 10);
//set_piezo(piezo_start, piezo_steps, piezo_step_size);
// piezo_start = 0; //in units of micrometer
// piezo_steps = 8; // in units of steps
// piezo_step_size = 23; // in units of micrometer

// Laser 3 //Laser line 4 being used to control brightfield
//set_exposure(3,0,0);
//set_intensity(3, 0, 100);
//set_delay (3, 0, 5);

// Laser 4 //Laser line 4 being used to control piezo
//set_exposure(4,0,0);
//set_intensity(4, 0, 100);
//set_delay (4,0,5);

  while (1)
  {

	  *laser_arr[0].reg_ref = laser_arr[0].intensity; // turn on IR when not running lasers
	  //HAL_Delay(1000);//comment out,just for looks and videos to see the IR
	  //trig = true; //comment out after testing
	  //set_startimaging(2,0,1); // comment off to start CAN interrupt trigering

	  if (trig == true) {

		  //stop IR before imaging
		  *laser_arr[0].reg_ref = 4; // 4/4096 bytes for a 12bit register produces a minimal duty cycle that creates a voltage very close to 0V.
		  HAL_Delay(1); //delay to ramp down the voltage
		  *laser_arr[0].reg_ref = 0; // step down to zero and iddle mode
		  delay_us(laser_arr[0].delay*1000); // This delay is in units MICRO-seconds (1000 times smaller that the units used for the exposure time).
		  HAL_Delay(100); //long delay before sequence, might need to reduce

		  for(int i=1; i <=(laser_seq); i++){ //laser for loop

			  if(laser_arr[i].exp_time>0){ //this if-statement makes sure that the code iterates throguh the laser line if an exposure value has been set
				  expose(laser_arr[i]);
				  HAL_Delay(100); //break at each step of sequence, might need to reduce, or eliminate
				  //expose(laser_arr[1]);
				  //expose(laser_arr[2]);
			  }

		  }

		  trig = false; // trigger == false will stop the illumination until trigger == true again...

	  } //end of imaging sequence

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  } 		//end of infinite WHILE LOOP

	//end of MAIN function below

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 36;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 3;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4095;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4.294967295E9;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4095;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4095;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Cam_exp_PB5_Pin */
  GPIO_InitStruct.Pin = Cam_exp_PB5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Cam_exp_PB5_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan1, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{

		if (HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &rxHeader, canRX)!= HAL_OK)
		{
			Error_Handler();
		}

	}

	ManageCAN_rx(canRX, canbus);

}

void can_write(int adresse, uint8_t* message){
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, message);
	//Can version // HAL_CAN_AddTxMessage(&hcan1,&txHeader,message,&canMailbox);

}

void set_exposure(uint8_t laser_spec, uint8_t data1_8, uint32_t data){laser_arr[laser_spec].exp_time = data;} //implementation of set_exposure

void set_intensity(uint8_t laser_spec, uint8_t data1_8, uint32_t data){  //implementation of set intensity (intensity input should be given in percentage)

	if (data <= 100) {

		switch(laser_spec) {

			case 0: //brightfield
				laser_arr[laser_spec].intensity = 4095*0.45* (data/100.); 	//limiting brightfield to 4.5V out of 5V max
				break;

			case 1: //laser 1
				//laser_arr[laser_spec].intensity = 4095*0.472* (data/100.); 	//the second factor is the tuning to the current output of the micrcontroller for the laser.
				//laser_arr[laser_spec].intensity = 4095*0.8* (data/100.); //camera testing line, comment out when done --> 80% of 12 bit output is about 4.08 volts
				//laser_arr[laser_spec].intensity = 4095*0.480* (data/100.); //hologram illumination finetuning //for 250 mA settings, outputs 0.470 of the max number of bits (in this case max. bits is 4095 + 1 bits) @ HiBW for the arroyo controller
				laser_arr[laser_spec].intensity = 4095*0.476* (data/100.); //hologram illumination finetuning //for 250 mA settings, outputs 0.470 of the max number of bits (in this case max. bits is 4095 + 1 bits) @ HiBW for the arroyo controller
				//laser_arr[laser_spec].intensity = 4095*(data/100.);	// 100% output testing code
				break;

			case 2: //laser 2
				laser_arr[laser_spec].intensity = 4095*0.325* (data/100.); //for 500 mA settings, outputs 0.325 of the max number of bits
				break; // optional

	 /*
	 case 3: //laser 3
      statement(s);
      break; //

	 case 4:
      statement(s); //laser 4
      break; //
	 */

   // you can have any number of case statements
   //default : // Optional default laser
   //laser_arr[laser_spec].intensity = 4095*0.070* (data/100.); //conservative estimate

		}

	}

}

void set_delay(uint8_t laser_spec, uint8_t data1_8, uint32_t data){laser_arr[laser_spec].delay = data;} //implementation of set-delay (CAN input should be given in milli-seconds)

void set_startimaging(uint8_t data0_8, uint8_t data1_8, uint32_t data){

//laser_seq = data0_8; // this code determines the amount of lasers to run
trig = data;

} //implementation of set_startimaging

void set_piezo(uint8_t data0_8, uint8_t data1_8, uint32_t data){
//piezo_start = data0_8 + 1;
piezo_start = data0_8;
piezo_steps = data1_8;
piezo_step_size = data;

} //implementation of set_piezo

void set_solopiezo(uint8_t data0_8, uint8_t data1_8, uint32_t data){

	if (data <= 200) { // making sure input for the piezo is not higher than 200 micrometers
	solo_piezo = data;
	piezo_position = data;
	}

} //implementation of set_solopiezo

void set_soloimaging(uint8_t data0_8, uint8_t data1_8, uint32_t data){

solo_imaging = data;

} //implementation of set_soloimaging


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
