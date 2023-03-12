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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CLCD_I2C.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
uint32_t a = 0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef struct{           // Gửi command, chu kì và ngưỡng đến task Uart_Threshold để thực thi
  uint32_t id;
  uint32_t command;
  uint32_t Frequency[3];
  uint32_t Threshold[3];
}myUartQueueData_t;

typedef struct{           // Gửi dữ liệu đến Task LCD và task Uart_Threshold
  uint32_t id;             // id = 1 gửi từ Task DHT
	uint32_t temp;           // id = 2 gửi từ Task HCSR 
  uint32_t humi;
  uint32_t distance;
}myQueueData_t;


//uint32_t temp = 0;
//uint32_t humi = 0;
uint32_t Threshold_tem = 0;
uint32_t Threshold_hum = 0;
uint32_t Lcd_temp = 0;
uint32_t Lcd_humi = 0;
uint32_t time_dht = 0;
uint32_t dist = 0;
uint32_t Threshold_dist = 0;
uint32_t Lcd_dist = 0;
uint32_t time_hcsr = 0;
uint32_t time_Threshold = 0;
uint32_t time_Threshold_now = 0;
uint32_t time_lcd = 0;
uint32_t time_button = 0; 
uint32_t time_uart = 0; 
uint32_t time_dht_read = 0;
uint32_t time_hcsr_read = 0;
uint32_t cnt_uart = 0;
uint32_t cnt_uart_rei = 0;
/* Tung cmt */

#define TransmitBuff_SIZE	70
#define ReceiveBuff_SIZE	10
#define MainBuff_SIZE		9
#define Queue_SIZE			45

typedef void (*Task_FunctionTypeDef) (void);
Task_FunctionTypeDef queue[Queue_SIZE];
Task_FunctionTypeDef UART_Transmit_Func = NULL;


/* Data of DHT11 Sensor */
float temp = 0;
float humi = 0;
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;
uint8_t Presence = 0;

/* Data line of LCD1602*/
char data_line_1[16];
char data_line_2[16];

/* Data of HC-SR04 Sensor */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
//uint8_t distance  = 0;
uint32_t distance  = 0;

/* Buffer Array */
uint8_t RxBuffer[ReceiveBuff_SIZE];
uint8_t TxBuffer[TransmitBuff_SIZE];
uint8_t MainBuffer[MainBuff_SIZE];

/* Period Time */
uint8_t DHT11_transmit_period;
uint8_t HCSR04_transmit_period;
uint8_t temp_transmit_period;
uint8_t humi_transmit_period;

/* Temperature Threshold */
uint8_t temp_threshold;

/* Humidity Threshold */
uint8_t humi_threshold;

/* Distance Threshold */
uint8_t dist_threshold;


/* Etc .. */
uint8_t cmd_index = 1;
uint8_t len = 0;

uint32_t start_tick = 0;
uint32_t stop_tick = 0;
uint32_t execute_tick = 0;

uint8_t humi_cnt = 0;
uint8_t temp_cnt = 0;
uint8_t dist_cnt = 0;

/* Enable Flag */
bool read_DHT11_flag = 0;
bool read_HCSR04_flag = 0;
bool transmit_DHT11_data_flag = 0;
bool transmit_temp_data_flag = 0;
bool transmit_humi_data_flag = 0;
bool transmit_dist_data_flag = 0;
bool transmit_DHT11_HCSR04_data_flag = 0;
bool transmit_temp_HCSR04_data_flag = 0;
bool transmit_humi_HCSR04_data_flag = 0;
bool lcd_display_flag = 0;
bool uart_enable_cmd_1 = 0;
bool uart_enable_cmd_2 = 0;

bool uart_enable_cmd_1_tmp = 0;
bool uart_enable_cmd_2_tmp = 0;

bool button_enable = 0;
bool isHumiTrans = 0;
bool isDistTrans = 0;

uint16_t Count = 0;
uint16_t cnt = 0;
uint16_t cnt_1 = 0;
uint8_t isr_cnt = 0;

int isr_tick_start;
int isr_tick_stop;
int isr_execute_tick;

int dht11_tick_old = 0;
int dht11_tick_new = 0;
int dht11_tick_period;

int lcd_tick_old = 0;
int lcd_tick_new = 0;
int lcd_tick_period;

int hcsr04_tick_old = 0;
int hcsr04_tick_new = 0;
int hcsr04_tick_period;

int old_tick = 0;
int new_tick = 0;

int task_idx = 0;
int old_task_idx;
int execute_time[46];
int idx = 4;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

osThreadId ButtonTaskHandle;               // task DHT
osThreadId HCSR04TaskHandle;            // task HCSR
osThreadId LCDTaskHandle;               // task LCD
osThreadId UARTTaskHandle;              // task xử lý dữ liệu nhận được từ UART
osThreadId DHTTaskHandle;            // task nút nhấn
osThreadId Uart_ThresholdTaskHandle;    // task xử lý ngưỡng và uart


osMessageQId myQueueDataHandle;                 // Gửi dữ liệu đến LCDTask
osMessageQId myQueueUart_ThresholdHandle;       // Gửi ngưỡng, chu kì đến Uart_ThresholdTask
osMessageQId myQueueUart_ThresholdDataHandle;   // Gửi dữ liệu đến Uart_ThresholdTask


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
CLCD_I2C_Name LCD1;
void ButtonTask(void const * argument);
void HCSR04Task(void const * argument);
void LCDTask(void const * argument);
void UARTTask(void const * argument);
void DHTTask(void const * argument);
void Uart_ThresholdTask(void const * argument);

// // Hiển thị LCD
//  void LCD_Display(void);

// Nhận UART
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);


// Delay microsecond
void delay_us(uint16_t time);

// Hàm Callback khi sóng siêu âm phản hồi v�? để đo khoảng cách
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

// Hàm ngắt Timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// Set chân cho DHT11
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

// Khởi tạo giao tiếp giữa DHT11 với Vi đi�?u khiển
void DHT11_Start(void);

// Kiểm tra phản hồi của DHT11
uint8_t DHT11_Check_Response(void);


uint8_t DHT11_Read(void);

// �?�?c cảm biến DHT11
void Read_DHT11_Sensor(void);

// �?�?c cảm biến HCSR04
void Read_HCSR04_Sensor(void);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Tung cmt */

// Set period, threshold value by button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	UNUSED(GPIO_Pin);
	if(GPIO_Pin == GPIO_PIN_0 ){
    BaseType_t xHigherPriority;
    xHigherPriority = xTaskResumeFromISR(ButtonTaskHandle);
    portEND_SWITCHING_ISR(xHigherPriority);
  }
}


void delay_us(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			distance = Difference * 0.034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
			//stop_tick = HAL_GetTick();
			//execute_tick = stop_tick - start_tick;

		}
	}
}

void Read_HCSR04_Sensor(void)
{
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART1){
		for(size_t i = 0; i<Size-1; i++) {
			if(RxBuffer[i] == 0x00){
				memcpy(MainBuffer, &RxBuffer[i], Size-i);
				//processRecCmd(Size-i);
				break;
			}
        
		}
        BaseType_t xHigherPriority;
        xHigherPriority = xTaskResumeFromISR(UARTTaskHandle);
        portEND_SWITCHING_ISR(xHigherPriority);
		
	}
}



void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_GPIO_Port, DHT11_Pin);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_GPIO_Port, DHT11_Pin, 0);   // pull the pin low
	delay_us(25000);   // wait for 25ms
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, 1);   // pull the pin high
	delay_us(25);   // wait for 25us
	Set_Pin_Input(DHT11_GPIO_Port, DHT11_Pin);    // set as input
}

uint8_t DHT11_Check_Response(void)
{
	uint8_t Response = 0;
	delay_us(40);
	if (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))
	{
		delay_us(80);
		if ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))) Response = 1;
		else Response = -1; // 255
	}
	else{
		return Response;
	}
	while ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read(void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)));   // wait for the pin to go high
		delay_us(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else{
			i|= (1<<(7-j));  // if the pin is high, write 1
		}
		while ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)));  // wait for the pin to go low
	}
	return i;
}

void Read_DHT11_Sensor(void){
	//start_tick = HAL_GetTick();
	DHT11_Start();
    Presence = DHT11_Check_Response();
    if(Presence){
    	Rh_byte1 = DHT11_Read();
    	Rh_byte2 = DHT11_Read();
    	Temp_byte1 = DHT11_Read();
    	Temp_byte2 = DHT11_Read();
    	SUM = DHT11_Read();
    	TEMP = Temp_byte1;
    	RH = Rh_byte1;
    	temp = (float)TEMP;
    	humi = (float)RH;
    }
    //stop_tick = HAL_GetTick();
    //execute_tick = stop_tick - start_tick;
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	CLCD_I2C_Init(&LCD1,&hi2c1,0x4e,16,2);
	CLCD_I2C_SetCursor(&LCD1, 0, 0);
	CLCD_I2C_WriteString(&LCD1,"Temp");
	CLCD_I2C_SetCursor(&LCD1, 6, 0);
	CLCD_I2C_WriteString(&LCD1,"Humi");
	CLCD_I2C_SetCursor(&LCD1, 12, 0);
	CLCD_I2C_WriteString(&LCD1,"Dist");

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, MainBuff_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    //lcd_init();
	  
		//HAL_Delay(200);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMailQDef(myQueueData, 14, myQueueData_t);
  myQueueDataHandle = osMailCreate(osMailQ(myQueueData), NULL);

  osMailQDef(myQueueUart, 14, myUartQueueData_t);
  myQueueUart_ThresholdDataHandle = osMailCreate(osMailQ(myQueueUart), NULL);

  osMailQDef(myQueueUart_Threshold, 14, myQueueData_t);
  myQueueUart_ThresholdHandle = osMailCreate(osMailQ(myQueueUart_Threshold), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  
  osThreadDef(ButtonTaskHandleH, ButtonTask, osPriorityNormal, 0, 128);
  ButtonTaskHandle  = osThreadCreate(osThread(ButtonTaskHandleH), NULL);

  osThreadDef(Uart_ThresholdTaskH, Uart_ThresholdTask, osPriorityNormal, 0, 128);
  Uart_ThresholdTaskHandle  = osThreadCreate(osThread(Uart_ThresholdTaskH), NULL);

  osThreadDef(HCSR04TaskH, HCSR04Task, osPriorityNormal, 0, 128);
  HCSR04TaskHandle = osThreadCreate(osThread(HCSR04TaskH), NULL);

  osThreadDef(LCDTaskH, LCDTask, osPriorityNormal, 0, 128);
  LCDTaskHandle  = osThreadCreate(osThread(LCDTaskH), NULL);

  osThreadDef(UARTTaskH, UARTTask, osPriorityNormal, 0, 128);
  UARTTaskHandle  = osThreadCreate(osThread(UARTTaskH), NULL);

  osThreadDef(DHTTaskH, DHTTask, osPriorityNormal, 0, 128);
  DHTTaskHandle  = osThreadCreate(osThread(DHTTaskH), NULL);



  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E_Pin|DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           TRIGGER_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : E_Pin DHT11_Pin */
  GPIO_InitStruct.Pin = E_Pin|DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RS_Pin */
  GPIO_InitStruct.Pin = RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(RS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// ButtonTask
void ButtonTask(void const * argument)
{
  uint32_t time_now = 0;
  uint32_t time_pre = 0;
  uint32_t cnt_button = 0;
 
  myUartQueueData_t *msg = osMailAlloc(myQueueUart_ThresholdHandle, osWaitForever);
  msg->id = 2; // gửi từ ButtonTask

  
  while(1)
  {
    vTaskSuspend(NULL);
    cnt_button++;
		/* Nhan nut lan 1, dat chu ky truyen UART cac cam bien la 1s */
		if(cnt_button == 1){
			msg->Frequency[0] = 1000;
      msg->Frequency[1] = 1000;
      msg->Frequency[2] = 1000;

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		}
		/* Nhan nut lan 2, dat chu ky truyen UART cac cam bien la 2s */
		else if(cnt_button == 2){
			msg->Frequency[0] = 2000;
      msg->Frequency[1] = 2000;
      msg->Frequency[2] = 2000;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		}
		/* Nhan nut lan 3, dat chu ky truyen UART cac cam bien la 3s */
		else if(cnt_button == 3){
			msg->Frequency[0] = 3000;
      msg->Frequency[1] = 3000;
      msg->Frequency[2] = 3000;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
			cnt_button = 0;
		}
    osMailPut(myQueueUart_ThresholdHandle, msg);
  }
}

// Task đ�?c và gửi khoảng cách
void HCSR04Task(void const * argument)
{
  uint32_t time_now = 0;
  uint32_t time_pre = 0;
  
  //int8_t Distance = 0;

  myQueueData_t *msg = osMailAlloc(myQueueDataHandle, osWaitForever);
  msg->id = 2;    // Gửi từ HCSR04Task tới LCDTask
  myQueueData_t *msg1 = osMailAlloc(myQueueUart_ThresholdDataHandle, osWaitForever);
  msg1->id = 2;   // Gửi từ HCSR04Task tới Uart_ThresholdTask
  
  while(1)
  {
  time_now = xTaskGetTickCount();
  time_hcsr = time_now - time_pre;
  time_pre = time_now;
  // �?ưa khoảng cách lưu vào biết Distance
  time_now = xTaskGetTickCount();
  Read_HCSR04_Sensor();
  time_hcsr_read = xTaskGetTickCount()-time_now;
  osDelay(100);

// dist = distance;
  msg->distance = distance;      // biến khoảng cách
  osMailPut(myQueueDataHandle, msg);

  msg1->distance = distance;      
  osMailPut(myQueueUart_ThresholdDataHandle, msg1);

   osDelay(2900);

   }
}



// Task hiển thị LCD
void LCDTask(void const * argument)
{
  uint32_t time_now = 0;
  uint32_t time_pre = 0;

  uint32_t temp_lcd = 0;
  uint32_t humi_lcd = 0;
  uint32_t dist_lcd = 0;

  char x[20];

  osEvent Recv_Task_data; // nhận dữ liệu từ Queue01
  while(1)
  {
  
  time_now = xTaskGetTickCount();
  time_lcd = time_now - time_pre;
  time_pre = time_now;

    //vTaskSuspend(NULL);
    //osDelay(1000);
    //uint8_t temp, humi, dist;
    // Nhận dữ liệu từ DHT và HCSR
    Recv_Task_data = osMailGet(myQueueDataHandle, 0);
    while (Recv_Task_data.status == osEventMail)
    {
    myQueueData_t *data = Recv_Task_data.value.p; // con tr�? đến cấu trúc lưu nhiệt độ, độ ẩm, khoảng cách
    if(data->id == 1) // Nhận nhiệt độ độ ẩm
    {
      temp_lcd = data->temp;
      humi_lcd = data->humi;

      // temp = data->temp;
      // humi = data->humi;
    }
    else              // Nhận Khoảng cách
    {
      dist_lcd = data->distance;

      //dist = data->distance;
    }
// nhận xong phải giải phóng
    osMailFree(myQueueDataHandle, data);
    Recv_Task_data = osMailGet(myQueueDataHandle, 0);
// Xử lý hiển thị
	CLCD_I2C_Clear(&LCD1);
	CLCD_I2C_SetCursor(&LCD1, 0, 0);
	CLCD_I2C_WriteString(&LCD1,"Temp");
	CLCD_I2C_SetCursor(&LCD1, 6, 0);
	CLCD_I2C_WriteString(&LCD1,"Humi");
	CLCD_I2C_SetCursor(&LCD1, 12, 0);
	CLCD_I2C_WriteString(&LCD1,"Dist");
	sprintf(x,"%d",temp_lcd); 
	CLCD_I2C_SetCursor(&LCD1, 0, 1);
	CLCD_I2C_WriteString(&LCD1,x);
  sprintf(x,"%d",humi_lcd); 
	CLCD_I2C_SetCursor(&LCD1, 6, 1);
	CLCD_I2C_WriteString(&LCD1,x);
  sprintf(x,"%d",dist_lcd);
	CLCD_I2C_SetCursor(&LCD1, 12, 1);
	CLCD_I2C_WriteString(&LCD1,x);
    }

    Lcd_temp = temp_lcd;
    Lcd_humi = humi_lcd;
    Lcd_dist = dist_lcd;

  osDelay(500);
  }
}
// // Task xử lý khi nhân được ngắt UART
// // Nhận dữ liệu và cập nhật ngưỡng vào Queue2 để các task khác xử lý
// // Thêm 1 task ngưỡng, dùng suspend và resume
void UARTTask(void const * argument)
{
  uint32_t time_now = 0;
  uint32_t time_pre = 0;
  myUartQueueData_t *msg = osMailAlloc(myQueueUart_ThresholdHandle, osWaitForever);
  msg->id = 1;
  while(1)
  {
    vTaskSuspend(NULL);


// // �?ưa các biến Command; Frequency; Threshold vào Queue2 để các task khác xử lý
if( MainBuffer[8] == 0x08){

// // �?ưa các biến Command; Frequency; Threshold vào Queue2 để các task khác xử lý
    cnt_uart = MainBuffer[2];
// gửi đến DHTTask, thay 1 bằng dữ liệu nhân được từ UART
     msg->command = MainBuffer[1];
     msg->Frequency[0] = MainBuffer[2];         // chu kì nhiệt độ DHTTask
     msg->Frequency[1] = MainBuffer[3];         // chu kì độ ẩm DHTTask
     msg->Frequency[2] = MainBuffer[4];         // chu kì HCSR04ask
     msg->Threshold[0] = MainBuffer[5];         // Ngưỡng nhiệt
     msg->Threshold[1] = MainBuffer[6];         // Ngưỡng độ ẩm
     msg->Threshold[2] = MainBuffer[7];         // Ngưỡng khoảng cách
     osMailPut(myQueueUart_ThresholdHandle, msg);
		

			}
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, MainBuff_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }
}
/* DHTTask */
void DHTTask(void const * argument)
{
    uint32_t time_now = 0;
  uint32_t time_pre = 0;
  myQueueData_t *msg = osMailAlloc(myQueueDataHandle, osWaitForever);
  msg->id = 1;    // Gửi từ HCSR04Task tới LCDTask
  myQueueData_t *msg1 = osMailAlloc(myQueueUart_ThresholdDataHandle, osWaitForever);
  msg1->id = 1;   // Gửi từ HCSR04Task tới Uart_ThresholdTask
  
  while(1)
  {
    // vTaskSuspend(NULL);
    // osDelay(1000);

  //uint8_t temp = 0;
  //uint8_t humi = 0;

    time_now = xTaskGetTickCount();
	  time_dht = time_now - time_pre;
	  time_pre = time_now;

  // time_now = xTaskGetTickCount();
	// time_dht = time_now - time_pre;
	//time_pre = time_now;
// �?ưa nhiệt độ, độ ẩm vào myQueueDataHandle để hiển thị LCD
// temp: nhiệt độ đo được bởi DHT
    
  time_now = xTaskGetTickCount();
  Read_DHT11_Sensor();
  time_dht_read = xTaskGetTickCount()-time_now;
    

     msg->humi = humi;      
		 msg->temp = temp;
     osMailPut(myQueueDataHandle, msg);
// gửi sang Task Threshold để xử lý
     msg1->humi = humi;     
     msg1->temp = temp; 		
     osMailPut(myQueueUart_ThresholdDataHandle, msg1);
 osDelay(4000-28);
  }

}

// /* Task xử lý ngưỡng */
void Uart_ThresholdTask(void const * argument)
{
  uint32_t Frequency_Temp = 2000;
  uint32_t Frequency_Humi = 4000;
  uint32_t Frequency_Dist = 6000;
	
  uint32_t Threshold_Temp = 100;
  uint32_t Threshold_Humi = 100;
  uint32_t Threshold_Dist = 100;

  uint32_t command = 0;
  uint32_t pre_command = 0;

  uint32_t temp1 = 100;
  uint32_t humi1 = 100;
  uint32_t dist1 = 100;

  uint32_t time_now = 0;
  uint32_t time_pre = 0;

  uint32_t time_temp = 0;
  uint32_t time_humi = 0;
  uint32_t time_dist = 0;
  
  osEvent Recv_Task_data_uart;  // nhận dữ liệu từ myQueueUart_ThresholdHandle
  osEvent Recv_Task_data; // nhận dữ liệu từ DHT và HCSR
  while(1)
  {
    time_now = xTaskGetTickCount();
	  time_Threshold = time_now - time_pre;
	  time_pre = time_now;


    // // Nhận dữ liệu từ myQueueUart_ThresholdHandle
    Recv_Task_data_uart = osMailGet(myQueueUart_ThresholdHandle, 0);
    while (Recv_Task_data_uart.status == osEventMail)
    {
        myUartQueueData_t *data = Recv_Task_data_uart.value.p; // con tr�? đến cấu trúc lưu nhiệt độ, độ ẩm
        if(data->id == 1) // Nhận từ UartTask
        {
        command = data->command;
        Frequency_Temp = data->Frequency[0];              // chu kì nhiệt độ DHT
        Frequency_Humi = data->Frequency[1];              // chu kì độ ẩm DHT
        Frequency_Dist = data->Frequency[2];              // chu kì khoảng cách
        Threshold_Temp = data->Threshold[0];              // Ngưỡng nhiệt
        Threshold_Humi = data->Threshold[1];              // Ngưỡng độ ẩm
        Threshold_Dist = data->Threshold[2];              // Ngưỡng khoảng cách
        }
        else              // nhận từ ButtonTask
        {
        Frequency_Temp = data->Frequency[0];              // chu kì nhiệt độ DHT
        Frequency_Humi = data->Frequency[1];              // chu kì độ ẩm DHT
        Frequency_Dist = data->Frequency[2];              // chu kì khoảng cách
        }
        // nhận xong phải giải phóng
        osMailFree(myQueueUart_ThresholdHandle, data);
        Recv_Task_data_uart = osMailGet(myQueueUart_ThresholdHandle, 0);

        time_temp = 0;
        time_humi = 0;
        time_dist = 0;
        Frequency_Temp = Frequency_Temp*1000;              // chu kì nhiệt độ DHT
        Frequency_Humi = Frequency_Humi*1000;              // chu kì độ ẩm DHT
        Frequency_Dist = Frequency_Dist*1000;              // chu kì khoảng cách
    }

    cnt_uart_rei = Frequency_Temp;
    
    if(command != pre_command)
    {
      pre_command = command;
      // Xử lý command
      switch (command) {
		case 0x01:
			/* some codes */

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
      break;
		
		case 0x02:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		  break;
    case 0x03:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		  break;
		default:
		  break;
	}


    }

    // Nhận dữ liệu từ myQueueUart_ThresholdHandle
    // Nhận dữ liệu từ DHT và HCSR
    Recv_Task_data = osMailGet(myQueueUart_ThresholdDataHandle, 0);
    while(Recv_Task_data.status == osEventMail)
    {
        myQueueData_t *data = Recv_Task_data.value.p; // con tr�? đến cấu trúc lưu nhiệt độ, độ ẩm, khoảng cách
        if(data->id == 1) // Nhận nhiệt độ độ ẩm
        {
        // Threshold_tem = data->temp;
        // Threshold_hum = data->humi;
        temp1 = data->temp;
        humi1 = data->humi;
        }
        else              // Nhận Khoảng cách
        {
        // Threshold_dist = data->distance;
        dist1 = data->distance;
        }
        // nhận xong phải giải phóng
        osMailFree(myQueueUart_ThresholdDataHandle, data);
        Recv_Task_data = osMailGet(myQueueUart_ThresholdDataHandle, 0);
    }

// Xử lý ngưỡng, Bật Led
   
    Threshold_tem = temp1;
    Threshold_hum = humi1;
    Threshold_dist = dist1;

if(((xTaskGetTickCount() - time_temp) > Frequency_Temp -6) && ((xTaskGetTickCount() - time_humi) > Frequency_Humi -6) && ((xTaskGetTickCount() - time_dist) > Frequency_Dist -6) ) // gửi sau chu kì Frequency_Dist
    {    
      time_temp = xTaskGetTickCount();
      time_humi = time_temp;
      time_dist = time_temp;
  memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "\nTemperature: %doC , Humidity: %d%% , Distance: %d , time: %d \n", (int) temp1, (int) humi1, (int) dist1,(int) (time_temp));
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, sizeof(TxBuffer));
    }
else if (((xTaskGetTickCount() - time_temp) > Frequency_Temp -6) && ((xTaskGetTickCount() - time_humi) > Frequency_Humi -6)  ) // gửi sau chu kì Frequency_Dist
    {    
      time_temp = xTaskGetTickCount();
      time_humi = time_temp;
  memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "\nTemperature: %doC , Humidity: %d%% , time: %d \n", (int) temp1, (int) humi1,(int) (time_temp));
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, sizeof(TxBuffer));
    }
else if (((xTaskGetTickCount() - time_temp) > Frequency_Temp -6) && ((xTaskGetTickCount() - time_dist) > Frequency_Dist -6) ) // gửi sau chu kì Frequency_Dist
    {    
      time_temp = xTaskGetTickCount();
      time_dist = time_temp;
  memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "\nTemperature: %doC , Distance: %d , time: %d \n", (int) temp1, (int) dist1,(int) (time_temp));
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, sizeof(TxBuffer));
    }
else if (((xTaskGetTickCount() - time_humi) > Frequency_Humi -6) && ((xTaskGetTickCount() - time_dist) > Frequency_Dist -6) ) // gửi sau chu kì Frequency_Dist
    {    
      time_humi = xTaskGetTickCount();
      time_dist = time_humi;
  memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "\nHumidity: %d%% , Distance: %d , time: %d \n",(int) humi1, (int) dist1,(int) (time_humi));
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, sizeof(TxBuffer));
    }
else if((xTaskGetTickCount() - time_temp) > Frequency_Temp - 6) // gửi sau chu kì Frequency_Dist
    {    
      time_temp = xTaskGetTickCount();
  memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "\nTemperature: %doC , time: %d \n", (int) temp1,(int) (time_temp));
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, sizeof(TxBuffer));
    }
else if((xTaskGetTickCount() - time_humi) > Frequency_Humi - 6 ) // gửi sau chu kì Frequency_Dist
    {    
      time_humi = xTaskGetTickCount();
  memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "\nHumidity: %d%% , time: %d \n", (int) humi1,(int) (time_humi));
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, sizeof(TxBuffer));
    }
else if((xTaskGetTickCount() - time_dist) > Frequency_Dist - 6) // gửi sau chu kì Frequency_Dist
    {    
      time_dist = xTaskGetTickCount();
  memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "\nDistance: %d , time: %d \n", (int) dist1,(int) (time_dist));
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, sizeof(TxBuffer));
    }
    osDelay(100);
}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		vTaskSuspend(NULL);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
