/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union{
        unsigned char uintData[8];
        struct floatsM{
            float fl1;
            float fl2;
        } floats;
        struct intsM{
        	int int1;
        	int int2;
        } ints;
    } CAN_TxRx_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void driverInit();
void transmitHardCanData(uint32_t CAN_ID, uint32_t DLC, uint8_t data[]);
void transmitFloatCanData(float data1, uint32_t CAN_ID);
void transmitIntCanData(int data1, uint32_t CAN_ID);
void transmitFloatFloatCanData(float data1, float data2, uint32_t CAN_ID);
void transmitIntIntCanData(int data1, int data2, uint32_t CAN_ID);
void transmitFloatIntCanData(float data1, int data2, uint32_t CAN_ID);
void transmitCanCommand(uint32_t COMMAND_CAN_ID);
double getPosition();
double getCurrent();
void setVoltage(float voltage);
uint16_t voltageToDAC(float voltage);

void transmitAllRatio();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int32_t encoderScaler = 0;
int32_t encoderValue = 0;

extern CAN_HandleTypeDef hcan1;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;
extern int repetitions;

CAN_TxRx_Data RxData;
CAN_RxHeaderTypeDef receiveMsgHeader;
CAN_TxRx_Data TxData;
CAN_TxHeaderTypeDef transmitMsgHeader;

uint8_t transmitMsgData[8];
uint32_t transmitMailBoxNum = 0;

uint8_t receiveMsgData[8];
uint32_t receiveMailBoxNum = 0;

uint32_t msgId = 0;

float positionProportionalRatio = 1;
float positionIntegralRatio = 0;
float positionDifferentialRatio = 0;

float speedProportionalRatio = 0.5;
float speedIntegralRatio = 0;
float speedDifferentialRatio = 0;

float currentTime = 0;
float prevCurrentTime = 0;
float regulatorForce = 0;

float currentPosition = 0;
float previousPosition = 0;
float desiredPosition = 2000;

float currentPositionError = 0;
float previousPositionError = 0;
float positionErrorDifferential = 0;
float positionErrorIntegral = 0;
float positionRegulatorForce = 0;

float currentSpeed = 0;
float previousSpeed = 0;
float desiredSpeed = 0;

float currentSpeedError = 0;
float previousSpeedError = 0;
float speedErrorDifferential = 0;
float speedErrorIntegral = 0;
float speedRegulatorForce = 0;

uint8_t cleanPlotBool = 0;
uint16_t voltage;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for speedTask */
osThreadId_t speedTaskHandle;
const osThreadAttr_t speedTask_attributes = {
  .name = "speedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for posPIDTask */
osThreadId_t posPIDTaskHandle;
const osThreadAttr_t posPIDTask_attributes = {
  .name = "posPIDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for spdPIDTask */
osThreadId_t spdPIDTaskHandle;
const osThreadAttr_t spdPIDTask_attributes = {
  .name = "spdPIDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void speedCalculationStart(void *argument);
void positionPIDStart(void *argument);
void speedPIDStart(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of speedTask */
  speedTaskHandle = osThreadNew(speedCalculationStart, NULL, &speedTask_attributes);

  /* creation of posPIDTask */
  posPIDTaskHandle = osThreadNew(positionPIDStart, NULL, &posPIDTask_attributes);

  /* creation of spdPIDTask */
  spdPIDTaskHandle = osThreadNew(speedPIDStart, NULL, &spdPIDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	float dt = 100;

	transmitMsgHeader.TransmitGlobalTime = DISABLE;
	transmitMsgHeader.RTR = CAN_RTR_DATA;
	transmitMsgHeader.IDE = CAN_ID_STD;

	while(!cleanPlotBool){
		osDelay(10);
		transmitCanCommand(CAN_RPi + T_CleanPlot);
	}
	transmitAllRatio();
	cleanPlotBool = 0;


	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, ENABLE);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, ENABLE);
	osDelay(100);

  /* Infinite loop */
  for(;;)
  {
	transmitFloatFloatCanData(currentPosition, currentTime, CAN_STM1 + T_Position);
	transmitFloatFloatCanData(currentSpeed, currentTime, CAN_STM1 + T_Speed);

    osDelay(dt);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_speedCalculationStart */
/* USER CODE END Header_speedCalculationStart */
void speedCalculationStart(void *argument)
{
  /* USER CODE BEGIN speedCalculationStart */
  float dt = 10;
  /* Infinite loop */
  for(;;)
  {
	currentTime += dt/1000;
	previousPosition = currentPosition;
	currentPosition = getPosition();
	currentSpeed = (currentPosition - previousPosition)/(dt/1000);

    osDelay(dt);
  }
  /* USER CODE END speedCalculationStart */
}

/* USER CODE BEGIN Header_positionPIDStart */
/* USER CODE END Header_positionPIDStart */
void positionPIDStart(void *argument)
{
  /* USER CODE BEGIN positionPIDStart */
  float dt = 10;
  /* Infinite loop */
  for(;;)
  {
	previousPositionError = currentPositionError;
	currentPositionError = desiredPosition - currentPosition;
	positionErrorDifferential = (currentPositionError - previousPositionError)/(dt/1000);
	positionErrorIntegral += currentPositionError*(dt/1000);
	positionRegulatorForce = positionProportionalRatio*(currentPositionError +
														positionDifferentialRatio*positionErrorDifferential+
														positionIntegralRatio*positionErrorIntegral);
    osDelay(dt);
  }
  /* USER CODE END positionPIDStart */
}

/* USER CODE BEGIN Header_speedPIDStart */
/* USER CODE END Header_speedPIDStart */
void speedPIDStart(void *argument)
{
  /* USER CODE BEGIN speedPIDStart */
  float dt = 20;
  /* Infinite loop */
  for(;;)
  {
	desiredSpeed = positionRegulatorForce;
	previousSpeedError = currentSpeedError;
	currentSpeedError = desiredSpeed - currentSpeed;
	speedErrorDifferential = (currentSpeedError - previousSpeedError)/(dt/1000);
	speedErrorIntegral += currentSpeedError*(dt/1000);
	speedRegulatorForce = speedProportionalRatio*(currentSpeedError +
												  speedDifferentialRatio*speedErrorDifferential+
												  speedIntegralRatio*speedErrorIntegral);
	setVoltage(speedRegulatorForce);

    osDelay(dt);
  }
  /* USER CODE END speedPIDStart */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void transmitFloatCanData(float data1, uint32_t CAN_ID){

	CAN_TxRx_Data tData;
	tData.floats.fl1 = data1;
	for (uint8_t i = 0; i < 4; i++)
	{
		transmitMsgData[i] = tData.uintData[i];
	}
	transmitHardCanData(CAN_ID, 4, transmitMsgData);
}

void transmitIntCanData(int data1, uint32_t CAN_ID){

	CAN_TxRx_Data tData;
	tData.ints.int1 = data1;
	for (uint8_t i = 0; i < 4; i++)
	{
		transmitMsgData[i] = tData.uintData[i];
	}
	transmitHardCanData(CAN_ID, 4, transmitMsgData);

}

void transmitFloatFloatCanData(float data1, float data2, uint32_t CAN_ID){

	CAN_TxRx_Data tData;
	tData.floats.fl1 = data1;
	tData.floats.fl2 = data2;
	for (uint8_t i = 0; i < 4; i++)
	{
		transmitMsgData[i] = tData.uintData[i];
		transmitMsgData[i+4] = tData.uintData[i+4];
	}
	transmitHardCanData(CAN_ID, 8, transmitMsgData);
}

void transmitIntIntCanData(int data1, int data2, uint32_t CAN_ID){

	CAN_TxRx_Data tData;
	tData.ints.int1 = data1;
	tData.ints.int2 = data2;
	for (uint8_t i = 0; i < 4; i++)
	{
		transmitMsgData[i] = tData.uintData[i];
		transmitMsgData[i+4] = tData.uintData[i+4];
	}
	transmitHardCanData(CAN_ID, 8, transmitMsgData);
}

void transmitFloatIntCanData(float data1, int data2, uint32_t CAN_ID){

	CAN_TxRx_Data tData;
	tData.floats.fl1 = data1;
	tData.ints.int2 = data2;
	for (uint8_t i = 0; i < 4; i++)
	{
		transmitMsgData[i] = tData.uintData[i];
		transmitMsgData[i+4] = TxData.uintData[i+4];
	}
	transmitHardCanData(CAN_ID, 8, transmitMsgData);
}

void transmitCanCommand(uint32_t COMMAND_CAN_ID){
	transmitHardCanData(COMMAND_CAN_ID, 0, transmitMsgData);
}

void transmitHardCanData(uint32_t CAN_ID, uint32_t DLC, uint8_t data[]){
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
	{
		transmitMsgHeader.StdId = CAN_ID;
		transmitMsgHeader.DLC = DLC;
		HAL_CAN_AddTxMessage(&hcan1, &transmitMsgHeader, data, &transmitMailBoxNum);
	}
}

void setVoltage(float voltage){
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, voltageToDAC(voltage));
}

double getPosition(){
	encoderValue = (uint32_t)(encoderScaler*(uint16_t)(-1)) + (uint32_t)TIM1->CNT;
	return 0.36*encoderValue;
}

void transmitAllRatio(){
	int interval = 50;
	osDelay(10);
	transmitFloatCanData(positionProportionalRatio, CAN_STM1 + T_PositionProportionalRatio);
	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		osDelay(interval);
	transmitFloatCanData(positionIntegralRatio, CAN_STM1 + T_PositionIntegralRatio);
	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		osDelay(interval);
	transmitFloatCanData(positionDifferentialRatio, 	CAN_STM1 + T_PositionDifferentialRatio);
	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		osDelay(interval);
	transmitFloatCanData(speedProportionalRatio, 	CAN_STM1 + T_SpeedProportionalRatio);
	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		osDelay(interval);
	transmitFloatCanData(speedIntegralRatio, 	CAN_STM1 + T_SpeedIntegralRatio);
	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		osDelay(interval);
	transmitFloatCanData(speedDifferentialRatio, 		CAN_STM1 + T_SpeedDifferentialRatio);
}

void driverInit(){

		transmitAllRatio();

		TIM1->CNT = 0;
		encoderValue = 0;
		encoderScaler = 0;

		regulatorForce = 0;

		currentPosition = 0;
		previousPosition = 0;

		currentPositionError = 0;
		previousPositionError = 0;
		positionErrorDifferential = 0;
		positionErrorIntegral = 0;
		positionRegulatorForce = 0;

		currentSpeed = 0;
		previousSpeed = 0;
		desiredSpeed = 0;

		currentSpeedError = 0;
		previousSpeedError = 0;
		speedErrorDifferential = 0;
		speedErrorIntegral = 0;
		speedRegulatorForce = 0;

		while(cleanPlotBool)
		{
			transmitCanCommand(CAN_STM1 + T_CleanPlot);
			osDelay(500);
		}
		cleanPlotBool = 0;

}

uint16_t voltageToDAC(float voltage){
	if(voltage > 10)
		return 0xFFF;
	else if(voltage < -10)
		return 0;
	else
		return 0xFFF/2 + voltage*204.8;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &receiveMsgHeader, receiveMsgData);
  msgId = receiveMsgHeader.StdId;

  switch(msgId){
  	  case CAN_All + Heartbeat:
	  	  transmitCanCommand(CAN_STM1 + HeartbeatRespond);
	  	  break;
  	  case CAN_STM1 + R_Position:
		  for(int i = 0; i < 4; ++i)
			RxData.uintData[i] = receiveMsgData[3-i];
		  desiredPosition = RxData.floats.fl1;
	  	  driverInit();
		  break;
	  case CAN_RPi+MovingStart:
	  	  driverInit();
	  	  break;
	  case CAN_All+R_CleanPlot:
		  cleanPlotBool = 1;
		  transmitAllRatio();
		  break;
	  case CAN_All+ToggleLockKey:
	      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  	  break;
	  case CAN_All+ToggleStopDriver:
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
	  	  break;
	  case CAN_STM1+R_PositionProportionalRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = receiveMsgData[3-i];
		  positionProportionalRatio = RxData.floats.fl1;
		  transmitFloatCanData(positionProportionalRatio, CAN_STM1+T_PositionProportionalRatio);
		  break;
	  case CAN_STM1+R_PositionIntegralRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = receiveMsgData[3-i];
		  positionIntegralRatio = RxData.floats.fl1;
		  transmitFloatCanData(positionIntegralRatio, CAN_STM1+T_PositionIntegralRatio);
		  break;
	  case CAN_STM1+R_PositionDifferentialRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = receiveMsgData[3-i];
		  positionDifferentialRatio = RxData.floats.fl1;
		  transmitFloatCanData(positionDifferentialRatio, CAN_STM1+T_PositionDifferentialRatio);
		  break;
	  case CAN_STM1+R_SpeedProportionalRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = receiveMsgData[3-i];
		  speedProportionalRatio = RxData.floats.fl1;
		  transmitFloatCanData(speedProportionalRatio, CAN_STM1+T_SpeedProportionalRatio);
		  break;
	  case CAN_STM1+R_SpeedIntegralRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = receiveMsgData[3-i];
		  speedIntegralRatio = RxData.floats.fl1;
		  transmitFloatCanData(speedIntegralRatio, CAN_STM1+T_SpeedIntegralRatio);
		  break;
	  case CAN_STM1+R_SpeedDifferentialRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = receiveMsgData[3-i];
		  speedDifferentialRatio = RxData.floats.fl1;
		  transmitFloatCanData(speedDifferentialRatio, CAN_STM1+T_SpeedDifferentialRatio);
		  break;
//	  case CAN_STM1+R_AllDataQuery:
//	  	  transmitCanCommand(CAN_RPi+RegulatorRatioReceiveAcknowledge);
  }

}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
