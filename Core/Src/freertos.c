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
        struct floatIntM{
            float fl;
            int in;
        } floatInt;
    } CAN_TxRx_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern CAN_HandleTypeDef hcan1;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;
extern int repetitions;

CAN_RxHeaderTypeDef msgHeader;
CAN_TxRx_Data TxData;
CAN_TxRx_Data RxData;

float positionProportionalRatio = 1;
float positionIntegralRatio = 0;
float positionDifferentialRatio = 0;

float speedProportionalRatio = 1;
float speedIntegralRatio = 0;
float speedDifferentialRatio = 0;

const float dt = 1;
float currentTime = 0;
float regulatorForce = 0;

float currentPosition = 0;
float previousPosition = 0;
float desiredPosition = 300;

float currentPositionError = 0;
float previousPositionError = 0;
float positionErrorDifferential = 0;
float positionErrorIntegral = 0;
float positionRegulatorForce = 0;

float currentSpeed = 0;
float previousSpeed = 0;
float desiredSpeed = 300;

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
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for speedTask */
osThreadId_t speedTaskHandle;
const osThreadAttr_t speedTask_attributes = {
  .name = "speedTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for posPIDTask */
osThreadId_t posPIDTaskHandle;
const osThreadAttr_t posPIDTask_attributes = {
  .name = "posPIDTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for spdPIDTask */
osThreadId_t spdPIDTaskHandle;
const osThreadAttr_t spdPIDTask_attributes = {
  .name = "spdPIDTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void speedCalculationStart(void *argument);
void positionPIDStart(void *argument);
void speedPIDStart(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	CAN_TxHeaderTypeDef msgHeader;
	    uint8_t msgData[1] = {1};
	    msgHeader.StdId = CAN_STM1 + T_CleanPlot;
	    msgHeader.DLC = 0;
	    msgHeader.TransmitGlobalTime = DISABLE;
	    msgHeader.RTR = CAN_RTR_DATA;
	    msgHeader.IDE = CAN_ID_STD;

	    uint32_t mailBoxNum = 0;
	    uint32_t msgId = 0;
	    while(1)
	   	    {
				HAL_CAN_AddTxMessage(&hcan1, &msgHeader, msgData, &mailBoxNum);
	    		if(cleanPlotBool){
	    			cleanPlotBool = 0;
	    			break;
	    		}
				osDelay(1000);
	   	    }
	    osDelay(10);
  /* Infinite loop */
  for(;;)
  {
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0)
	  		  {
	  		    CAN_TxHeaderTypeDef msgHeader;
	  		    TxData.floats.fl1 = currentPosition;
	  		    TxData.floats.fl2 = currentTime;
	  		    uint8_t msgData[8];
	  		    msgHeader.StdId = CAN_STM1 + T_Position;
	  		    msgHeader.DLC = 8;
	  		    msgHeader.TransmitGlobalTime = DISABLE;
	  		    msgHeader.RTR = CAN_RTR_DATA;
	  		    msgHeader.IDE = CAN_ID_STD;

	  		    uint32_t mailBoxNum = 0;

	  		    for (uint8_t i = 0; i < 4; i++)
	  		    {
	  		      msgData[i] = TxData.uintData[3-i];
	  		      msgData[i+4] = TxData.uintData[7-i];
	  		    }

	  		    HAL_CAN_AddTxMessage(&hcan1, &msgHeader, msgData, &mailBoxNum);

	  		    TxData.floats.fl1 = currentSpeed;
	  		    msgHeader.StdId = CAN_STM1 + T_Speed;
	  		    for (uint8_t i = 0; i < 4; i++)
	  		    {
	  		      msgData[i] = TxData.uintData[3-i];
	  		    }
	  		    HAL_CAN_AddTxMessage(&hcan1, &msgHeader, msgData, &mailBoxNum);
	  		  }
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_speedCalculationStart */
/**
* @brief Function implementing the speedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_speedCalculationStart */
void speedCalculationStart(void *argument)
{
  /* USER CODE BEGIN speedCalculationStart */

  /* Infinite loop */
  for(;;)
  {
	previousPosition = currentPosition;
	currentPosition = (float)TIM1->CNT*0.36;
	currentSpeed = currentPosition - previousPosition;


    osDelay(5);
  }
  /* USER CODE END speedCalculationStart */
}

/* USER CODE BEGIN Header_positionPIDStart */
/**
* @brief Function implementing the posPIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_positionPIDStart */
void positionPIDStart(void *argument)
{
  /* USER CODE BEGIN positionPIDStart */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END positionPIDStart */
}

/* USER CODE BEGIN Header_speedPIDStart */
/**
* @brief Function implementing the spdPIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_speedPIDStart */
void speedPIDStart(void *argument)
{
  /* USER CODE BEGIN speedPIDStart */
  /* Infinite loop */
  for(;;)
  {
	currentTime += dt/1000;


    osDelay(1);
  }
  /* USER CODE END speedPIDStart */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t msgId = 0;
  uint8_t msgData[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msgHeader, msgData);

  msgId = msgHeader.StdId;
  switch(msgId){
  	  case CAN_RPi+Init:
		  for(int i = 0; i < 4; ++i)
		  RxData.uintData[i] = msgData[3-i];
		  desiredPosition = RxData.floats.fl1;

		  TIM1->CNT = 1000*32;
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
		  break;
	  case CAN_RPi+T_CleanPlot:
		  cleanPlotBool = 1;
		  break;
	  case CAN_STM1+R_PositionProportionalRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = msgData[3-i];
		  positionProportionalRatio = RxData.floats.fl1;
		  break;
	  case CAN_STM1+R_PositionIntegralRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = msgData[3-i];
		  positionIntegralRatio = RxData.floats.fl1;
		  break;
	  case CAN_STM1+R_PositionDifferentialRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = msgData[3-i];
		  positionDifferentialRatio = RxData.floats.fl1;
		  break;
	  case CAN_STM1+R_SpeedProportionalRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = msgData[3-i];
		  speedProportionalRatio = RxData.floats.fl1;
		  break;
	  case CAN_STM1+R_SpeedIntegralRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = msgData[3-i];
		  speedIntegralRatio = RxData.floats.fl1;
		  break;
	  case CAN_STM1+R_SpeedDifferentialRatio:
		  for(int i = 0; i < 4; ++i)
			 RxData.uintData[i] = msgData[3-i];
		  speedDifferentialRatio = RxData.floats.fl1;
		  break;
	  case CAN_RPi+MovingStart:

		  TIM1->CNT = 2147483648-1;

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
	  	  break;
  }

}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
