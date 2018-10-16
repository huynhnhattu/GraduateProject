/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
	
}
/** @Variable **/
uint8_t 	temp1, temp2, temp3, ind = 0;
char 			U2_Message[50][20], U6_Message[50][20];
double 		lat,lon,cor[2];
uint32_t  time_tick = 0, sample_time = 10000, time_temp = 0; //Default: 10ms sampling times
uint16_t  EncM1= 0, EncM2 = 0, PreEncM1 = 0, PreEncM2 = 0;
uint32_t	DisEncM1, DisEncM2;
double 		VelM1, VelM2, PIDM1, PIDM2, rollM1, rollM2, time, Angle, SetAngle = 0, SendAngle = 0, A_Error, A_Error_dot, Pre_A, Fuzzy;
BOOLEAN 	GPS_RxFlag = false, ManAuto_Control = true, Stop = false;
uint8_t 	multiM1 = 0,multiM2 = 0;
/*--------------- Functions -------------------*/
void SampleTime(void)
{
	time     = sample_time * pow(10,-4);
}
void VelocityControl(void)
{
	DisEncM1 = EncM1 + (multiM1-1)*65535;
	DisEncM2 = EncM2 + (multiM2-1)*65535;
	rollM1   = (double)(DisEncM1 - PreEncM1)/ 39400;
  rollM2   = (double)(DisEncM2 - PreEncM2)/ 39400;
	multiM1  = 1;
	multiM2  = 1;
	VelM1 	 = (rollM1 * 60) / time;
	VelM2 	 = (rollM2 * 60) / time;
	PIDM1 	 = PID_ControlM1(VelM1,velocity,time);
  PIDM2 	 = PID_ControlM2(VelM2,velocity,time);
	Robot_Forward(PIDM1,PIDM2);
	PreEncM1 = EncM1;
	PreEncM2 = EncM2;
}

void SendData(void)
{
	ind 		 = ToChar(VelM1,U6_TxBuffer);
	U6_TxBuffer[ind] = (uint8_t)',';
	ind++;
	ind  		+= ToChar(VelM2,&U6_TxBuffer[ind]);
	U6_TxBuffer[ind] = (uint8_t)',';
	ind++;
	ind   	+= ToChar(Angle,&U6_TxBuffer[ind]);
	U6_TxBuffer[ind] = (uint8_t)',';
	ind++;
	if(GPS_RxFlag)
	{
		GPS_RxFlag = false;
		U6_TxBuffer[ind] = (uint8_t)',';
		ind++;
		U6_TxBuffer[ind] = (uint8_t)'Y';
		ind++;
		ind   		+= ToChar(cor[0],&U6_TxBuffer[ind]);
		U6_TxBuffer[ind] = (uint8_t)',';
		ind++;
		ind				+= ToChar(cor[1],&U6_TxBuffer[ind]);
	}
	U6_TxBuffer[ind] = (uint8_t)'N';
	ind++;
	U6_TxBuffer[ind] = 0x0D;
	ind++;
	U6_TxBuffer[ind] = 0x0A;
	U6_SendData(ind);
}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	if(time_tick < time_temp)
	{
		time_tick++;
	}
	else
	{
		time_tick = 0;
		time_temp = sample_time;
		SampleTime();
		EncM1 	 = TIM_GetCounter(TIM3);
		EncM2		 = TIM_GetCounter(TIM4);
		Angle 	 = GetAngle();
		if(ManAuto_Control)   //Manual control
		{
			SetAngle = Angle + SendAngle;
		  if(SetAngle > 360)
			{
				SetAngle = SetAngle - 360;
			}
			else if(SetAngle < 0)
			{
				SetAngle = SetAngle + 360;
			}
			A_Error = ToRadian(SetAngle - Angle);
			A_Error_dot = ToRadian(-(Angle - Pre_A)) / time;
			Pre_A = Angle;
			Fuzzy = Defuzzification_Max_Min(K1 * A_Error, K2 * A_Error_dot);
			DisEncM1 = EncM1 + (multiM1-1)*65535;
			DisEncM2 = EncM2 + (multiM2-1)*65535;
			rollM1   = (double)(DisEncM1 - PreEncM1)/ 39400;
			rollM2   = (double)(DisEncM2 - PreEncM2)/ 39400;
			multiM1  = 1;
			multiM2  = 1;
			VelM1 	 = (rollM1 * 60) / time;
			VelM2 	 = (rollM2 * 60) / time;
			if(Fuzzy < 0)
			{
				PIDM1  = PID_ControlM1(VelM1,velocity,time);
				PIDM2  = PID_ControlM2(VelM2,velocity + velocity*(-Fuzzy),time);
			}
			else
			{
				PIDM1 	 = PID_ControlM1(VelM1,velocity + velocity*Fuzzy,time);
				PIDM2 	 = PID_ControlM2(VelM2,velocity,time);
			}
			Robot_Forward(PIDM1,PIDM2);
			PreEncM1 = EncM1;
			PreEncM2 = EncM2;
			SendData();
		}
		else // Stanley control
		{
			
		}
	}
}
/** @brief : USART1 interrupt Rx handler **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET)
	{
		USART_ClearFlag(USART1,USART_FLAG_IDLE);
		temp1 = USART_ReceiveData(USART1);
		DMA_Cmd(DMA2_Stream5,DISABLE);
	}
}
void DMA2_Stream5_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
	DMA_Cmd(DMA2_Stream5,ENABLE);
}
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** @brief : USART2 interrupt Rx Handler **/
/** @GPS @interrupt **/
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_IDLE))
	{
		USART_ClearFlag(USART2,USART_FLAG_IDLE);
		temp2 = USART_ReceiveData(USART2);
		DMA_Cmd(DMA1_Stream5,DISABLE);
	}
}
void DMA1_Stream5_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5);
	GetMessageInfo((char*)U2_RxBuffer,U2_Message);
	if(IsCorrectHeader(&U2_Message[0][0]))
	{
		if(IsValidData(&U2_Message[5][0]))
		{
			GPS_RxFlag = true;
			lat = LLToDegree(GetValueFromString(&U2_Message[1][0]));
			lon = LLToDegree(GetValueFromString(&U2_Message[3][0]));
			LatLonToUTM(lat,lon,cor);
		}
	}
	DMA_Cmd(DMA1_Stream5,ENABLE);
}
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** @brief : USART6 interrupt Rx Handler **/
/** PC USART **/
void USART6_IRQHandler(void)
{
	if(USART_GetITStatus(USART6,USART_IT_IDLE))
	{
		USART_ClearFlag(USART6,USART_FLAG_IDLE);
		temp3 = USART_ReceiveData(USART6);
		DMA_Cmd(DMA2_Stream2,DISABLE);
	}
}
void DMA2_Stream2_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
	GetMessageInfo((char*)U6_RxBuffer,U6_Message);
	control = (uint8_t)U6_Message[0][0];
	DMA_Cmd(DMA2_Stream2,ENABLE);
}
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** @brief : Encoder interrupt **/
void TIM3_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	multiM1++;
}

void TIM4_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	multiM2++;
}
/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
