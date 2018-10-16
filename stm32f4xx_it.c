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
uint32_t  time_tick = 0, sample_time = 100000, time_temp = 0; //Default: 10ms sampling times
double 		time, Angle, SetAngle = 0, SendAngle = 0, A_Error, A_Error_dot, Pre_A, Fuzzy;
BOOLEAN 	GPS_RxFlag = false, ManAuto_Control = true, Stop = false;
/*--------------- Functions -------------------*/
void SampleTime(void)
{
	time     = sample_time * pow(10,-6);
}

void SendData(void)
{
	ind 		 = ToChar(M1.Velocity,U6_TxBuffer);
	U6_TxBuffer[ind] = (uint8_t)',';
	ind++;
	ind  		+= ToChar(M2.Velocity,&U6_TxBuffer[ind]);
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
	else
	{
		U6_TxBuffer[ind] = (uint8_t)'N';
		ind++;
	}
	U6_TxBuffer[ind] = 0x0D;
	ind++;
	U6_TxBuffer[ind] = 0x0A;
	U6_SendData(ind);
	ind = 0;
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
		M1.Enc 	 = TIM_GetCounter(TIM3);
		M2.Enc	 = TIM_GetCounter(TIM4);
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
			//A_Error = ToRadian(SetAngle - Angle);
			//A_Error_dot = ToRadian(-(Angle - Pre_A)) / time;
			//Pre_A = Angle;
			//Fuzzy = Defuzzification_Max_Min(K1 * A_Error, K2 * A_Error_dot);
			M1.TotalEnc = M1.Enc + (M1.OverFlow - 1)*65535;
			M2.TotalEnc = M2.Enc + (M2.OverFlow - 1)*65535;
			M1.Velocity = (((double)(M1.TotalEnc - M1.PreEnc)/ 39400) * 60) / time;
			M2.Velocity = (((double)(M2.TotalEnc - M2.PreEnc)/ 39400) * 60) / time;
			M1.OverFlow = 1;
			M2.OverFlow = 1;
			/*if(Fuzzy < 0)
			{
				PIDM1  = PID_ControlM1(VelM1,velocity,time);
				PIDM2  = PID_ControlM2(VelM2,velocity + 0*K3 * (-Fuzzy),time);
			}
			else
			{
				PIDM1 	 = PID_ControlM1(VelM1,velocity + 0*K3 * Fuzzy,time);
				PIDM2 	 = PID_ControlM2(VelM2,velocity,time);
			}*/
			M1.Duty_Cycle = PID_ControlM1(M1.Velocity,velocity,time);
			M1.Duty_Cycle	= PID_ControlM2(M2.Velocity,velocity,time);
			Robot_Backward(M1.Duty_Cycle,M2.Duty_Cycle);
			M1.PreEnc     = M1.Enc;
			M2.PreEnc			= M2.Enc;
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
	M1.OverFlow++;
}

void TIM4_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	M2.OverFlow++;
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
