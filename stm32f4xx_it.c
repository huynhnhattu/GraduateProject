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
#include <math.h>
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
/** @Variable **/\
/*C,0.3,0,0.8,0.5,0.001,0.8,0.3,0.001*/
uint8_t 				temp3, temp2;
/*--------------- Functions -------------------*/
void GetVehicleVelocity(void)
{
			if(GPIO_ReadOutputDataBit(Dir_GPIOx,Dir_GPIO_Pin_M1) == 0) // Backward up counting
			{
				M1.Current_Vel = (((double)((M1.Enc + (M1.OverFlow - 1) * 65535) - M1.PreEnc)/ 39400) * 60) / Timer.T;
			}
			else // Front down 
			{
				M1.Current_Vel = (((double)(((65535 - M1.Enc) + (M1.OverFlow - 1) * 65535) - (65535 - M1.PreEnc))/ 39400) * 60) / Timer.T;
			}
			if(M1.Current_Vel < 0)
				M1.Current_Vel = 0;
			if(GPIO_ReadOutputDataBit(Dir_GPIOx,Dir_GPIO_Pin_M2) == 0)
			{
				M2.Current_Vel = (((double)((M2.Enc + (M2.OverFlow - 1) * 65535) - M2.PreEnc)/ 39400) * 60) / Timer.T;
			}
			else 
			{
				M2.Current_Vel = (((double)(((65535 - M2.Enc) + (M2.OverFlow - 1) * 65535) - (65535 - M2.PreEnc))/ 39400) * 60) / Timer.T;
			}
			if(M2.Current_Vel < 0)
				M2.Current_Vel = 0;
}

/**
  * @brief  This function Save data to internal flash memory
  * @param  None
  * @retval None
  */
void SaveDataToInternalFlash(int key)
{
	Flash.Length = 0;
	switch(key)
	{
		/* ------ PID parameters ------- */
		case 1:
			PID_SavePIDParaToFlash(&Flash,&M1,&M2);
			break;
		
		/* ------- GPS parameter -------*/
		case 2:
			GPS_SavePathCoordinateToFlash(&GPS_NEO,&Flash);
			break;
	}
}
/**
  * @brief  This function reset motor
  * @param  None
  * @retval None
  */
void Reset_Motor()
{
	M1.Set_Vel 				= 0;
	M2.Set_Vel 				= 0;
	Robot_Forward(0,0);
	Veh.Manual_Velocity   = 0;
	Veh.Manual_Angle 			= 0;
}


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/* M1 - Right motor, M2 - Left motor */
void SysTick_Handler(void)
{
	if(!Status_CheckStatus(&VehStt.Veh_Send_Data))
	{
		if(Timer.Time_Send_Count < Timer.Send_Time)
		{
			Timer.Time_Send_Count++;
		}
		else
		{
			Timer.Time_Send_Count = 0;
			Status_UpdateStatus(&VehStt.Veh_Send_Data,Check_OK);
		}
	}
	
	if(!Status_CheckStatus(&VehStt.Veh_Sample_Time))
	{
		if(Timer.Time_Sample_Count < Timer.Sample_Time)
		{
			Timer.Time_Sample_Count++;
		}
		else
		{
			Timer.Time_Sample_Count = 0;
			Status_UpdateStatus(&VehStt.Veh_Sample_Time,Check_OK);
		}
	}
}

/** brief : USART1 interrupt Rx handler **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
void DMA2_Stream5_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
	IMU_UpdateAngle(&Mag,IMU_GetValue(U1_RxBuffer,3));
	if(!Status_CheckStatus(&VehStt.IMU_FirstSetAngle))
	{
		Status_UpdateStatus(&VehStt.IMU_FirstSetAngle,Check_OK);
		IMU_UpdateSetAngle(&Mag,0);
	}
	DMA_Cmd(DMA2_Stream5,ENABLE);
}

/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** brief : USART2 interrupt Rx Handler **/
/** GPS interrupt **/
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
	if(GPS_GetMessage("GNGLL",(char*)U2_RxBuffer,U2.Message))
	{
		if(IsValidData(U2.Message[6][0]))
		{
			GPS_GetLatFromString(&GPS_NEO,&U2.Message[1][0]);
			GPS_GetLonFromString(&GPS_NEO,&U2.Message[3][0]);
			GPS_LatLonToUTM(&GPS_NEO);
			Status_UpdateStatus(&VehStt.GPS_Coordinate_Reveived,Check_OK);
		}
	}
	DMA_Cmd(DMA1_Stream5,ENABLE);
}

/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** brief : USART6 interrupt Rx Handler **/
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

/*------------------ Command sequence -----------------*/
/*-----------------------------------------------------*/
/*-----------------------------------------------------*/
/*
*********     1. $VEHCF,Max_Velocity,M1.Kp,M1.Ki,M1.Kd,M2.Kp,M2.Ki,M2.Kd,CC
*********     2. $TSAMP,Sample_Time,CC     (-- in ms --)
*********     3. $TSEND,Send_Time,CC       (-- in ms --)
*********     4. $IMUCF,IMU commands sequences,CC      (-- Command sequences in documents --)
*********     5. $SFRST,CC     
*********     6. $MACON,Manual commands sequences      (-- 1: Manual mode enable, 0: Stop motor
																													 and disable manual mode
																													 W/S/A/D: Vehicle control Key --)
*********     7. $AUCON,Auto commands sequences        (-- 1: Auto mode enable, 0: Disable mode --)
*********     8. $VPLAN,NBOfWP,lat,lon,....									 (-- Numbers of lat lon --)
*/
void DMA2_Stream2_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
	DMA_Cmd(DMA2_Stream6,DISABLE);
	if(Veh_GetCommandMessage((char*)U6_RxBuffer,U6.Message))
	{
		switch((int)GetNbOfReceiveHeader(&U6.Message[0][0]))
			{
				/*----------------- Vehicle Config --------------------------*/
				case None:
					U6_SendData(FeedBack(U6_TxBuffer,'0'));
					break;
				
				case Vehicle_Config:
					Veh_UpdateMaxVelocity(&Veh,ToRPM(GetValueFromString(&U6.Message[1][0])));
					PID_ParametersUpdate(&M1,GetValueFromString(&U6.Message[2][0]),GetValueFromString(&U6.Message[3][0]),GetValueFromString(&U6.Message[4][0]));
					PID_ParametersUpdate(&M2,GetValueFromString(&U6.Message[5][0]),GetValueFromString(&U6.Message[6][0]),GetValueFromString(&U6.Message[7][0]));
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					break;
				
				/*---------------- Sample time config (us)------------------*/	
				case Sample_Time: 
					Time_SampleTimeUpdate(&Timer,(uint32_t)GetValueFromString(&U6.Message[1][0]));
					Time_GetSampleTime(&Timer);
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					break;
				
				/*---------------- Send data time config (ms)---------------*/
				case Send_Time: 
					Time_SendTimeUpdate(&Timer,(uint32_t)GetValueFromString(&U6.Message[1][0]));
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					break;
				
				/*---------------- IMU command and read data config --------*/
				case IMU_Config: 
					U1_TxBuffer[0] = (uint8_t)'$';
					U1_SendData((uint16_t)IMU_GetCommandMessage(&U6.Message[1][0],&U1_TxBuffer[1]));
					if((U6.Message[1][0] == 'M') && (U6.Message[1][1] == 'A') && (U6.Message[1][2] == 'G') && (U6.Message[1][3] == '2') && (U6.Message[1][4] == 'D'))
					{
						if(Veh.Mode != Calib_Mode)
						{
							Reset_Motor();
							Veh.Mode = Calib_Mode;
							PID_UpdateSetVel(&M1,30);
							PID_UpdateSetVel(&M2,30);
						}
					}
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					break;
				
				/*---------------- Software reset --------------------------*/
				case Soft_Reset: 
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					NVIC_SystemReset();
					break;
				
				/*---------------- Manual mode config ----------------------*/
				case Manual_Config: 
					if(U6.Message[1][0] == '1')
					{
						if(Veh.Mode != Manual_Mode)
						{
							Reset_Motor();
							Veh.Mode = Manual_Mode;
						}
					}
					else if(U6.Message[1][0] == '0')
						Reset_Motor();
					else
						Veh.ManualCtrlKey = U6.Message[1][0];
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					break;
				
				/*---------------- Auto mode config ------------------------*/
				case Auto_Config: 
					if(U6.Message[1][0] == '1')
					{
						if(Veh.Mode != Auto_Mode)
						{
							Reset_Motor();
							Veh.Mode = Auto_Mode;
						}
					}
					else if(U6.Message[1][0] == '0')
						Reset_Motor();
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					break;
				
				/*---------------- Receive path coordinate -----------------*/
				case Path_Plan: 
					GPS_UpdatePathCoordinate(&GPS_NEO,U6.RxTempBuffer);
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					break;
					
				/*--------------- Save data in internal flash memory --------*/
				case Flash_Save:
					SaveDataToInternalFlash(1);
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					break;
				
				/*--------------- Save data in internal flash memory --------*/
				/* Format: $KCTRL,1,max velocity 
									 $KCTRL,0
									 $KCTRL,W,S,A,D,level
				*/
				case KeyBoard_Control:
					if(U6.Message[1][0] == '1')
					{
						if(Veh.Mode != KeyBoard_Mode)
						{
							Reset_Motor();
							Veh.Mode = KeyBoard_Mode;
							Veh_UpdateMaxVelocity(&Veh,ToRPM(GetValueFromString(&U6.Message[2][0])));
						}
					}
					else if(U6.Message[1][0] == '0')
					{
						Veh.Mode = None_Mode;
						Reset_Motor();
					}
					else
					{
						if((U6.Message[1][0] == 'W') && (U6.Message[2][0] == '!') && (U6.Message[3][0] == '!') && (U6.Message[4][0] == '!'))
						{
							PID_UpdateSetVel(&M1,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							PID_UpdateSetVel(&M2,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
						}
						else if((U6.Message[1][0] == '!') && (U6.Message[2][0] == '!') && (U6.Message[3][0] == 'A') && (U6.Message[4][0] == '!'))
						{
							PID_UpdateSetVel(&M1,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							PID_UpdateSetVel(&M2,0);
						}
						else if((U6.Message[1][0] == '!') && (U6.Message[2][0] == '!') && (U6.Message[3][0] == '!') && (U6.Message[4][0] == 'D'))
						{
							PID_UpdateSetVel(&M1,0);
							PID_UpdateSetVel(&M2,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
						}
						else if((U6.Message[1][0] == 'W') && (U6.Message[2][0] == '!') && (U6.Message[3][0] == 'A') && (U6.Message[4][0] == '!'))
						{
							PID_UpdateSetVel(&M1,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							PID_UpdateSetVel(&M2,(((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity) * 0.6);
						}
						else if((U6.Message[1][0] == 'W') && (U6.Message[2][0] == '!') && (U6.Message[3][0] == '!') && (U6.Message[4][0] == 'D'))
						{
							PID_UpdateSetVel(&M1,(((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity) * 0.6);
							PID_UpdateSetVel(&M2,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
						}
						else
						{
							PID_UpdateSetVel(&M1,0);
							PID_UpdateSetVel(&M2,0);
						}
					}
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					break;
			}
	}
	else
	{
		U6_SendData(FeedBack(U6_TxBuffer,'0'));
	}
}
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
void TIM2_IRQHandler(void)
{
//	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
//	SendData();
}
/** brief : Encoder interrupt **/
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
