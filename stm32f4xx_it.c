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
double 					cor[2];
int    					count = 0;
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
/*$VINFO, M1.SetVelocity, M2.SetVelocity, M1.Velocity, M2Velocity, 
SetAngle, Angle, GPS.Status (Y/N), PosX, PosY, lat, lon, CC*/
int ind = 0;
void SendData(void)
{
	U6_TxBuffer[ind++] = (uint8_t)'$';
	U6_TxBuffer[ind++] = (uint8_t)'V';
	U6_TxBuffer[ind++] = (uint8_t)'I';
	U6_TxBuffer[ind++] = (uint8_t)'N';
	U6_TxBuffer[ind++] = (uint8_t)'F';
	U6_TxBuffer[ind++] = (uint8_t)'O';
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind     += ToChar(M1.Set_Vel,&U6_TxBuffer[ind]); // M1 SetVelocity
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind			+= ToChar(M2.Set_Vel,&U6_TxBuffer[ind]); // M2 SetVelocity
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind			+= ToChar(M1.Current_Vel,&U6_TxBuffer[ind]); // M1 Velocity
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind  		+= ToChar(M2.Current_Vel,&U6_TxBuffer[ind]); // M2 velocity
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind  		+= ToChar(M1.PID_Out,&U6_TxBuffer[ind]); // M1 PID 
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind  		+= ToChar(M2.PID_Out,&U6_TxBuffer[ind]); // M2 PID
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind			+= ToChar(Mag.Set_Angle,&U6_TxBuffer[ind]); // Set angle
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind   	+= ToChar(Mag.Angle,&U6_TxBuffer[ind]); // Current angle
	U6_TxBuffer[ind++] = (uint8_t)',';
	if(GPS_NEO.Send_Flag)
	{
		U6_TxBuffer[ind++] = (uint8_t)'Y';
		U6_TxBuffer[ind++] = (uint8_t)',';
	}
	else
	{
		U6_TxBuffer[ind++] = (uint8_t)'N';
		U6_TxBuffer[ind++] = (uint8_t)',';
	}
	ind   		+= ToChar(GPS_NEO.CorX,&U6_TxBuffer[ind]);
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind				+= ToChar(GPS_NEO.CorY,&U6_TxBuffer[ind]);
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind				+= ToChar(GPS_NEO.Latitude,&U6_TxBuffer[ind]);
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind				+= ToChar(GPS_NEO.Longitude,&U6_TxBuffer[ind]);
	U6_TxBuffer[ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],ind - 1);
	U6_TxBuffer[ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[ind++] = 0x0D;
	U6_TxBuffer[ind++] = 0x0A;
	U6_SendData(ind);
	ind = 0;
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
	if(Timer.Time_Count < Timer.Sample_Time)
	{
		Timer.Time_Count++;
	}
	else
	{
		Timer.Time_Count = 0;
		PID_UpdateEnc(&M1,TIM_GetCounter(TIM3));
		PID_UpdateEnc(&M2,TIM_GetCounter(TIM4));
		switch(Veh.Mode)
		{
			/*-------------- Auto mode section ------------------*/
			/*---------------------------------------------------*/
			case 1: 
				GetVehicleVelocity();
				if(GPS_NEO.Rx_Flag)
				{
					GPS_NEO.Rx_Flag = false;
					GPS_NEO.Send_Flag = true;
					GPS_StanleyControl(&GPS_NEO);
					IMU_UpdateSetAngle(&Mag,&GPS_NEO.Delta_Angle);
				}
				IMU_UpdateFuzzyInput(&Mag,&Timer.T);
				Defuzzification_Max_Min(&Mag);
				if(Mag.Fuzzy_Out >= 0)
				{
					PID_UpdateSetVel(&M1,Veh.Manual_Velocity);
					PID_UpdateSetVel(&M2,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
				}
				else
				{
					PID_UpdateSetVel(&M1,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
					PID_UpdateSetVel(&M2,Veh.Manual_Velocity);
				}
				PID_Compute(&M1);
				PID_Compute(&M2);
				Robot_Forward(M1.PID_Out,M2.PID_Out);   //Forward down counting Set bit
				IMU_UpdatePreAngle(&Mag);
				break;
			
			/*--------------- Manual mode section ----------------*/
			/*----------------------------------------------------*/
			case 2: 
				GetVehicleVelocity();
				Veh_UpdateVehicleFromKey(&Veh);
				IMU_UpdateFuzzyInput(&Mag,&Timer.T);
				Defuzzification_Max_Min(&Mag);
				if(Mag.Fuzzy_Out >= 0)
				{
					PID_UpdateSetVel(&M1,Veh.Manual_Velocity);
					PID_UpdateSetVel(&M2,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
				}
				else
				{
					PID_UpdateSetVel(&M1,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
					PID_UpdateSetVel(&M2,Veh.Manual_Velocity);
				}
			  PID_Compute(&M1);
				PID_Compute(&M2);
				Robot_Forward(M1.PID_Out,M2.PID_Out);   //Forward down counting Set bit
				IMU_UpdatePreAngle(&Mag);
				Veh.ManualCtrlKey = 0;
				break;
			
			/*--------------- Calibration section ------------------*/
			/*------------------------------------------------------*/
			case 3: 
				GetVehicleVelocity();
				PID_UpdateSetVel(&M1,30);
				PID_UpdateSetVel(&M2,30);
				PID_Compute(&M1);
				PID_Compute(&M2);
				Robot_Spin_ClockWise(M1.PID_Out,M2.PID_Out);
				break;
			
			/*-------------- Test section --------------------------*/
			/*------------------------------------------------------*/
			case 4:
				GetVehicleVelocity();
				PID_UpdateSetVel(&M1,Veh.Max_Velocity);
				PID_UpdateSetVel(&M2,Veh.Max_Velocity);
				PID_Compute(&M1);
				PID_Compute(&M2);
				Robot_Forward(M1.PID_Out,M2.PID_Out);   //Forward down counting Set bit
			break;
		}
		PID_ResetEncoder(&M1);
		PID_ResetEncoder(&M2);
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
	if(!Mag.First_RxFlag)
	{
		Mag.First_RxFlag = true;
		IMU_UpdateSetAngle(&Mag,&Mag.Angle);
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
	Readline(U2_RxBuffer,U2.RxTempBuffer);
	GetMessageInfo((char*)U2.RxTempBuffer,U2.Message,',');
	if(StringHeaderCompare(&U2.Message[0][0],"$GNGLL"))
	{
		if(IsCorrectMessage(&U2_RxBuffer[1],46,(uint8_t)U2.Message[7][2],(uint8_t)U2.Message[7][3]))
		{
			if(IsValidData(U2.Message[6][0]))
			{
				GPS_NEO.Rx_Flag = true;
				GPS_GetLatFromString(&GPS_NEO,&U2.Message[1][0]);
				GPS_GetLonFromString(&GPS_NEO,&U2.Message[3][0]);
				GPS_LatLonToUTM(&GPS_NEO);
			}
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
	Veh.LengthOfCommands = Readline(U6_RxBuffer,U6.RxTempBuffer);
//	if(IsCorrectMessage(&U6.RxTempBuffer[1],Veh.LengthOfCommands - 3,U6.RxTempBuffer[Veh.LengthOfCommands - 2],U6.RxTempBuffer[Veh.LengthOfCommands - 1]))
//	{
	if(1)
	{
		GetMessageInfo((char*)U6.RxTempBuffer,U6.Message,',');
		switch(GetNbOfReceiveHeader(&U6.Message[0][0]))
			{
				/*----------------- Vehicle Config --------------------------*/
				case 1:
					Veh_UpdateMaxVelocity(&Veh,ToRPM(GetValueFromString(&U6.Message[1][0])));
					PID_ParametersUpdate(&M1,GetValueFromString(&U6.Message[2][0]),GetValueFromString(&U6.Message[3][0]),GetValueFromString(&U6.Message[4][0]));
					PID_ParametersUpdate(&M2,GetValueFromString(&U6.Message[5][0]),GetValueFromString(&U6.Message[6][0]),GetValueFromString(&U6.Message[7][0]));
					break;
				
				/*---------------- Sample time config (us)------------------*/	
				case 2: 
					Time_SampleTimeUpdate(&Timer,(uint32_t)GetValueFromString(&U6.Message[1][0]));
					Time_GetSampleTime(&Timer);
					break;
				
				/*---------------- Send data time config (ms)---------------*/
				case 3: 
					Time_SendTimeUpdate(&Timer,(uint32_t)GetValueFromString(&U6.Message[1][0]));
					TIM2_TimeBaseConfig(Timer.Send_Time);
					break;
				
				/*---------------- IMU command and read data config --------*/
				case 4: 
					U1_TxBuffer[0] = (uint8_t)'$';
					U1_SendData((uint16_t)IMU_GetCommandMessage(&U6.Message[1][0],&U1_TxBuffer[1]));
					if((U6.Message[1][0] == 'M') && (U6.Message[1][1] == 'A') && (U6.Message[1][2] == 'G') && (U6.Message[1][3] == '2') && (U6.Message[1][4] == 'D'))
					{
						if(Veh.Mode != 3)
						{
							Reset_Motor();
							Veh.Mode = 3;
						}
					}
					break;
				
				/*---------------- Software reset --------------------------*/
				case 5: 
					U6_SendData(FeedBack(U6_TxBuffer,'1'));
					NVIC_SystemReset();
					break;
				
				/*---------------- Manual mode config ----------------------*/
				case 6: 
					if(U6.Message[1][0] == '1')
					{
						if(Veh.Mode != 2)
						{
							Reset_Motor();
							Veh.Mode = 2;
						}
					}
					else if(U6.Message[1][0] == '0')
						Reset_Motor();
					else
						Veh.ManualCtrlKey = U6.Message[1][0];
					break;
				
				/*---------------- Auto mode config ------------------------*/
				case 7: 
					if(U6.Message[1][0] == '1')
					{
						if(Veh.Mode != 1)
						{
							Reset_Motor();
							Veh.Mode = 1;
						}
					}
					else if(U6.Message[1][0] == '0')
						Reset_Motor();
					break;
				
				/*---------------- Receive path coordinate -----------------*/
				case 8: 
					GPS_UpdatePathCoordinate(&GPS_NEO,U6.RxTempBuffer);
					break;
					
				/*--------------- Save data in internal flash memory --------*/
				case 9:
					SaveDataToInternalFlash(1);
					break;
			}
			U6_SendData(FeedBack(U6_TxBuffer,'1'));
	}
	else
	{
		U6_SendData(FeedBack(U6_TxBuffer,'0'));
	}
	DMA_Cmd(DMA2_Stream2,ENABLE);
}
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
void TIM2_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	SendData();
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
