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
void SampleTime(void)
{
	Timer.T     = Timer.Sample_Time * pow(10,-6);
}
void GetVehicleVelocity(void)
{
			if(GPIO_ReadOutputDataBit(Dir_GPIOx,Dir_GPIO_Pin_M1) == 0) // Backward up counting
			{
				if((M1.Enc - M1.PreEnc) > 0)
				{
					M1.TotalEnc = M1.Enc + (M1.OverFlow - 1) * 65535;
					M1.Velocity = (((double)(M1.TotalEnc - M1.PreEnc)/ 39400) * 60) / Timer.T;
				}
				else
				{
					M1.Velocity = 0;
				}
			}
			else // Front down 
			{
				if((M1.Enc - M1.PreEnc) < 0)
				{
					M1.TotalEnc = (65535 - M1.Enc) + (M1.OverFlow - 1) * 65535;
					M1.Velocity = (((double)(M1.TotalEnc - (65535 - M1.PreEnc))/ 39400) * 60) / Timer.T;
				}
				else
				{
					M1.Velocity = 0;
				}
			}
			
			if(GPIO_ReadOutputDataBit(Dir_GPIOx,Dir_GPIO_Pin_M2) == 0)
			{
				if((M2.Enc - M2.PreEnc) > 0)
				{
					M2.TotalEnc = M2.Enc + (M2.OverFlow - 1) * 65535;
					M2.Velocity = (((double)(M2.TotalEnc - M2.PreEnc)/ 39400) * 60) / Timer.T;
				}
				else
				{
					M2.Velocity = 0;
				}
			}
			else 
			{
				if((M2.Enc - M2.PreEnc) < 0)
				{
					M2.TotalEnc = (65535 - M2.Enc) + (M2.OverFlow - 1) * 65535;
					M2.Velocity = (((double)(M2.TotalEnc - (65535 - M2.PreEnc))/ 39400) * 60) / Timer.T;
				}
				else
				{
					M2.Velocity = 0;
				}
			}
}
/*$VINFO, M1.SetVelocity, M2.SetVelocity, M1.Velocity, M2Velocity, 
M1Duty, M2Duty, SetAngle, Angle, GPS.Status, PosX, PosY, CC*/
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
	ind     += ToChar(Ang.Duty_Cycle,&U6_TxBuffer[ind]); // Ang duty cyble
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind			+= ToChar(Veh.Mode,&U6_TxBuffer[ind]);
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind			+= ToChar(M1.SetVelocity,&U6_TxBuffer[ind]);
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind			+= ToChar(M2.SetVelocity,&U6_TxBuffer[ind]);
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind 		+= ToChar(M1.Velocity,&U6_TxBuffer[ind]); //M1 velocity
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind  		+= ToChar(M2.Velocity,&U6_TxBuffer[ind]); // M2 velocity
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind			+= ToChar(Mag.Set_Angle,&U6_TxBuffer[ind]); // Set angle
	U6_TxBuffer[ind++] = (uint8_t)',';
	ind   	+= ToChar(Mag.Angle,&U6_TxBuffer[ind]); // Current angle
	U6_TxBuffer[ind++] = (uint8_t)',';
	if(GPS_NEO.Send_Flag)
	{
		GPS_NEO.Send_Flag = false;
		U6_TxBuffer[ind++] = (uint8_t)'Y';
		U6_TxBuffer[ind++] = (uint8_t)',';
		ind   		+= ToChar(GPS_NEO.CorX,&U6_TxBuffer[ind]);
		U6_TxBuffer[ind++] = (uint8_t)',';
		ind				+= ToChar(GPS_NEO.CorY,&U6_TxBuffer[ind]);
		U6_TxBuffer[ind++] = (uint8_t)',';
		ind				+= ToChar(GPS_NEO.Latitude,&U6_TxBuffer[ind]);
		U6_TxBuffer[ind++] = (uint8_t)',';
		ind				+= ToChar(GPS_NEO.Longitude,&U6_TxBuffer[ind]);
		U6_TxBuffer[ind++] = (uint8_t)',';
	}
	else
	{
		U6_TxBuffer[ind++] = (uint8_t)'N';
		U6_TxBuffer[ind++] = (uint8_t)',';
	}
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],ind - 1);
	U6_TxBuffer[ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[ind++] = 0x0D;
	U6_TxBuffer[ind++] = 0x0A;
	U6_SendData(ind);
	ind = 0;
}

void Reset_Motor()
{
	Stop_Motor();
	M1.SetVelocity 				= 0;
	M2.SetVelocity 				= 0;
	Veh.Manual_Velocity   = 0;
	Veh.Manual_Angle 			= 0;
}

void Reset_Encoder()
{
		M1.PreEnc = M1.Enc;
		M2.PreEnc = M2.Enc;
		M1.OverFlow = 1;
		M2.OverFlow = 1;
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/* M1 - Right motor, M2 - Left motor */
void SysTick_Handler(void)
{
	if(Timer.Time_Count < Timer.Set_Time)
	{
		Timer.Time_Count++;
	}
	else
	{
		Timer.Time_Count = 0;
		Timer.Set_Time   = Timer.Sample_Time;
		SampleTime();
		M1.Enc 	     	 = TIM_GetCounter(TIM3);
		M2.Enc	 		 	 = TIM_GetCounter(TIM4);
		if(Mag.Rx_Flag)
			Mag.Angle 	 = Mag.Rx_Angle;
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
					Mag.Delta_Angle = StanleyControl(GPS_NEO.CorX, GPS_NEO.CorY, Mag.Angle, GPS_NEO.Path_X, GPS_NEO.Path_Y, GPS_NEO.Path_Yaw, GPS_NEO.NbOfWayPoints, Veh.Max_Velocity);
					Mag.Set_Angle   = fix(Mag.Angle + Mag.Delta_Angle);
				}
				Ang.Duty_Cycle = Defuzzification_Max_Min((double)((Mag.Set_Angle - Mag.Angle)/180),(double)(-(Mag.Angle - Mag.Pre_Angle)/Timer.T)/30);
				if(Ang.Duty_Cycle >= 0)
				{
					M1.SetVelocity = Veh.Max_Velocity;
					M2.SetVelocity = (1 + fabs(Ang.Duty_Cycle)) * Veh.Max_Velocity;
				}
				else
				{
					M1.SetVelocity = (1 + fabs(Ang.Duty_Cycle)) * Veh.Max_Velocity;
					M2.SetVelocity = Veh.Max_Velocity;
				}
				M1.Duty_Cycle = PID_ControlM1(M1.Velocity,M1.SetVelocity,Timer.T);
				M2.Duty_Cycle = PID_ControlM2(M2.Velocity,M2.SetVelocity,Timer.T);
				Robot_Forward(M1.Duty_Cycle,M2.Duty_Cycle);   //Forward down counting Set bit
				Mag.Pre_Angle = Mag.Angle;
				break;
			
			/*--------------- Manual mode section ----------------*/
			/*----------------------------------------------------*/
			case 2: 
				GetVehicleVelocity();
				if(Veh.ManualCtrlKey == 'F')
				{
					Reset_Motor();
				}
				if(Veh.ManualCtrlKey == 'W')
				{
					Veh.Manual_Velocity += 0.2 * Veh.Max_Velocity;
					SelectFuzzyOutput(Veh.Manual_Velocity);
				}
				if(Veh.ManualCtrlKey == 'S')
				{
					Veh.Manual_Velocity -= 0.2 * Veh.Max_Velocity;
					SelectFuzzyOutput(Veh.Manual_Velocity);
				}
				if(Veh.Manual_Velocity < 0) Veh.Manual_Velocity = 0;
				else if(Veh.Manual_Velocity > Veh.Max_Velocity) Veh.Manual_Velocity = Veh.Max_Velocity;
				if(Veh.ManualCtrlKey == 'A') // Turn left - slow down the left motor
				{
					Veh.Manual_Angle -= 30;
					Mag.Set_Angle = fix(Veh.Manual_Angle + Mag.Angle);
				}
				if(Veh.ManualCtrlKey == 'D') // Turn right - slow down the right motor
				{
					Veh.Manual_Angle += 30;
					Mag.Set_Angle = fix(Veh.Manual_Angle + Mag.Angle);
				}
				Ang.Duty_Cycle = Defuzzification_Max_Min((double)((Mag.Set_Angle - Mag.Angle)/180),(double)(-(Mag.Angle - Mag.Pre_Angle)/Timer.T)/30);
				if(Ang.Duty_Cycle >= 0)
				{
					M1.SetVelocity = Veh.Manual_Velocity;
					M2.SetVelocity = (1 + fabs(Ang.Duty_Cycle)) * Veh.Manual_Velocity;
				}
				else
				{
					M1.SetVelocity = (1 + fabs(Ang.Duty_Cycle)) * Veh.Manual_Velocity;
					M2.SetVelocity = Veh.Manual_Velocity;
				}
			  M1.Duty_Cycle = PID_ControlM1(M1.Velocity,M1.SetVelocity,Timer.T); // Right motor
				M2.Duty_Cycle = PID_ControlM2(M2.Velocity,M2.SetVelocity,Timer.T); // Left motor
				Robot_Forward(M1.Duty_Cycle,M2.Duty_Cycle);
				Mag.Pre_Angle = Mag.Angle;
				Veh.ManualCtrlKey = 0;
				break;
			
			/*--------------- Calibration section ------------------*/
			/*------------------------------------------------------*/
			case 3: 
				GetVehicleVelocity();
				M1.Duty_Cycle = PID_ControlM1(M1.Velocity,30,Timer.T);
				M2.Duty_Cycle = PID_ControlM2(M2.Velocity,30,Timer.T);
				Robot_Spin_ClockWise(M1.Duty_Cycle,M2.Duty_Cycle);
				break;
			
			/*-------------- Test section --------------------------*/
			/*------------------------------------------------------*/
			case 4:
				GetVehicleVelocity();
				M1.Duty_Cycle = PID_ControlM1(M1.Velocity,30,Timer.T);
				M2.Duty_Cycle = PID_ControlM2(M2.Velocity,30,Timer.T);
				Robot_Forward(M1.Duty_Cycle,M2.Duty_Cycle);
				break;
		}
		Reset_Encoder();
	}
}

/** brief : USART1 interrupt Rx handler **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
void DMA2_Stream5_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
	Mag.Rx_Angle = IMU_GetValue(U1_RxBuffer,3);
	if(!Mag.Rx_Flag)
	{
			Mag.Rx_Flag = true;
			Mag.Set_Angle = Mag.Rx_Angle;
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
				GPS_NEO.Latitude 	 = LLToDegree(GetLatFromString(&U2.Message[1][0]));
				GPS_NEO.Longitude  = LLToDegree(GetLonFromString(&U2.Message[3][0]));
				LatLonToUTM(GPS_NEO.Latitude,GPS_NEO.Longitude,cor);
				GPS_NEO.CorX = cor[0];
				GPS_NEO.CorY = cor[1];
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
//	if(IsCorrectMessage(&U6.RxTempBuffer[1],length - 3,U6.RxTempBuffer[length - 2],U6.RxTempBuffer[length - 1]))
//	{
	if(1)
	{
		GetMessageInfo((char*)U6.RxTempBuffer,U6.Message,',');
		switch(GetNbOfReceiveHeader(&U6.Message[0][0]))
			{
				/*----------------- Vehicle Config --------------------------*/
				case 1: 
					Veh.Max_Velocity 		  		= ToRPM(GetValueFromString(&U6.Message[1][0]));
					M1.Kp    		  						= GetValueFromString(&U6.Message[2][0]);
					M1.Ki   		  						= GetValueFromString(&U6.Message[3][0]);
					M1.Kd 			  						= GetValueFromString(&U6.Message[4][0]);
					M2.Kp		 		  						= GetValueFromString(&U6.Message[5][0]);
					M2.Ki 	 		  						= GetValueFromString(&U6.Message[6][0]);
					M2.Kd		 		  						= GetValueFromString(&U6.Message[7][0]);
					break;
				
				/*---------------- Sample time config (us)------------------*/
				case 2: 
					Timer.Sample_Time = (uint32_t)GetValueFromString(&U6.Message[1][0]);
					break;
				
				/*---------------- Send data time config (ms)---------------*/
				case 3: 
					Timer.Send_Time = (uint32_t)GetValueFromString(&U6.Message[1][0]);
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
					Veh.Software_Reset = true;
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
					GPS_NEO.NbOfWayPoints = U6.Message[1][0] - 48;
					for(int i = 0; i < GPS_NEO.NbOfWayPoints; i++)
					{
						LatLonToUTM(GetValueFromString(&U6.Message[i * 2 + 2][0]),GetValueFromString(&U6.Message[i * 2 + 3][0]),cor);
						GPS_NEO.Path_X[i] = cor[0];
						GPS_NEO.Path_Y[i] = cor[1];
					}
					CalculatePathCoordinate(GPS_NEO.NbOfWayPoints,GPS_NEO.Path_X,GPS_NEO.Path_Y,GPS_NEO.Path_Yaw);
					break;
			}
			U6_SendData(FeedBack(U6_TxBuffer,'1'));
			if(Veh.Software_Reset)
			{
				Veh.Software_Reset = false;
				NVIC_SystemReset();
			}
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
