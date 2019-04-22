#include "main.h"

/** @Control @Characters **/
/** Motor Control
** @A  : 
** @W  :
** @S  :
** @D  : 
** @C  : Config Peripheral.
** Format: C , v , angle , M1kp , M1kd , M1ki , M2kp , M2kd , M2ki
** Send PC format: VelM1,VelM2,angle,Y/N,xcor,ycor
**/
/* Global Variables */
TIM_TimeBaseInitTypeDef			Main_TIM_Struct;
NVIC_InitTypeDef						Main_NVIC_Struct;
Srf05_InitTypeDef						Srf05_Struct;
Srf05_Data									Mid, Left, Right, Back;
/* Function */
void Srf05_MidFront(void)
{
	Srf05_Struct.Srf05_ICTIM					= TIM2;
	Srf05_Struct.Srf05_GPIO						= GPIOA;
	Srf05_Struct.Srf05_Echo_Pin				= GPIO_Pin_15;
	Srf05_Struct.Srf05_IC_Channel			= TIM_Channel_1;
	Srf05_Struct.Srf05_TIM_IT_CC			= TIM_IT_CC1;
	Srf05_Struct.Srf05_PreemptionPriority = 2;
	Srf05_Struct.Srf05_SubPriority		= 0;
	Srf05_Initial(&Srf05_Struct);
}

void Srf05_LeftFront(void)
{
	Srf05_Struct.Srf05_ICTIM					= TIM2;
	Srf05_Struct.Srf05_GPIO						= GPIOB;
	Srf05_Struct.Srf05_Echo_Pin				= GPIO_Pin_3;
	Srf05_Struct.Srf05_IC_Channel			= TIM_Channel_2;
	Srf05_Struct.Srf05_TIM_IT_CC			= TIM_IT_CC2;
	Srf05_Struct.Srf05_PreemptionPriority = 2;
	Srf05_Struct.Srf05_SubPriority		= 1;
	Srf05_Initial(&Srf05_Struct);
}

void Srf05_RightFront(void)
{
	Srf05_Struct.Srf05_ICTIM					= TIM2;
	Srf05_Struct.Srf05_GPIO						= GPIOA;
	Srf05_Struct.Srf05_Echo_Pin				= GPIO_Pin_10;
	Srf05_Struct.Srf05_IC_Channel			= TIM_Channel_3;
	Srf05_Struct.Srf05_TIM_IT_CC			= TIM_IT_CC3;
	Srf05_Struct.Srf05_PreemptionPriority = 2;
	Srf05_Struct.Srf05_SubPriority		= 2;
	Srf05_Initial(&Srf05_Struct);
}

void Srf05_BackEnd(void)
{
	Srf05_Struct.Srf05_ICTIM					= TIM2;
	Srf05_Struct.Srf05_GPIO						= GPIOA;
	Srf05_Struct.Srf05_Echo_Pin				= GPIO_Pin_11;
	Srf05_Struct.Srf05_IC_Channel			= TIM_Channel_4;
	Srf05_Struct.Srf05_TIM_IT_CC			= TIM_IT_CC4;
	Srf05_Struct.Srf05_PreemptionPriority = 2;
	Srf05_Struct.Srf05_SubPriority		= 3;
	Srf05_Initial(&Srf05_Struct);
}

/** @brief  : TIM1 interrupt count config
**  @agr    : void
**  @retval : void
**/
void TIM2_TimeBaseConfig(uint32_t time)   // ms
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	Main_TIM_Struct.TIM_Prescaler  				=  50000 - 1; //ms
	Main_TIM_Struct.TIM_Period     				=  time*2 - 1;
	Main_TIM_Struct.TIM_ClockDivision 		=  TIM_CKD_DIV1;
	Main_TIM_Struct.TIM_CounterMode  			=  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&Main_TIM_Struct);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	Main_NVIC_Struct.NVIC_IRQChannel  		=  TIM2_IRQn;
	Main_NVIC_Struct.NVIC_IRQChannelCmd 	=  ENABLE;
	Main_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 3;
	Main_NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&Main_NVIC_Struct);
	TIM_Cmd(TIM2,ENABLE);
}

/** @brief  : Peripheral config
**  @agr    : void
**  @retval : void
**/
void Peripheral_Config(void)
{
	Srf05_TriggerPinConfig(GPIOB,GPIO_Pin_2);
	Srf05_MidFront();
	USART1_Config(115200);
	USART2_Config(9600); 			//GPS USART first priority 
	USART6_Config(9600); 			//Sending and controling USART1
	Encoder_Config();				
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/** @brief  : Delay core clock 100MHZ
**  @agr    : time
**  @retval : void
**/
void Delay(uint32_t time)
{ 	
	while(time--)
	{};
}

/** @brief  : Read PID parameters from internal flash memory
**  @agr    : void
**  @retval : void
**/
void PID_ReadParametersFromFlash(void)
{
	ReadFromFlash(&Flash,FLASH_PIDPara_BaseAddr);
	GetMessageInfo((char*)Flash.ReadOutBuffer,Flash.Message,',');
	PID_ParametersUpdate(&M1,GetValueFromString(&Flash.Message[0][0]),GetValueFromString(&Flash.Message[1][0]),GetValueFromString(&Flash.Message[2][0]));
	PID_ParametersUpdate(&M2,GetValueFromString(&Flash.Message[3][0]),GetValueFromString(&Flash.Message[4][0]),GetValueFromString(&Flash.Message[5][0]));
}

void PID_SaveManual(void)
{
	Flash.Length    += ToChar(0.060000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.080000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.001000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.050000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.100000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.001000,&Flash.WriteInBuffer[Flash.Length]);
	EraseMemory(FLASH_Sector_7);
	WriteToFlash(&Flash,FLASH_Sector_7,FLASH_PIDPara_BaseAddr);
	Flash.Length		= 0;
}

void SendData(void)
{
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'$';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'V';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'I';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'F';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'O';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M1.Set_Vel,&U6_TxBuffer[Veh.SendData_Ind]); // M1 SetVelocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind			+= ToChar(M2.Set_Vel,&U6_TxBuffer[Veh.SendData_Ind]); // M2 SetVelocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind			+= ToChar(M1.Current_Vel,&U6_TxBuffer[Veh.SendData_Ind]); // M1 Velocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind  		+= ToChar(M2.Current_Vel,&U6_TxBuffer[Veh.SendData_Ind]); // M2 velocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind  		+= ToChar(M1.PID_Out,&U6_TxBuffer[Veh.SendData_Ind]); // M1 PID 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind  		+= ToChar(M2.PID_Out,&U6_TxBuffer[Veh.SendData_Ind]); // M2 PID
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind			+= ToChar(Mag.Set_Angle,&U6_TxBuffer[Veh.SendData_Ind]); // Set angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind   		+= ToChar(Mag.Angle,&U6_TxBuffer[Veh.SendData_Ind]); // Current angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	if(Status_CheckStatus(&VehStt.GPS_Coordinate_Sending))
	{
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'Y';
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	}
	else
	{
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	}
	Veh.SendData_Ind   			+= ToChar(GPS_NEO.CorX,&U6_TxBuffer[Veh.SendData_Ind]);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.CorY,&U6_TxBuffer[Veh.SendData_Ind]);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.Latitude,&U6_TxBuffer[Veh.SendData_Ind]);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.Longitude,&U6_TxBuffer[Veh.SendData_Ind]);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	U6_SendData(Veh.SendData_Ind);
	Veh.SendData_Ind = 0;
}

/** @brief  : Initial parameters for input
**  @agr    : void
**  @retval : void
**/
void Parameters_Init(void)
{
	/*-----------------Timer Init ---------------*/
	Time_ParametersInit(&Timer,50000,1000000);
	Time_GetSampleTime(&Timer);
	/*------------PID Parameter Init-------------*/
	PID_ReadParametersFromFlash();
	PID_ParametersInitial(&M1);
	PID_ParametersInitial(&M2);
	/*------------Fuzzy parametes Init ----------*/
	Fuzzy_ParametersInit();
	/*------------------AngleControl-------------*/
	IMU_ParametesInit(&Mag);
	IMU_UpdateFuzzyCoefficients(&Mag,(double)1/180,(double)1/30,(double)1);
	/*------------------StanleyParameter---------*/
	GPS_ParametersInit(&GPS_NEO);
	/*------------------Vehicleinit--------------*/
	Veh_ParametersInit(&Veh);
	/*------------------Read/Write Message Init--*/
}

int main(void)
{
	Parameters_Init();
	Peripheral_Config();
	//PID_SaveManual();
	SysTick_Config(SystemCoreClock/1000000); // (us)
	while(1)
	{
		if(Status_CheckStatus(&VehStt.Srf05_TimeOut_Flag))
		{
			Srf05_StartDevice();
			Srf05_ResetCounter(TIM2);
			Status_UpdateStatus(&VehStt.Srf05_TimeOut_Flag,Check_NOK);
		}
		
		if(Status_CheckStatus(&VehStt.Veh_Send_Data))
		{
			if(Status_CheckStatus(&VehStt.Veh_SendData_Flag))
			{
				//SendData();
			}
			Status_UpdateStatus(&VehStt.Veh_Send_Data,Check_NOK);
		}
		
		if(Status_CheckStatus(&VehStt.Veh_Sample_Time))
		{
			PID_UpdateEnc(&M1,TIM_GetCounter(TIM3));
			PID_UpdateEnc(&M2,TIM_GetCounter(TIM4));
			GetVehicleVelocity();
			switch((int)Veh.Mode)
			{
				/*-------------- Auto mode section ------------------*/
				/*---------------------------------------------------*/
				case Auto_Mode: 
					if(Status_CheckStatus(&VehStt.GPS_Coordinate_Reveived))
					{
						Status_UpdateStatus(&VehStt.GPS_Coordinate_Reveived,Check_NOK);
						Status_UpdateStatus(&VehStt.GPS_Coordinate_Sending,Check_OK);
						GPS_StanleyControl(&GPS_NEO, Timer.T);
						IMU_UpdateSetAngle(&Mag,GPS_NEO.Delta_Angle);
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
					if(Status_CheckStatus(&GPS_NEO.Goal_Flag))
					{
						PID_UpdateSetVel(&M1,0);
						PID_UpdateSetVel(&M2,0);
					}
					PID_Compute(&M1);
					PID_Compute(&M2);
					Robot_Forward(M1.PID_Out,M2.PID_Out);   //Forward down counting Set bit
					IMU_UpdatePreAngle(&Mag);
					break;
				
				/*--------------- Manual mode section ----------------*/
				/*----------------------------------------------------*/
				case Manual_Mode: 
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
				case Calib_Mode: 
					PID_Compute(&M1);
					PID_Compute(&M2);
					Robot_Spin_AntiClockWise(M1.PID_Out,M2.PID_Out);
					break;
				
				/*-------------- Test section --------------------------*/
				/*------------------------------------------------------*/
				case KeyBoard_Mode:
					PID_Compute(&M1);
					PID_Compute(&M2);
					Robot_Forward(M1.PID_Out,M2.PID_Out);   //Forward down counting Set bit
				break;
			}
			PID_ResetEncoder(&M1);
			PID_ResetEncoder(&M2);
			Status_UpdateStatus(&VehStt.Veh_Sample_Time,Check_NOK);
		}
	}
}




















