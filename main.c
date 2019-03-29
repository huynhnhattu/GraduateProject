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
/* Function */
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
	Main_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 1;
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
	USART1_Config(115200);
	//USART2_Config(9600); 			//GPS USART first priority 
	USART6_Config(9600); 			//Sending and controling USART1
	Encoder_Config();				
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_SetPriority(SysTick_IRQn,1);
	TIM2_TimeBaseConfig(Timer.Send_Time);
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
	Flash.Length    += ToChar(0.300000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.200000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.001000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.400000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.200000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.001000,&Flash.WriteInBuffer[Flash.Length]);
	Flash.WriteInBuffer[Flash.Length++] = 0x0D;
	Flash.WriteInBuffer[Flash.Length++] = 0x0A;
	EraseMemory(FLASH_Sector_7);
	WriteToFlash(&Flash,FLASH_Sector_7,FLASH_PIDPara_BaseAddr);
	Flash.Length = 0;
}
/** @brief  : Initial parameters for input
**  @agr    : void
**  @retval : void
**/
void Parameters_Init(void)
{
	/*-----------------Timer Init ---------------*/
	Time_ParametersInit(&Timer,50000,800);
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


/*C,0.7,0,0.035,0.35,0.0005,0.2,0.4,0.002*/
int main(void)
{
	Parameters_Init();
	Peripheral_Config();
	//PID_SaveManual();
	SysTick_Config(SystemCoreClock/1000000);
	while(1)
	{
		
	}
}




















