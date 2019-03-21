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
	TIM2_TimeBaseConfig(800);
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

/** @brief  : Initial parameters for input
**  @agr    : void
**  @retval : void
**/
void Parameters_Init(void)
{
	/*------------PID Parameter Init-------------*/
	M1.Kp 				= 0;
	M1.Ki 				= 0;
	M1.Kd 				= 0;
	M1.Pre2_Error = 0;
	M1.Pre_Error  = 0;
	M1.Pre_PID 		= 0;
	M1.OverFlow   = 1;
	M1.PreEnc 		= 0;
	M1.Duty_Cycle = 0;
	M2.Kp 				= 0;
	M2.Ki 				= 0;
	M2.Kd 				= 0;
	M2.Pre2_Error = 0;
	M2.Pre_Error  = 0;
	M2.Pre_PID 		= 0;
	M2.OverFlow   = 1;
	M2.PreEnc 		= 0;
	M2.Duty_Cycle = 0;
	/*------------------Timer Init ---------------------*/
	Timer.Sample_Time  = 50000;
	Timer.Time_Count   = 0;
	Timer.Set_Time     = 50000;
	/*------------------Angle Control-------------------*/
	Mag.Pre_Angle = 0;
	Mag.Set_Angle = 0;
	Mag.Rx_Angle  = 0;
	Mag.Rx_Flag = false;
	/*------------------Stanley Parameter---------------*/
	GPS_NEO.Pre_CorX = 0;
	GPS_NEO.Pre_CorY = 0;
	GPS_NEO.Rx_Flag = false;
	GPS_NEO.Send_Flag = false;
	GPS_NEO.NbOfWayPoints = 0;
	/*------------------Manual Init---------------------*/
	M1.SetVelocity = 0;
	M2.SetVelocity = 0;
	M1.Velocity = 0;
	M2.Velocity = 0;
	/*------------------Vehicle init-----------------------*/
	Veh.Mode = 4;  // 0 - default mode, 1 - auto, 2 - manual, 3 - Calib spin robot mode
	Veh.ManualCtrlKey = 0;
	Veh.Max_Velocity = 0.0;
	Veh.Manual_Angle = 0;
	Veh.Software_Reset = false;
	/*------------------Read/Write Message Init-------------*/
}


/*C,0.7,0,0.035,0.35,0.0005,0.2,0.4,0.002*/
int main(void)
{
	Parameters_Init();
	//Peripheral_Config();
	//SelectFuzzyOutput(0);
	//SysTick_Config(SystemCoreClock/1000000);
	while(1)
	{
		
	}
}




















