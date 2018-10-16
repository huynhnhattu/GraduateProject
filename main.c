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
double 				Mxyz[3], Mx, My, Mz, D;
char   				dir[2], a, b, *r;
uint16_t 			counterM1 = 0, counterM2 = 0;
uint8_t  			control = 0;
uint8_t       duty = 0;
uint16_t 			angle = 0, velocity = 0;

/* Function */
void Peripheral_Config()
{
	USART2_Rx_Config(38400); //GPS USART first priority 
	USART6_Config(38400); 		//Sending and controling USART1
	Encoder_Config();
	ECompass_Config();
	SysTick_Config(SystemCoreClock/1000000);  // 1us
}
void Delay(uint32_t time)
{
	while(time--)
	{}
}
void ResetPID(void)
{
	M1_Pre_Error = 0;
	M1_Pre2_Error = 0;
	M1_Pre_PID = 0;
	M2_Pre_Error = 0;
	M2_Pre2_Error = 0;
	M2_Pre_PID = 0;
	Ang_Pre_Error = 0;
	Ang_Pre2_Error = 0;
	Ang_Pre_PID = 0;
}
int main(void)
{
	Parameters_Init();
	Peripheral_Config();
	while(1)
	{
		switch((char)control)
		{
			case 'A': // Left
				control = 0;
				//Robot_Forward(
				break;
			case 'S': // Back
				control = 0;
			  
				break;
			case 'W': // Forward
				control = 0;
				//Robot_Forward(GetValueFromString(&U6_Message[1][0]),GetValueFromString(&U6_Message[1]
				break;
			case 'D': // Right
				control = 0;
			
				break;
			case 'C': // Config
				control 		 = 0;
				Stop    		 = false;
				velocity 		 = (uint16_t)GetValueFromString(&U6_Message[1][0]);
			  velocity		 = ToRPM(velocity);
			  SendAngle    = (uint16_t)GetValueFromString(&U6_Message[2][0]);
				M1_Kp   		 = GetValueFromString(&U6_Message[3][0]);
			  M1_Kd    		 = GetValueFromString(&U6_Message[4][0]);
				M1_Ki 			 = GetValueFromString(&U6_Message[5][0]);
				M2_Kp		 		 = GetValueFromString(&U6_Message[6][0]);
				M2_Kd  	 		 = GetValueFromString(&U6_Message[7][0]);
				M2_Ki		 		 = GetValueFromString(&U6_Message[8][0]);
				break;
			
			case 'T': //Sample time for robot
				control = 0;
				sample_time = (uint32_t)GetValueFromString(&U6_Message[1][0]);
				break;
			
			case 'R':
				control = 0;
				Stop_Motor();
				PreEncM1 = 0;
				PreEncM2 = 0;
				Stop = true;
			  //Reset PID
				ResetPID();
				break;
		}
	}
}




















