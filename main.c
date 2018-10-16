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
double 				velocity = 0;

/* Function */
void Peripheral_Config()
{
	//USART2_Rx_Config(38400); //GPS USART first priority 
	USART6_Config(38400); 		//Sending and controling USART1
	Encoder_Config();
	ECompass_Config();
	SysTick_Config(SystemCoreClock/1000000);  // 1us
}
/** @brief  : Initial parameters for input
**  @agr    : void
**  @retval : void
**/
void Parameters_Init()
{
	/*----------Fuzzy parameter init ------------------*/
	/*   Input 1 (e = Set_theta - theta)  */
	// NB : 0.4 - 1
	In1_NB.h1 = -2;
	In1_NB.h2 = -1;
	In1_NB.h3 = -0.45;
	In1_NB.h4 = -0.4;
	// NS : 0.15 - 0.45
	In1_NS.a1 = -0.45;
	In1_NS.a2 = -0.2;
	In1_NS.a3 = -0.15;
	// ZE : 0 - 0.2
	In1_ZE.a1 = -0.2;
	In2_ZE.a2 = 0;
	In2_ZE.a3 = 0.2;
	// PS : 0.15 - 0.45
	In1_PS.a1 = 0.15;
	In1_PS.a2 = 0.2;
	In1_PS.a3 = 0.45;
	// PB : 0.4 - 1
	In1_PB.h1 = 0.4;
	In1_PB.h2 = 0.45;
	In1_PB.h3 = 1;
	In1_PB.h4 = 2;
	/* Input 2 (edot = Set_thetadot - thetadot) */
	// NE : 0.3 - 1
	In2_NE.h1 = -2;
	In2_NE.h2 = -1;
	In2_NE.h3 = -0.4;
	In2_NE.h4 = -0.3;
	// ZE : 0 - 0.4
	In2_ZE.a1 = -0.4;
	In2_ZE.a2 = 0;
	In2_ZE.a3 = 0.4;
	// PO : 0.3 - 1
	In2_PO.h1 = 0.3;
	In2_PO.h2 = 0.4;
	In2_PO.h3 = 1;
	In2_PO.h4 = 2;
	/* Output value */
	NB = -0.75;
	NM = -0.4;
	NS = -0.175;
	ZE = 0;
	PS = 0.175;
	PM = 0.4;
	PB = 0.75;
	/*------------PID Parameter Init-------------*/
	M1.Kp 				= 0.25;
	M1.Ki 				= 0.08;
	M1.Kd 				= 0.005;
	M1.Pre2_Error = 0;
	M1.Pre_Error  = 0;
	M1.Pre_PID 		= 0;
	M1.OverFlow   = 0;
	M1.PreEnc 		= 0;
	M2.Kp 				= 0.25;
	M2.Ki 				= 0.08;
	M2.Kd 				= 0.005;
	M2.Pre2_Error = 0;
	M2.Pre_Error  = 0;
	M2.Pre_PID 		= 0;
	M2.OverFlow   = 0;
	M2.PreEnc 		= 0;
}
void Delay(uint32_t time)
{
	while(time--)
	{}
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
				velocity 		 = (double)GetValueFromString(&U6_Message[1][0]);
			  velocity		 = ToRPM(velocity);
			  SendAngle    = (uint16_t)GetValueFromString(&U6_Message[2][0]);
				M1.Kp   		 = GetValueFromString(&U6_Message[3][0]);
			  M1.Ki   		 = GetValueFromString(&U6_Message[4][0]);
				M1.Kd 			 = GetValueFromString(&U6_Message[5][0]);
				M2.Kp		 		 = GetValueFromString(&U6_Message[6][0]);
				M2.Ki  	 		 = GetValueFromString(&U6_Message[7][0]);
				M2.Kd		 		 = GetValueFromString(&U6_Message[8][0]);
				break;
			
			case 'T': //Sample time for robot
				control = 0;
				sample_time = (uint32_t)GetValueFromString(&U6_Message[1][0]);
				break;
			
			case 'R':
				break;
		}
	}
}




















