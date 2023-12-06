// StepperTestMain.c
// Runs on TM4C123
// Test the functions provided by Stepper.c,
// 
// Before connecting a real stepper motor, remember to put the
// proper amount of delay between each CW() or CCW() step.
// Daniel Valvano
// September 12, 2013
// Modified by Min HE

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Example 4.1, Programs 4.4, 4.5, and 4.6
   Hardware circuit diagram Figure 4.27

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// PD3 connected to driver for stepper motor coil A/In1
// PD2 connected to driver for stepper motor coil A'/In2
// PD1 connected to driver for stepper motor coil B/In3
// PD0 connected to driver for stepper motor coil B'/In4
#include <stdint.h>
#include <stdbool.h>
#include "stepper.h"
#include "systick.h"
#include "tm4c123gh6pm.h"


#define T1ms 8000    // assumes using 16 MHz Clock 

bool Direction;
bool Move;
bool Drive;
bool Sensor;
bool Update_State;
uint32_t counter;
int i;


// Function Prototypes (from startup.s)
void DisableInterrupts(); // Disable interrupts
void EnableInterrupts();  // Enable interrupts
void WaitForInterrupt();  // Go to low power mode while waiting for the next interrupt

void Buttons_Init();
void sensor_init();
void GPIOPortF_Handler(); // Handle GPIO Port F interrupts
void GPIOPortD_Handler();
void Sensor_Init();

void Move_Forward();
void Move_Backward();
void Turn_Left();
void Turn_Right();
int input;
int Index;

void GPIOPortF_Handler(){
	Update_State = true;
	if( ( !(GPIO_PORTF_RIS_R & 0x01)) && input != 0)		//sw 1 (LEFT SWITCH)
	{
		input = 0;											//input is already 0 so if doesnt pass, but update state is false unless this runs
		//Update_State = true;
	}
	else if( (!(GPIO_PORTF_RIS_R & 0x10)) && (input != 1)) 	//sw 2 (RIGHT SWITCH)
	{
		input = 1;
		//Update_State = true;
	}
	GPIO_PORTF_ICR_R |= 0xFF;
}

void GPIOPortA_Handler(){
	Update_State = true;
	input = 2;
	GPIO_PORTA_ICR_R |= 0xFF;
}

enum Command{
	Forward,
	Backward,
	Left,
	Right,
	Idle,
	Continuous
};

struct State{
	int Function_Calls;		//number of times movement function needs to be called
	enum Command Movement;	
	bool wait;		//whether or not it waits for interrupt
	int next[4];	// forward sw pressed, backwards sw pressed, Ir sensor, finish run
};

typedef const struct State StateType;

StateType FSM[9]={      
	{0, Idle  , true , {1, 6, 0, 0}}, 	// idle
	{7, Forward , false, {1, 1, 1, 2}}, 	//move forward 720 then turn left
	{1, Left , false, {2, 2, 2, 3}},		//turn left		takes 5 90 deg turns to turn vehicle 90
	{69420, Continuous , false, {3, 3, 4, 3}},		// move forward until obstacle
	{0, Idle , true , {4, 4, 5, 4}},		// wait for obstacle
	{4, Forward, false, {5, 5, 5, 0}},		// forward 360
	{4, Backward, false, {6, 6, 6, 7}},		//reverse 360
	{1, Right, false, {7, 7, 7, 8}},		//turn right
	{7, Forward, false, {8, 8, 8, 0}}		//move forward 720 then idle
};


int main(void){
	SysTick_Init();
	Stepper_Init();
	Buttons_Init();
	Sensor_Init();
	Index = 0;
	input = 0;
	while(1){
		
		if(Update_State)
		{
			Index = FSM[Index].next[input];
			Update_State = false;
		}

		if(FSM[Index].Movement == Forward)
		{
			input = 3;
			
			for(int q = 0; q < FSM[Index].Function_Calls; q++)
				{
					Move_Forward();
				}
			Update_State = true;
		}
		
		else if(FSM[Index].Movement == Backward)
		{
			input = 3;
			for(int i = 0; i < FSM[Index].Function_Calls; i++)
			{
				Move_Backward();
			}
			Update_State = true;
		}
		
		if(FSM[Index].Movement == Left)
		{
			Turn_Left();
			Update_State = true;
			input = 3;
		}
		
		if(FSM[Index].Movement == Right)
		{
			Turn_Right();
			Update_State = true;
			input = 3;
		}
		
		else if(FSM[Index].Movement == Continuous)
		{
			Move_Forward();
		}
		
		else if(FSM[Index].Movement == Idle)
		{
			//WaitForInterrupt();
		}
}
}


void Buttons_Init() {
	
	SYSCTL_RCGC2_R |= 0x00000020;		//activate port F clock
	
	//enable port f interrupts
	NVIC_EN0_R |= 0x40000000;

	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // Unlock PF  
	
	GPIO_PORTF_CR_R |= 0x1F;

	GPIO_PORTF_DIR_R &= ~0x11; 	//PF0 (BTN 2), 4 btn1 is an input
	GPIO_PORTF_AFSEL_R &= ~0x11;	//no alternate select function on PF0,4
	GPIO_PORTF_DEN_R |= 0x11;			//Digital enable for PF0,4
	GPIO_PORTF_PCTL_R &= ~0x0000001F;	//PF0,4 as GPIO
	GPIO_PORTF_AMSEL_R &= ~0x11;		//disable analog on PF0,4
	GPIO_PORTF_PUR_R |= 0x11;				//enable pull up resistors for PF0,4
	
	GPIO_PORTF_IS_R &= ~0x11; 			// PF0,4 is edge sensitive
	GPIO_PORTF_IBE_R &= ~0x11;			//PF0,4 not both edges
	GPIO_PORTF_IEV_R &= 0x00; 		//PF0 is a falling edge event
	GPIO_PORTF_ICR_R |= 0x11;					//clear flag 0,4
	GPIO_PORTF_IM_R |= 0x11;					//arm interrupt on PF0.4
	NVIC_PRI7_R = NVIC_PRI7_R | (0x00000001 <<21);		//priority number set to 1
	EnableInterrupts();
}

void Sensor_Init(void){		//port A
	SYSCTL_RCGC2_R |= 0x01; // 1) activate port D
  GPIO_PORTA_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
  GPIO_PORTA_PCTL_R &= ~0x0000FFFF; // 4) GPIO configure PA as GPIO
  GPIO_PORTA_DIR_R &= 0x00;   // 5) make PA3-0 in
  GPIO_PORTA_AFSEL_R &= ~0x0F;// 6) disable alt funct on PA3-0
  //GPIO_PORTD_DR8R_R |= 0x0F;  // enable 8 mA drive
  GPIO_PORTA_DEN_R |= 0x0F;   // 7) enable digital I/O on PA3-0 
	
	NVIC_EN0_R |= 0x01;
	GPIO_PORTA_IS_R &= ~0x14; 			// PA0,4 is edge sensitive
	GPIO_PORTA_IBE_R |= 0x1F;			//PA0,4 is both edges
	//GPIO_PORTD_IEV_R &= 0x0F; 		//PA0 is a rising edge event
	GPIO_PORTA_ICR_R |= 0xFF;					//clear flag 0,4
	GPIO_PORTA_IM_R |= 0x0F;					//arm interrupt on PA0.4
	NVIC_PRI0_R = NVIC_PRI0_R | (0x00000001 << 5);		//priority number set to 2

}

void Move_Forward(void){
	for (int i = 0; i < 550; ++i)	//Move forwards 90
					{			
						Stepper_L_CCW(4*T1ms);   // output every 10ms, frequency for the stepper motor is 100Hz.
						Stepper_R_CW(4*T1ms); 
						if(Update_State)
						{
							break;
						}
					}
}

void Move_Backward(void){
	for (int i = 0; i < 550; ++i)	//Move forwards 90
					{			
						Stepper_L_CW(4*T1ms);   // output every 10ms, frequency for the stepper motor is 100Hz.
						Stepper_R_CCW(4*T1ms); 
					}
}

void Turn_Left(void){
	for (int w = 0; w < 2750; ++w)	//Move forwards 90
					{			
						Stepper_L_CCW(4*T1ms); 
					}
}

void Turn_Right(void){
	for (int z = 0; z < 2750; ++z)	//Move forwards 90
					{			
						Stepper_L_CW(4*T1ms);   // output every 10ms, frequency for the stepper motor is 100Hz.
					}
}
