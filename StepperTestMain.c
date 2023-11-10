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


#define T1ms 16000    // assumes using 16 MHz Clock 

bool Direction;
bool Move;

// Function Prototypes (from startup.s)
void DisableInterrupts(); // Disable interrupts
void EnableInterrupts();  // Enable interrupts
void WaitForInterrupt();  // Go to low power mode while waiting for the next interrupt

void Buttons_Init();
void GPIOPortF_Handler(); // Handle GPIO Port F interrupts



int main(void){
	unsigned int i=0;
	Direction = true;
	Move = true;
	SysTick_Init();
  Stepper_Init();
	Buttons_Init();
  while(1){
		
		
		//turn clockwise 180 degrees:0.18 degree for each step
		for (i=0;i<1000; i++) {
      Stepper_CW(10*T1ms);   // output every 10ms, frequency for the stepper motor is 100Hz.
		}		
	  //SysTick_Wait(500*T1ms);  // wait for 0.5s
		
		// turn counter clockwise 180 degrees
		for (i=0;i<1000; i++) {
      Stepper_CCW(10*T1ms);   // output every 10ms
		}
	  //SysTick_Wait(500*T1ms);  // wait for 0.5s
  
	} 
}

void Buttons_Init() {
	
	SYSCTL_RCGC2_R |= 0x00000020;		//activate port F clock
	
	NVIC_EN1_R |= 0x00004000;		//enable port f interrupts

	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // Unlock PF  

	GPIO_PORTF_DIR_R &= ~0x11; 	//PF0 (BTN 2), 4 btn1 is an input
	//GPIO_PORTF_AFSEL_R &= ~0x11;	//no alternate select function on PF0,4
	GPIO_PORTF_DEN_R |= 0x11;			//Digital enable for PF0,4
	//GPIO_PORTF_PCTL_R &= ~0x0000001F;	//PF0,4 as GPIO
	GPIO_PORTF_AMSEL_R &= ~0x11;		//disable analog on PF0,4
	GPIO_PORTF_PUR_R |= 0x11;				//enable pull up resistors for PF0,4
	GPIO_PORTF_IS_R &= ~0x11; 			// PF0,4 is edge sensitive
	GPIO_PORTF_IBE_R &= ~0x11;			//PF0,4 not both edges
	GPIO_PORTF_IEV_R &= 0x00; 		//PF0 is a falling edge event
	GPIO_PORTF_ICR_R |= 0x11;					//clear flag 0,4
	GPIO_PORTF_IM_R |= 0x11;					//arm interrupt on PF0.4
	NVIC_PRI7_R = NVIC_PRI7_R | (0x00000001 <<14);		//priority number set to 1
	EnableInterrupts();
}


void GPIOPortF_Handler() {
	if( (GPIO_PORTF_RIS_R & 0x01) != 0x00){ 	// if PF0 BTN 2 is pressed
		Direction = true;		/// true = clockwise, false = counter cw
		Move = true;
		GPIO_PORTF_ICR_R = 0x01;
	}
	
	if( (GPIO_PORTF_RIS_R & 0x10) != 0x00){ 	// if PF4 BTN 1 is pressed
		Direction = false;		/// true = clockwise, false = counter cw
		Move = true;
		GPIO_PORTF_ICR_R = 0x10;
	}
	
	//GPIO_PORTF_ICR_R = 0x11; //clear flags
	
}