// Stepper.c
// Runs on TM4C123
// Provide functions that step the motor once clockwise, step
// once counterclockwise, and initialize the stepper motor
// interface.
// Daniel Valvano
// September 12, 2013
// Modified by Dr. Min He April 28, 2017

// PD3 connected to driver for stepper motor coil A
// PD2 connected to driver for stepper motor coil A'
// PD1 connected to driver for stepper motor coil B
// PD0 connected to driver for stepper motor coil B'

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "systick.h"

struct State{
  uint8_t Out;     // Output
  uint8_t Next[2]; // CW/CCW Next[0]->CW, Next[1]->CCW
};
typedef const struct State StateType;

#define CLOCKWISE 0        // Next index
#define COUNTERCLOCKWISE 1 // Next index

StateType fsm[4]={
	// index 0: state 0,state goes from 0 to 3,  output 1100,
	// if next state index is 0: move clockwise, next state for clockwise movement is 1
	// CW state transition is: 0->1->2->3 then repeat
	// CCW state transition is: 0->3->2->1 then repeat
  {12,{1,3}}, // state 0, PD3-0:1100. CW next state->1, CCW next state->3
  { 6,{2,0}}, // state 1, PD3-0:0110
  { 3,{3,1}}, // state 2, PD3-0:0011
  { 9,{0,2}}  // state 3, PD3-0:1001
};
unsigned char s; // current state

#define STEPPER  (*((volatile uint32_t *)0x4000703C))  // PORT D, pin: 0,1,2,3
	
// Move 1.8 degrees clockwise, delay is the time to wait after each step
void Stepper_CW(uint32_t delay){
  s = fsm[s].Next[CLOCKWISE]; // clock wise circular
  STEPPER = fsm[s].Out; // step motor
  SysTick_Wait(delay);
}
// Move 1.8 degrees counterclockwise, delay is wait after each step
void Stepper_CCW(uint32_t delay){
  s = fsm[s].Next[COUNTERCLOCKWISE]; // counter clock wise circular
  STEPPER = fsm[s].Out; // step motor
  SysTick_Wait(delay); // blind-cycle wait
}
// Initialize Stepper interface
void Stepper_Init(void){
//  SYSCTL_RCGCGPIO_R |= 0x08; // 1) activate port D
  SYSCTL_RCGC2_R |= 0x08; // 1) activate port D
  SysTick_Init();
  s = 0; 
                                    // 2) no need to unlock PD3-0
  GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
  GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // 4) GPIO configure PD3-0 as GPIO
  GPIO_PORTD_DIR_R |= 0x0F;   // 5) make PD3-0 out
  GPIO_PORTD_AFSEL_R &= ~0x0F;// 6) disable alt funct on PD3-0
  GPIO_PORTD_DR8R_R |= 0x0F;  // enable 8 mA drive
  GPIO_PORTD_DEN_R |= 0x0F;   // 7) enable digital I/O on PD3-0 
}
