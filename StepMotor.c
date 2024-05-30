//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Blinky (blinky)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

//*****************************************************************************
//
// Control the step motor in 
//
//*****************************************************************************

uint32_t Clock, t , i  ,j ;
uint32_t SysTickPeriod,Period,Timer,t,ui32SysClock;
void Delay(uint32_t time);
int
	
//uint32_t t;

main(void)
{
    volatile uint32_t ui32Loop;
 
	
	//Clock=SysCtlClockFreqSet(SYSCTL_OSC_MAIN | SYSCTL_XTAL_25MHZ|SYSCTL_USE_OSC,25000000);
     Clock=SysCtlClockFreqSet(SYSCTL_OSC_INT| SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480,120000000);
//	  ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000);

 	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

 	 while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
     {
     }

	  	GPIO_PORTF_AHB_DIR_R=0xf;		//Config Port F bit 0,1,2,3 as output ,here AHB means F is an AHB bus control
		//  													also can use the diriverlib function as below line
	  //   GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	 
		 
	  GPIO_PORTF_AHB_DEN_R=0xf;// Enable  Port F bit 0,1,2,3 
    
    // Loop forever.
    
    while(1)
    {
   //*The stepping motor works in the counter clockwise state 
 
		 for( t=0;t<64*8;t++) 				//for 64*8=512 times,	the step motor run a circle 
			{		 
  //   GPIO_PORTF_AHB_DATA_R=1; 	 	//PF0=1,PF1=0,PF2=0,	PF3=0;
			/*
			also can use the drivelib function as below line 
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,0x01);
			*/
//		 Delay(300);
//		 GPIO_PORTF_AHB_DATA_R=3;			//PF0=1,PF1=1,PF2=0,	PF3=0;
//		 Delay(300);
//	//	 GPIO_PORTF_AHB_DATA_R=2;			//PF0=0,PF1=1,PF2=0,	PF3=0;
//		 Delay(300);
//		 GPIO_PORTF_AHB_DATA_R=6;			//PF0=0,PF1=1,PF2=1,	PF3=0;
//		 Delay(300);
//	//	 GPIO_PORTF_AHB_DATA_R=4;			//PF0=0,PF1=0,PF2=1,	PF3=0;
//		 Delay(300);
//		 GPIO_PORTF_AHB_DATA_R=0xc;		//PF0=0,PF1=0,PF2=1,	PF3=1;
//		 Delay(300);
//	//	 GPIO_PORTF_AHB_DATA_R=8;			//PF0=0,PF1=0,PF2=0,	PF3=1;
//		 Delay(300);
//	//	 GPIO_PORTF_AHB_DATA_R=9;			//PF0=1,PF1=0,PF2=0,	PF3=1;
//		 Delay(300);
						 Delay(300);
		 GPIO_PORTF_AHB_DATA_R=9;			//PF0=1,PF1=1,PF2=0,	PF3=0;
		 Delay(300);
		 GPIO_PORTF_AHB_DATA_R=8;			//PF0=0,PF1=1,PF2=0,	PF3=0;
		 Delay(300);
		 GPIO_PORTF_AHB_DATA_R=0xc;			//PF0=0,PF1=1,PF2=1,	PF3=0;
		 Delay(300);
		 GPIO_PORTF_AHB_DATA_R=4;			//PF0=0,PF1=0,PF2=1,	PF3=0;
		 Delay(300);
		 GPIO_PORTF_AHB_DATA_R=6;		//PF0=0,PF1=0,PF2=1,	PF3=1;
		 Delay(300);
		 GPIO_PORTF_AHB_DATA_R=2;			//PF0=0,PF1=0,PF2=0,	PF3=1;
		 Delay(300);
		 GPIO_PORTF_AHB_DATA_R=3;			//PF0=1,PF1=0,PF2=0,	PF3=1;
		 Delay(300);
			}
		 GPIO_PORTF_AHB_DATA_R=1;
     Delay(100000);		//Pause for a while 

    }
}

void Delay(uint32_t time)
{
	uint32_t i , j;
	
	for( i=0;i<time;i++)
	{
		for(j=0;j<1000;j++)
		{
			;
		}
			}
}
