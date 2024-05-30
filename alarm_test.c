
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "hw_ints.h"
#include "pwm.h"








#define HAL_MAX_DELAY      		0xFFFFFFFFU
#define MAX_RECE_BUFF					16
#define SYSTICK_FREQUENCY		1000			//1000hz

#define	I2C_FLASHTIME				500				//500mS
#define GPIO_FLASHTIME			300				//300mS

#define PWM_PERIOD					4000
//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT							0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06
//systick function for HAL_Delay use
void     HAL_IncTick(void);
void     HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);

void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
void 		S800_UART_Init(void);
void 		S800_Uart_Echo(char * p);
//systick software counter define
volatile uint32_t uwTick;
//uart receive buffer define and pointer define
char rece_buff[MAX_RECE_BUFF];
uint16_t	rece_buff_pos;
const uint16_t CTune[] =
{
    1,      //0
    262,    //1.Do    ---Low
    294,    //2.Re
    330,    //3.Mi
    349,    //4.Fa
    392,    //5.So
    440,    //6.La
    494,    //7.Si
    0,      //8
    0,      //9
    0,      //10
    523,    //11.Do    ---Middle
    587,    //12.Re
    659,    //13.Mi
    698,    //14.Fa
    784,    //15.So
    880,    //16.La
    988,    //17.Si
    0,      //18
    0,      //19
    0,      //20
    1046,   //21.Do    ---High
    1175,   //22.Re
    1318,   //23.Mi
    1397,   //24.Fa
    1568,   //25.So
    1760,   //26.La
    1976,   //27.Si
};
//Ò»ÉÁÒ»ÉÁÁÁ¾§¾§
const uint16_t MusicNote1[] =
{
1,0,1,0,5,0,5,0,6,0,6,0,5,0,
4,0,4,0,3,0,3,0,2,0,2,0,1,0,
5,0,5,0,4,0,4,0,3,0,3,0,2,0,
5,0,5,0,4,0,4,0,3,0,3,0,2,0,
1,0,1,0,5,0,5,0,6,0,6,0,5,0,
4,0,4,0,3,0,3,0,2,0,2,0,1,0,
};
const uint8_t  MusicBeat1[] =
{
3,1,3,1,3,1,3,1,3,1,3,1,7,1,
3,1,3,1,3,1,3,1,3,1,3,1,7,1,
3,1,3,1,3,1,3,1,3,1,3,1,7,1,
3,1,3,1,3,1,3,1,3,1,3,1,7,1,
3,1,3,1,3,1,3,1,3,1,3,1,7,1,
3,1,3,1,3,1,3,1,3,1,3,1,7,8,
};
//°®ÊÇÄãÎÒ
const uint16_t MusicNote3[] =
{
11,7,0,5,3,3,3,3,3,6,6,6,6,11,5,3,3,3,3,3,
11,7,0,5,6,7,6,6,0,6,2,2,2,2,2,1,6,2,3,3,
3,3,3,11,7,6,13,12,12,6,11,12,7,5,5,6,6,6,13,13,
11,6,13,12,12,12,12,12,6,6,7,12,7,7,7,0,11,6,6,6,
6,0,0,13,16,16,16,16,16,16,16,15,16,16,13,13,13,13,13,13,
13,13,16,16,16,16,16,16,15,16,15,15,15,15,15,15,15,16,13,12,
13,11,16,6,0,3,5,3,5,5,6,11,6,12,13,0,16,16,15,13,
12,13,15,13,13,13,13,0,12,11,12,12,13,11,6,5,6,12,12,12,
12,13,11,6,0,12,13,5,3,3,12,12,13,11,6,6,13,16,15,15,
15,15,15,15,15,12,13,5,3,0,11,6,6,6,6,6,12,13,5,3,
0,11,6,6,6,6,6,0,
};

const uint8_t  MusicBeat3[] =
{
4,4,1,2,2,2,4,4,4,2,2,4,2,2,2,2,2,2,2,2,
4,4,2,2,2,2,4,4,2,2,2,2,2,2,4,2,2,2,2,2,
4,4,4,4,4,2,4,4,4,2,4,2,4,4,2,4,2,4,2,2,
4,2,4,4,2,2,4,4,2,2,2,4,2,4,4,4,4,4,2,2,
4,4,4,4,2,4,2,2,4,4,4,4,4,4,4,2,2,4,4,4,
2,2,4,4,2,2,4,4,2,2,2,4,2,2,2,4,4,4,4,4,
4,2,2,4,2,2,2,2,4,2,2,4,4,4,2,2,4,4,4,4,
4,2,2,2,2,2,2,2,2,2,4,2,2,2,2,4,4,4,4,4,
2,2,4,4,4,2,2,4,2,2,4,2,2,2,2,4,4,2,2,2,
2,2,2,2,2,2,2,4,4,4,4,2,2,2,2,2,2,2,4,4,
4,4,4,4,4,4,4,8,
};
uint16_t i=0;
uint8_t  ex=0;
volatile uint16_t systick_10ms_couter,systick_100ms_couter;
volatile uint8_t	systick_10ms_status,systick_100ms_status;

volatile uint8_t result,cnt,key_value,gpio_status;
volatile uint8_t rightshift = 0x01;
uint32_t ui32SysClock,ui32IntPriorityGroup,ui32IntPriorityMask;
uint32_t ui32IntPrioritySystick,ui32IntPriorityUart0;

uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};

void PWM_Init(uint32_t period)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

  GPIOPinConfigure(GPIO_PK5_M0PWM7);

	GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);	
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3,period );	
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,
                     period/2);
	PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}
int main(void)
{
	volatile uint16_t	i2c_flash_cnt,gpio_flash_cnt;
	uint16_t beep_status;
	//use internal 16M oscillator, PIOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
	//ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 8000000);		
	//use external 25M oscillator, MOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_OSC), 25000000);		

	//use external 25M oscillator and PLL to 120M
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);;		
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000);
	
  SysTickPeriodSet(ui32SysClock/(SYSTICK_FREQUENCY));
	SysTickEnable();
	SysTickIntEnable();																		//Enable Systick interrupt
	  

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	
	IntEnable(INT_UART0);
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	//Enable UART0 RX,TX interrupt
  IntMasterEnable();		
	ui32IntPriorityMask				= IntPriorityMaskGet();
	IntPriorityGroupingSet(3);														//Set all priority to pre-emtption priority
	
	IntPrioritySet(INT_UART0,3);													//Set INT_UART0 to highest priority
	IntPrioritySet(FAULT_SYSTICK,0x0e0);									//Set INT_SYSTICK to lowest priority
	
	ui32IntPriorityGroup			= IntPriorityGroupingGet();

	ui32IntPriorityUart0			= IntPriorityGet(INT_UART0);
	ui32IntPrioritySystick		= IntPriorityGet(FAULT_SYSTICK);
	ex=MusicBeat1[i];
//	PWM_Init(uint16_t period);
	while (1)
	{
		S800_Uart_Echo(rece_buff);
		
		if (systick_10ms_status)
		{
//			if(--ex!=0)
//			  PWM_Init(ui32SysClock/CTune[MusicNote1[i]]);
//			else
//			{
//			  i=(i+1)%sizeof(MusicBeat1);
//				ex=MusicBeat1[i];
//			}
			
			systick_10ms_status		= 0;
			if (++gpio_flash_cnt	>= GPIO_FLASHTIME/10)
			{
				gpio_flash_cnt			= 0;
				if (gpio_status)
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_PIN_0 );
				else
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,0);
				gpio_status					= !gpio_status;
			
			}
		}
		if (systick_100ms_status)
		{
			systick_100ms_status	= 0;
//			PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, 0);
 //   	GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,GPIO_PIN_5 );
	//		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,0 );
///*			if (beep_status)
//					GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,GPIO_PIN_5 );
//				else
//					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5,~GPIO_PIN_5);
//				beep_status					= !beep_status;
//		
//*/	
//			
//			
//			for(i = 0; i < sizeof(MusicBeat1); i++)
//    {
//			PWM_Init(ui32SysClock/CTune[MusicNote1[i]]);
//			HAL_Delay(MusicBeat1[i] * 125);
//		}
//		for(i = 0; i < sizeof(MusicBeat3); i++)
//    {
//			PWM_Init(ui32SysClock/CTune[MusicNote3[i]]);
//			HAL_Delay(MusicBeat1[i] * 125);
//		}
			if(--ex!=0)
			  PWM_Init(ui32SysClock/CTune[MusicNote1[i]]);
			else
			{
			  i=(i+1)%sizeof(MusicBeat1);
				ex=MusicBeat1[i];
			}
			if (++i2c_flash_cnt		>= I2C_FLASHTIME/100)
			{
//				PWM_Init(notes [i]);
				i2c_flash_cnt				= 0;
				result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[cnt+1]);	//write port 1 				
				result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
		
				result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~rightshift);	

				cnt++;
				rightshift= rightshift<<1;

				if (cnt		  >= 0x8)
				{
					rightshift= 0x01;
					cnt 			= 0;
				}
			
			}
			
    }
		
	}



}




void UARTStringPut(char *cMessage)
{
	while((*cMessage!='\0') && UARTSpaceAvail)
		UARTCharPut(UART0_BASE,*(cMessage++));
}
void UARTStringPutNonBlocking(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPutNonBlocking(UART0_BASE,*(cMessage++));
}

void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTStringPut((uint8_t *)"\r\nHello, world!\r\n");
}
void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));			//Wait for the GPIO moduleN ready		
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);			//Set PN0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);			//Set PN1 as Output pin	

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	//TEST THE BEEPER PK5
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);						//Enable PortK
  GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);			//Set PK5 as Output pin
	GPIOPadConfigSet(GPIO_PORTK_BASE,GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);	
	
}

void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	HAL_Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
		HAL_Delay(1);
	return value;
}

/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/
void SysTick_Handler(void)
{
	HAL_IncTick();
	if (systick_100ms_couter	!= 0)
		systick_100ms_couter--;
	else
	{
		systick_100ms_couter	= SYSTICK_FREQUENCY/10;
		systick_100ms_status 	= 1;
	}
	
	if (systick_10ms_couter	!= 0)
		systick_10ms_couter--;
	else
	{
		systick_10ms_couter		= SYSTICK_FREQUENCY/100;
		systick_10ms_status 	= 1;
	}
	while (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0)
	{
		systick_100ms_status	= systick_10ms_status = 0;
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,GPIO_PIN_0);		
	}
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,0);		
}

/*
	Corresponding to the startup_TM4C129.s vector table UART0_Handler interrupt program name
*/
void UART0_Handler(void)
{
	int32_t uart0_int_status;
  uart0_int_status 		= UARTIntStatus(UART0_BASE, true);		// Get the interrrupt status.

  UARTIntClear(UART0_BASE, uart0_int_status);								//Clear the asserted interrupts

  while(UARTCharsAvail(UART0_BASE))    											// Loop while there are characters in the receive FIFO.
  {
		rece_buff[rece_buff_pos++]					= (char)UARTCharGetNonBlocking(UART0_BASE);
		if (rece_buff_pos										>= MAX_RECE_BUFF)
		{
			memset(rece_buff,0,sizeof(char)*MAX_RECE_BUFF);				//clear the buffer and reset the rece_buff_pos
			rece_buff_pos											= 0;
		}
	}
}

void S800_Uart_Echo(char * p)
{
	if (strchr(p,'\n')									!= NULL) 
	{
		UARTStringPut(p);
		memset(p,0,sizeof(char)*MAX_RECE_BUFF);
		rece_buff_pos											= 0;
	}
}
/*
		///Read the next character from the UART and write it back to the UART.
    UARTCharPutNonBlocking(UART0_BASE,UARTCharGetNonBlocking(UART0_BASE));
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,GPIO_PIN_1 );		
//		HAL_Delay(1000);



*/
/**
  * @brief  This function is called to increment  a global variable "uwTick"
  *         used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *         in SysTick ISR.
  * @note This function is declared as __weak to be overwritten in case of other 
  *         implementations  in user file.
  * @retval None
  */
__weak void HAL_IncTick(void)
{
  uwTick++;
}

/**
  * @brief  Povides a tick value in millisecond.
  * @note   The function is declared as __Weak  to be overwritten  in case of other 
  *         implementations  in user file.
  * @retval tick value
  */
__weak uint32_t HAL_GetTick(void)
{
  return uwTick;  
}

/**
  * @brief  This function provides accurate delay (in milliseconds) based 
  *         on variable incremented.
  * @note   In the default implementation , SysTick timer is the source of time base. 
  *         It is used to generate interrupts at regular time intervals where uwTick
  *         is incremented.
  *         The function is declared as __Weak  to be overwritten  in case of other
  *         implementations  in user file.
  * @param  Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
__weak void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;
  
  /* Add freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
    wait ++;
  
  while((HAL_GetTick() - tickstart) < wait)
  {
  }
}



