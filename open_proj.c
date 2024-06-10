#include <stdbool.h>
#include <stdint.h>
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "i2c.h"
#include "interrupt.h"
#include "pin_map.h"
#include "string.h"
#include "sysctl.h"
#include "systick.h"
#include "uart.h"
#include "pwm.h"
//#include "tm4c1294ncpdt.h"


#define SYSTICK_FREQUENCY 1000// 1000hz

#define I2C_FLASHTIME 500  // 500mS
#define GPIO_FLASHTIME 300 // 300mS

#define PWM_PERIOD					4000
#define HAL_MAX_DELAY      		0xFFFFFFFFU
#define MAX_RECE_BUFF					16
//*****************************************************************************
//
// I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 0x22
#define PCA9557_I2CADDR 0x18

#define PCA9557_INPUT 0x00
#define PCA9557_OUTPUT 0x01
#define PCA9557_POLINVERT 0x02
#define PCA9557_CONFIG 0x03

#define TCA6424_CONFIG_PORT0 0x0c
#define TCA6424_CONFIG_PORT1 0x0d
#define TCA6424_CONFIG_PORT2 0x0e

#define TCA6424_INPUT_PORT0 0x00
#define TCA6424_INPUT_PORT1 0x01
#define TCA6424_INPUT_PORT2 0x02

#define TCA6424_OUTPUT_PORT0 0x04
#define TCA6424_OUTPUT_PORT1 0x05
#define TCA6424_OUTPUT_PORT2 0x06

#define normal 0
#define timeset 1
#define alarmset 2
#define stopwatch 3

#define SW1 0xFE
#define SW2 0xFD
#define SW3 0xFB
#define SW7 0xBF
#define SW8 0x7F



struct clock{
	uint32_t hour,minute,second;
};


//----------------------------------
// 蜂鸣器部分
//----------------------------------








//----------------------------------


void Delay(uint32_t value);
void S800_GPIO_Init(void);
uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void S800_I2C0_Init(void);
void S800_UART_Init(void);


// 10ms计数和标志
volatile uint16_t systick_10ms_couter;
volatile uint8_t systick_10ms_status;


// 100ms计数和标志
volatile uint16_t systick_100ms_couter;
volatile uint8_t systick_100ms_status;


// 1s计数和标志
volatile uint16_t systick_1s_couter;
volatile uint8_t systick_1s_status;

// 数码管显示计数和标志
volatile uint16_t digiseg_display_couter;
volatile uint8_t digiseg_display_status;


volatile uint8_t mode_cur = 0; //当前模式，0-普通，1-时间设置，2-闹钟设置，3-秒表


volatile uint16_t SWscan_couter=0;

volatile uint8_t SW1_cur,SW1_last; // 8个按键状态
volatile uint8_t SW1_state=0; // 按键按下状态。0-没按下，1-按下
volatile uint16_t SW1_couter=0;
volatile uint8_t SW1_status=0;// 按键触发标志

volatile uint8_t SW2_cur,SW2_last; // 8个按键状态
volatile uint8_t SW2_state=0; // 按键按下状态。0-没按下，1-按下
volatile uint16_t SW2_couter=0;
volatile uint8_t SW2_status=0;// 按键触发标志


volatile uint8_t flash_state = 0; //闪烁状态,0不闪，1，2，3分别对应秒分时
volatile bool is_flash_state =0 ; // 是否闪烁标志
// 闪烁状态计数和标志
volatile uint16_t flash_couter;
volatile uint8_t flash_status;

volatile uint8_t result, cnt, key_value, gpio_status;
volatile uint8_t rightshift = 0x01;
uint32_t ui32SysClock;

uint8_t is_alarm_set = 0; // 闹钟设定标志
uint8_t is_alarm_arrive =0; //闹钟到达标志
uint8_t alarm_count =0;



//----------------------------
//秒表相关
uint8_t is_stopwatch_run=0;// 当前是否正在计时


void ClockAdd_stopwatch(struct clock *myclock,uint32_t add_time);



uint8_t seg7[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d,  0x7d, 0x07, 0x7f,
                  0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x079, 0x71, 0x5c};

uint8_t uart_receive_char;

unsigned char RxBuf[256];            // 命令字符串缓冲区

char time_str[13]; // 此时时间字符串

									

struct clock my_clock={0,0,0};
struct clock alarm_clock={0,0,0};
struct clock watch_clock={0,0,0};

void GetTime(struct clock *clock,char *time);
void ClockAdd(struct clock *my_clock,uint32_t add_time);

void DigSegDisplay(struct clock * clk,bool is_flash,uint8_t flahs_mode);


void UARTStringGetNonBlocking();
void UARTStringPutNonBlocking(const char *cMessage);

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

uint16_t beep_count=0;


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
int main(void) {

	
    volatile uint16_t i2c_flash_cnt, gpio_flash_cnt;
    // use internal 16M oscillator, PIOSC
    // ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT
    // |SYSCTL_USE_OSC), 16000000); ui32SysClock =
    // SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC),
    // 8000000); use external 25M oscillator, MOSC
    // ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN
    // |SYSCTL_USE_OSC), 25000000);

    // use external 25M oscillator and PLL to 120M
    // ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |
    // SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);;
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ | SYSCTL_OSC_INT |
                                       SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                      20000000);

    SysTickPeriodSet(ui32SysClock / SYSTICK_FREQUENCY);
    SysTickEnable();
    SysTickIntEnable(); // Enable Systick interrupt

    S800_GPIO_Init();
    S800_I2C0_Init();
    S800_UART_Init();

    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE,
                  UART_INT_RX | UART_INT_RT); // Enable UART0 RX,TX interrupt
									
	
    IntMasterEnable();
	
    while (1) {
					
				if (systick_1s_status) 
				{
					ClockAdd(&my_clock,1);
					//GetTime(&my_clock,time_str);
					//UARTStringPutNonBlocking(time_str);
					systick_1s_status=0;
					//SW_cur = I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0);
		//UARTCharPutNonBlocking(UART0_BASE,SW_cur);
				}
				if (flash_status) 
						{
							is_flash_state = !is_flash_state;
							if (is_alarm_arrive==1) {
							alarm_count++;
							if (alarm_count%2) { 
							PWM_Init(ui32SysClock/CTune[15]);
								result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT,0xFE);
							}
							else{
								PWM_Init(0);
								result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT,0xFF);
							}
								if (alarm_count==10){is_alarm_arrive=0;alarm_count=0;}
							}
							flash_status=0;
						}
				if (is_alarm_set) {
					result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT,0xFE);
					if (my_clock.hour == alarm_clock.hour && my_clock.minute == alarm_clock.minute && my_clock.second == 0) {
						is_alarm_arrive =1;
						is_alarm_set=0;
						UARTStringPutNonBlocking("alarm arrive!!\r\n");
						
					}
					}
				
				switch (mode_cur){
					case normal: { 
						if (digiseg_display_status) 
						{
								DigSegDisplay(&my_clock,0,0);
								digiseg_display_status=0;
						}
						GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_0);
						break;
					}
					case timeset:{
						
						if (digiseg_display_status) 
						{
								DigSegDisplay(&my_clock,is_flash_state,flash_state);
								digiseg_display_status=0;
						}
						GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1);	
						break;
					}
					case alarmset:{
						
						if (digiseg_display_status) 
						{
								DigSegDisplay(&alarm_clock,is_flash_state,flash_state);
								digiseg_display_status=0;
						}
						GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_2);	
						break;
					}
					case stopwatch:{
						if (systick_10ms_status && is_stopwatch_run) 
						{
							ClockAdd_stopwatch(&watch_clock,1);
							systick_10ms_status=0;
						}
						if (digiseg_display_status) 
						{
								DigSegDisplay(&watch_clock,0,0);
								digiseg_display_status=0;
						}
						
						GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_3);
						break;
					}
				
			}
				
			
		
			
    }
}

void SW1_scan()
{
	SW1_cur =GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0);
	
	if (SW1_cur!=SW1_last&& SW1_cur==0 && SW1_state ==0) {SW1_state =1;SW1_couter=0;}
 	else if (SW1_state==1 && SW1_cur == 0 && SW1_cur==SW1_last ){
		SW1_couter ++;
		if (SW1_couter == 20) {SW1_state=2;SW1_couter=0;}
	}
	else if (SW1_state==2 && SW1_cur == 1 && SW1_cur!=SW1_last) {SW1_state=0;SW1_status = 1;}
	else if (SW1_state==2 && SW1_cur == 0 ) {
		SW1_couter++;
		if (SW1_couter>=1500) {
		SW1_state=0;
		SW1_status = 2;
		}
	}
	if (SW1_status == 1 && SW1_cur ==1 ){
		
						switch(mode_cur){
							case normal:{mode_cur=timeset;flash_state=3;beep_count=80;break;}
							case timeset:{
								flash_state--;
								if (flash_state == 0) {flash_state=3;mode_cur=normal;}
								beep_count=80;
								break;
							}
							case alarmset:{
								flash_state--;
								if (flash_state == 1) {flash_state=3;mode_cur=normal;is_alarm_set = 1;}
								beep_count=80;
								break;
							}
							case stopwatch:{
								if (!is_stopwatch_run)
								{
									is_stopwatch_run=!is_stopwatch_run;
									UARTStringPutNonBlocking("Start Timing!\r\n");
									beep_count=80;
								}
								else {
									is_stopwatch_run=!is_stopwatch_run;
									GetTime(&watch_clock,time_str);
									UARTStringPutNonBlocking("WATCH");
									UARTStringPutNonBlocking(time_str);
									beep_count=80;
								}
								break;
							}
							
						}
					
					SW1_status = 0;
					SW1_state =0;
				}
	if (SW1_status == 2 && SW1_cur ==0 ){
		
						switch(mode_cur){
							case normal:{mode_cur=stopwatch;is_stopwatch_run=0;beep_count=80;break;}
							case timeset:break;
							case alarmset:break;
							case stopwatch:{
							mode_cur=normal;
							is_stopwatch_run=0;
							watch_clock.hour=0;
							watch_clock.minute=0;
							watch_clock.second=0;
								beep_count=80;
							break;}
						}
					
					SW1_status = 0;
					SW1_state =0;
				}

			
	
			
	SW1_last = SW1_cur;

				
}

void SW2_scan()
{
	SW2_cur =GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1);
			if (SW2_cur!=SW2_last&& SW2_cur==0 && SW2_state ==0) {SW2_state =1;SW2_couter=0;}
	else if (SW2_state==1 && SW2_cur == 0 && SW2_cur==SW2_last ){
		SW2_couter ++;
		if (SW2_couter == 20) {SW2_state=2;SW2_couter=0;}
	}
	else if (SW2_state==2 && SW2_cur == 2 && SW2_cur!=SW2_last) {SW2_state=0;SW2_status = 1;}
		
	if (SW2_status == 1 && SW2_cur ==2){
		
						switch(mode_cur){
							case normal:{
								mode_cur=alarmset;
								if (is_alarm_set==0) {
								alarm_clock.hour=my_clock.hour;
								alarm_clock.minute=my_clock.minute;
								}
								flash_state=3;
								beep_count=80;
								break;
							}
							case timeset:{
								switch(flash_state){
									case 3: {ClockAdd(&my_clock,3600);beep_count=80;break;}
									case 2: {ClockAdd(&my_clock,60);beep_count=80;break;}
									case 1: {ClockAdd(&my_clock,1);beep_count=80;break;}
			
								break;
							}
						}
							case alarmset:{
								switch (flash_state) {
									case 3:{ClockAdd(&alarm_clock,3600);beep_count=80;break;}
									case 2:{ClockAdd(&alarm_clock,60);beep_count=80;break;}
								}
								break;
							}
							case stopwatch:{
								if (is_stopwatch_run){
								GetTime(&watch_clock,time_str);
								UARTStringPutNonBlocking("WATCH");
								UARTStringPutNonBlocking(time_str);
									beep_count=80;
								} 
								else if ((watch_clock.hour!=0 || watch_clock.minute!=0 || watch_clock.second!=0)) {
									watch_clock.hour=0;
									watch_clock.minute=0;
									watch_clock.second=0;
									beep_count=80;
								}
								break;
							}
					
				}
						SW2_status = 0;
					SW2_state =0;
			}
	
		
			
		SW2_last = SW2_cur;
}



void DigSegDisplay(struct clock * clk,bool is_flash,uint8_t flahs_mode)
{
	static uint8_t i=0;
	uint8_t seg_array[8];
	seg_array[0]=(is_flash&&(flahs_mode == 1))? 0x00:seg7[(clk->second)%10];
	seg_array[1]=(is_flash&&(flahs_mode == 1))? 0x00:seg7[(clk->second)/10];
	seg_array[2]=0x40;
	seg_array[3]=(is_flash&&(flahs_mode == 2))? 0x00:seg7[(clk->minute)%10];
	seg_array[4]=(is_flash&&(flahs_mode == 2))? 0x00:seg7[(clk->minute)/10];
	seg_array[5]=0x40;
	seg_array[6]=(is_flash&&(flahs_mode == 3))? 0x00:seg7[(clk->hour)%10];
	seg_array[7]=(is_flash&&(flahs_mode == 3))? 0x00:seg7[(clk->hour)/10];
	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg_array[i]);										
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0x80 >> i));
		i++;
		if (i==8) i=0;
	
}

void GetTime(struct clock *clock,char *time)
{
	time[0] = 'T';
	time[1] = 'I';
	time[2] = 'M';
	time[3] = 'E';
	time[4] = '0'+(clock->hour)/10;
	time[5] = '0'+(clock->hour)%10;
	time[6] = ':';
	time[7] = '0'+(clock->minute)/10;
	time[8] = '0'+(clock->minute)%10;
	time[9] = ':';
	time[10] = '0'+(clock->second)/10;
	time[11] = '0'+(clock->second)%10;
	time[12] = '\n';

	
}

void ClockAdd(struct clock *myclock,uint32_t add_time)
{
		
				
					myclock->second+=add_time;
					if (myclock->second>=60) 
					{
						myclock->minute+=myclock->second/60;
						myclock->second%=60;
						
					}
					if (myclock->minute>=60)
						{
							myclock->hour+=myclock->minute/60;
						myclock->minute%=60;
						}
						if (myclock->hour>=24)
							{
								myclock->hour%=24;
							}
				
}

void ClockAdd_stopwatch(struct clock *myclock,uint32_t add_time)
{
		
				
					myclock->second+=add_time;
					if (myclock->second>=100) 
					{
						myclock->minute+=myclock->second/100;
						myclock->second%=100;
						
					}
					if (myclock->minute>=60)
						{
							myclock->hour+=myclock->minute/60;
						myclock->minute%=60;
						}
						if (myclock->hour>=24)
							{
								myclock->hour%=24;
							}
				
}

void Delay(uint32_t value) {
    uint32_t ui32Loop;
    for (ui32Loop = 0; ui32Loop < value; ui32Loop++) {
    };
}
void UARTStringPut(uint8_t *cMessage) {
    while (*cMessage != '\0')
        UARTCharPut(UART0_BASE, *(cMessage++));
}
void UARTStringPutNonBlocking(const char *cMessage) {
    while (*cMessage != '\0') {
        if (UARTSpaceAvail(UART0_BASE)) // 发送FIFO有空位
            UARTCharPutNonBlocking(UART0_BASE, *(cMessage++));
    }
}

void UARTStringGetNonBlocking() {
    uint8_t cnt = 0;
    while (UARTCharsAvail(UART0_BASE)) {
        RxBuf[cnt++] = UARTCharGetNonBlocking(UART0_BASE);
    }
    RxBuf[cnt] = '\0';
    cnt = 0;
   
}
// void UARTStringGetNonBlocking(char *msg)
//{
//    while( UARTCharsAvail(UART0_BASE) )
//	 {
//      *msg++ = UARTCharGetNonBlocking(UART0_BASE);
//    }
//    *msg='\0';
// }

void S800_UART_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable PortA
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
        ; // Wait for the GPIO moduleA ready

    GPIOPinConfigure(GPIO_PA0_U0RX); // Set GPIO A0 and A1 as UART pins.
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(
        UART0_BASE, ui32SysClock, 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTStringPut((uint8_t *)"\r\nHello, world!\r\n");
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX2_8, UART_FIFO_RX7_8);
}
void S800_GPIO_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable PortF
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
        ; // Wait for the GPIO moduleF ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Enable PortJ
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
        ; // Wait for the GPIO moduleJ ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Enable PortN
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
        ; // Wait for the GPIO moduleN ready

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); // Set PF0 as Output pin
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); // Set PN0 as Output pin
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1); // Set PN1 as Output pin

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,
                         GPIO_PIN_0 |
                             GPIO_PIN_1); // Set the PJ0,PJ1 as input pin
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void S800_I2C0_Init(void) {
    uint8_t result;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, true); // config I2C0 400k
    I2CMasterEnable(I2C0_BASE);
	
	

    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT0,
                            0x0ff); // config port 0 as input
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT1,
                            0x0); // config port 1 as output
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT2,
                            0x0); // config port 2 as output

    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_CONFIG,
                            0x00); // config port as output
    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT,
                            0x0ff); // turn off the LED1-8
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData) {
    uint8_t rop;
    while (I2CMasterBusy(I2C0_BASE)) {
    };
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    I2CMasterDataPut(I2C0_BASE, RegAddr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {
    };
    rop = (uint8_t)I2CMasterErr(I2C0_BASE);

    I2CMasterDataPut(I2C0_BASE, WriteData);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE)) {
    };

    rop = (uint8_t)I2CMasterErr(I2C0_BASE);
    return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr) {
    uint8_t value, rop;
    while (I2CMasterBusy(I2C0_BASE)) {
    };
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    I2CMasterDataPut(I2C0_BASE, RegAddr);
    //	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    rop = (uint8_t)I2CMasterErr(I2C0_BASE);
    Delay(1);
    // receive data
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    value = I2CMasterDataGet(I2C0_BASE);
    Delay(1);
    return value;
}





void SysTick_Handler(void) {
	
	if (systick_1s_couter != 0)
        systick_1s_couter--;
    else {
        systick_1s_couter = SYSTICK_FREQUENCY ;
        systick_1s_status = 1;
    }
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
		
	
    if (digiseg_display_couter != 0)
        digiseg_display_couter--;
    else {
        digiseg_display_couter = 1 ;
        digiseg_display_status = 1;
    }
		
		if (mode_cur == 1 || mode_cur == 2 || is_alarm_arrive) {
		if (flash_couter != 0)
        flash_couter--;
    else {
        flash_couter = SYSTICK_FREQUENCY/3 ;
        flash_status = 1;
    }
		}
	

		SW1_scan();
		SW2_scan();
		
		if (beep_count!=0) {
			PWM_Init(ui32SysClock/CTune[15]);
			beep_count--;
			if (beep_count==0) {PWM_Init(0);}
		}
		
		
	
}

void UART0_Handler(void) {
    int32_t uart0_int_status;
    uart0_int_status =
        UARTIntStatus(UART0_BASE, true); // Get the interrrupt status.

    UARTIntClear(UART0_BASE, uart0_int_status); // Clear the asserted interrupts

    UARTStringGetNonBlocking(); // 获取字符串
    // UARTStringPutNonBlocking(RxBuf);
		if (strncasecmp("SET",RxBuf,3)==0) 
		{
			my_clock.hour=(RxBuf[3]-'0')*10+(RxBuf[4]-'0');
			my_clock.minute=(RxBuf[6]-'0')*10+(RxBuf[7]-'0');
			my_clock.second=(RxBuf[9]-'0')*10+(RxBuf[10]-'0');
			GetTime(&my_clock,time_str);
			UARTStringPutNonBlocking(time_str);
			
		}
		if (strncasecmp("INC",RxBuf,3)==0) 
		{
			//my_clock.hour+=(RxBuf[3]-'0')*10+(RxBuf[4]-'0');
			//my_clock.minute+=(RxBuf[6]-'0')*10+(RxBuf[7]-'0');
			//my_clock.second+=(RxBuf[9]-'0')*10+(RxBuf[10]-'0');
			ClockAdd(&my_clock,(((RxBuf[3]-'0')*10+(RxBuf[4]-'0'))*3600+((RxBuf[6]-'0')*10+(RxBuf[7]-'0'))*60+((RxBuf[9]-'0')*10+(RxBuf[10]-'0'))));
			//ClockAdd(&my_clock,0);
			GetTime(&my_clock,time_str);
			UARTStringPutNonBlocking(time_str);
		}
		if (strncasecmp("ALARM",RxBuf,5)==0) 
		{
			alarm_clock.hour=(RxBuf[5]-'0')*10+(RxBuf[6]-'0');
			alarm_clock.minute=(RxBuf[8]-'0')*10+(RxBuf[9]-'0');
			is_alarm_set=1;
			GetTime(&alarm_clock,time_str);
			UARTStringPutNonBlocking("ALARM");
			UARTStringPutNonBlocking(time_str);
		}
		if (strcasecmp("GETTIME",RxBuf)==0)
		{
			GetTime(&my_clock,time_str);
			UARTStringPutNonBlocking(time_str);
		}
   
}
