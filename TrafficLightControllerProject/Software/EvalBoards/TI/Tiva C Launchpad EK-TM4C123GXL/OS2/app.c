#include "app_cfg.h"
#include <cpu_core.h>
#include <os.h>
#include "..\bsp\bsp.h"
#include "..\bsp\bsp_led.h"
#include "..\bsp\bsp_sys.h"
// SAR Addition
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
//#include <time.h>
#include <stdlib.h>
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"
/*
***************************************************************************************************
******
* LOCAL DEFINES
***************************************************************************************************
******
*/
#define SYSCTL 0x400FE108
#define SYSCTL_BASE           0x400FE000
#define SYSCTL_RCGC2_OFFSET   0x108
//#define SYSCTL_RCGC2_R        (*((volatile uint32_t *)(SYSCTL_BASE + SYSCTL_RCGC2_OFFSET)))

#define GPIO_PORTA_BASE      0x40004000
#define GPIO_PORTA_DATA      (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x3FC)))
#define GPIO_PORTA_DIR       (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x400)))
#define GPIO_PORTA_AFSEL     (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x420)))
#define GPIO_PORTA_DEN       (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x51C)))
#define GPIO_PORTA_PCTL      (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x52C)))

#define GPIO_PORTB_BASE      0x40005000
#define GPIO_PORTB_DATA      (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x3FC)))
#define GPIO_PORTB_DIR       (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x400)))
#define GPIO_PORTB_AFSEL     (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x420)))
#define GPIO_PORTB_DEN       (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x51C)))
#define GPIO_PORTB_PCTL      (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x52C)))

#define TASK_STK_SIZE 128
#define SENSOR_GPIO_PORT GPIO_PORTB_BASE
#define SENSOR_GPIO_PIN GPIO_PIN_4
#define LED_GPIO_PORT GPIO_PORTA_BASE
#define LED_GPIO_PIN GPIO_PIN_3
#define CAR_FLAG 1



#define SENSOR_MONITOR_PRIO  5   // Sensor monitor task priority
#define MAIN_ROAD_SEQ1_PRIO  6   // Main road sequence 1 task priority
#define MAIN_ROAD_SEQ2_PRIO  10  // Main road sequence 2 task priority
#define SIDE_ROAD_SEQ1_PRIO  7   // Side road sequence 1 task priority
#define SIDE_ROAD_SEQ2_PRIO  11  // Side road sequence 2 task priority

/*
***************************************************************************************************
******
* LOCAL CONSTANTS
***************************************************************************************************
******
*/

/*

***************************************************************************************************
******
* LOCAL DATA TYPES
***************************************************************************************************
******
*/

/*$PAGE*/
/*
***************************************************************************************************
******
* LOCAL GLOBAL VARIABLES
***************************************************************************************************
******
*/
static OS_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK Task1Stk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK Task2Stk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK sensorMonitor2Stk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK mainRoad2Stk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK sideRoad2Stk[APP_CFG_TASK_START_STK_SIZE];
OS_FLAG_GRP *carDetEvent;
OS_EVENT *UART_BINARY_SEMAPHORE;
OS_TCB blinkyTCB;
volatile bool sensorTriggered = false;
volatile bool runMainRoadSeq2 = false;
volatile bool runSideRoadSeq2 = false;
volatile bool sequence2Done = false;
int trafficTimeCounter;
/*
* LOCAL FUNCTION PROTOTYPES
***************************************************************************************************
******
*/
static void AppTaskCreate (void);
static void AppTaskStart (void *p_arg);
static void Task1 (void *p_arg);
static void Task2 (void *p_arg);

static void sensorMonitor(void *p_arg);
static void mainRoadLight_seq1(void *p_arg);
static void sideRoadLight_seq1(void *p_arg);
static void mainRoadLight_seq2(void *p_arg);
static void sideRoadLight_seq2(void *p_arg);

void Sensor_LEDs_Init(void);
void Interrupt_Init(void);
void Sensor_ISR(void);
void InitConsole(void);

// Define your GPIO ports and pins here
#define Red_LED_main GPIO_PIN_6
#define Green_LED_main GPIO_PIN_3
#define Yellow_LED_main GPIO_PIN_4

#define Red_LED_side GPIO_PIN_1
#define Green_LED_side GPIO_PIN_3
#define Yellow_LED_side GPIO_PIN_1 + GPIO_PIN_3



int main(void) {
	int i;
	int test = 1;
    // Set up system clock, GPIO, etc
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
		
    // Initialize sensor and LED GPIOs
   Sensor_LEDs_Init();
	
    // Configure and enable the sensor GPIO pin interrupt
    Interrupt_Init();
		
    // Enable processor interrupts
    IntMasterEnable();
		InitConsole();
	UARTprintf("test message\n");
	
	while(1){
    // Sequence 1
		sequence2Done = false;
		trafficTimeCounter = 0;
		
    while (1) {
			UARTprintf("dun\n");
        GPIOPinWrite(GPIO_PORTA_BASE, Green_LED_main, Green_LED_main); // Turn on PA3
				GPIOPinWrite(GPIO_PORTF_BASE, Red_LED_side, Red_LED_side);
				for(trafficTimeCounter = 0; trafficTimeCounter<10; trafficTimeCounter++){
				SysCtlDelay((SysCtlClockGet() / 3) * 1); 
				if(sequence2Done) break; //Break out of loop and restart sequence 1 from beginning
			   UARTprintf("%d\n", trafficTimeCounter);
				}
				if(sequence2Done) break;
				GPIOPinWrite(GPIO_PORTA_BASE, Green_LED_main, 0);
				UARTprintf("2\n");
				runMainRoadSeq2 = false;
				runSideRoadSeq2 = false;
				GPIOPinWrite(GPIO_PORTA_BASE, Yellow_LED_main, Yellow_LED_main); // Turn on PA4
				SysCtlDelay((SysCtlClockGet() / 3) * 3);   
				GPIOPinWrite(GPIO_PORTA_BASE, Yellow_LED_main, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, Red_LED_side, 0);
			
				
				GPIOPinWrite(GPIO_PORTB_BASE, Red_LED_main, Red_LED_main); // Turn on PB6
				GPIOPinWrite(GPIO_PORTF_BASE, Green_LED_side, Green_LED_side);
				SysCtlDelay((SysCtlClockGet() / 3) * 6);  
				GPIOPinWrite(GPIO_PORTF_BASE, Green_LED_side, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, Yellow_LED_side, Yellow_LED_side);
				SysCtlDelay((SysCtlClockGet() / 3) * 3);
				GPIOPinWrite(GPIO_PORTB_BASE, Red_LED_main, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, Yellow_LED_side, 0);
    }
		UARTprintf("nun\n");
	}
}

// ISR for sensor input
void Sensor_ISR(void) {
    // Clear the interrupt (specific to your MCU and library)
    GPIOIntClear(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
	UARTprintf("int triggered\n");
		if(GPIOPinRead(GPIO_PORTA_BASE, Green_LED_main) == Green_LED_main){
			if(!runMainRoadSeq2 && !runSideRoadSeq2 && (trafficTimeCounter > 5)){ //So it doesnt keep triggering
					runMainRoadSeq2 = true;
					runSideRoadSeq2 = true;
				UARTprintf("11\n");
				//Turn off all LEDs from previous sequence
				//Main Road
				GPIOPinWrite(GPIO_PORTA_BASE, Green_LED_main, 0);
				GPIOPinWrite(GPIO_PORTA_BASE, Yellow_LED_main, 0);
				GPIOPinWrite(GPIO_PORTB_BASE, Red_LED_main, 0);
				//Side Road
				GPIOPinWrite(GPIO_PORTF_BASE, Red_LED_side, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, Green_LED_side, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, Yellow_LED_side, 0);
				
				//New Sequence
				GPIOPinWrite(GPIO_PORTF_BASE, Red_LED_side, Red_LED_side);
				GPIOPinWrite(GPIO_PORTA_BASE, Yellow_LED_main, Yellow_LED_main);
				SysCtlDelay((SysCtlClockGet() / 3) * 3); 

				GPIOPinWrite(GPIO_PORTF_BASE, Red_LED_side, 0);
				GPIOPinWrite(GPIO_PORTA_BASE, Yellow_LED_main, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, Green_LED_side, Green_LED_side);
				GPIOPinWrite(GPIO_PORTB_BASE, Red_LED_main, Red_LED_main);
				SysCtlDelay((SysCtlClockGet() / 3) * 6); 
				
				GPIOPinWrite(GPIO_PORTF_BASE, Green_LED_side, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, Yellow_LED_side, Yellow_LED_side);
				SysCtlDelay((SysCtlClockGet() / 3) * 3);
				
				GPIOPinWrite(GPIO_PORTF_BASE, Yellow_LED_side, 0);
				//GPIOPinWrite(GPIO_PORTF_BASE, Red_LED_side, Red_LED_side);
				GPIOPinWrite(GPIO_PORTB_BASE, Red_LED_main, 0);
				
					
				sequence2Done = true;
			}
			
		}
		//UARTprintf("3\n");
    // Traffic light control logic
    // Change the state of the LEDs based on the sensor input
    // ...

    // Example: Toggle a LED
    
}

void Sensor_LEDs_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOInput(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
		GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);
	
	//GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4); //Enable sw1 for input 
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);  //Enable green led for output
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 + GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
}

void Interrupt_Init(void){
	
		GPIOIntDisable(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
    GPIOIntClear(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
    GPIOIntTypeSet(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN, GPIO_RISING_EDGE);
    GPIOIntEnable(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
		GPIOIntRegister(SENSOR_GPIO_PORT, Sensor_ISR);
}

void InitConsole(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 9600, 16000000);
}