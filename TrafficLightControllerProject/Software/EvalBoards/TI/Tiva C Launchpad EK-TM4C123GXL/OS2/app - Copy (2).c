#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include <os.h>
#include "driverlib/timer.h"  
#include "driverlib/uart.h"
#include "utils/uartstdio.h"



#define GREEN_LED   GPIO_PIN_3 
#define BLUE_LED     GPIO_PIN_2 
#define RED_LED     GPIO_PIN_1 
#define YELLOW_LED  GREEN_LED+RED_LED

#define SYSCTL 0x400FE108
#define SYSCTL_BASE           0x400FE000
#define SYSCTL_RCGC2_OFFSET   0x108
#define SYSCTL_RCGC2_R        (*((volatile uint32_t *)(SYSCTL_BASE + SYSCTL_RCGC2_OFFSET)))



#define GPIO_PORTA_BASE      0x40004000

// GPIO Port A Registers
#define GPIO_PORTA_BASE      0x40004000
#define GPIO_PORTA_DATA      (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x3FC)))
#define GPIO_PORTA_DIR       (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x400)))
#define GPIO_PORTA_AFSEL     (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x420)))
#define GPIO_PORTA_DEN       (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x51C)))
#define GPIO_PORTA_PCTL      (*((volatile uint32_t *)(GPIO_PORTA_BASE + 0x52C)))

// GPIO Port B Registers
#define GPIO_PORTB_BASE      0x40005000
#define GPIO_PORTB_DATA      (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x3FC)))
#define GPIO_PORTB_DIR       (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x400)))
#define GPIO_PORTB_AFSEL     (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x420)))
#define GPIO_PORTB_DEN       (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x51C)))
#define GPIO_PORTB_PCTL      (*((volatile uint32_t *)(GPIO_PORTB_BASE + 0x52C)))

// Timer 0 Registers
#define TIMER0_BASE          0x40030000
#define TIMER0_CTL           (*((volatile uint32_t *)(TIMER0_BASE + 0x00C)))
#define TIMER0_CFG           (*((volatile uint32_t *)(TIMER0_BASE + 0x000)))
#define TIMER0_TAMR          (*((volatile uint32_t *)(TIMER0_BASE + 0x004)))
#define TIMER0_ICR           (*((volatile uint32_t *)(TIMER0_BASE + 0x024)))
#define TIMER0_RIS           (*((volatile uint32_t *)(TIMER0_BASE + 0x01C)))
#define TIMER0_TAR           (*((volatile uint32_t *)(TIMER0_BASE + 0x048)))

// Timer 1 Registers
#define TIMER1_BASE          0x40031000
#define TIMER1_CTL           (*((volatile uint32_t *)(TIMER1_BASE + 0x00C)))
#define TIMER1_CFG           (*((volatile uint32_t *)(TIMER1_BASE + 0x000)))
#define TIMER1_TAMR          (*((volatile uint32_t *)(TIMER1_BASE + 0x004)))
#define TIMER1_TAILR         (*((volatile uint32_t *)(TIMER1_BASE + 0x028)))
#define TIMER1_ICR           (*((volatile uint32_t *)(TIMER1_BASE + 0x024)))
#define TIMER1_RIS           (*((volatile uint32_t *)(TIMER1_BASE + 0x01C)))



#define LED_TASK_PRIO        6    // Unique priority for LED Task
#define SENSOR_TASK_PRIO     7    // Unique priority for Sensor Task
#define TASK_STK_SIZE        128  // Define a suitable stack size for both tasks
static OS_STK LedTaskStk[TASK_STK_SIZE];
static OS_STK SensorTaskStk[TASK_STK_SIZE];

INT8U err; 

// Function prototypes
static void LEDTask(void *p_arg);
uint32_t Measure_distance(void);
void Timer0ACapture_init(void);
void InitConsole(void);
void Delay_MicroSecond(int time);
static void SensorTask(void *p_arg);

void Sensor_LEDs_Init(void);

int main(void) {
	
	
	
	 SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    
			Sensor_LEDs_Init();
	// Initialize UART for communication
    InitConsole();
			
			OSInit();
			
			UARTprintf("hello\n");
			 OSTaskCreate(LEDTask, NULL, (void *)&LedTaskStk[TASK_STK_SIZE - 1], LED_TASK_PRIO);
    OSTaskCreate(SensorTask, NULL, (void *)&SensorTaskStk[TASK_STK_SIZE - 1], SENSOR_TASK_PRIO);
UARTprintf("hello2\n");

			
			OSStart();
    return 0;
	
}

void Sensor_LEDs_Init(void){
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    // Set the pin on Port B as an input
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_4);

	
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    // Set the direction of the pins as output and enable the GPIO pins for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6); // PB6
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_3); // PA4, PA3

	
}



static void LEDTask(void *p_arg) {
    (void)p_arg;
		
	
    UARTprintf("hello3\n");
while(1){
	UARTprintf("hello4.1\n");
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3); // Turn on PA3
		SysCtlDelay((SysCtlClockGet() / 3 * 10));
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
				
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4); // Turn on PA4
		SysCtlDelay((SysCtlClockGet() / 3 * 2));
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
				
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn on PB6
		SysCtlDelay((SysCtlClockGet() / 3 * 5));
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
	 OSTimeDlyHMSM(0, 0, 1, 0);
			UARTprintf("hello4\n");
    }
}

static void SensorTask(void *p_arg){
	UARTprintf("hello5\n");
	    while(1) {
				UARTprintf("hello6\n");
        // Read the sensor output from Port B
        if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4)) {
            // Object detected
					UARTprintf("not detected\n");
					//UARTprintf("%d\n", GPIO_PIN_4);
        }
        else {
            // No object detected
					UARTprintf("detected\n");
					//UARTprintf("%d\n", GPIO_PIN_4);
        }
				OSTimeDlyHMSM(0, 0, 0, 500);
				UARTprintf("hello7\n");
			}
			
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

