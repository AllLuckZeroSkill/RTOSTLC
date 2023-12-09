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



#define TASK_STK_SIZE 128 
OS_STK TaskStk[TASK_STK_SIZE];

// Function prototypes
static void LEDTask(void *p_arg);
uint32_t Measure_distance(void);
void Timer0ACapture_init(void);
void InitConsole(void);
void Delay_MicroSecond(int time);

int main(void) {
	
	 SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    
    // Enable the GPIO port that is used for the IR sensor (Port B)
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

    // Turn on the LEDs.
			while(1){
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3); // Turn on PA3
		SysCtlDelay((SysCtlClockGet() / 3 * 10));
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
				
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4); // Turn on PA4
		SysCtlDelay((SysCtlClockGet() / 3 * 2));
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
				
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn on PB6
		SysCtlDelay((SysCtlClockGet() / 3 * 5));
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
			}
	// Initialize UART for communication
    InitConsole();
	
    while(1) {
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
				SysCtlDelay(SysCtlClockGet() / 4);
			}
	/*	uint32_t distance;
	
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}

    // Enable the GPIO pin for the Green LED (PF3). Set the direction as output, and enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GREEN_LED);

    // Turn on the Green LED.
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, GREEN_LED);

    // Delay for 30 seconds.
    SysCtlDelay(SysCtlClockGet() * 5);
GPIOPinWrite(GPIO_PORTF_BASE, 0, 0);
			
			
			
    // Turn off the Green LED.
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
   // OSInit(); // Initialize uC/OS-II
	//SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                  // SYSCTL_XTAL_16MHZ);
    // Initialize peripherals
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GREEN_LED | YELLOW_LED | RED_LED);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    // Initialize Timer0A for ultrasonic sensor
    Timer0ACapture_init();

    

    // Create RTOS task
   // OSTaskCreate(LEDTask, NULL, &TaskStk[TASK_STK_SIZE - 1], 5);
		UARTprintf("test1\n");
		while (1) {
			UARTprintf("test2\n");
         distance = Measure_distance();			// Get distance reading
			UARTprintf("test3\n");
        UARTprintf("Distance: %u\n", distance); // Send distance over UART

        // Optional: Delay between measurements
        SysCtlDelay(SysCtlClockGet() / 3); // Approximately 1 second delay
    }
    // Start multitasking
    //OSStart();
		
    // Main loop
    */

    return 0;
}

/*
static void LEDTask(void *p_arg) {
    (void)p_arg;

    while (1) {
        // Green for 10 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, GREEN_LED);
        OSTimeDlyHMSM(0, 0, 10, 0); // Delay for 10 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0); // Turn off green LED

        // Yellow for 1 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, YELLOW_LED, YELLOW_LED);
        OSTimeDlyHMSM(0, 0, 1, 0); // Delay for 10 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, YELLOW_LED, 0); // Turn off yellow LED

        // Red for 5 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
        OSTimeDlyHMSM(0, 0, 5, 0); // Delay for 5 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0); // Turn off red LED
    }
}*/

void Delay_MicroSecond(int time) {
    int i;
    SYSCTL_RCGC2_R |= (1 << 2); // Enable clock to Timer Block 1 (assuming Timer 1 is under Bit 2)

    TIMER1_CTL = 0;            // Disable Timer1 before initialization
    TIMER1_CFG = 0x04;         // 16-bit option
    TIMER1_TAMR = 0x02;        // Periodic mode and down-counter
    TIMER1_TAILR = 16 - 1;     // TimerA interval load value reg (for 1 microsecond delay)
    TIMER1_ICR = 0x1;          // Clear the TimerA timeout flag
    TIMER1_CTL |= 0x01;        // Enable Timer1 after initialization

    for (i = 0; i < time; i++) {
        while ((TIMER1_RIS & 0x1) == 0); // Wait for TimerA timeout flag
        TIMER1_ICR = 0x1;      // Clear the TimerA timeout flag
    }
}


uint32_t Measure_distance(void) {
    uint32_t lastEdge = 0, thisEdge = 0;
    uint32_t distance;
    uint32_t timeout = 0;

    // Trigger pulse
    GPIO_PORTA_DATA &= ~(1<<4);
    OSTimeDlyHMSM(0, 0, 0, 10);
    GPIO_PORTA_DATA |= (1<<4);
    OSTimeDlyHMSM(0, 0, 0, 10);
    GPIO_PORTA_DATA &= ~(1<<4);

    UARTprintf("test4\n");

    // Wait for echo signal
    while(1) {
        UARTprintf("test5\n");
        TIMER0_ICR = 4;
        while((TIMER0_RIS & 4) == 0) {
            timeout++;
            if (timeout > 1000000) { // Adjust timeout value as needed
                UARTprintf("Timeout\n");
                return 0; // Timeout occurred
            }
        }

        if(GPIO_PORTB_DATA & (1<<6)) {
            lastEdge = TIMER0_TAR;

            TIMER0_ICR = 4;
            while((TIMER0_RIS & 4) == 0);
            thisEdge = TIMER0_TAR;

            distance = (thisEdge - lastEdge);
            UARTprintf("Distance: %u\n", distance);
            return distance;
        }
    }
}



void Timer0ACapture_init(void) {
    // Enable clock for Timer 0, GPIO Port A and Port B
    SYSCTL_RCGC2_R |= (1 << 1); // Enable clock for PORTB (Bit 1)
    SYSCTL_RCGC2_R |= (1 << 0); // Enable clock for PORTA (Bit 0)

    // Configure PB6 as an input pin for Timer0 capture
    GPIO_PORTB_DIR &= ~0x40;       // Make PB6 an input pin
    GPIO_PORTB_DEN |= 0x40;        // Enable digital functionality for PB6
    GPIO_PORTB_AFSEL |= 0x40;      // Enable alternate function for PB6
    GPIO_PORTB_PCTL &= ~0x0F000000; // Clear PMC6 bits
    GPIO_PORTB_PCTL |= 0x07000000;  // Configure PB6 for T0CCP0 function

    // Configure PA4 as a digital output pin for the ultrasonic trigger
    GPIO_PORTA_DIR |= 0x10;       // Make PA4 an output pin
    GPIO_PORTA_DEN |= 0x10;       // Enable digital functionality for PA4

    // Initialize Timer 0 in input-edge time mode
    TIMER0_CTL &= ~1;              // Disable Timer0A
    TIMER0_CFG = 4;                // 16-bit timer mode
    TIMER0_TAMR = 0x17;            // Up-count, edge-time, capture mode
    TIMER0_CTL |= 0x0C;            // Capture the rising edge
    TIMER0_CTL |= 1;               // Enable Timer0A
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

