#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
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

OS_STK LedTaskStk[TASK_STK_SIZE];
OS_STK SensorMonitoringStk[TASK_STK_SIZE];
OS_FLAG_GRP *SensorEventFlag;
OS_EVENT *UART_BINARY_SEMAPHORE;

INT8U err;

// Function prototypes
static void LEDTask(void *p_arg);
static void SensorMonitoring(void *p_arg);
void Sensor_LEDs_Init(void);
void InitConsole(void);
void SensorInterruptHandler(void);
void LEDTask2();

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    Sensor_LEDs_Init();
    InitConsole();

    OSInit();

    SensorEventFlag = OSFlagCreate(0, &err); // Initialize flag with a clear (0) value
    UART_BINARY_SEMAPHORE = OSSemCreate(1);  // Create a semaphore for UART access

    // Create tasks
    err = OSTaskCreate(SensorMonitoring, NULL, (void *)&SensorMonitoringStk[TASK_STK_SIZE - 1], 10);
    err = OSTaskCreate(LEDTask, NULL, (void *)&LedTaskStk[TASK_STK_SIZE - 1], 5);


    OSStart(); // Start the OS Scheduler
    return 0;
}

bool CheckSensorFlag(void) {
    OS_FLAGS flags = OSFlagQuery(SensorEventFlag, &err);
    return (flags & 1) != 0;
}

void Sensor_LEDs_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
    GPIOPinTypeGPIOOutput(LED_GPIO_PORT, LED_GPIO_PIN);

    // Interrupt setup
    GPIOIntDisable(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
    GPIOIntClear(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
    GPIOIntRegister(SENSOR_GPIO_PORT, SensorInterruptHandler);
    GPIOIntTypeSet(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN, GPIO_RISING_EDGE);
    GPIOIntEnable(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
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

static void LEDTask(void *p_arg) {
    while (1) {
        OS_FLAGS flags = OSFlagPend(SensorEventFlag, 1, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &err);

       // if (flags != 0) {
            // Your code to blink LEDs here
           GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3); // Turn on PA3
            OSTimeDlyHMSM(0, 0, 3, 0);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);

            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4); // Turn on PA4
            OSTimeDlyHMSM(0, 0, 1, 0);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);

            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn on PB6
            OSTimeDlyHMSM(0, 0, 3, 0);
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
        //}
    }
}


static void SensorMonitoring(void *p_arg) {
    while (1) {
        if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4)) {
            OSSemPend(UART_BINARY_SEMAPHORE, 0, &err); // Wait for UART access semaphore
            UARTprintf("not detected\n");
            OSSemPost(UART_BINARY_SEMAPHORE); // Release UART access semaphore
        } else {
            OSSemPend(UART_BINARY_SEMAPHORE, 0, &err); // Wait for UART access semaphore
            UARTprintf("detected\n");
            OSSemPost(UART_BINARY_SEMAPHORE); // Release UART access semaphore
            OSFlagPost(SensorEventFlag, 1, OS_FLAG_SET, &err); // Set the flag
        }
        OSTimeDlyHMSM(0, 0, 0, 500); // Delay for debounce, adjust as needed
    }
}

void LEDTask2() {
    OSSemPend(UART_BINARY_SEMAPHORE, 0, &err); // Wait for UART access semaphore
    UARTprintf("test\n");
    OSSemPost(UART_BINARY_SEMAPHORE); // Release UART access semaphore

    // Alternative behavior for the LED
    // Example: blink faster or in a different pattern
}

void SensorInterruptHandler(void) {
    GPIOIntClear(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
    // Handle the sensor interrupt (e.g., toggle an LED or send a signal)
    //GPIOPinWrite(LED_GPIO_PORT, LED_GPIO_PIN, GPIOPinRead(LED_GPIO_PORT, LED_GPIO_PIN) ^ LED_GPIO_PIN);
	OSSemPend(UART_BINARY_SEMAPHORE, 0, &err); // Wait for UART access semaphore
            UARTprintf("detected\n");
            OSSemPost(UART_BINARY_SEMAPHORE); // Release UART access semaphore

}
