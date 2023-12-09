#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include <os.h>

#define GREEN_LED   GPIO_PIN_1 // Assuming PF1 is green LED
#define YELLOW_LED  GPIO_PIN_2 // Assuming PF2 is yellow LED
#define RED_LED     GPIO_PIN_3 // Assuming PF3 is red LED

// Stack sizes should be aligned to the 8-byte boundary
#define TASK_STK_SIZE 128 // Adjust stack size as needed
OS_STK TaskStk[TASK_STK_SIZE];

// Function prototypes
static void LEDTask(void *p_arg);

int main(void) {
    OSInit(); // Initialize uC/OS-II

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable the GPIO port for the LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GREEN_LED | YELLOW_LED | RED_LED); // Set LED pins as outputs

    OSTaskCreate(LEDTask, // Create the LED task
                 NULL,
                 &TaskStk[TASK_STK_SIZE - 1],
                 5); // Task Priority

    OSStart(); // Start multitasking
    return 0;
}

static void LEDTask(void *p_arg) {
    (void)p_arg;

    while (1) {
        // Green for 10 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, GREEN_LED);
        OSTimeDlyHMSM(0, 0, 10, 0); // Delay for 10 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0); // Turn off green LED

        // Yellow for 10 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, YELLOW_LED, YELLOW_LED);
        OSTimeDlyHMSM(0, 0, 10, 0); // Delay for 10 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, YELLOW_LED, 0); // Turn off yellow LED

        // Red for 5 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
        OSTimeDlyHMSM(0, 0, 5, 0); // Delay for 5 seconds
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0); // Turn off red LED
    }
}
