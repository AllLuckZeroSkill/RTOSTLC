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
#include <time.h>
#include <stdlib.h>
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
#define BLINK_FLAG 1
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

int main (void)
{
	
	#if (OS_TASK_NAME_EN > 0)
	CPU_INT08U err;
	#endif
	#if (CPU_CFG_NAME_EN == DEF_ENABLED)
	CPU_ERR cpu_err;
	#endif

	#if (CPU_CFG_NAME_EN == DEF_ENABLED)
	CPU_NameSet((CPU_CHAR *)"TM4C129XNCZAD",
	(CPU_ERR *)&cpu_err);
	#endif
	CPU_IntDis(); /* Disable all interrupts. */
	OSInit(); /* Initialize "uC/OS-II, The Real-Time Kernel" */
	OSTaskCreateExt((void (*)(void *)) AppTaskStart, /* Create the start task
	*/
	(void *) 0,
	(OS_STK *)&AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE - 1],
	(INT8U ) APP_CFG_TASK_START_PRIO,

	(INT16U ) APP_CFG_TASK_START_PRIO,
	(OS_STK *)&AppTaskStartStk[0],
	(INT32U ) APP_CFG_TASK_START_STK_SIZE,
	(void *) 0,
	(INT16U )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
	#if (OS_TASK_NAME_EN > 0)
	OSTaskNameSet(APP_CFG_TASK_START_PRIO, "Start", &err);
	#endif
	
		Sensor_LEDs_Init();
	OSStart(); /* Start multitasking (i.e. give control to uC/OS-II) */
	for (;;) {
	}
}
static void AppTaskStart (void *p_arg)
{
	uint32_t sw2_status;

	CPU_INT32U cpu_clk_freq;

	CPU_INT32U cnts;

	(void)p_arg; /* See Note #1 */

	(void)&p_arg;
	BSP_Init(); /* Initialize BSP functions */
	cpu_clk_freq = BSP_SysClkFreqGet(); /* Determine SysTick reference freq.
	*/
	cnts = cpu_clk_freq /* Determine nbr SysTick increments */
	/ (CPU_INT32U)OS_TICKS_PER_SEC;
	OS_CPU_SysTickInit(cnts);
	CPU_Init(); /* Initialize the uC/CPU services */
	#if (OS_TASK_STAT_EN > 0)

	OSStatInit(); /* Determine CPU capacity */
	#endif

	Mem_Init();
	#ifdef CPU_CFG_INT_DIS_MEAS_EN
	CPU_IntDisMeasMaxCurReset();
	#endif

	BSP_LED_Toggle(0);
	OSTimeDlyHMSM(0, 0, 0, 200);
	BSP_LED_Toggle(0);
	BSP_LED_Toggle(1);
	OSTimeDlyHMSM(0, 0, 0, 200);
	BSP_LED_Toggle(1);
	BSP_LED_Toggle(2);
	OSTimeDlyHMSM(0, 0, 0, 200);
	BSP_LED_Toggle(2);
	OSTimeDlyHMSM(0, 0, 1, 0);
	AppTaskCreate(); /* Creates all the necessary application tasks.

	*/

	while (DEF_ON) {

		OSTimeDlyHMSM(0, 0, 0, 100);

		// Demonstrating button interface
		//sw2_status = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
//		if (sw2_status == 0)
//		{
//			UARTprintf("\n\r Button2 was pressed\n\r"); // Probably needs to be protected by semaphore

//		}

	}
}

/*
***************************************************************************************************
******
* AppTaskCreate()
*
* Description : Create the application tasks.
*
* Argument(s) : none.
*
* Return(s) : none.
*
* Caller(s) : AppTaskStart()
*
* Note(s) : none.
***************************************************************************************************
******
*/
static void AppTaskCreate (void)
{
	//OSTaskCreate((void (*)(void *)) Task1, /* Create the second task */
	//(void *) 0, // argument
	//(OS_STK *)&Task1Stk[APP_CFG_TASK_START_STK_SIZE - 1],
	//(INT8U ) 5 ); // Task Priority

	//OSTaskCreate((void (*)(void *)) Task2, /* Create the second task */
	// (void *) 0, // argument
	// (OS_STK *)&Task2Stk[APP_CFG_TASK_START_STK_SIZE - 1],
	// (INT8U ) 6 ); // Task Priority
	INT8U err;

	OSTaskCreate((void 			(*)(void 			*)) sensorMonitor,
 (void 			*) 0, (OS_STK			*)&sensorMonitor2Stk[APP_CFG_TASK_START_STK_SIZE - 1] , (INT8U			 ) 5 );
	//OSTaskCreate((void (*)(void *)) mainRoadLight_seq1, (void *) 0, (OS_STKmainRoad2Stk[APP_CFG_TASK_START_STK_SIZE - 1] , (INT8U ) 6 );
	OSTaskCreateExt((void 		(*)(void		 *)) mainRoadLight_seq1,(void 			*) 0,(OS_STK			 *)&mainRoad2Stk[APP_CFG_TASK_START_STK_SIZE - 1],(INT8U 			) 6,
		(INT16U			 ) 6,(OS_STK 			*)&mainRoad2Stk[0],(INT32U 			)APP_CFG_TASK_START_STK_SIZE,(void *) 0,(INT16U 			)(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
	OSTaskCreate((void			 (*)(void *)) sideRoadLight_seq1, (void 			*) 0, (OS_STK*)&sideRoad2Stk[APP_CFG_TASK_START_STK_SIZE - 1] , (INT8U 			) 7 );
	// CREATE FLAG 
	UART_BINARY_SEMAPHORE = OSSemCreate(1);
	// CREATE SEMAPHOREs
	carDetEvent = OSFlagCreate(0, &err);
}
static void sideRoadLight_seq1(void *p_arg)
{
	INT8U err;
	INT8U blinkyPriority = 6;
	OS_TCB *p_tcb = &blinkyTCB;
	
		for(;;){
			//OSTimeDlyHMSM(0, 0, 5, 0);
			UARTprintf("red\n");
	//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn on PB6
			OSTimeDlyHMSM(0, 0, 13, 0);
	//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
			UARTprintf("green\n");
	//		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3); // Turn on PA3
			OSTimeDlyHMSM(0, 0, 6, 0);
	//		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
UARTprintf("yellow\n");
	//		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4); // Turn on PA4
			OSTimeDlyHMSM(0, 0, 3, 0);
	//		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);

			
			
			OSTaskQuery(blinkyPriority, &blinkyTCB);
			
			OSSemPost(UART_BINARY_SEMAPHORE);
		}
}


static void sensorMonitor(void *p_arg)
{

	uint32_t sensor1;
	uint32_t sensor2;
	INT8U err;
	for(;;)
	{
		OSTimeDlyHMSM(0, 0, 0, 150);
		//UARTprintf("hello1");
		sensor1 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4);
		//UARTprintf("hello2");
	//	sensor2 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
		if(sensor1 == 0 || sensor2 == 0)
		{
			//UARTprintf("hello3");
		OSSemPend(UART_BINARY_SEMAPHORE, 0, &err);
			if(sensor1 == 0) UARTprintf("Object Detected on Sensor 1:\n");
			//if(sensor2 == 0) UARTprintf("Object Detected on Sensor 2:\n");
			OSSemPost(UART_BINARY_SEMAPHORE);
			OSFlagPost(carDetEvent, BLINK_FLAG, OS_FLAG_SET, &err);
		}
	}
}
static void mainRoadLight_seq1(void *p_arg)
{
	INT8U err;
	uint32_t newBlinkRate, newColor;
	for(;;)
	{
		OSFlagAccept(carDetEvent, BLINK_FLAG, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, &err);
	
		if(err == OS_ERR_NONE){
			
			
			
			
			OSSemPend(UART_BINARY_SEMAPHORE, 0, &err);
			
			OSSemPost(UART_BINARY_SEMAPHORE);
		}

						GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3); // Turn on PA3
            OSTimeDlyHMSM(0, 0, 10, 0);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);

            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4); // Turn on PA4
            OSTimeDlyHMSM(0, 0, 3, 0);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);

            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn on PB6
            OSTimeDlyHMSM(0, 0, 9, 0);
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
	
	}
}
static void Task1 (void *p_arg)
{
	(void)p_arg;
	for(;;) {
		BSP_LED_Toggle(1);

		UARTprintf("T1 "); // Probably needs to be protected by semaphore

		OSTimeDlyHMSM(0, 0, 0, 400);

	}

}
static void Task2 (void *p_arg)
{
	(void)p_arg;
	while (1) {
		BSP_LED_Toggle(2);
		UARTprintf("T2 "); // Probably needs to be protected by semaphore
		OSTimeDlyHMSM(0, 0, 0, 700);

	}

}

static void mainRoadLight_seq2(void *p_arg) {
    while (sensorTriggered) {
        // Sequence 2 logic for main road
    }
    // Switch back to sequence 1 or suspend this task
}

// Traffic light control task for side road (seq2)
static void sideRoadLight_seq2(void *p_arg) {
    while (sensorTriggered) {
        // Sequence 2 logic for side road
    }
    // Switch back to sequence 1 or suspend this task
}


void Sensor_LEDs_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(SENSOR_GPIO_PORT, SENSOR_GPIO_PIN);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
		GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);
}