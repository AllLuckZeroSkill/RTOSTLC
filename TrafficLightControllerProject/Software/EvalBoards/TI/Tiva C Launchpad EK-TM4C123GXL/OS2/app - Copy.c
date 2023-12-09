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
static OS_STK ButtonMonitor2Stk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK Blinky2Stk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK DebuggingVars2Stk[APP_CFG_TASK_START_STK_SIZE];
OS_FLAG_GRP *blinkEvent;
OS_EVENT *UART_BINARY_SEMAPHORE;
OS_TCB blinkyTCB;
/*
***************************************************************************************************
******
* LOCAL MACRO'S
***************************************************************************************************
******
*/

/*
***************************************************************************************************
******

* LOCAL FUNCTION PROTOTYPES
***************************************************************************************************
******
*/
static void AppTaskCreate (void);
static void AppTaskStart (void *p_arg);
static void Task1 (void *p_arg);
static void Task2 (void *p_arg);
static void ButtonMonitor(void *p_arg);
static void Blinky(void *p_arg);
static void DebuggingVars(void *p_arg);
static uint32_t generateRandomBlinkRate(void *p_arg);
static uint32_t generateNewColor(void *p_arg);
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

	OSTaskCreate((void 			(*)(void 			*)) ButtonMonitor,
 (void 			*) 0, (OS_STK			*)&ButtonMonitor2Stk[APP_CFG_TASK_START_STK_SIZE - 1] , (INT8U			 ) 5 );
	//OSTaskCreate((void (*)(void *)) Blinky, (void *) 0, (OS_STKBlinky2Stk[APP_CFG_TASK_START_STK_SIZE - 1] , (INT8U ) 6 );
	OSTaskCreateExt((void 		(*)(void		 *)) Blinky,(void 			*) 0,(OS_STK			 *)&Blinky2Stk[APP_CFG_TASK_START_STK_SIZE - 1],(INT8U 			) 6,
		(INT16U			 ) 6,(OS_STK 			*)&Blinky2Stk[0],(INT32U 			)APP_CFG_TASK_START_STK_SIZE,(void *) 0,(INT16U 			)(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
	OSTaskCreate((void			 (*)(void *)) DebuggingVars, (void 			*) 0, (OS_STK*)&DebuggingVars2Stk[APP_CFG_TASK_START_STK_SIZE - 1] , (INT8U 			) 7 );
	// CREATE FLAG 
	UART_BINARY_SEMAPHORE = OSSemCreate(1);
	// CREATE SEMAPHOREs
	blinkEvent = OSFlagCreate(0, &err);
}
static void DebuggingVars(void *p_arg)
{
	INT8U err;
	INT8U blinkyPriority = 6;
	OS_TCB *p_tcb = &blinkyTCB;
	
		for(;;){
			OSTimeDlyHMSM(0, 0, 5, 0);
			OSSemPend(UART_BINARY_SEMAPHORE, 0, &err);
			UARTprintf("CPU Usage: %d%%\r\n", OSCPUUsage);
			UARTprintf("Context Switches: %d\r\n", OSCtxSwCtr);
			UARTprintf("Idle Counter: %d\r\n", OSIdleCtr);
			OSTaskQuery(blinkyPriority, &blinkyTCB);
			UARTprintf("TCB: %d\r\n", p_tcb->OSTCBCtxSwCtr);
			OSSemPost(UART_BINARY_SEMAPHORE);
		}
}
static void ButtonMonitor(void *p_arg)
{

	uint32_t SW1_Button_Press;
	uint32_t SW2_Button_Press;
	INT8U err;
	for(;;)
	{
		OSTimeDlyHMSM(0, 0, 0, 150);
		SW1_Button_Press = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
		SW2_Button_Press = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
		if(SW1_Button_Press == 0 || SW2_Button_Press == 0)
		{
		OSSemPend(UART_BINARY_SEMAPHORE, 0, &err);
			if(SW1_Button_Press == 0) UARTprintf("Button 1 was pressed:\n");
			if(SW2_Button_Press == 0) UARTprintf("Button 2 was pressed:\n");
			OSSemPost(UART_BINARY_SEMAPHORE);
			OSFlagPost(blinkEvent, BLINK_FLAG, OS_FLAG_SET, &err);
		}
	}
}
static void Blinky(void *p_arg)
{
	INT8U err;
	uint32_t newBlinkRate, newColor;
	for(;;)
	{
		OSFlagAccept(blinkEvent, BLINK_FLAG, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, &err);
	
		if(err == OS_ERR_NONE){
			
			
			newBlinkRate = generateRandomBlinkRate(NULL);
			newColor = generateNewColor(NULL);
			OSSemPend(UART_BINARY_SEMAPHORE, 0, &err);
			UARTprintf("New Blink Rate: %d \n", newBlinkRate);
			OSSemPost(UART_BINARY_SEMAPHORE);
		}

		BSP_LED_Toggle(newColor);
		OSTimeDlyHMSM(0, 0, 0, newBlinkRate);
		BSP_LED_Toggle(newColor);
		OSTimeDlyHMSM(0, 0, 0, newBlinkRate);
		BSP_LED_Toggle(newColor);
		OSTimeDlyHMSM(0, 0, 0, newBlinkRate);
		BSP_LED_Toggle(newColor);
		
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
static uint32_t generateRandomBlinkRate(void *p_arg)
{
	uint32_t random;
	uint32_t ranMin = 50;
	uint32_t ranMax = 1000;
	random = ranMin + rand() % (ranMax - ranMin + 1);
	return random;
}
static uint32_t generateNewColor(void *p_arg)
{
	uint32_t random;
	uint32_t ranMin = 0;

	uint32_t ranMax = 2;
	random = ranMin + rand() % (ranMax - ranMin + 1);
	return random;
}