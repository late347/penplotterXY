/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here
#include "FreeRTOS.h"
#include "task.h"
#include "ITM_write.h"
#include "semphr.h"
#include "DigitalIoPin.h"
#include <mutex>
#include "Fmutex.h"
#include "user_vcom.h"
#include <cstring>
#include <atomic>
#include "queue.h"
#include <string>
#include <vector>

// TODO: insert other definitions and declarations here
#include "GcodeParser.h"
#include "CommandStruct.h"


SemaphoreHandle_t syncSemph;


/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
/* end runtime statictics collection */

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);

}

/* send data and toggle thread */
static void send_task(void *pvParameters) {
bool LedState = false;
char ok[]="OK\r\n";
int len = strlen(ok);
char initialMessage[]="M10 XY 380 310 0.00 0.00 A0 B0 H0 S80 U160 D90\r\nOK\r\n";
int initlen= strlen(initialMessage);
bool isFirstRound=true;

	while (1) {

		xSemaphoreTake(syncSemph, portMAX_DELAY);

		if(isFirstRound){
			isFirstRound=false;
			USB_send((uint8_t*)initialMessage,initlen);
		}
		else{
			USB_send((uint8_t *)ok, len);
		}
		Board_LED_Set(0, LedState);
		LedState = (bool) !LedState;
//		vTaskDelay(configTICK_RATE_HZ / 50);
	}
}


/* usb receive thread */
static void receive_task(void *pvParameters) {

	bool LedState = false;
	GcodeParser parser;
	const int allocsize=65;
	char initialMessage[]="M10 XY 380 310 0.00 0.00 A0 B0 H0 S80 U160 D90\r\nOK\r\n";
	int initlen= strlen(initialMessage);


	while (1) {
		char str[allocsize]{0};
		uint32_t len = USB_receive((uint8_t *)str, allocsize-1);
		str[len] = 0; /* make sure we have a zero at the end so that we can print the data */



		ITM_write(str);

		Board_LED_Set(1, LedState);
		LedState = (bool) !LedState;
	}
}




static void parse_task(void*pvParameters){
	bool LedState = false;
	GcodeParser parser;
	const int allocsize=120+1;
	char initialMessage[]="M10 XY 380 310 0.00 0.00 A0 B0 H0 S80 U160 D90\r\nOK\r\n";
	int initlen= strlen(initialMessage);
	char okMessage[]="OK\r\n";
	char badMessage[]="nok\r\n";
	int oklen=strlen(okMessage);
	std::string temp;

	while (1) {
		//str buffer is allocated
		char str[allocsize]{0};
		uint32_t len = USB_receive((uint8_t *)str, allocsize-1);
		str[len] = 0; /* make sure we have a zero at the end so that we can print the data */
		//extract away delimiters M10\r\n len==5
		// m4\r len==3
		if (len > 1) {
			if (str[len - 1] == '\r' || str[len - 1] == '\n') {
				str[len - 1] = 0;
			}		//extract away delimiters \r\n
			if (str[len - 2] == '\r' || str[len - 1] == '\n') {
				str[len - 2] = 0;
			}
		}

		temp="";
		temp=std::string(str);
		ITM_write(temp.c_str());
		ITM_write("\r\n");
		vTaskDelay(5);
		CommandStruct cmd {CommandStruct::M1, -2147483648, false,-2147483648,-2147483648 };
		  cmd = parser.parseCommand(temp);
		if(cmd.commandWord==CommandStruct::M10 && cmd.isLegal){
			USB_send((uint8_t*) initialMessage, initlen );
		}
		else if(cmd.isLegal ){
			//other legal message send ok
			USB_send((uint8_t*) okMessage, oklen );
		}
		else {
			USB_send((uint8_t*) badMessage, strlen(badMessage) );
		}
		//NOTE important TO CLEAR TOKENS from parser AFTER EACH ROUND!! otherwise buggy behavior

		//EDITED parseCommand has clearTokens() at the end
		//parser.clearTokens();
		memset(str, 0, allocsize);
		Board_LED_Set(1, LedState);
		LedState = (bool) !LedState;
	}
}


int main(void) {

	prvSetupHardware();
	ITM_init();

	syncSemph = xSemaphoreCreateBinary();
	/* usb parser  thread */
	xTaskCreate(parse_task, "usbparse",
				configMINIMAL_STACK_SIZE * 5, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

//	/* usb receive thread */
//	xTaskCreate(receive_task, "Rx",
//				configMINIMAL_STACK_SIZE * 3, NULL, (tskIDLE_PRIORITY + 1UL),
//				(TaskHandle_t *) NULL);

	/* cdc thread */
	xTaskCreate(cdc_task, "CDC",
				configMINIMAL_STACK_SIZE * 3, NULL, (tskIDLE_PRIORITY  +1UL +1UL),
				(TaskHandle_t *) NULL);


	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
