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

//global semaphores and variables
SemaphoreHandle_t syncSemph;
volatile uint32_t RIT_count;
SemaphoreHandle_t sbRIT;
SemaphoreHandle_t sbGo;
static  std::atomic<bool> calibrationFinished(false);
//does this pulseState have to be volatile??
static volatile std::atomic<bool> pulseState(true);
QueueHandle_t commandQueue;
DigitalIoPin*limit1P;
DigitalIoPin*limit2P;
DigitalIoPin*stepP;
DigitalIoPin*dirP;






/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}


//EXAMPLE RIT_interrupt_handler is used to control the pulses of the motor!!!
extern "C" {
void RIT_IRQHandler(void) {
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	// Tell timer that we have processed the interrupt.
	// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag

	if (limit1P != NULL && limit2P != NULL) {
		if (RIT_count > 0) {
			RIT_count--;
			// do something useful here...
			if (!limit1P->read() && !limit2P->read()) {
				//move motor and toggle pulsestate rit_halfpulses
				stepP->write(pulseState);
				pulseState = !pulseState;
			} else {
				stepP->write(false);
				dirP->write(!dirP->read());
				RIT_count = 0; //WE HIT THE WALL, set rit_count=0 and soon the motor will stop, hopefully
			}
		} else {
			Chip_RIT_Disable(LPC_RITIMER); // disable timer
			// Give semaphore and set context switch flag if a higher priority task was woken up
			xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
		}
		// End the ISR and (possibly) do a context switch
		portEND_SWITCHING_ISR(xHigherPriorityWoken);
	}
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

//EXAMPLE FUNCTION RIT_START IS USED TO MOVE THE MOTOR
/*
 * example function call for starrting to move the motor
 *
 * RIT_start( 2*stepamount, (1000000/(2*currentPPS)) );
 *
 * because rit_interrupt happens every halfstep, must double wanted stepamount to have correct stepamount
 * second parameter is calculated like as follows below:
 * 					 * initializing RIT thing...
					 * count == steps == pulses
					 * we need interrupt "every half pulse" lowtime or hightime in ISR ritTimer (isr toggle pulseState)
					 * therefore rit_Count == steps *2
					 * if pps==1000 and left 800(fullsteps) is called,
					 * then you have
					 * 800*2 pulses in ISR in rit_counter (because isr has halfpulses)
					 * then you have pps==1000 =>implies =>
					 * 1000 fullsteps per million us, flip it around => million microsecs per 1000 fullsteps
					 * reduce fraction...
					 * == 1000 microsecs per 1 fullstep
					 * == 500 microsecs per 0.5 fullstep
 * */
void RIT_start(int count, int us) {
	uint64_t cmp_value;
	// Determine approximate compare value based on clock rate and passed interval
	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) us
			/ 1000000;
	// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);
	RIT_count = count;
	// enable automatic clear on when compare value==timer value
	// this makes interrupts trigger periodically
	Chip_RIT_EnableCompClear(LPC_RITIMER);
	// reset the counter
	Chip_RIT_SetCounter(LPC_RITIMER, 0);
	Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);
	// start counting
	Chip_RIT_Enable(LPC_RITIMER);
	// Enable the interrupt signal in NVIC (the interrupt controller)
	NVIC_EnableIRQ(RITIMER_IRQn);
	// wait for ISR to tell that we're done
	if (xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
		// Disable the interrupt signal in NVIC (the interrupt controller)
		NVIC_DisableIRQ(RITIMER_IRQn);
	} else {
		// unexpected error
	}
}












//EXAMPLE FUNCTION SCTSETUP THREE COUNTERS FOR THREE RGB COLORS
void setupSCTLED(){
	Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT0_O,  0,3);	//greenled port0_pin3
	Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT1_O,  0,25);	//reassigns output to redled
	Chip_SWM_MovablePortPinAssign( SWM_SCT1_OUT0_O,  1,1);	/*reassign output to blueled*/

	/*setup counters sct1low and sct0high and sct0low*/
	LPC_SCT1->CONFIG |= (1<<17); //sct1 lowcounter used autolimit
	LPC_SCT1->CTRL_L |= (72-1)<<5; //prescale lowcounter sct1
	LPC_SCT1->MATCHREL[0].L= 255-1;	//sct1 lowcoutner freq

	LPC_SCT0->CONFIG |= (3<<17); //autolimit lowcounter and highcounter
	LPC_SCT0->CTRL_L |= (72-1) << 5; //prescale lowcounter
	LPC_SCT0->CTRL_H |= (72-1) << 21; //prescale highcounter
	LPC_SCT0->MATCHREL[0].L = 255-1;	//sct0 lowcounter  freq
	LPC_SCT0->MATCHREL[0].H = 255-1;	//sct0 highcounter  freq

	/*set the pulsewidths into matchreload*/
	LPC_SCT0->MATCHREL[2].L = 250;	// sct0 lowcounter pulsewidth GREENLED
	LPC_SCT0->MATCHREL[1].H = 250;	// sct0 highcounter pulsewidth REDLED
	LPC_SCT1->MATCHREL[1].L= 250; //sct1 lowcounter pulsewidth BLUELED

	/*events configured
	 * 1st frequecny events*/
	LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF;	//all states allowed event0
	LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF;	//all states allowed event1
	LPC_SCT1->EVENT[2].STATE = 0xFFFFFFFF; //all states allowed event2

	LPC_SCT0->EVENT[0].CTRL= 1<<12;	//event0 sct0 lowcounter frequency match, select reg0
	LPC_SCT0->EVENT[1].CTRL = (1<<4) |(1<<12)  ; //event1 sct0 highcounter frequency match, select reg0
	LPC_SCT1->EVENT[2].CTRL = (1<<12); //event2 sct1 lowcounter frequency match, select reg0

	/*2ndly we have the COUNTER-MATCH events*/
	LPC_SCT0->EVENT[3].STATE= 0xFFFFFFFF;
	LPC_SCT0->EVENT[4].STATE= 0xFFFFFFFF;
	LPC_SCT1->EVENT[5].STATE= 0xFFFFFFFF;

	LPC_SCT0->EVENT[3].CTRL = (1<<12) | (2); //event3 sct0 lowcounter match, select reg2
	LPC_SCT0->EVENT[4].CTRL = (1<<4) | (1<<12) | (1); //event4 sct0 highcounter match, HEVENTbitTrue,  select reg1
	LPC_SCT1->EVENT[5].CTRL = (1<<12) | (1); //event5 sct1 lowcounter match, select reg1 (default)

	/*set outputs*/
	LPC_SCT0->OUT[0].SET =  (1<<0); //event0 sets  sct0 output0
	LPC_SCT0->OUT[1].SET = (1<<1); //event1 sets sct0 output1 //1<<0
	LPC_SCT1->OUT[0].SET = (1<<2); //event2 sets sct1 output0

	/*clear outputs with countermatches*/
	LPC_SCT0->OUT[0].CLR = 	1<<3;			//event3 clears sct0 output0
	LPC_SCT0->OUT[1].CLR =	1<<4;		//event4 clears sct0 output1
	LPC_SCT1->OUT[0].CLR =	1<<5;		//event5 clears sct1 output0

	/*unhalt timers*/
	LPC_SCT0->CTRL_L &=  ~(1<<2);
	LPC_SCT0->CTRL_H &= ~(1<<2);
	LPC_SCT1->CTRL_L &= ~(1<<2);

}


//getters and setters for SCTimer for three RGB colors
uint16_t getRed(){
	return LPC_SCT0->MATCHREL[1].H;
}
//getters and setters for SCTimer for three RGB colors
uint16_t getGreen(){
	return LPC_SCT0->MATCHREL[2].L;
}
//getters and setters for SCTimer for three RGB colors
uint16_t getBlue(){
	return LPC_SCT1->MATCHREL[1].L;
}

//getters and setters for SCTimer for three RGB colors
void setRGBValues(uint8_t red, uint8_t green, uint8_t blue){


	red = 255-red;
	green= 255 - green;
	blue=255 -blue;

	LPC_SCT0->MATCHREL[1].H = red;	// sct0 highcounter pulsewidth REDLED
	LPC_SCT0->MATCHREL[2].L = green;	// sct0 lowcounter pulsewidth GREENLED
	LPC_SCT1->MATCHREL[1].L= blue; //sct1 lowcounter pulsewidth BLUELED
}




//EXAMPLE FUNCTION SCTSETUP FOR GREENLED
void startUpSCT( ){
	LPC_SCT0->CONFIG |= (1 << 17); // two 16-bit timers, auto limit
	LPC_SCT0->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock == 72mhz / 72 == 1mhz
	LPC_SCT0->MATCHREL[0].L = 1000-1; // match 0 @ 1000/1MHz = 1000 us (1 kHz PWM freq)
	LPC_SCT0->MATCHREL[1].L = 950; // match 1 used for duty cycle (initialize at 50% pwm hopefully)

	/*NOTE!!! SET TO CORRECT PLOTTER PIN BEFORE USAGE!!!
	 *
	 *
	 * */
	/*reassigns output to greenled*/
	Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT0_O,  0,3);	//greenled port0_pin3
	LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
	LPC_SCT0->EVENT[0].CTRL = (1 << 12); // match 0 condition only
	LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
	LPC_SCT0->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only
	LPC_SCT0->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0
	LPC_SCT0->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0
	LPC_SCT0->CTRL_L &= ~(1 << 2);// unhalt it by clearing bit 2 of CTRL reg
}

//EXAMPLE FUNCTION SCTSETUP FOR SERVOACTUATING
void startUpServoSCT(){
	LPC_SCT0->CONFIG |= (1 << 17); // two 16-bit timers, auto limit
	LPC_SCT0->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock == 72mhz / 72 == 1mhz

	/*NOTE PERIOD = t must be equal to 20ms => freq= 50hz
	 * MATCHREL[1].L == 15000 BECAUSE 15000/ 1000000hz  = 0,015sec = 15ms =servo should be centered at this value of duty cycle
	 * MARCHREL[0].L == 20000 BECAUSE 20000 / 1000000hz = 0,02sec = 20ms period of signal = 50hz freq of pwm signal*/

	LPC_SCT0->MATCHREL[0].L = 20000-1; // match 0 @ 1000/1MHz = 1000 us (1 kHz PWM freq == 1000hz)
	LPC_SCT0->MATCHREL[1].L = 1500; // match 1 used for duty cycle (initialize at center for servo)

	/*reassigns output to pin
	 *
	 *NOTE !!! CHANGE THE MOVABLE PIN TO THE CORRECT PIN FOR PLOTTERPIN!!!
	 *
	 *
	 *
	 **/
	Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT0_O,  0,8);	//servopin P0.8 drive it to the center EDITED::TODO::

	LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
	LPC_SCT0->EVENT[0].CTRL = (1 << 12); // match 0 condition only
	LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
	LPC_SCT0->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only
	LPC_SCT0->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0
	LPC_SCT0->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0
	LPC_SCT0->CTRL_L &= ~(1 << 2);// unhalt it by clearing bit 2 of CTRL reg

}

//set sctimer pulsewidth FOR SERVOACTUATING
void setPWM( int amount ){
	LPC_SCT0->MATCHREL[1].L = amount;
}
//sctimer getpulsewidth FOR SERVOACTUATING
uint16_t getPulseWidth(){
	auto val= LPC_SCT0->MATCHREL[1].L;
	return val;
}

//sctimer getPeriod FOR SERVOACTUATING
uint16_t getPeriod(){
	auto temp =  LPC_SCT0->MATCHREL[0].L;
	return temp;
}



//PARSER TASK gets USB-receive and parses the Gcode commands
static void parse_task(void*pvParameters){
	bool LedState = false;
	GcodeParser parser;
	const int allocsize=80+1;
	char initialMessage[]="M10 XY 380 310 0.00 0.00 A0 B0 H0 S80 U160 D90\r\nOK\r\n";
	int initlen= strlen(initialMessage);
	char okMessage[]="OK\r\n";
	char badMessage[]="nok\r\n";
	int oklen=strlen(okMessage);
	std::string temp="";
	std::string cppstring="";
	std::string gcode="";
	while (1) {
		//str buffer is allocated
		char str[allocsize]{0};
		uint32_t len = USB_receive((uint8_t *)str, allocsize-1);
		str[len] = 0; /* make sure we have a zero at the end so that we can print the data */

		/*append strbuffer into the "oldcppstring" */
		cppstring += std::string(str);
		/*find enterIndex*/
		auto enterInd = cppstring.find('\n',0);
		//vTaskDelay(10); //vtaskdelay is used for purpose of itmprint not bugging-out
		if (enterInd != std::string::npos) {
			/*found entersign, get one line of gcode*/
			gcode = cppstring.substr(0, enterInd + 1);

			/*erase the gotten gcodeline, from the cppstring front portion, BECAUSE WE PROCESSED IT ALREADY ==>sent to parser*/
			cppstring.erase(0, enterInd + 1);

			/*send the gcode string into the parser
			 * ACTUALLY must remove entersign and cr from the gcodestring first */
			gcode.erase(enterInd,1);

			ITM_write(gcode.c_str());
			ITM_write("\r\n");
			vTaskDelay(20);
			CommandStruct cmd {CommandStruct::M1, -2147483648, false,-2147483648,-2147483648 };
			cmd = parser.parseCommand(gcode);
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

			/*EDITED parseCommand has clearTokens() at the end
			parser.clearTokens();*/
			memset(str, 0, allocsize);
			Board_LED_Set(1, LedState);
			LedState = (bool) !LedState;
			gcode="";
		}
		else{
		//else, YOU DIDNT FIND COMPLETE GCODE, WAIT FOR MORE STRBUFFER APPENDED CHARS!
		}
	}
}












int main(void) {

	prvSetupHardware();
	ITM_init();

	/*All SCtimers must be initialized in the main before usage!!!
	 *
	 * */
//	Chip_SCT_Init(LPC_SCT0);
//	Chip_SCT_Init(LPC_SCT1);

	/*sets up three SCtimers for three RGB colors*/
//	setupSCTLED();

	  // initialize RIT (= enable clocking etc.)
	    Chip_RIT_Init(LPC_RITIMER);
	    // set the priority level of the interrupt
	    // The level must be equal or lower than the maximum priority specified in FreeRTOS config
	    // Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	    NVIC_SetPriority( RITIMER_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY + 1 );

	    /*craete semaphores and commandqueue*/
	    sbRIT=xSemaphoreCreateBinary();
	    commandQueue = xQueueCreate(5, sizeof(CommandStruct));
		sbGo=xSemaphoreCreateBinary();

		/*use queueregistry to register semaphores and queues*/
		vQueueAddToRegistry( commandQueue, "comQueue" );
		vQueueAddToRegistry( sbRIT, "sbRIT" );
		vQueueAddToRegistry( sbGo, "sbGo" );


	//create binary semph
	syncSemph = xSemaphoreCreateBinary();
	/* usb parser  thread */
	xTaskCreate(parse_task, "usbparse",
				configMINIMAL_STACK_SIZE * 5, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);



	/* cdc thread */
	xTaskCreate(cdc_task, "CDC",
				configMINIMAL_STACK_SIZE * 3, NULL, (tskIDLE_PRIORITY  +1UL +1UL),
				(TaskHandle_t *) NULL);


	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
