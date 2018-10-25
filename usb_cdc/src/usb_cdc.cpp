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
#include <cmath>
#include "event_groups.h"
//regular and freertos includes ends

// User-defined classes, structs and includes
#include "GcodeParser.h"
#include "CommandStruct.h"
#include "PlotterSettings.h"
//user defined includes ends


/*CONDITINAL COMPILATION OPTIONS****************************/
//#define useLoopingBresenham	 // NOTE! this option chooses if you want to use RIT_interrupt-driven Bresenham algorithm, OR forlooping Bresenham algorithm
//#define logicAnalyzerTest

/*options and variables when using RITinterruptBresingham*/
 //variables for RITinterruptBresingham

volatile std::atomic<int> g_dx, g_dy, g_nabla;
volatile std::atomic<int> g_x_1, g_x_0, g_y_1, g_y_0;


/*********************************************************/


//freertos globals defined
QueueHandle_t commandQueue;
SemaphoreHandle_t sbRIT; //NOTE!! THIS SEMAPHORE IS NECESSARY! DOUBLECHECK MAIN() TO SEE IF CREATED CORRECTLY...
EventGroupHandle_t eventGroup;



/*Bresenham's algorithm variables, and notation is based on Bresenham's published paper
"Algorithm for computer control of a digital plotter":
IBM SYSTEMS JOURNAL * VOL. 4 * NO. 1 . 1965
by J. E. Bresenham

Bresenham's paper  can be useful reference for the pictures of motormovements for m1, and m2,
and also the tabulated results section near the end was useful. Some functions such as decidem2parameters,
decidem1parameters, getOctant, and SwapAxes were based on the journal article results.
*/

//NOTE! loopingBresenham version of Interrupt Handler function uses THIS EXTRA VARIABLE, USE CONDITIONAL COMPILATION...
volatile std::atomic<int> g_executeM1orM2(0); //check 1 or 2 inside isr to decide USED FOR loopingBresenham versio


//These global variables are shared usage between either case of conditional compilation Interrupt Handlers...
volatile std::atomic<int> g_m1parameter(0);//  determine which pin is driven into which direction, straight motorMove horiz OR vert
volatile std::atomic<int> g_m2parameter(0);//   determine which two pins are driven into which direction at the same time, diagonal motorMove
const int ppsValue = 2000;	//2000 pps works quite well with RIT-interrupt-driven-bresenhamPENCIL, AND ALSO forloopBresenhamPENCIL
volatile uint32_t g_RIT_count; //NOTE! this variable is used as amountOfHalfpulses which is double the amount of fullsteps, to drive either version of Bresenham
volatile std::atomic<bool> g_pulseState(true);//NOTE!! THIS VARIABLE IS NECESSARY for rit interrupt handlers! It is used for halfpulsing
volatile std::atomic<bool> g_limitStatusOK(true); //global variable that the tasks can read, but RIT_isr can modify if you hit the limits.
volatile std::atomic<bool> g_isEven(true);
volatile std::atomic<bool> g_expectm2(false); //boolean keeps track if you are writing to m1Pin or m2Pin in a particular halfpulse

PlotterSettings savedplottersettings; //global object keeps track of saveable and saved mDraw settings

//these are device current coords global variables for plottercoords
//REMEMBER TO UPDATE AFTER PLOTS!
/*maybe not required to be volatile?
 * NOTE the units of g_curX, g_curY and g_originX and g_originY should be in units of fullsteps!*/
volatile std::atomic<int> g_curX(250);
volatile std::atomic<int> g_curY(250);
volatile std::atomic<int> g_OriginX(0);
volatile std::atomic<int> g_OriginY(0);
const int g_STEPS_FROM_EDGE(360); //keeps the origin for G28 in a safely calibrated place, so doesn't hit the limits exactly when goes to (0,0) point
 double g_xFullstepsPerMM; //double fullstepsPerMillimetre ratio
 double g_yFullstepsPerMM;


//NOTE!! THESE GLOBAL POINTERS ARE NECESSARY!
DigitalIoPin *limitYMinP;
DigitalIoPin *limitYMaxP;
DigitalIoPin *limitXMaxP;
DigitalIoPin *limitXMinP;
DigitalIoPin *stepXP;
DigitalIoPin *dirXP;
DigitalIoPin *stepYP;
DigitalIoPin *dirYP;
DigitalIoPin *laserP;
DigitalIoPin *penP;





/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}

/**********  RIT interrupt handler for interrupt-driven Bresenham algorithm  *** USE CONDITIONAL COMPILATION TO ENABLE ***********/
#ifndef useLoopingBresenham
extern "C" {
void RIT_IRQHandler(void) { //THIS VERSION IS FOR RITinterruptBresenham

	portBASE_TYPE xHigherPriorityWoken = pdFALSE;	// This used to check if a context switch is required
	Chip_RIT_ClearIntStatus(LPC_RITIMER);	// clear IRQ flag
	if (g_RIT_count > 0) {
		g_limitStatusOK = (limitYMinP->read() && limitYMaxP->read() && limitXMaxP->read() && limitXMinP->read()); //check limits status inside ISR
		g_isEven = (g_RIT_count % 2 == 0);
		if (g_limitStatusOK) { //CALIBRATION MODE SHOULD NEVER USE RIT IRQ
			/*at each fullstep, in the beginning,
			 * use decisionparameter to iterate over bresenham
			 * When RIT_count is odd, => implies that current ISR round is basically
			 * writing false cycle to the steppins, to complete any given fullstep*/
			if (g_isEven) {
				if (g_nabla >= 0) {
					g_expectm2 = true;
					g_nabla = g_nabla + 2 * g_dy - 2 * g_dx;
				} else {
					g_expectm2 = false;
					g_nabla = g_nabla + 2 * g_dy;
				}
			}
			if (!g_expectm2) {
				if (g_m1parameter == 1 || g_m1parameter == 5) {
					stepXP->write(g_pulseState);
				} else if (g_m1parameter == 3 || g_m1parameter == 7) {
					stepYP->write(g_pulseState);
				}
				if (g_isEven) {
					switch (g_m1parameter) {
					case 1: ++g_curX; break;
					case 5: --g_curX; break;
					case 3: ++g_curY; break;
					case 7: --g_curY; break;
					}
				}
			}else {
				stepYP->write(g_pulseState);
				stepXP->write(g_pulseState);
				if (g_isEven) {
					switch (g_m2parameter) {
					case 2: ++g_curX; ++g_curY; break;
					case 4: --g_curX; ++g_curY;	break;
					case 6: --g_curX; --g_curY; break;
					case 8: ++g_curX; --g_curY; break;
					}
				}
			}
			g_pulseState = !g_pulseState;
			g_RIT_count--;
//			if (RIT_count == 0) {
//				pulseState = true; //prepare pulsestate for next G1command
//				expectm2 = false; //reset boolean in preparation for the beginning of next G1 command, so it will be false in beginning of ritstart
//				isEven = true;
//				RIT_count = 0; //reset RIT_count also, probably not needed though, because ritstart sets it up again
//				Chip_RIT_Disable(LPC_RITIMER); // disable timer
//				// Give semaphore and set context switch flag if a higher priority task was woken up
//				xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
//			}
		} else { //WE HIT THE WALL, set rit_count=0, stop stepPins, and soon the motor will stop, hopefully
			stepXP->write(false);
			stepYP->write(false);
			g_RIT_count = 0;
		}
	} else { // "iterate bresenham" has ended, prepare to reset variables and stop interrupt
		g_pulseState = true; //prepare pulsestate for next G1command
		g_expectm2 = false; //reset boolean in preparation for the beginning of next G1 command, so it will be false in beginning of ritstart
		g_RIT_count = 0; //reset RIT_count also, probably not needed though, because ritstart sets it up again
		Chip_RIT_Disable(LPC_RITIMER); // disable timer
		// Give semaphore and set context switch flag if a higher priority task was woken up
		xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	}
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);

}
}

#endif
/*********************************************************************************************************************************/


/*** RIT interrupt handler for looping-style Bresenham algorithm, USE CONDITIONAL COMPILATION TO ENABLE ***/
#ifdef useLoopingBresenham
extern "C" {
void RIT_IRQHandler(void) {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;	// This used to check if a context switch is required
	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag


		if (g_RIT_count > 0) {
			g_isEven= (g_RIT_count % 2 == 0);
			g_RIT_count--;
			// do something useful here...
			g_limitStatusOK = (  limitYMinP->read() && limitYMaxP->read() && limitXMaxP->read() && limitXMinP->read()  );
			if (g_limitStatusOK) {
				if(g_executeM1orM2==1) { //execute M1 pattern move
					switch(g_m1parameter) { //actuate only one motor straight move
					case 1: stepXP->write(g_pulseState); break;
					case 3: stepYP->write(g_pulseState); break;
					case 5: stepXP->write(g_pulseState); break;
					case 7: stepYP->write(g_pulseState); break;
					default: break;//shouldnt be here
					}
					if(g_isEven){
						switch(g_m1parameter){
						case 1: g_curX++; break;
						case 3: g_curY++; break;
						case 5: g_curX--; break;
						case 7: g_curY--; break;
						}
					}
				}else if(g_executeM1orM2 == 2){//execute M2 pattern move
					stepXP->write(g_pulseState);//actuate both motors diagonal move
					stepYP->write(g_pulseState);
					if (g_isEven) {
						switch(g_m2parameter){
						case 2: g_curX++; g_curY++; break;
						case 4: g_curX--; g_curY++; break;
						case 6: g_curX--; g_curY--; break;
						case 8: g_curX++; g_curY--; break;
						}
					}
				}
				g_pulseState = !g_pulseState;//move motor and toggle pulsestate rit_halfpulses
				if(g_RIT_count == 0){
					g_pulseState = true; //prepare pulsestate for next G1command
					g_expectm2 = false; //reset boolean in preparation for the beginning of next G1 command, so it will be false in beginning of ritstart
					g_isEven=true;
//					stepXP->write(false);
//					stepYP->write(false);
					g_RIT_count = 0; //reset RIT_count also, probably not needed though, because ritstart sets it up again
					Chip_RIT_Disable(LPC_RITIMER); // disable timer
					// Give semaphore and set context switch flag if a higher priority task was woken up
					xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
				}
			} else {//WE HIT THE WALL, set rit_count=0 and soon the motor will stop, hopefully
				stepXP->write(false);
				stepYP->write(false);
				g_RIT_count = 0;
			}
		} else {
			g_pulseState=true;
//			stepXP->write(false);
//			stepYP->write(false);
			Chip_RIT_Disable(LPC_RITIMER); // disable timer
			// Give semaphore and set context switch flag if a higher priority task was woken up
			xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
		}
		// End the ISR and (possibly) do a context switch
		portEND_SWITCHING_ISR(xHigherPriorityWoken);


}
}
#endif
/**********************************************************************************************************/







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
	g_RIT_count = count;
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


void swapDigitalIoPins(DigitalIoPin &p1, DigitalIoPin &p2){

	std::swap(p1,p2);

}

/*helper functions for RIT_INTERRUPT BRESENHAM */
int refactored_getOctant(const int orig_dx, const int orig_dy){
	/*check booleans to determine octant number
	later inside adjustm1motor and adjustm2motor
	we determine those movemeentpatterns with the octant number
	A = deltaX < 0
	B = deltaY < 0
	C = 0 < abs(x1 - x0) - abs(y1 - y0)

	This helper function was codedd basd on bresenhams tabulated information table at the end
	NOTE!!
	based on my test it should be executed before any swaps are made. You can verify
	this function with pen and paper and draw triangles into unit circle and look at the angle
	*/
	bool A, B, C;
	A = (orig_dx >= 0);
	B = (orig_dy >= 0);
	C = ( abs(orig_dx) - abs(orig_dy) >= 0  );

	if (A && B && C) {
		return 1; //oct1
	} else if (A && B && !C) {
		return 2; //oct2
	} else if (A && !B && C) {
		return 8; //oct8
	} else if (A && !B && !C) {
		return 7; //oct7
	} else if (!A && B && C) {
		return 4; //oct4
	} else if (!A && B && !C) {
		return 3; //oct3
	} else if (!A && !B && C) {
		return 5; //oct 5
	} else {
		return 6; //oct 6
	}


}

/*helper functions for RIT_INTERRUPT BRESENHAM */
int refactored_decideM1Parameter(const int octant){
	/*from bresenhams original researchpaper, near the
	tabulated data section

	this function requires the correctly working and correct
	octant value for the G1move to deduce the m1pattern*/
	int pattern1(0);
	switch (octant) {
		case 1: pattern1 = 1; break;
		case 2: pattern1 = 3; break;
		case 3: pattern1 = 3; break;
		case 4: pattern1 = 5; break;
		case 5: pattern1 = 5; break;
		case 6: pattern1 = 7; break;
		case 7: pattern1 = 7; break;
		case 8: pattern1 = 1; break;
		default:break;//shoudlnt be here
	}

	return pattern1;

}

/*helper functions for RIT_INTERRUPT BRESENHAM */
int refactored_decideM2Parameter(const int octant){
	/*from bresenhams original researchpaper, near the
	tabulated data section

	this function requires the correctly working and correct
	octant value for the G1move to deduce the m2pattern*/
	int pattern2(0);
	switch (octant) {
		case 1: pattern2 = 2; break;
		case 2: pattern2 = 2; break;
		case 3: pattern2 = 4; break;
		case 4: pattern2 = 4; break;
		case 5: pattern2 = 6; break;
		case 6: pattern2 = 6; break;
		case 7: pattern2 = 8; break;
		case 8: pattern2 = 8; break;
		default:break;//shoudlnt be here
	}

	//decide and set global variable m2parameter
	return pattern2;

}

/*helper functions for RIT_INTERRUPT BRESENHAM */
char refactored_decideSwapAxes(const int octant){
	/*function is used BEFORE executing bresenhams looping
	OR bresenhams ISR handler function

	with the already computed data from getOctant
	we decide if swaps of global variables are necessary

	it is needed to implement bresenhams for all angles

	NOTICE SWAPS GLOBAL VARIABLES!!!
	dy, dx, x, y, nabla
	returns the drivingAxis char or some information for debugging
	maybe full version doesnt need to return
	if swapped, returns y axis
	else returns regular x axis*/

	/*if true, then perform swap of driving axis, else keep variables
	 * and driving axis will be x-axis*/
	if( octant == 2 ||
		octant == 7 ||
		octant == 3 ||
		octant == 6 ) {
		/*recompute global variables in preparation
		for first check of the bresenham*/

		std::swap(g_x_1, g_y_1);
		std::swap(g_x_0, g_y_0);
		g_dx = abs(g_x_1 - g_x_0 );
		g_dy = abs( g_y_1 - g_y_0 );
		g_nabla = 2 * g_dy - g_dx;	//initial value of nabla for first check inside ISR
		return 'Y';
	}

	else{
		/*apparently ISRversion of Bresenham uses absolute value
		 * according to the original paper */
		g_dx = abs(g_dx);
		g_dy = abs(g_dy);
		g_nabla = 2 * g_dy - g_dx;
		return 'X';
	}
}

/*helper functions for RIT_INTERRUPT BRESENHAM */
void testing_setupBresenhamDirPins(const int m1param, const int m2param){

	switch(m1param){
	case 1:
		if(m2param==2){ //prepare m2Motormove for diagonal right&up oct1, and m1Motormove horiz +
			dirYP->write(false);
			dirXP->write(false);
		}

		else if(m2param==8){//prepare m2Motormove for diagonal right&down oct8, and m1Motormove horiz +
			dirXP->write(false);
			dirYP->write(true);
		}
		break;

	case 3:
		if(m2param==2){ //prepare for m2Motormove for diagonal right&up oct2, and m1Motormove vert +
			dirYP->write(false);
			dirXP->write(false);
		}
		else if(m2param==4){//prepare for m2Motormove diagonal left&up oct3, and m1Motormove vert +
			dirYP->write(false);
			dirXP->write(true);
		}
		break;
	case 5:
		if(m2param==4){//prepare for m2Motormove diagonal left&up oct4, and m1Motormove horiz -
			dirYP->write(false);
			dirXP->write(true);
		}
		else if(m2param==6){//prepare for m2Motormove diagonal left&down oct5, m1Motormove horiz -
			dirYP->write(true);
			dirXP->write(true);
		}
		break;
	case 7:
		if(m2param==8){ //prepare for m2Motormove diagonal right&down oct7, m1Motormove vert -
			dirXP->write(false);
			dirYP->write(true);
		}
		else if(m2param==6){//prepare for m2Motormove diagonal left&down oct6, m1Motormove vert -
			dirXP->write(true);
			dirYP->write(true);
		}
		break;
	default: break; //never should happen!
	}
}


/*main function for  RIT_INTERRUPT BRESENHAM, starts interrupt bresenham*/
void refactored_BresenhamInterruptAlgorithm(int x0, int y0, int x1, int y1){

	/*NOTE!
	 * call this function only with the integer amount of rounded stepCoordinates
	 * such that you are moving the closest possible amount (rounded amount) of fullsteps
	 * which correspond to the required G1 command millimetres movement */

	const int raw_dx = x1 - x0;
	const int raw_dy = y1 - y0;
	g_x_0 = x0;
	g_x_1 = x1;
	g_y_0 = y0;
	g_y_1 = y1;
	g_dx = g_x_1 - g_x_0;
	g_dy = g_y_1 - g_y_0;
	g_nabla = 0;
	//global nabla is hopefully properly initialized in the refactored_decideSwapAxes
	const int oct = refactored_getOctant(raw_dx, raw_dy);
	g_m1parameter = refactored_decideM1Parameter(oct);
	g_m2parameter = refactored_decideM2Parameter(oct);

	//refactored_setupBresenhamDirPins(m1parameter, m2parameter);
	testing_setupBresenhamDirPins(g_m1parameter, g_m2parameter); //this function is cleaner code, it was refactored from the other refactored_setupBresenhamDirPins()
	refactored_decideSwapAxes(oct); //knowing the drivingAxis returnvalue from refactored_decideSwapAxes() isnt important except for debugging,


	const int fullsteps =  abs(g_dx);


	//NOTE! always ritstart with EVEN numbers because by definition, you are halfpulsing the fullpulses
	RIT_start( 2 * fullsteps,  ( 1000000 / (2*ppsValue) )    );
	vTaskDelay(2);

}




//helper functions for FORLOOPING BRESENHAM
//returns currently active motorpattern (horiz or vert motormove) and sets pins
int adjustm1motor(int curOctant) {
	/*m1 and m1 patter motor movements are based on Bresenham's original paper
	 * "Algorithm for computer control of a digital plotter (Figure 5)"
	 * where he describes movements of digital plotter in conjunction with
	 * the bresenham line algorithm (m1 and m2 patterns are the
	 * actual paths of plotter)
	 * typically m1 pattern is either horizontal or vertical move of one motor
	 * typically m2 pattern is a diagonal move e.g. 45deg angle, which reguires both motors stepping*/
	int M1pattern = 0;
	switch (curOctant) {
		case 1: M1pattern = 1;break;
		case 2: M1pattern = 3;break;
		case 3: M1pattern = 3;break;
		case 4: M1pattern = 5;break;
		case 5: M1pattern = 5;break;
		case 6: M1pattern = 7;break;
		case 7: M1pattern = 7;break;
		case 8: M1pattern = 1;break;
			default:break;//shoudlnt be here
	}
	switch (M1pattern) {
		case 1:
			//XMOTOR RIGHT
			dirXP->write(false); break;
		case 3:
			//YMOTOR UP
			dirYP->write(false); break;
		case 5:
			//XMOTOR LEFT
			dirXP->write(true); break;
		case 7:
			//YMOTOR DOWN
			dirYP->write(true); break;
		default:break;//shoudlnt be here

	}
	return M1pattern;

}

//helper functions for FORLOOPING BRESENHAM
//returns currently active motorpattern, (diagonal motormove) and sets pins
int adjustm2motor(int curOctant) {
	/*m1 and m1 patter motor movements are based on Bresenham's original paper
	 * "Algorithm for computer control of a digital plotter (Figure 5)"
	 * where he describes movements of digital plotter in conjunction with
	 * the bresenham line algorithm (m1 and m2 patterns are the
	 * actual paths of plotter)
	 * typically m1 pattern is either horizontal or vertical move of one motor
	 * typically m2 pattern is a diagonal move e.g. 45deg angle, which reguires both motors stepping*/
	int M2pattern = 0;
	switch (curOctant) {
		case 1: M2pattern = 2; break;
		case 2: M2pattern = 2; break;
		case 3: M2pattern = 4; break;
		case 4: M2pattern = 4; break;
		case 5: M2pattern = 6; break;
		case 6: M2pattern = 6; break;
		case 7: M2pattern = 8; break;
		case 8: M2pattern = 8; break;
		default:break; //shoudlnt be here
	}

	switch(M2pattern){

	case 2: //xmotor right ymotor up
		dirXP->write(false);
		dirYP->write(false);
		break;
	case 4: //xmotor left ymotor up
		dirXP->write(true);
		dirYP->write(false);
		break;
	case 6: //xmotor left ymotor down
		dirXP->write(true);
		dirYP->write(true);
		break;
	case 8: //xmotor right ymotor down
		dirXP->write(false);
		dirYP->write(true);
		break;
	default:break;
	}


	return M2pattern;
}


/*LASER SETUP, using SCTimer0Large highcounter*/
void setupLaserPWM(){

	LPC_SCT0->CONFIG |= (3 << 17);	 // two 16-bit timers, auto limit, use  the HIGHCOUNTER FOR LASER, (use lowcounter for pencilservo)
	LPC_SCT0->CTRL_H |= (72 - 1) << 21; 	// set prescaler highcounter, SCTimer/PWM clock == 72mhz / 72 == 1mhz
	LPC_SCT0->MATCHREL[0].H = 1000 - 1; 	//sct0 highcounter  freq
	LPC_SCT0->MATCHREL[1].H = 235; 	// sct0 highcounter pulsewidth laser
	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT1_O, 0, 12);  //set laserpin to sct0 output1

	/*configure events, freq event for laser, and pulsewidthmatch event for laser*/
	LPC_SCT0->EVENT[2].STATE = 0xFFFFFFFF; //all states allowed event2, use for freqmatch
	LPC_SCT0->EVENT[3].STATE = 0xFFFFFFFF; //all states allowed event3, use for pulsewidthmatch

	LPC_SCT0->EVENT[2].CTRL = (1<<4) |(1<<12)  ; 	//event2 sct0 highcounter frequency match, select reg0 (matchrel[0].h)
	LPC_SCT0->EVENT[3].CTRL = (1<<4) | (1<<12) | (1); 	//event3 sct0 highcounter pulsewidthmatch, HEVENTbitTrue,  select reg1 (matchrel[1].h)

	/*set outputs with freq events*/
	LPC_SCT0->OUT[1].SET = (1<<2); //event2 sets sct0output1

	/*clears outputs with pulsewidthmatch events*/
	LPC_SCT0->OUT[1].CLR = 	1<<3;	//event3 clears sct0output1

	/*unhalt timers*/
	LPC_SCT0->CTRL_H &= ~(1<<2);  //unhalt sct0 highcounter
}

/*LASER SET NEW VALUE, using SCTimer0Large highcounter*/
void setLaserValue(int amount){
	bool allowed = (amount >= 0) &&  (amount <= 255);
	int newval = 0;

	/*put the amount [0,255] directly into laser pulsewidthmatch register, after it was legally within range*/
	if(allowed){
		newval = 3 * amount +235;
		LPC_SCT0->MATCHREL[1].H = newval;
	}

}


/*PEN-SERVO SETUP FOR THE PROJECT, using SCTimer0Large*/
void setupPenServo(){
	LPC_SCT0->CONFIG |= (1 << 17); // two 16-bit timers, auto limit, use only the one counter... leave other un-used
		LPC_SCT0->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock == 72mhz / 72 == 1mhz

		//matchrel1 controls duty cycle , min=1000, max=2000, center=1500
		//matchrel0 controls the main frequency
		LPC_SCT0->MATCHREL[0].L = 20000-1; // match 0 @ 20000 / 1000000Hz =  0,02s period = 50Hz drivingfrequency, as required
		LPC_SCT0->MATCHREL[1].L = 1500; // match 1 used for duty cycle (initialize at center for servo), period = 1500 / 1000000 = 0,0015s = 1.5ms period, as required, for center position

		/*reassigns output to pin
		 *
		 *NOTE !!! CHANGE THE MOVABLE PIN TO THE CORRECT PIN FOR PLOTTERPIN!!!
		 *
		 **/
		Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT0_O,  0,10);	//SERVOPIN FOR PEN P_0.10 drive it to the center

		LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
		LPC_SCT0->EVENT[0].CTRL = (1 << 12); // match 0 condition only
		LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
		LPC_SCT0->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only
		LPC_SCT0->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0 output, then we get dutycycle
		LPC_SCT0->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0
		LPC_SCT0->CTRL_L &= ~(1 << 2);// unhalt it by clearing bit 2 of CTRL reg
}

/*PEN-SERVO SET NEW VALUE FOR THE PROJECT, using SCTimer0Large*/
void setPenValue(int amount) {
	bool allowed = false;
	int newval = 0;
	allowed = (0 <= amount) && (amount <= 255);

	//map the smaller values from [0, 1, 2, ... 254, 255]
	//into the larger values from [1000,1001,1002... 1500,1501...1999,2000]
	if (allowed) {
		if (amount > 250) {
			newval = 2000;
		} else {
			//check what value it was and increment by correct amount
			newval = 4 * amount + 1000;
		}
		LPC_SCT0->MATCHREL[1].L = newval;
	}
}



//useful functions for calibrating!!! calibration mode functions to count the steps and get current location
void stepVert(){
	stepYP->write(true);
	vTaskDelay(1);
	stepYP->write(false);
	vTaskDelay(1);
}


void stepHoriz(){
	stepXP->write(true);
	vTaskDelay(1);
	stepXP->write(false);
	vTaskDelay(1);
}





//helper functions for FORLOOPING BRESENHAM
int getOctant(bool A, bool B, bool C) {
	/*function is based on Bresingham's original paper, in the tabulated information section about the algorithm
	 *
	 * check booleans to determine octant number
	later inside adjustm1motor and adjustm2motor
	we determine those movemeentpatterns with the octant number
	A = deltaX < 0
	B = deltaY < 0
	C = 0 < abs(x1 - x0) - abs(y1 - y0)
	*/
	if (!A && !B && !C) {
		return 1; //oct1
	}
	else if (!A && !B && C) {
		return 2; //oct2
	}
	else if (!A && B && !C) {
		return 8; //oct8
	}
	else if (!A && B && C) {
		return 7; //oct7
	}
	else if (A && !B && !C) {
		return 4;
	}
	else if (A && !B && C) {
		return 3;
	}
	else if (A && B && !C) {
		return 5;
	}
	else {
		return 6;
	}
}

//helper functions for FORLOOPING BRESENHAM
//special case for purely horizontal moves BRESENHAM loopingBresenham
void plotLineHoriz(int x0, int y0, int x1, int y1) {
	int dx = x1 - x0;
	g_executeM1orM2 = 1; //notify isr global variable which patternm1 or patternm2 to execute, in this case only one motor move => hence m1pattern
	if (dx > 0) {
		//xmotordir right
		dirXP->write(false);
		g_m1parameter = 1; //set global volatile variable to notify isr which m1 motor to actjuate
	} else {
		//xmotordir left
		dirXP->write(true);
		g_m1parameter = 5;
	}


	for (int x = 1; x <= abs(dx); x++) {
		//write to motordir to drive left or right
		RIT_start(2, 1000000 / (2 * ppsValue));
	}
}

//helper functions for FORLOOPING BRESENHAM
//special case for purely vertical moves BRESENHAM loopingBresenham
void plotLineVert(int x0, int y0, int x1, int y1) {

	int dy = y1 - y0;
	g_executeM1orM2 = 1; //notify isr which patternm1 or patternm2 to execute, in this case only one motor move => hence m1pattern
	if (dy > 0) {
		//y motor up
		g_m1parameter = 3; //notify isr with global variable to expect ymotor up
		dirYP->write(false);
	} else {
		//ymotor down
		g_m1parameter = 7; //notify isr with global variable to expect ymotor down
		dirYP->write(true);
	}

	for (int y = 1; y <= abs(dy); y++) {
		//write motorpin up or down
		RIT_start(2, 1000000 / (2 * ppsValue));
	}
}

//helper functions for FORLOOPING BRESENHAM
void plotLineLow(int x0, int y0, int x1, int y1, bool makeSwap) {
	if (makeSwap) {
		std::swap(x0, x1);
		std::swap(y0, y1);
	}
	int dx=x1-x0, dy=y1-y0;

	dx = abs(x1 - x0);
	dy = abs(y1 - y0);

	int nabla = 2 * dy - dx; //decision parameter
	int x = x0;

	for (x; x < x1; x++) { //x is driving axis
		if (nabla > 0) {
			nabla = nabla - 2 * dx; //update decisionparameter nabla
			/*execute diagonal m2 move (step x and y motor at same time)???
			 m2 move type depends on the octant
			 m1 move type depends on octant also (m1 is either horiz or vert move)*/
			g_executeM1orM2 = 2; //set global volatile boolean, so that interrupthandler knows which movementpattern to execute
			RIT_start(2, 1000000 / (2 * ppsValue));

		}
		else {
			//execute m1 move
			g_executeM1orM2 = 1;
			RIT_start(2, 1000000/(2*ppsValue) );
		}
		nabla = nabla + 2 * dy;//update decisionparameter nabla
	}
}

//helper functions for FORLOOPING BRESENHAM
void plotLineHigh(int x0, int y0, int x1, int y1, bool makeSwap) {
	if(makeSwap){
		std::swap(x0,x1);
		std::swap(y0,y1);
	}


	int dx=x1-x0, dy=y1-y0;
	 dx = abs(x1 - x0);
	 dy = abs(y1 - y0);

	int nabla = 2 * dx - dy; //decision parameter
	int y = y0;

	for (y; y < y1; y++) { //y is driving axis
		if (nabla > 0) {
			nabla = nabla - 2 * dy;//update decisionparameter nabla
			//execute diagonal m2 move (step x and y motor at same time)
			// m2 move type depends on the octant
			//m1 move type depends on octant also (m1 is either horiz or vert move)
			g_executeM1orM2 = 2;
			RIT_start(2, 1000000/(2*ppsValue) );
		}
		else {
			//execute m1 move
			g_executeM1orM2 = 1;
			RIT_start(2, 1000000/(2*ppsValue) );
		}
		nabla = nabla + 2 * dx;//update decisionparameter nabla
	}
}

//BRESENHAM MAIN FUNCTION FORLOOPING BRESENHAM
void plotLineGeneral(int x0, int y0, int x1, int y1) {
	/*octant notation goes from
	 angle 0deg to 45deg == oct 1
	 angle 45deg to 90deg == oct 2
	 etc...
	 */
	int oct = 0;

	/*NOTE!! if you get same startpoint and endingpoint, DONT draw anything, GOTO ENDINGPOINT*/
	if (x0 == x1 && y0 == y1) { //zero length line, dont draw anything for safety purposes!
		goto endingpoint;
	}

	if (x0 == x1) {
		//plot vertical case, special case
		plotLineVert(x0, y0, x1, y1);
	} else if (y0 == y1) {
		//plot horizontal case, special case
		plotLineHoriz(x0, y0, x1, y1);
	}

	/*Plot regular case looping bresenham*/
	else {
		bool res = abs(y1 - y0) < abs(x1 - x0);
		bool doSwap = false;
		oct = getOctant((x1 - x0 < 0),
						(y1 - y0 < 0),
						(abs(x1 - x0) - abs(y1 - y0) < 0));

		g_m2parameter = adjustm2motor(oct);
		g_m1parameter = adjustm1motor(oct);

		if (res) {
			if (x0 > x1) {
				doSwap = true;
			}
			plotLineLow(x0, y0, x1, y1, doSwap);
		} else {
			if (y0 > y1) {
				doSwap = true;
			}
			plotLineHigh(x0, y0, x1, y1, doSwap);
		}
	}

	endingpoint:  //ending label for goto, for the special case of same startpoint and same endpoint, dont do anything
	vTaskDelay(2);
	int kakka1=0;	//I think we maybe have to have the statement after endingpoint label to compile?!
}




//execute_task unblocks AFTER CALIBRATION PHASE ENDS (execute_task blocks on evengroup)
static void execute_task(void*pvParameters) {


	xEventGroupWaitBits(eventGroup, 0x1, pdFALSE, pdFALSE, portMAX_DELAY);	//wait until real_calibration_task finishes!

	char badMessage[] = "not_ack_wasIllegalCommand\r\n";
	const int badlen = strlen(badMessage);
	char okMessage[] = "OK\r\n";
	const int oklen = strlen(okMessage);
	CommandStruct curcmd;

	vTaskDelay(100);


/*execute_task mainloop is conditional based on the status of limitStatusOK, which
 * can change, during regualar operations, only inside RIT_isr (you could push
 *  the limitswitch with your finger, but then motor will not be "the limithitting part")
 *
 * NOTE! the motor can move, ONLY thru the usage of RIT_isr during regular operation, when
 * limitswitches must be polled
 *
 * NOTE! you are allowd to hit the limits during real_calibration_task, BUT NO LONGER ALLOWED
 * during regular operation between producer-consumer (parse_task and execute_task)
 * */
	for( ; g_limitStatusOK == true ; ){
		xQueueReceive(commandQueue, &curcmd, portMAX_DELAY); //get command from queue

		/*process G1 command, and make appropriate usb_send response*/
		if(curcmd.commandWord == CommandStruct::G1){
			//assume 1 fullstep == 1.0mm, in keijosimulator (it was, actually, 1step == 1.0mm )
			/*NOTE!
			 * you must use mDraw imagesize == keijosimulator imagesize, for the real_calibration_task
			 * to properly count the steps and calibrate the simulator and scale the image.
			 * */

			/* how to plot from Gcode millimetres into plotter steps - the PlottingProcedure
			 *
			 * 1.) convert int hundredthsXCoordMillimetre into double millimetres
			 * 2.)  multiply double millimetres  with double XfullstepsPerMillimetre (the latter came from real_calibration_task)
			 * 3.) get double fullstepsamount as result of multiplication
			 * 4.) round double fullstepsamount into int roundX
			 * 5.) always move int roundX amount of fullsteps  with plotter, plotter moves inside int valued coordinate-system
			 * 6.) remember to update int valued globalXcoord and globalYcoord after each line plot move explicitly after having plotted*/

			double tempx = g_xFullstepsPerMM * ( ((double)curcmd.xCoord) / 100.0); //note real program should have "identical" or "very much the same" Xstep/mm ratio == Ystep/mm ratio
			double tempy = g_yFullstepsPerMM * ( ((double)curcmd.yCoord) / 100.0);
			int roundX = std::round(tempx);
			int roundY = std::round(tempy);

#ifndef useLoopingBresenham //ritInterrupt-Driven Bresenham
			refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, roundX,roundY );

//			g_curX = roundX;//update current coords to the "dest coords, after move"
//			g_curY = roundY;
			USB_send((uint8_t*) okMessage, oklen);
#endif
#ifdef useLoopingBresenham //forlooping Bresenham
			plotLineGeneral(g_curX, g_curY, roundX,roundY);
//			g_curX = roundX;
//			g_curY = roundY;
			USB_send((uint8_t*) okMessage, oklen);
#endif

		}

		/*process M1 command (pencilServo move), and make appropriate usb_send response*/
		else if(curcmd.commandWord == CommandStruct::M1){
			setPenValue(curcmd.commandNumber);
			USB_send((uint8_t*) okMessage, oklen);
			vTaskDelay(100); //small delay to allow pencilservo time to settle
		}

		/*process M10 command, and make appropriate usb_send response*/
		else if(curcmd.commandWord == CommandStruct::M10){
			std::string M10responsemessage = savedplottersettings.getM10ResponseMessage(); //responds to M10message properly, with formated reply
			USB_send((uint8_t*) M10responsemessage.c_str(), strlen(M10responsemessage.c_str()));
		}

		/*process M11 limitquery, and make appropriate usb_send response */
		else if(curcmd.commandWord == CommandStruct::M11){
			std::string M11LimitResponse = savedplottersettings.getM11LimitResponseMessage();
			USB_send((uint8_t*) M11LimitResponse.c_str(), strlen(M11LimitResponse.c_str()));
		}

		/*process M2 (save pencilservo values), and make appropriate usb_send response*/
		else if(curcmd.commandWord == CommandStruct::M2){
			savedplottersettings.setPenUp(curcmd.penUp); //saves new pencilServovalues to memory
			savedplottersettings.setPenDown(curcmd.penDown);
			USB_send((uint8_t*) okMessage, oklen);
		}

		/*process M5 (getNewValues)*/
		else if(curcmd.commandWord == CommandStruct::M5){
			savedplottersettings.updateM5Values(curcmd); //save M5values to memory, in order to prepare for next M10message
			USB_send((uint8_t*) okMessage, oklen);
		}

		/*process G28 (goto origin), and make appropriate usb_send response*/
		else if(curcmd.commandWord == CommandStruct::G28){
			/*if already at origincoords, dont move!*/
			if(g_curX == 0 && g_curY == 0){
				USB_send((uint8_t*) okMessage, oklen);//dontmove, BUT DO make appropriate usb_send response
			}
			else{
#ifndef useLoopingBresenham  	//use ritInterrupt-Driven Bresenham, for G28 move
				refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, g_OriginX, g_OriginY ); 	//goto origin, from curCoords
//				g_curX = g_OriginX; 	//update current coords to the "dest coords, after move"
//				g_curY = g_OriginY;
				USB_send((uint8_t*) okMessage, oklen);
#endif
#ifdef useLoopingBresenham 	//use forlooping Bresenham, for G28 move
				plotLineGeneral(g_curX, g_curY, g_OriginX, g_OriginY);
//				g_curX = g_OriginX;
//				g_curY = g_OriginY;
				USB_send((uint8_t*) okMessage, oklen);
#endif
			}
		}

		/*process M4 (lasercommand) AND M28_start_stepper_calibration DONT ALLOW THESE COMMANDS ANYMORE at this stage*/
		else if(curcmd.commandWord == CommandStruct::M4 ){
			setLaserValue(curcmd.commandNumber);
			USB_send((uint8_t*) okMessage, oklen);
		}
		else{
			//send nak_message (notAcknowledge_wasIllegalCommand)
			USB_send((uint8_t*) badMessage, badlen);
		}

	}

	vTaskSuspend(NULL); 	//NOTE! if you hit the limitswitches, mainloop will end-up here! CRITICAL ERROR IN PROGRAM!

}


//parse_task unblocks after calibration phase ends (parse_task blocks on eventgroup)
static void parse_task(void*pvParameters) {


	xEventGroupWaitBits(eventGroup, 0x1, pdFALSE, pdFALSE, portMAX_DELAY);	//wait until real_calibration_task finishes!


	GcodeParser parser;
	const int allocsize = 80 + 1;
	char badMessage[] = "nok\r\n";
	const int badlen = strlen(badMessage);
	std::string cppstring;
	std::string gcode;

	vTaskDelay(100);	//small initial delay

	/*parse_task mainloop is conditional based on the status of limitStatusOK, which
	 * can change, during regualar operations, only inside RIT_isr (you could push
	 * the limitswitch with your finger, but then motor will not be "the limithitting part")
	 *
	 * NOTE! the motor can move, ONLY thru the usage of RIT_isr during regular operation, when
	 * limitswitches must be polled...
	 *
	 * NOTE! you are allowd to hit the limitswitches during real_calibration_task, BUT NO LONGER ALLOWED
	 * during regular operation between producer-consumer (parse_task and execute_task)
	 * */

	for ( ; g_limitStatusOK == true ; ) {

		char str[allocsize] { 0 };		//str buffer is allocated
		uint32_t len = USB_receive((uint8_t *) str, allocsize - 1);
		str[len] = 0; 	// make sure we have a zero at the end so that we can print the data
		cppstring += std::string(str); 	//append strbuffer into the "oldcppstring"
		auto enterInd = cppstring.find('\n', 0); 	//find enterIndex

		if (enterInd != std::string::npos) {
			gcode = cppstring.substr(0, enterInd);//found entersign, get one line of gcode
			cppstring.erase(0, enterInd + 1);//erase the gotten gcodeline, from the cppstring front portion, BECAUSE WE PROCESSED IT ALREADY ==>sent to parser

			CommandStruct cmd = parser.parseCommand(gcode);	//send the gcode string into the parser

			if (cmd.isLegal && cmd.commandWord != CommandStruct::uninitialized) {
				xQueueSendToBack(commandQueue, &cmd, portMAX_DELAY); 	//any legal message => send into queue => executetask reads command =>  appropriate USB_send reply
			} else {
				USB_send((uint8_t*) badMessage, badlen);
			}
			memset(str, 0, allocsize);
			gcode = "";
		}


		//else, YOU DIDNT FIND COMPLETE GCODE, WAIT FOR MORE STRBUFFER APPENDED CHARS!

	}

	vTaskSuspend(NULL); 	//NOTE! if you hit the limitswitches, mainloop will end-up here! CRITICAL ERROR IN PROGRAM!

}


//real_calibrate_task starts first among user-defined tasks, and allows servopencil calibration
//and also steppermotor calibration. And, after the steppercalibration has calibrated,
//then, real_calibration_task sets eventgroup bit true, and unblocks the parse_task and execute_task
//finally real_calibration_task suspends itself
static void real_calibrate_task(void*pvParameters){
	/*NOTE! this calibration task starts first and initializes pins, assigns pointers
	 * and also calibrates steppermotors,
	 * NOTE! moving commands from mDraw will be disabled/ignored during calibration phase
	 * ONLY pencilServo commands and plotArea changes are allowed during calibration phase*/

	/*Allowable commands from mDraw at this point during calibration are as follows:
	 * M1 pencilServo
	 * M10 settings command
	 * M2 savePencilValues
	 * M5 savePlotArea etc... into PlotterSettings
	 * M11 limitQuery
	 * M28 begin stepper calibration procedure
	 *
	 * Then, after M28 is gotten, you wait until steppercalibration finishes, and then you set EventGroupBit true
	 *  other user tasks were waiting on the EventGroup bit, so now they are unblocked and ready to work*/

	//laser and pen pins, hopefully correct???
	DigitalIoPin pen(0,10, DigitalIoPin::pullup, true);
	DigitalIoPin laser(0,12,DigitalIoPin::output, true);

	//drive laserpin low
	laser.write(false);
	laserP = &laser;

	//regular pin setup, as per pdf pin layout guide!!!
	DigitalIoPin dirX(0, 28, DigitalIoPin::output, true);
	DigitalIoPin dirY(1, 0, DigitalIoPin::output, true);
	DigitalIoPin stepY(0, 24, DigitalIoPin::output, true);
	DigitalIoPin stepX(0, 27, DigitalIoPin::output, true);

	DigitalIoPin limitYMin(1, 3, DigitalIoPin::pullup, false);
	DigitalIoPin limitYMax(0, 0, DigitalIoPin::pullup, false);
	DigitalIoPin limitXMax(0, 9, DigitalIoPin::pullup, false);
	DigitalIoPin limitXMin(0, 29, DigitalIoPin::pullup, false);

	//assign global pointers
	limitYMinP = &limitYMin;
	limitYMaxP = &limitYMax;
	limitXMinP = &limitXMin;
	limitXMaxP = &limitXMax;

	//step, dir and pen pointers should remain same always
	stepXP = &stepX;
	dirXP = &dirX;
	stepYP = &stepY;
	dirYP = &dirY;
	penP = &pen;

	/*NOTE!
	 * because of the DigitalIoPin class was written in a particular way,
	 * we can use a simple strategy to expectCertainLimitpin and detectALimitpin in
	 * calibration phase and if those pins are different, then swap-by-value.
	 *
	 * We can simply use swap-by-value, because DigitalIoPin constructor sets-up
	 * the ChipAPIfunctions, but afterwards DigitalIoPin object relies on its own datamembers
	 * for any and all function calls for memberfunctions
	 *
	 * Therefore, it doesnt matter "inside which DigitalIoPin's constructor" certain pins and ports were initialized for the purpose of PinMuxSet,
	 * BUT the only thing what matters, is that the correct pin and port datamember is used to call the DigitalIoPin memberfunctions.
	 * HENCE, it should be enough that you keep the limitpointers pointing at the correct limitpin objects, but swap-by-value the datamembers.
	 *
	 * Therefore, for ease of use it is best to avoid pointerswapping. Simply swap-by-value, so you can still get
	 * the correct datamembers copied into the correct DigitalIoPin, and then you can simply use
	 * the correct name of the pinObject as you would regularly.
	 *
	 * pointer swapping isn't necessary either.
	 * */
	savedplottersettings.setLimitPointers(limitYMaxP, limitYMinP, limitXMaxP, limitXMinP);

	GcodeParser calibrationParser; 		//should  allow ONLY configuration commands and savesettings commands and pencilservo commands from mDraw during calibrationphase
	std::atomic<int> xsteps(0), ysteps(0); 		//counted steps
	std::atomic<int> yTouches(0), xTouches(0);	    //count the touchesAtEdge, to make it only count one full length between limits
	std::atomic<bool> countingY(false), countingX(false);  //booleans to trigger counting on/off

	/*variables to detect limits and identifies which is which*/
	std::atomic<bool> limit1detected(false), limit2detected(false), limit3detected(false), limit4detected(false);
	std::atomic<bool> isCalibratingServo(true);

	const int allocsize = 80 + 1;
	char badMessage[] = "ILLEGAL_command_during_calibration!\r\n";
	const int badLength = strlen(badMessage);
	char okMessage[] = "OK\r\n";
	const int oklen = strlen(okMessage);
	char stepperCalibrationBegins[] = "M28received_steppercalibration_begins_now\r\n";
	const int calibrationLength = strlen(stepperCalibrationBegins);
	char calibrationError[] = "CALIBRATION ERROR! critical_limit_error!\r\n";
	const int calibrationErrorLen = strlen(calibrationError);
	std::string cppstring;
	std::string gcode;



	vTaskDelay(100);	//small initial delay



	/*Allowable commands from mDraw at this point during servocalibration are as follows:
	 * M1 pencilServo
	 * M10 settings command
	 * M2 savePencilValues
	 * M5 savePlotArea etc... into PlotterSettings
	 * M11 limitQuery
	 * M28 begin stepper calibration procedure
	 *
	 * Then, you wait until steppercalibration finishes,
	 *
	 * then and then you set EventGroupBit true
	 *  to allow other tasks to do work*/

	while (isCalibratingServo) {

		char str[allocsize] { 0 }; 	//str buffer is allocated
		uint32_t len = USB_receive((uint8_t *) str, allocsize - 1);
		str[len] = 0; 	// make sure we have a zero at the end so that we can print the data
		cppstring += std::string(str); 	//append strbuffer into the "oldcppstring"
		auto enterInd = cppstring.find('\n', 0); 	//find enterIndex

		if (enterInd != std::string::npos) {

			gcode = cppstring.substr(0, enterInd); 	//found entersign, get one complete line of gcode
			cppstring.erase(0, enterInd + 1);	//erase the gotten gcodeline, from the cppstring front portion, BECAUSE WE PROCESSED IT ALREADY ==>sent to parser
			CommandStruct calibrationcmd = calibrationParser.parseCommand(gcode);

			/*deal with M10, get response to settings and send it to mdraw*/ /*NOTE!SEEMS TO WORK WHEN DEBUGGING! :)*/
			if (calibrationcmd.commandWord == CommandStruct::M10 && calibrationcmd.isLegal) {
				std::string M10responsemessage = savedplottersettings.getM10ResponseMessage(); //responds to M10message properly, with formated reply
				USB_send((uint8_t*) M10responsemessage.c_str(), strlen(M10responsemessage.c_str()));
			}
			/*deal with M2, savepencilsettings*/ /*NOTE! SEEMS TO WORK WHEN DEBUGGING! :)*/
			else if (calibrationcmd.commandWord == CommandStruct::M2 && calibrationcmd.isLegal) {
				savedplottersettings.setPenUp(calibrationcmd.penUp); //saves new pencilServovalues to memory
				savedplottersettings.setPenDown(calibrationcmd.penDown);
				USB_send((uint8_t*) okMessage, oklen);
			}
			/*deal with M1, movepencil*/ /*NOTE! SEEMS TO WORK WHEN DEBUGGING! :)*/
			else if (calibrationcmd.commandWord == CommandStruct::M1 && calibrationcmd.isLegal ){
				setPenValue(calibrationcmd.commandNumber); // move the pencilServo
				vTaskDelay(100); //small delay to allow pencilservo time to settle
				USB_send((uint8_t*) okMessage, oklen);
			}
			/*deal with M5, updatevalues*/ /*NOTE! SEEMS TO WORK WHEN DEBUGGING! :)*/
			else if(calibrationcmd.commandWord == CommandStruct::M5 && calibrationcmd.isLegal){
				savedplottersettings.updateM5Values(calibrationcmd); //save M5values to memory, in order to prepare for next M10message
				USB_send((uint8_t*) okMessage, oklen);
			}
			/*deal with M11 limitquery, get response and send it to mdraw*/ /*NOTE! SEEMS TO WORK WHEN DEBUGGING! :)*/
			else if (calibrationcmd.commandWord == CommandStruct::M11 && calibrationcmd.isLegal){
				std::string M11responsemessage = savedplottersettings.getM11LimitResponseMessage();
				USB_send((uint8_t*) M11responsemessage.c_str(), strlen(M11responsemessage.c_str()));
			}

			/*deal with M28 start steppercalibration message*/ /*NOTE! SEEMS TO WORK WHEN DEBUGGING! :)
			 * NOTE! remember to send  it thru the mDraw GUI, because command doesn't exist in mDraw natively*/
			else if(calibrationcmd.commandWord == CommandStruct::M28 && calibrationcmd.isLegal){
				USB_send((uint8_t*) stepperCalibrationBegins, calibrationLength);
				isCalibratingServo = false; //BREAKS FROM LOOP, start stepper calibrations
			}
			/*deal with laser command M4*/
			else if(calibrationcmd.commandWord == CommandStruct::M4 && calibrationcmd.isLegal){
				setLaserValue(calibrationcmd.commandNumber);
				USB_send((uint8_t*) okMessage, oklen);
			}
			else {			/*deal with illegal commands during calibration phase*/ /*NOTE! SEEMS TO WORK WHEN DEBUGGING! :)*/
				USB_send((uint8_t*) badMessage, badLength);
			}
			memset(str, 0, allocsize);
			gcode = "";
		}
	}



	/*initiate steppermotors calibration phase
	 * start steppercalibration with Y-axis, with
	 * direction towards expected Ymax
	 * count the Ysteps along the way when traversing
	 *
	 * same applies to X-axis*/


	dirX.write(false);//set dirs so that you move outwards from origin
	dirY.write(false);


	while (yTouches < 1) { //drive stepper towards expected Ymax, then detect which limitpin, then switchDir, and move away from that limit.
		/*if limit read() true => not contacting limit*/
		limit1detected = limitYMax.read();
		limit2detected = limitYMin.read();
		limit3detected = limitXMax.read();
		limit4detected = limitXMin.read();

		if ( limit1detected && limit2detected && limit3detected && limit4detected) { //all limits open, keep moving outwards toward Ymax
			stepVert();
		} else if (  (!limit1detected && limit2detected && limit3detected && limit4detected) ||		/*check if only single limitDetected, otherwise if multiple limits triggered => calibration got entirely fucked up*/
					 (limit1detected && !limit2detected && limit3detected && limit4detected) ||
					 (limit1detected && limit2detected && !limit3detected && limit4detected) ||
					 (limit1detected && limit2detected && limit3detected && !limit4detected)
				) {
			if(!limit1detected){ 	// detected Ymax, expected Ymax, do nothing
			}else if (!limit2detected){ 	//Ymin detected, Ymax expected, swap them
				swapDigitalIoPins(limitYMax, limitYMin);

			}else if(!limit3detected){ 	//Xmax detected, Ymax expected, swap them
				swapDigitalIoPins(limitYMax, limitXMax);
			}else if(!limit4detected){ 	//Xmin detected, Ymax expected, swap them
				swapDigitalIoPins(limitYMax, limitXMin);
			}

			++yTouches;
			countingY = true; //sets up boolean so that we are getting ready toBeginCountingYsteps
			dirYP->write(!dirYP->read()); //invert dirPin value, turn around

			/*move a safe and deterministic distance away from edge,
			 * count the "TurningSteps" from limitcontact also,
			 * that way, we get accurate Ystepcount for stepcount/millimetre ratio, while
			 * still easily moving safely away from the limitEdge*/
			for(int j = 0; j < g_STEPS_FROM_EDGE; j++){
				stepVert();
				++ysteps; 	//count "TurningSteps" but still move deterministic amount of TurningSteps (later use whileloop for counting)
			}


		}else{ //multiple limits triggered at same time, CALIBRATION ERROR! CRITICAL ERROR!!! stay in foreverloop!!!
			USB_send((uint8_t*) calibrationError, calibrationErrorLen); //send debug message to mDraw!
			stepX.write(false);
			stepY.write(false);
			while (1) {}; //busyloop
		}
	}

	/*add small delay so that the when the the previous forloop from the edge has run (hopefully clearing the limits!),
	 * Then, for safety you should have small delay, before you are again trying to write
	 * highState to any steppins*/
	vTaskDelay(5);


	//yTouches is 1, and stepper should be just off-the-edge nearby the Ymax, keep dirPin same, initially
	//then move to the expected Ymin
	while(yTouches < 2){
		limit1detected = limitYMax.read(); 	// dont expect to get limit1 again!, we just hit it earlier!
		limit2detected = limitYMin.read(); // DO EXPECT to find Ymin among the three remaining booleans at next contact
		limit3detected = limitXMax.read();
		limit4detected = limitXMin.read();
		if( limit1detected && limit2detected && limit3detected && limit4detected ){ 	//allLimitsOpen, keep moving towards Ymin direction
			stepVert();
			if(countingY) 		//keep counting from where the earlier forloop ended (as intended)
				++ysteps;
		} else if(  (!limit1detected && limit2detected && limit3detected && limit4detected) ||
					(limit1detected && !limit2detected && limit3detected && limit4detected) ||
					(limit1detected && limit2detected && !limit3detected && limit4detected) ||
					(limit1detected && limit2detected && limit3detected && !limit4detected)
				){
			if(!limit1detected){ 	//detect Ymax, even though we started the move from there??? error happened
				USB_send((uint8_t*) calibrationError, calibrationErrorLen); //send debug message to mDraw! 	//something went terribly wrong,critical error, we hit the earlier limit again?!
				stepX.write(false);
				stepY.write(false);
				while (1) {}; //busyloop
			}
			if (!limit2detected){ 	//detect Ymin, expected Ymin, do nothing
			}else if(!limit3detected){ 	// detect Xmax, expect Ymin, swap
				swapDigitalIoPins(limitYMin, limitXMax);
			}else if (!limit4detected){ 	//detect Xmin, expect Ymin, swap
				swapDigitalIoPins(limitYMin, limitXMin);
			}

			++yTouches;
			dirYP->write(!dirYP->read()); //invert dirPin value, turn around

			for(int k = 0; k < g_STEPS_FROM_EDGE; k++){ //NOTE!! turn safely&deterministicaly  atTheEdge,
				stepVert(); 	//ALSO, NOTE! stop counting ysteps, BUT still make deterministic turningSteps away fromTheEdge

			}
			/*after the forloop has run, we should be at the  G28 y-coordinate (origin's y-coordinate)*/
			 g_curY = g_OriginY = 0; //initialize curYcoord, and originYcoord both to zero stepYcoord

		}else{
			USB_send((uint8_t*) calibrationError, calibrationErrorLen); //send debug message to mDraw!//multiple limits triggered at same time, CALIBRATION ERROR! CRITICAL ERROR!!! stay in foreverloop!!!
			stepX.write(false);
			stepY.write(false);
			while (1) {}; //busyloop
		}
	}

	/*small delay just for safety purposes*/
	vTaskDelay(5);

	/*try to find the Xmax limit, expect to find that Xmax first...*/
	while(xTouches < 1){
		limit1detected = limitYMax.read(); 	// dont expect to get limit1 again!, we just hit it earlier!
		limit2detected = limitYMin.read(); // dont expect to get limit2 again!, we just hit it earlier!
		limit3detected = limitXMax.read(); //DO EXPECT Xmax
		limit4detected = limitXMin.read();

		if(limit1detected && limit2detected && limit3detected && limit4detected){
			stepHoriz(); 	//no limits triggered, then move in x-axis toward expected Xmax
		}else if (  (!limit1detected && limit2detected && limit3detected && limit4detected) ||
					(limit1detected && !limit2detected && limit3detected && limit4detected) ||
					(limit1detected && limit2detected && !limit3detected && limit4detected) ||
					(limit1detected && limit2detected && limit3detected && !limit4detected)
				){
			if(!limit1detected || !limit2detected){ //critical error, hit again same limits as earlier!?
				USB_send((uint8_t*) calibrationError, calibrationErrorLen); //send debug message to mDraw!// CALIBRATION ERROR! CRITICAL ERROR!!! stay in foreverloop!!!
				stepX.write(false);
				stepY.write(false);
				while (1) {}; //busyloop
			} else if(!limit3detected){ 	//detect Xmax, expect Xmax, do nothing
			}else{
				//detect Xmin, expected Xmax, do swap
				swapDigitalIoPins(limitXMin, limitXMax);	//Xmin detected, but Xmax expected, swap
			}

			++xTouches;
			countingX = true; //prepare bool to start counting xsteps in next loop
			dirXP->write( !dirXP->read() ); //invert xdirPin

			for(int i = 0; i < g_STEPS_FROM_EDGE; i++){ //move safely from XmaxEdge, BUT ALSO count these "TurningSteps"
				stepHoriz();
				++xsteps;
			}

		}else{
			USB_send((uint8_t*) calibrationError, calibrationErrorLen); //send debug message to mDraw!//multiple limits triggered at same time, CALIBRATION ERROR! CRITICAL ERROR!!! stay in foreverloop!!!
			stepX.write(false);
			stepY.write(false);
			while (1) {}; //busyloop
		}
	}

	/*small delay just for safety purposes*/
	vTaskDelay(5);


	while(xTouches < 2){
		limit1detected = limitYMax.read(); 	// dont expect to get limit1 again!, we just hit it earlier!
		limit2detected = limitYMin.read(); // dont expect to get limit2 again!, we just hit it earlier!
		limit3detected = limitXMax.read(); //dont expect  Xmax,  we just hit it earlier!
		limit4detected = limitXMin.read(); // DO EXPECT Xmin

		if(limit1detected && limit2detected && limit3detected && limit4detected){ 	//all limits are open, keep stepping toward Xmin
			stepHoriz();
			if(countingX) //keep counting Xsteps, from where turning forloop ended at (as intended)
				++xsteps;
		} else if( (!limit1detected && limit2detected && limit3detected && limit4detected) ||
				   (limit1detected && !limit2detected && limit3detected && limit4detected) ||
				   (limit1detected && limit2detected && !limit3detected && limit4detected) ||
				   (limit1detected && limit2detected && limit3detected && !limit4detected)
				){
			if(!limit4detected){ 	//detected Xmin, expected Xmin, do nothing
			} else{ 	// in any other case, critical error occured
				USB_send((uint8_t*) calibrationError, calibrationErrorLen); //send debug message to mDraw!// CALIBRATION ERROR! CRITICAL ERROR!!! stay in foreverloop!!!
				stepX.write(false);
				stepY.write(false);
				while (1) {}; //busyloop
			}

			++xTouches;
			dirXP->write( !dirXP->read() ); //invert xdirPin
			//stop counting xsteps

			for(int i = 0; i < g_STEPS_FROM_EDGE; i++){ //move safely from edge, BUT don't count the "TurningSteps"
				stepHoriz();

			}

			/*after the forloop has run,
			 * we have basically allowed the G28 origin coordinates to be in a safe location
			 * that will be safe distance away from limitswitches,
			 *
			 * we should be at the  G28 x-coordinate (origin's x-coordinate)*/
			g_curX = g_OriginX = 0;

		}else {
			USB_send((uint8_t*) calibrationError, calibrationErrorLen); //send debug message to mDraw!//multiple limits triggered at same time, CALIBRATION ERROR! CRITICAL ERROR!!! stay in foreverloop!!!
			stepX.write(false);
			stepY.write(false);
			while (1) {}; //busyloop
		}
	}






/*UPDATE GLOBAL CUR_COORDS AND GLOBAL ORIGIN_COORDS*/
// 	 g_curX=g_OriginX;
// 	 g_curY=g_OriginY;

 /*count the ratio of both
  * Xstepcount/ heightMM
  * and ratio Ystepcount / widthMM
  *
  * NOTE! both ratios are expected to be equal of course!!!
  * */
 	 g_xFullstepsPerMM = (double)xsteps / (double)savedplottersettings.getWidth();
 	 g_yFullstepsPerMM = (double)ysteps / (double)savedplottersettings.getHeight();
	vTaskDelay(1000);

	/*set eventbit0 true
	 * WAKES UP OTHER TASKS to prepare for mdraw commands*/
	xEventGroupSetBits(eventGroup, 0x1);
	vTaskSuspend( NULL);

}



static void logic_test_initialize_task(void*pvParameters){
		//laser and pen pins, hopefully correct???
		DigitalIoPin pen(0,10, DigitalIoPin::pullup, true);
		DigitalIoPin laser(0,12,DigitalIoPin::output, true);
		//drive laserpin low
		laser.write(false);
		laserP = &laser;
		//regular pin setup, as per pdf pin layout guide!!!
		DigitalIoPin dirX(0, 28, DigitalIoPin::output, true);
		DigitalIoPin dirY(1, 0, DigitalIoPin::output, true);
		DigitalIoPin stepY(0, 24, DigitalIoPin::output, true);
		DigitalIoPin stepX(0, 27, DigitalIoPin::output, true);
		DigitalIoPin limitYMin(1, 3, DigitalIoPin::pullup, false);
		DigitalIoPin limitYMax(0, 0, DigitalIoPin::pullup, false);
		DigitalIoPin limitXMax(0, 9, DigitalIoPin::pullup, false);
		DigitalIoPin limitXMin(0, 29, DigitalIoPin::pullup, false);
		//assign global pointers
		limitYMinP = &limitYMin;
		limitYMaxP = &limitYMax;
		limitXMinP = &limitXMin;
		limitXMaxP = &limitXMax;
		//step, dir and pen pointers should remain same always
		stepXP = &stepX;
		dirXP = &dirX;
		stepYP = &stepY;
		dirYP = &dirY;
		penP = &pen;

		vTaskDelay(100);
		/*set eventbit0 true
		 * WAKES UP OTHER TASKS to prepare for mdraw commands*/
		xEventGroupSetBits(eventGroup, 0x1);
		vTaskSuspend( NULL);

}

static void logic_test_rit_bresenham_task(void*pvParameters){

	xEventGroupWaitBits(eventGroup, 0x1, pdFALSE, pdFALSE, portMAX_DELAY);	//wait until real_calibration_task finishes!


	vTaskDelay(10);

		refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, g_curX+2000, g_curY); //right
		vTaskDelay(2);
		refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, g_curX, g_curY+2000); //up
		vTaskDelay(2);
		refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, g_curX-2000, g_curY); //left
		vTaskDelay(2);
		refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, g_curX, g_curY-2000); //down
		vTaskDelay(2);
		refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, g_curX+2000, g_curY+2000); //45deg angle
		vTaskDelay(2);
		refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, g_curX+500, g_curY+300);//30.964deg angle
		vTaskDelay(2);
		refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, g_curX-500, g_curY-300);//-30.964deg angle
		vTaskDelay(2);
		refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, g_curX+300, g_curY+500);//59,036deg angle
}

/*here are testing tasks for bresenham plotting, to verify the algorithms in plottersimulator*/
static void testdraw_isr_bresenham_task(void*pvParameters){
	vTaskDelay(500);
	setPenValue(90);
	refactored_BresenhamInterruptAlgorithm(250, 250, 300, 250); //0deg

	refactored_BresenhamInterruptAlgorithm(300,250,	250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250, 250, 300, 260); //oct1 regular
	refactored_BresenhamInterruptAlgorithm(300,260,	250,250); //go back


	refactored_BresenhamInterruptAlgorithm(250,250,	300,300); //45deg angle
	refactored_BresenhamInterruptAlgorithm(300,300, 250,250);//go back

	refactored_BresenhamInterruptAlgorithm(250,250, 260, 300); //oct2 regular
	refactored_BresenhamInterruptAlgorithm(260,300, 250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250,	250,300);//90deg angle
	refactored_BresenhamInterruptAlgorithm(250,300,	250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250,	240,300);//oct3 regular
	refactored_BresenhamInterruptAlgorithm(240,300, 250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250,	200,300);//135deg angle
	refactored_BresenhamInterruptAlgorithm(200,300,	250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250,	200,260);//oct4reg
	refactored_BresenhamInterruptAlgorithm(200,260,	250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250,	200,250);//180deg
	refactored_BresenhamInterruptAlgorithm(200,250, 250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250,	200,240);//oct5 reg
	refactored_BresenhamInterruptAlgorithm(200,240, 250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250,	200,200);//225deg
	refactored_BresenhamInterruptAlgorithm(200,200, 250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250, 240,200);//oct6reg
	refactored_BresenhamInterruptAlgorithm(240,200, 250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250, 250,200);//270deg
	refactored_BresenhamInterruptAlgorithm(250,200, 250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250,	260,200);//oct7reg
	refactored_BresenhamInterruptAlgorithm(260,200,	 250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250,	300,200);//315deg
	refactored_BresenhamInterruptAlgorithm(300,200, 250,250); //go back

	refactored_BresenhamInterruptAlgorithm(250,250, 300,240);//oct8reg
	refactored_BresenhamInterruptAlgorithm(300,240,	250,250); //go back


	for(;;){
		vTaskDelay(10000);
	}


}


static void draw_square_task(void*pvParameters) {

	vTaskDelay(200);
	setPenValue(90);
	vTaskDelay(100); //give servo time to move down, before draw
	plotLineGeneral(250,250, 300,250);
	plotLineGeneral(300,250, 300,300);
	plotLineGeneral(300,300, 250,300);
	plotLineGeneral(250,300, 250,250);

	//plot 45deg twisted square
	plotLineGeneral(250, 250, 325, 325);
	/*BUG IS SOMEWHERE AFTER THIS LINE!!!
	 * global coord variables dont update*/
	plotLineGeneral(325, 325, 400, 250);
	plotLineGeneral(400, 250, 325, 175);
	plotLineGeneral(325, 175, 250, 250);

	//diamond buggy!!!??? sometiems prints sometimes doesnt depedn on beginning delay before drawing???
	plotLineGeneral(250, 250, 300, 270);
	plotLineGeneral(300, 270, 350, 250);
	plotLineGeneral(350, 250, 300, 230);
	plotLineGeneral(300, 230, 250, 250);

	//another diamond
	/*this diamond works ok*/
	plotLineGeneral(250, 250, 240, 270);
	plotLineGeneral(240, 270, 230, 250);
	plotLineGeneral(230, 250, 240, 230);
	plotLineGeneral(240, 230, 250, 250);

	//reset the diamond and prepare to draw square
	plotLineGeneral(250, 250, 240, 270);
	plotLineGeneral(240, 270, 230, 250);

	//draw rectangle
	plotLineGeneral(230, 250, 230, 270);
	plotLineGeneral(230, 270, 350, 270);
	plotLineGeneral(350, 270, 350, 230);
	plotLineGeneral(350, 230, 230, 230);
	plotLineGeneral(230, 230, 230, 270);

	for (;;) {
		vTaskDelay(100);
	}
}

static void draw_square_task1(void*pvParameters) {

	vTaskDelay(200);
	setPenValue(90);
	vTaskDelay(100); //give servo time to move down, before draw
	plotLineGeneral(g_curX,g_curY, 300,250);
	plotLineGeneral(g_curX,g_curY, 300,300);
	plotLineGeneral(g_curX,g_curY, 250,300);
	plotLineGeneral(g_curX,g_curY, 250,250);

	//plot 45deg twisted square
	plotLineGeneral(g_curX,g_curY, 325, 325);
	/*BUG IS SOMEWHERE AFTER THIS LINE!!!
	 * global coord variables dont update EDITED::BUG FIxed sort of... currently all g_curX and g_curY update
	 * after complete line plot (never one by one increment in a loop)*/
	plotLineGeneral(g_curX,g_curY, 400, 250);
	plotLineGeneral(g_curX,g_curY, 325, 175);
	plotLineGeneral(g_curX,g_curY, 250, 250);

	//diamond
	plotLineGeneral(g_curX,g_curY, 300, 270);
	plotLineGeneral(g_curX,g_curY, 350, 250);
	plotLineGeneral(g_curX,g_curY, 300, 230);
	plotLineGeneral(g_curX,g_curY, 250, 250);

	//another diamond
	/* works ok*/
	plotLineGeneral(g_curX,g_curY, 240, 270);
	plotLineGeneral(g_curX,g_curY, 230, 250);
	plotLineGeneral(g_curX,g_curY, 240, 230);
	plotLineGeneral(g_curX,g_curY, 250, 250);

	//reset the diamond and prepare to draw square
	plotLineGeneral(g_curX,g_curY, 240, 270);
	plotLineGeneral(g_curX,g_curY, 230, 250);

	//draw rectangle
	plotLineGeneral(g_curX,g_curY, 230, 270);
	plotLineGeneral(g_curX,g_curY, 350, 270);
	plotLineGeneral(g_curX,g_curY, 350, 230);
	plotLineGeneral(g_curX,g_curY, 230, 230);
	plotLineGeneral(g_curX,g_curY, 230, 270);

	for (;;) {
		vTaskDelay(100);
	}
}


int main(void) {

	prvSetupHardware();
	ITM_init();
	Chip_RIT_Init(LPC_RITIMER);// initialize RIT (= enable clocking etc.)
	Chip_SCT_Init(LPC_SCT0);//init SCtimer0Large
	setupPenServo();//init servo pwm into center pos for the servo
	setupLaserPWM();
	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority(RITIMER_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY + 5);

	/*create semaphores and commandqueue*/
	sbRIT = xSemaphoreCreateBinary();
	commandQueue = xQueueCreate(20, sizeof(CommandStruct));

	/*//eventgroup is used so that real_calibrate_task makes all initializations,
	 * and runs first, and after calibration has run,
	 * allow other tasks to unblock, such as parse_task, and execute_task*/
    eventGroup = xEventGroupCreate();
    xEventGroupSetBits( eventGroup, 0x0 );

	/*use queueregistry to register semaphores and queues*/
	vQueueAddToRegistry(commandQueue, "comQueue");
	vQueueAddToRegistry(sbRIT, "sbRIT");

#ifndef logicAnalyzerTest
	/* usb parser mdraw commands thread */
	xTaskCreate(parse_task, "parse_task",
			configMINIMAL_STACK_SIZE * 6, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	/*execute mdraw commands thread*/
	xTaskCreate(execute_task, "execute_task",
			configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	/*servopencil and steppermotor calibration thread*/
	xTaskCreate(real_calibrate_task, "real_calibrate_task",
			configMINIMAL_STACK_SIZE * 6, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);
#endif

	/*here are some testing tasks maybe so you can see in logicanalyzer???*/
#ifdef logicAnalyzerTest
	xTaskCreate(logic_test_rit_bresenham_task, "logic_test_rit_bresenham_task",
			configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(logic_test_initialize_task, "logic_test_initialize_task",
			configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);
#endif

	/* cdc thread */
	xTaskCreate(cdc_task, "CDC",
			configMINIMAL_STACK_SIZE * 3, NULL, (tskIDLE_PRIORITY  +1UL),
			(TaskHandle_t *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
