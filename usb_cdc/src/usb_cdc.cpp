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
//user defined includes ends


/*CONDITINAL COMPILATION OPTIONS****************************/

#define useLoopingBresenham
#define keijoSimulator


/*options and variables when using RITinterruptBresingham*/
 //variables for RITinterruptBresingham

volatile int g_dx, g_dy, g_nabla;
int g_x_1, g_x_0, g_y_1, g_y_0;
char g_drivingAxis = 'X';


/*********************************************************/


//freertos globals defined
SemaphoreHandle_t syncSemph;
QueueHandle_t commandQueue;
SemaphoreHandle_t sbRIT; //NOTE!! THIS SEMAPHORE IS NECESSARY! DOUBLECHECK MAIN() TO SEE IF CREATED CORRECTLY...
SemaphoreHandle_t sbGo;
EventGroupHandle_t eventGroup;
//freertos globals ends


/*Bresenham's algorithm variables, and notation is based on Bresenham's published paper
"Algorithm for computer control of a digital plotter":
IBM SYSTEMS JOURNAL * VOL. 4 * NO. 1 . 1965
by J. E. Bresenham

Bresenham's paper  can be useful reference for the pictures of motormovements for m1, and m2,
and also the tabulated results section near the end was useful. Some functions such as decidem2parameters,
decidem1parameters, getOctant, and SwapAxes were based on the journal article results.
*/

//NOTE! loopingBresenham version of Interrupt Handler function uses THIS EXTRA VARIABLE, USE CONDITIONAL COMPILATION...

volatile std::atomic<int> executeM1orM2(0); //check 1 or 2 inside isr to decide USED FOR loopingBresenham versio
const int forLoopDelay = 1;

//These global variables are shared usage between either case of conditional compilation Interrupt Handlers...
volatile int m1parameter = 0;//  determine which pin is driven into which direction, straight motorMove horiz OR vert
volatile int m2parameter = 0;//   determine which two pins are driven into which direction at the same time, diagonal motorMove
const int ppsValue=800;	//arbitrary value for pps
volatile uint32_t RIT_count; //NOTE!! THIS VARIABLE IS NECESSARY!
static  std::atomic<bool> calibrationFinished(false);	//note, currently not used yet for anything important, at least...
static volatile std::atomic<bool> pulseState(true);//NOTE!! THIS VARIABLE IS NECESSARY for rit interrupt handlers!


//these are device current coords global variables for plottercoords
volatile int g_curX = 250;
volatile int g_curY = 250;
/*global variables ends*/


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
//GlobalPointers End here


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
extern "C"{
void RIT_IRQHandler(void){ //THIS VERSION IS FOR RITinterruptBresenham
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	// Tell timer that we have processed the interrupt.
	// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER);// clear IRQ flag

	static bool expectm2 = false; //boolean keeps track of halfpulsing for m2pinwrite in the same ISR round as the m1pinwrite

	if (limitYMinP != NULL && limitYMaxP != NULL && limitXMaxP != NULL && limitXMinP != NULL) { //should never be nullpointers when starting ritstart

		bool isOK = ( !limitYMinP->read() && !limitYMaxP->read() && !limitXMaxP->read() && !limitXMinP->read() ); //check limits status inside ISR

		if (RIT_count > 0) { //regular case, "iterate bresenham algorithm" inside interrupt handler

			if (isOK) { //CALIBRATION MODE SHOULD NEVER USE RIT IRQ
				/*at each fullstep, in the beginning,
				 * use decisionparameter to iterate over bresenham
				 * When RIT_count is odd, => implies that current ISR round is basically
				 * writing false cycle to the steppins, to complete any given fullstep*/
				if(RIT_count % 2 == 0){

					if(g_nabla >= 0){
						expectm2=true;
						g_nabla = g_nabla + 2* g_dy - 2* g_dx;
					}else{
						expectm2=false;
						g_nabla = g_nabla + 2* g_dy;
					}
				}

				/*prepare to halfpulse the m1pin
				 * m1pin is notation based on Bresenhams original paper
				 * the dirPins are already pre-computed before ISR
				 * for each line plot, for each G1command, dirPins stay same
				 * */
				if (!expectm2) {
					if(m1parameter==1 || m1parameter==5 ){
						stepXP->write(pulseState);
					}
					else if(m1parameter==3 || m1parameter==7){
						stepYP->write(pulseState);
					}
				}
				/*else perform  halfpulse m2pin for diagonal M2Move
				 * only perform the necessary checks inside ISR,
				 * rely on the setupBresenham to have correctly
				 * configured dirPins before line plot
				 * NOTE!!!
				 *   expectm2 is static bool initialized at false, so that it keeps
				 *   track of its own state after each ISR iterating round
				 *   then, at the end of final halfpulse ISR iterating round, remember to reset expectm2 to false
				 *   */
				else {
					stepYP->write(pulseState);
					stepXP->write(pulseState);
				}

				pulseState= !pulseState;
				RIT_count--;
				/*ISR will still be called, when RIT_count == 0,
				 * but then the interrupt doesnt write to any pins
				 * when RIT_count==0, interrupt simply ends and
				 * resets RIT_count= and expectm2=false  */

			} else { //WE HIT THE WALL, set rit_count=0, stop stepPins, and soon the motor will stop, hopefully
				stepXP->write(false);
				stepYP->write(false);
				RIT_count = 0;
			}
		} else { // "iterate bresenham" has ended, prepare to reset variables and stop interrupt
			pulseState=true; //prepare pulsestate for next G1command
			expectm2=false; //reset boolean in preparation for the beginning of next G1 command, so it will be false in beginning of ritstart
			RIT_count=0; //reset RIT_count also, probably not needed though, because ritstart sets it up again
			Chip_RIT_Disable(LPC_RITIMER); // disable timer
			// Give semaphore and set context switch flag if a higher priority task was woken up
			xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
		}
		// End the ISR and (possibly) do a context switch
		portEND_SWITCHING_ISR(xHigherPriorityWoken);
	}else{
		//do nothing the limitpointers are null// SHOULD NOT HAPPEN!!!
	}


}
}

#endif
/*********************************************************************************************************************************/


/*** RIT interrupt handler for looping-style Bresenham algorithm, USE CONDITIONAL COMPILATION TO ENABLE ***/
#ifdef useLoopingBresenham
extern "C" {
void RIT_IRQHandler(void) {
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	// Tell timer that we have processed the interrupt.
	// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag

	if (limitYMinP != NULL && limitYMaxP != NULL && limitXMaxP != NULL && limitXMinP != NULL) {

		if (RIT_count > 0) {
			RIT_count--;
			// do something useful here...
			bool isOK = (  !limitYMinP->read() && !limitYMaxP->read() && !limitXMaxP->read() && !limitXMinP->read()  );

			if (isOK) {
				if(executeM1orM2==1) { //execute M1 pattern move
					switch(m1parameter) { //actuate only one motor straight move
					case 1: stepXP->write(pulseState); break;
					case 3: stepYP->write(pulseState); break;
					case 5: stepXP->write(pulseState); break;
					case 7: stepYP->write(pulseState); break;
					default: break;//shouldnt be here
					}
				}else if(executeM1orM2 == 2){//execute M2 pattern move
					stepXP->write(pulseState);//actuate both motors diagonal move
					stepYP->write(pulseState);
				}
				pulseState = !pulseState;//move motor and toggle pulsestate rit_halfpulses
			} else {//WE HIT THE WALL, set rit_count=0 and soon the motor will stop, hopefully
				stepXP->write(false);
				stepYP->write(false);
				RIT_count = 0;
			}
		} else {
			pulseState=true;
			Chip_RIT_Disable(LPC_RITIMER); // disable timer
			// Give semaphore and set context switch flag if a higher priority task was woken up
			xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
		}
		// End the ISR and (possibly) do a context switch
		portEND_SWITCHING_ISR(xHigherPriorityWoken);

	}
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
void refactored_decideM1Parameter(const int octant){
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

	//decide and set global variable m1pattern
	m1parameter = pattern1;

}

/*helper functions for RIT_INTERRUPT BRESENHAM */
void refactored_decideM2Parameter(const int octant){
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
	m2parameter = pattern2;

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


///*helper functions for RIT_INTERRUPT BRESENHAM */
//void refactored_setupBresenhamDirPins(const int m1param, const int m2param){
//	//dir == true, implies decreasing coordinates
//	//dir == false, implies increasing coordinates
//	switch(m1param){
//	case 1:
//		dirXP->write(true);
//		break; //hopefuly go right
//	case 3:
//		dirYP->write(true);
//		break; //hopefuly go up
//	case 5:
//		dirXP->write(false);
//		break; //hopefuly go left
//	case 7:
//		dirYP->write(false);
//		break; //hopefuly go down
//	default: break;
//	}
//
//
//	if (m1param==1) {
//		if (m2param == 2)
//			dirYP->write(true);
//		else if(m2param==8)
//			dirYP->write(false);
//	}
//
//	else if (m1param==3) {
//		if (m2param == 2)
//			dirXP->write(true);
//		else if (m2param == 4)
//			dirXP->write(false);
//	}
//
//	else if (m1param==5) {
//		if (m2param == 4)
//			dirYP->write(true);
//		else if (m2param == 6)
//			dirYP->write(false);
//	}
//
//	else if (m1param == 7) {
//		if (m2param == 6)
//			dirXP->write(false);
//		else if (m2param == 8)
//			dirXP->write(true);
//	}
//	else{
//		//should never get here
//	}
//int kakka = 0;//for debug only!
//
//}
//

/*main function for  RIT_INTERRUPT BRESENHAM, starts interrupt bresenham*/
void refactored_BresenhamInterruptAlgorithm(int x0, int y0, int x1, int y1){

	/*ASSUME
	 * initially that each xcoord, ycoord == 1.0mm
	 * */

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
	refactored_decideM1Parameter(oct);
	refactored_decideM2Parameter(oct);

	//refactored_setupBresenhamDirPins(m1parameter, m2parameter);
	testing_setupBresenhamDirPins(m1parameter,m2parameter); //this function is cleaner code, it was refactored from the other refactored_setupBresenhamDirPins()
	const char drivingAxis = refactored_decideSwapAxes(oct); //knowing the drivingAxis return value isnt important except for debugging,


	const int fullsteps =  abs(g_dx);


	//NOTE! always ritstart with even numbers because by definition, you are halfpulsing the fullpulses
	RIT_start( 2 * fullsteps,  ( 1000000 / (2*ppsValue) )    );


}







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
	m1parameter = M1pattern; //set the global variable to notify isr about step parameter
	return m1parameter;

}

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

	m2parameter = M2pattern; //set the global variable to notify isr about step parameter
	return m2parameter;
}

//
////EXAMPLE FUNCTION SCTSETUP THREE COUNTERS FOR THREE RGB COLORS
//void setupSCTLED(){
//	Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT0_O,  0,3);	//greenled port0_pin3
//	Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT1_O,  0,25);	//reassigns output to redled
//	Chip_SWM_MovablePortPinAssign( SWM_SCT1_OUT0_O,  1,1);	/*reassign output to blueled*/
//
//	/*setup counters sct1low and sct0high and sct0low*/
//	LPC_SCT1->CONFIG |= (1<<17); //sct1 lowcounter used autolimit
//	LPC_SCT1->CTRL_L |= (72-1)<<5; //prescale lowcounter sct1
//	LPC_SCT1->MATCHREL[0].L= 255-1;	//sct1 lowcoutner freq
//
//	LPC_SCT0->CONFIG |= (3<<17); //autolimit lowcounter and highcounter
//	LPC_SCT0->CTRL_L |= (72-1) << 5; //prescale lowcounter
//	LPC_SCT0->CTRL_H |= (72-1) << 21; //prescale highcounter
//	LPC_SCT0->MATCHREL[0].L = 255-1;	//sct0 lowcounter  freq
//	LPC_SCT0->MATCHREL[0].H = 255-1;	//sct0 highcounter  freq
//
//	/*set the pulsewidths into matchreload*/
//	LPC_SCT0->MATCHREL[2].L = 250;	// sct0 lowcounter pulsewidth GREENLED
//	LPC_SCT0->MATCHREL[1].H = 250;	// sct0 highcounter pulsewidth REDLED
//	LPC_SCT1->MATCHREL[1].L= 250; //sct1 lowcounter pulsewidth BLUELED
//
//	/*events configured
//	 * 1st frequecny events*/
//	LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF;	//all states allowed event0
//	LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF;	//all states allowed event1
//	LPC_SCT1->EVENT[2].STATE = 0xFFFFFFFF; //all states allowed event2
//
//	LPC_SCT0->EVENT[0].CTRL= 1<<12;	//event0 sct0 lowcounter frequency match, select reg0
//	LPC_SCT0->EVENT[1].CTRL = (1<<4) |(1<<12)  ; //event1 sct0 highcounter frequency match, select reg0
//	LPC_SCT1->EVENT[2].CTRL = (1<<12); //event2 sct1 lowcounter frequency match, select reg0
//
//	/*2ndly we have the COUNTER-MATCH events*/
//	LPC_SCT0->EVENT[3].STATE= 0xFFFFFFFF;
//	LPC_SCT0->EVENT[4].STATE= 0xFFFFFFFF;
//	LPC_SCT1->EVENT[5].STATE= 0xFFFFFFFF;
//
//	LPC_SCT0->EVENT[3].CTRL = (1<<12) | (2); //event3 sct0 lowcounter match, select reg2
//	LPC_SCT0->EVENT[4].CTRL = (1<<4) | (1<<12) | (1); //event4 sct0 highcounter match, HEVENTbitTrue,  select reg1
//	LPC_SCT1->EVENT[5].CTRL = (1<<12) | (1); //event5 sct1 lowcounter match, select reg1 (default)
//
//	/*set outputs*/
//	LPC_SCT0->OUT[0].SET =  (1<<0); //event0 sets  sct0 output0
//	LPC_SCT0->OUT[1].SET = (1<<1); //event1 sets sct0 output1 //1<<0
//	LPC_SCT1->OUT[0].SET = (1<<2); //event2 sets sct1 output0
//
//	/*clear outputs with countermatches*/
//	LPC_SCT0->OUT[0].CLR = 	1<<3;			//event3 clears sct0 output0
//	LPC_SCT0->OUT[1].CLR =	1<<4;		//event4 clears sct0 output1
//	LPC_SCT1->OUT[0].CLR =	1<<5;		//event5 clears sct1 output0
//
//	/*unhalt timers*/
//	LPC_SCT0->CTRL_L &=  ~(1<<2);
//	LPC_SCT0->CTRL_H &= ~(1<<2);
//	LPC_SCT1->CTRL_L &= ~(1<<2);
//}
//
//
////getters and setters for SCTimer for three RGB colors
//uint16_t getRed(){
//	return LPC_SCT0->MATCHREL[1].H;
//}
////getters and setters for SCTimer for three RGB colors
//uint16_t getGreen(){
//	return LPC_SCT0->MATCHREL[2].L;
//}
////getters and setters for SCTimer for three RGB colors
//uint16_t getBlue(){
//	return LPC_SCT1->MATCHREL[1].L;
//}
//
////getters and setters for SCTimer for three RGB colors
//void setRGBValues(uint8_t red, uint8_t green, uint8_t blue){
//
//
//	red = 255-red;
//	green= 255 - green;
//	blue=255 -blue;
//
//	LPC_SCT0->MATCHREL[1].H = red;	// sct0 highcounter pulsewidth REDLED
//	LPC_SCT0->MATCHREL[2].L = green;	// sct0 lowcounter pulsewidth GREENLED
//	LPC_SCT1->MATCHREL[1].L= blue; //sct1 lowcounter pulsewidth BLUELED
//}
//
////EXAMPLE FUNCTION SCTSETUP FOR GREENLED
//void startUpSCT( ){
//	LPC_SCT0->CONFIG |= (1 << 17); // two 16-bit timers, auto limit
//	LPC_SCT0->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock == 72mhz / 72 == 1mhz
//	LPC_SCT0->MATCHREL[0].L = 1000-1; // match 0 @ 1000/1MHz = 1000 us (1 kHz PWM freq)
//	LPC_SCT0->MATCHREL[1].L = 950; // match 1 used for duty cycle (initialize at 50% pwm hopefully)
//
//	/*NOTE!!! SET TO CORRECT PLOTTER PIN BEFORE USAGE!!!
//	 *
//	 *
//	 * */
//	/*reassigns output to greenled*/
//	Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT0_O,  0,3);	//greenled port0_pin3
//	LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
//	LPC_SCT0->EVENT[0].CTRL = (1 << 12); // match 0 condition only
//	LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
//	LPC_SCT0->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only
//	LPC_SCT0->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0
//	LPC_SCT0->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0
//	LPC_SCT0->CTRL_L &= ~(1 << 2);// unhalt it by clearing bit 2 of CTRL reg
//}
//
////EXAMPLE FUNCTION SCTSETUP FOR SERVOACTUATING
//void startUpServoSCT(){
//	LPC_SCT0->CONFIG |= (1 << 17); // two 16-bit timers, auto limit
//	LPC_SCT0->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock == 72mhz / 72 == 1mhz
//
//	/*NOTE PERIOD = t must be equal to 20ms => freq= 50hz
//	 * MATCHREL[1].L == 15000 BECAUSE 15000/ 1000000hz  = 0,015sec = 15ms =servo should be centered at this value of duty cycle
//	 * MARCHREL[0].L == 20000 BECAUSE 20000 / 1000000hz = 0,02sec = 20ms period of signal = 50hz freq of pwm signal*/
//
//	LPC_SCT0->MATCHREL[0].L = 20000-1; // match 0 @ 1000/1MHz = 1000 us (1 kHz PWM freq == 1000hz)
//	LPC_SCT0->MATCHREL[1].L = 1500; // match 1 used for duty cycle (initialize at center for servo)
//
//	/*reassigns output to pin
//	 *NOTE !!! CHANGE THE MOVABLE PIN TO THE CORRECT PIN FOR PLOTTERPIN!!!
//	 **/
//	Chip_SWM_MovablePortPinAssign( SWM_SCT0_OUT0_O,  0,8);	//servopin P0.8 drive it to the center EDITED::TODO::
//
//	LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
//	LPC_SCT0->EVENT[0].CTRL = (1 << 12); // match 0 condition only
//	LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
//	LPC_SCT0->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only
//	LPC_SCT0->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0
//	LPC_SCT0->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0
//	LPC_SCT0->CTRL_L &= ~(1 << 2);// unhalt it by clearing bit 2 of CTRL reg
//
//}
//
////set sctimer pulsewidth FOR SERVOACTUATING
//void setPWM( int amount ){
//	LPC_SCT0->MATCHREL[1].L = amount;
//}
////sctimer getpulsewidth FOR SERVOACTUATING
//uint16_t getPulseWidth(){
//	auto val= LPC_SCT0->MATCHREL[1].L;
//	return val;
//}
//
////sctimer getPeriod FOR SERVOACTUATING
//uint16_t getPeriod(){
//	auto temp =  LPC_SCT0->MATCHREL[0].L;
//	return temp;
//}


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
		 *
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
	//dirXP->write(true);
	stepXP->write(true);
	vTaskDelay(1);
	stepXP->write(false);
	vTaskDelay(1);
}





//same function as in Bresenham's notation helperfunction F() which deduces the correct octant to draw inside of
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

//special case for purely horizontal moves BRESENHAM loopingBresenham
void plotLineHoriz(int x0, int y0, int x1, int y1) {
	int dx = x1 - x0;
	int dy = y1 - y0;
	int xincr = 0;
	int localX=x0;
	executeM1orM2 = 1; //notify isr global variable which patternm1 or patternm2 to execute, in this case only one motor move => hence m1pattern
	if (dx > 0) {
		//xmotordir right
		xincr=1;
		dirXP->write(false);
		m1parameter = 1; //set global volatile variable to notify isr which m1 motor to actjuate
	} else {
		//xmotordir left
		dirXP->write(true);
		m1parameter = 5;
		xincr = -1;

	}


	for (int x = 1; x <= abs(dx); x++) {
		//write to motordir to drive left or right
		RIT_start(2, 1000000 / (2 * ppsValue));
		localX += xincr; //update  coords
	}
	g_curX=localX;
}

//special case for purely vertical moves BRESENHAM loopingBresenham
void plotLineVert(int x0, int y0, int x1, int y1) {

	int dx = x1 - x0;
	int dy = y1 - y0;
	executeM1orM2 = 1; //notify isr which patternm1 or patternm2 to execute, in this case only one motor move => hence m1pattern
    int yincr=0;
    int localY=y0;
	if (dy > 0) {
		//y motor up
		m1parameter = 3; //notify isr with global variable to expect ymotor up
		dirYP->write(false);
		yincr = 1;
	} else {
		//ymotor down
		m1parameter = 7; //notify isr with global variable to expect ymotor down
		dirYP->write(true);
		yincr = -1;
	}

	for (int y = 1; y <= abs(dy); y++) {
		//write motorpin up or down
		RIT_start(2, 1000000 / (2 * ppsValue));
		localY += yincr;
	}
	g_curY=localY;
}

//BRESENHAM HELPER FUNCTION loopingBresenham
void plotLineLow(int x0, int y0, int x1, int y1, bool makeSwap) {
	if (makeSwap) {
		std::swap(x0, x1);
		std::swap(y0, y1);
	}
	int yincr = 1;
	int xincr = 1;
	int dx=x1-x0, dy=y1-y0;
	if(dy < 0)
		yincr = -1;
	if(dx < 0)
		xincr = -1;


	dx = abs(x1 - x0);
	dy = abs(y1 - y0);

	int localX=x0, localY=y0;

	int nabla = 2 * dy - dx; //decision parameter
	int x = x0;

	for (x; x < x1; x++) { //x is driving axis
		if (nabla > 0) {
			nabla = nabla - 2 * dx; //update decisionparameter nabla
			/*execute diagonal m2 move (step x and y motor at same time)???
			 m2 move type depends on the octant
			 m1 move type depends on octant also (m1 is either horiz or vert move)*/
			executeM1orM2 = 2; //set global volatile boolean, so that interrupthandler knows which movementpattern to execute
			RIT_start(2, 1000000 / (2 * ppsValue));

		}
		else {
			//execute m1 move
			executeM1orM2 = 1;
			RIT_start(2, 1000000/(2*ppsValue) );
		}
		nabla = nabla + 2 * dy;//update decisionparameter nabla
	}


}


//BRESENHAM HELPER FUNCTION loopingBresenham
void plotLineHigh(int x0, int y0, int x1, int y1, bool makeSwap) {
	if(makeSwap){
		std::swap(x0,x1);
		std::swap(y0,y1);
	}

	int xincr = 1;
    int yincr = 1;
	int dx=x1-x0, dy=y1-y0;
	if(dx < 0)
		xincr = -1;
	if(dy<0)
		yincr = -1;



	 dx = abs(x1 - x0);
	 dy = abs(y1 - y0);




	int nabla = 2 * dx - dy; //decision parameter
	int y = y0;
	int localX=x0, localY=y0;

	for (y; y < y1; y++) { //y is driving axis
		if (nabla > 0) {
			nabla = nabla - 2 * dy;//update decisionparameter nabla
			//execute diagonal m2 move (step x and y motor at same time)
			// m2 move type depends on the octant
			//m1 move type depends on octant also (m1 is either horiz or vert move)
			executeM1orM2 = 2;
			RIT_start(2, 1000000/(2*ppsValue) );
		}
		else {
			//execute m1 move
			executeM1orM2 = 1;
			RIT_start(2, 1000000/(2*ppsValue) );
		}
		nabla = nabla + 2 * dx;//update decisionparameter nabla
	}
}

//BRESENHAM MAIN FUNCTION loopingBresenham
void plotLineGeneral(int x0, int y0, int x1, int y1) {
	/*octant notation goes from
	 angle 0deg to 45deg == oct 1
	 angle 45deg to 90deg == oct 2
	 etc...
	 */
	int oct = 0;
	int kakka = -1;

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
		oct = getOctant((x1 - x0 < 0), (y1 - y0 < 0),
				(abs(x1 - x0) - abs(y1 - y0) < 0));
		m2parameter = adjustm2motor(oct);
		m1parameter = adjustm1motor(oct);

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

	endingpoint:  //label for the special case of same startpoint and same endpoint, dont do anything
	kakka = 0; //for debug only!
	/*UPDATE GLOBAL COORDS, ASSUME ALGORITHM WORKED AND SET CURCOORDS=ENDCOORDS
	 * seems to work, in example test_draw_tasks */
	g_curX=x1, g_curY=y1;

}







//executes G1movementcommands, pencilservocommands, and lasercommands
static void execute_task(void*pvParameters) {

	//laser and pen pins, hopefully correct???
	DigitalIoPin pen(0,10, DigitalIoPin::pullup, true);
	DigitalIoPin laser(0,12,DigitalIoPin::output, true);

	//drive laserpin low
	laser.write(false);
	laserP = &laser;

	//movement pins for axes

#ifdef keijoSimulator //swap pins-setup for easier debugging for keijosimulator, limit pins and dirpins and step pins so that Y->X and X<-Y
	//invert limitpins and dirPins and stepPins
	DigitalIoPin dirX(1, 0, DigitalIoPin::output, true);
	DigitalIoPin dirY(0, 28, DigitalIoPin::output, true);
	DigitalIoPin stepX(0, 24, DigitalIoPin::output,true);
	DigitalIoPin stepY(0, 27, DigitalIoPin::output,true);

	DigitalIoPin limitYMin(0, 29, DigitalIoPin::pullup, true);
	DigitalIoPin limitYMax(0 ,9 , DigitalIoPin::pullup, true);
	DigitalIoPin limitXMax( 0, 0, DigitalIoPin::pullup, true);
	DigitalIoPin limitXMin(1, 3, DigitalIoPin::pullup, true);
#endif

#ifndef keijoSimulator	//regular pin setup, as per pdf pin layout guide!!!
	DigitalIoPin dirX(0, 28, DigitalIoPin::output, true);
	DigitalIoPin dirY(1, 0, DigitalIoPin::output, true);
	DigitalIoPin stepY(0, 24, DigitalIoPin::output, true);
	DigitalIoPin stepX(0, 27, DigitalIoPin::output, true);
	DigitalIoPin limitYMin(1, 3, DigitalIoPin::pullup, true);
	DigitalIoPin limitYMax(0, 0, DigitalIoPin::pullup, true);
	DigitalIoPin limitXMax(0, 9, DigitalIoPin::pullup, true);
	DigitalIoPin limitXMin(0, 29, DigitalIoPin::pullup, true);
#endif


	//assign global pointers
	limitYMinP = &limitYMin;
	limitYMaxP = &limitYMax;
	limitXMinP = &limitXMin;
	limitXMaxP = &limitXMax;
	stepXP = &stepX;
	dirXP = &dirX;
	stepYP = &stepY;
	dirYP = &dirY;
	penP = &pen;

	CommandStruct curcmd{CommandStruct::M1, 0, true, 0, 0};

	vTaskDelay(50);

	//wait until calibration task finishes!
	/*TODO:: add something in calibration mode to get the currentlocation in coords*/
	/*TODO:: CALIBRATION TASK IS BUGGED!!! LIMIT PINS DONT WORK PROPERLY IN KEIJOSIMULATOR*/

	xEventGroupWaitBits(eventGroup, 0x1, pdFALSE, pdFALSE, portMAX_DELAY);

	for(;;){
		xQueueReceive(commandQueue, &curcmd, portMAX_DELAY); //get command from queue

		if(curcmd.commandWord == CommandStruct::G1 && curcmd.isLegal){

			//assume 1 fullstep == 1.0mm,
			float tempx = curcmd.xCoord / 100.0;
			float tempy = curcmd.yCoord / 100.0;

			int roundX = std::round(tempx);
			int roundY = std::round(tempy);

			int kakka1 = 0; //for debug only!!
#ifndef useLoopingBresenham //ritInterrupt-Driven Bresenham
			refactored_BresenhamInterruptAlgorithm(g_curX, g_curY, roundX,roundY );
			g_curX=roundX;//update current coords to the "dest coords, after move"
			g_curY=roundY;
			int kakka3=0;//for debug only!
#endif
#ifdef useLoopingBresenham //forlooping Bresenham
			//forlooping bresenham updates globalcoords internally! this is desired (updates coords in loopcounting)!
			plotLineGeneral(g_curX, g_curY, roundX,roundY);
			int kakka2=0;//for debug only!
#endif


			int kakka = 0;//for debug only!


		}else if(curcmd.commandWord == CommandStruct::M1 && curcmd.isLegal){
			setPenValue(curcmd.commandNumber);
			vTaskDelay(100); //small delay to allow pencilservo time to settle
		}
	}

}

//PARSER TASK gets USB-receive and parses the Gcode commands
static void parse_task(void*pvParameters) {

	//bool LedState = false;
	GcodeParser parser;
	const int allocsize = 80 + 1;
	char initialMessage[] =
			"M10 XY 380 310 0.00 0.00 A0 B0 H0 S80 U160 D90\r\nOK\r\n";
	int initlen = strlen(initialMessage);
	char okMessage[] = "OK\r\n";
	char badMessage[] = "nok\r\n";
	int oklen = strlen(okMessage);
	std::string temp = "";
	std::string cppstring = "";
	std::string gcode = "";

	//small initial delay
	vTaskDelay(75);

	//wait until calibration task finishes!
	xEventGroupWaitBits(eventGroup, 0x1, pdFALSE, pdFALSE, portMAX_DELAY);
	while (1) {

		char str[allocsize] { 0 }; //str buffer is allocated
		uint32_t len = USB_receive((uint8_t *) str, allocsize - 1);
		str[len] = 0; /* make sure we have a zero at the end so that we can print the data */
		cppstring += std::string(str); //append strbuffer into the "oldcppstring"
		auto enterInd = cppstring.find('\n', 0); //find enterIndex
		//vTaskDelay(5); //vtaskdelay is used for purpose of itmprint not bugging-out

		if (enterInd != std::string::npos) {
			gcode = cppstring.substr(0, enterInd); //found entersign, get one line of gcode
			cppstring.erase(0, enterInd + 1);//erase the gotten gcodeline, from the cppstring front portion, BECAUSE WE PROCESSED IT ALREADY ==>sent to parser
			ITM_write(gcode.c_str());
			ITM_write("\r\n");
			//vTaskDelay(20);
			CommandStruct cmd {
				CommandStruct::M1,
				-2147483648,
				false,
				-2147483648,
				-2147483648
			};
			cmd = parser.parseCommand(gcode);//send the gcode string into the parser

			if (cmd.commandWord == CommandStruct::M10 && cmd.isLegal) {
				USB_send((uint8_t*) initialMessage, initlen);
			} else if (cmd.isLegal) {
				//any legal message send ok, but also send command to queue
				xQueueSendToBack(commandQueue, &cmd, portMAX_DELAY);
				USB_send((uint8_t*) okMessage, oklen);
			} else {
				USB_send((uint8_t*) badMessage, strlen(badMessage));
			}
			memset(str, 0, allocsize);
			gcode = "";
		}


		//else, YOU DIDNT FIND COMPLETE GCODE, WAIT FOR MORE STRBUFFER APPENDED CHARS!

	}

}



static void calibrate_task(void*pvParameters){

	int xsteps = 0;
	int ysteps = 0;
	int yTouches = 0;
	int xTouches = 0;
	int halfY;
	int halfX;
	float temp1,temp2;
	bool countingY = false;
	bool countingX = false;
goto endpoint;
	//find y limits and count ysteps
	while (yTouches < 2) { //count ysteps
		if (!limitYMinP->read() && !limitYMaxP->read()) {
			stepVert();
			if (countingY) {
				++ysteps;
			}
		} else if (limitYMinP->read() != limitYMaxP->read()) {
			++yTouches;
			if (ysteps == 0) {
				countingY = true;
			}
			dirYP->write(!dirYP->read());
			while (limitYMinP->read() || limitYMaxP->read()) {
				stepVert();
			}
		}
	}

	//drive back y axis to the center
	temp1 = (float)ysteps/2.0;
	halfY = std::round(temp1);


	for (int k = 0; k < halfY; k++) {
		stepVert();
	}

	while (xTouches < 2) { //count xsteps
		if (!limitXMinP->read() && !limitXMaxP->read()) {
			stepHoriz();
			if (countingX) {
				++xsteps;
			}
		} else if (limitXMinP->read() != limitXMaxP->read()) {
			++xTouches;
			if (xsteps == 0) {
				countingX = true;
			}
			dirXP->write(!dirXP->read());
			while (limitXMinP->read() || limitXMaxP->read()) {
				stepHoriz();
			}
		}
	}
	/*driveback to halfpoint x axis*/

	temp2 = (float)xsteps/2.0;
	halfX = std::round(temp2);

	for (int j = 0; j < halfX; j++) {
		stepHoriz();
	}

	endpoint:
	vTaskDelay(1000);
	g_curX=500;
	g_curY=500;
	/*set eventbit0 true
	 * WAKES UP OTHER TASKS to prepare for mdraw commands*/
	xEventGroupSetBits(eventGroup, 0x1);
	 vTaskSuspend( NULL );
}



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
	 * global coord variables dont update*/
	plotLineGeneral(g_curX,g_curY, 400, 250);
	plotLineGeneral(g_curX,g_curY, 325, 175);
	plotLineGeneral(g_curX,g_curY, 250, 250);

	//diamond buggy!!!??? sometiems prints sometimes doesnt depedn on beginning delay before drawing???
	plotLineGeneral(g_curX,g_curY, 300, 270);
	plotLineGeneral(g_curX,g_curY, 350, 250);
	plotLineGeneral(g_curX,g_curY, 300, 230);
	plotLineGeneral(g_curX,g_curY, 250, 250);

	//another diamond
	/*this diamond works ok*/
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
	setupPenServo();//init servo pwm into center pos originally

	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority(RITIMER_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY + 5);

	/*craete semaphores and commandqueue*/
	sbRIT = xSemaphoreCreateBinary();
	commandQueue = xQueueCreate(20, sizeof(CommandStruct));
	sbGo = xSemaphoreCreateBinary();
    eventGroup = xEventGroupCreate();
    xEventGroupSetBits( eventGroup, 0x0 );

	/*use queueregistry to register semaphores and queues*/
	vQueueAddToRegistry(commandQueue, "comQueue");
	vQueueAddToRegistry(sbRIT, "sbRIT");
	vQueueAddToRegistry(sbGo, "sbGo");

	//create binary semph
	syncSemph = xSemaphoreCreateBinary();

	/* usb parser mdraw commands thread */
	xTaskCreate(parse_task, "parse_task",
			configMINIMAL_STACK_SIZE * 5, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	/*NOTE!! here are algorithm test drawing tasks for plottersimulator*/
#ifdef keijoSimulator
	#ifdef useLoopingBresenham
	xTaskCreate(draw_square_task1, "drawsquare1",
			configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);
	#endif


	#ifndef useLoopingBresenham
	xTaskCreate(testdraw_isr_bresenham_task, "isrtestdraw",
			configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	#endif
#endif




	/*execute mdraw commands thread*/
	xTaskCreate(execute_task, "execute_task",
			configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(calibrate_task, "calibrate_task",
			configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);


	/* cdc thread */
	xTaskCreate(cdc_task, "CDC",
			configMINIMAL_STACK_SIZE * 3, NULL, (tskIDLE_PRIORITY + 1UL + 1UL),
			(TaskHandle_t *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
