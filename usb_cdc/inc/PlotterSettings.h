/*
 * PlotterSettings.h
 *
 *  Created on: 12 Oct 2018
 *      Author: Lauri
 *
 *
 *      CLASS IDEA is to make one global object of PlotterSettings
 *      Then, you can keep the saveable settings stored in PlotterSettings ps; object
 *
 *      DONT store PLOTTER STATE in the PlotterSettings object, only mDraw saveable settings like height, width, speed, penUp, penDown
 *
 *      the state of the plotter cannot really be encapsulated in one object
 *      because ISR requires access to some variables
 *
 *       curX, curY which contains current location of plotter
 *      Likewise limitswitch state is only gotten with polling the limitswitches inside ISR  when moving has occurred
 */

#ifndef PLOTTERSETTINGS_H_
#define PLOTTERSETTINGS_H_
#include <string>
#include "DigitalIoPin.h"
#include "CommandStruct.h"

class PlotterSettings {
public:
	/*initial constructor constructs the PlotterSettings based on default mDraw values,
	 * NOTE! it leaves limitpointers as NULL, if you dont set the limitpointers
	 *
	 * Initially you should maybe just put in whatever order possible, the limitpin addresses into
	 * these DigitalIoPin pointers, just so they are still pointing to existings iopin objects
	 *
	 * NOTE! remember to calibrate the limitpointers and later update the limitpointers to the correct pointers after calibration!
	 * */
	PlotterSettings(
			int newpenup = 160,
			int newpendown = 90,
			bool newXclockwise=true,
			bool newYclockwise = true,
			int newheight = 310,
			int newwidth = 380,
			int newspeed = 80,
			DigitalIoPin *l1 = NULL,
			DigitalIoPin *l2 = NULL,
			DigitalIoPin *l3 = NULL,
			DigitalIoPin *l4 = NULL
			) :
			penUp(newpenup),
			penDown(newpendown),
			xMotorClockwise(newXclockwise),
			yMotorClockwise(newYclockwise),
			height(newheight),
			width(newwidth),
			speed(newspeed),
			limit1p(l1),
			limit2p(l2),
			limit3p(l3),
			limit4p(l4)	{}






	PlotterSettings(CommandStruct &);
	PlotterSettings(CommandStruct &, DigitalIoPin*, DigitalIoPin*, DigitalIoPin*, DigitalIoPin*);
	virtual ~PlotterSettings();

	void setPenUp(int);
	void setPenDown(int);
	void setXMotorDir(bool);
	void setYMotorDir(bool);
	void setHeight(int);
	void setWidth(int);
	void setSpeed(int);

	void updateM5Values(CommandStruct &);
	void setLimitPointers(DigitalIoPin*, DigitalIoPin*, DigitalIoPin*, DigitalIoPin*);

	std::string getM10ResponseMessage(); // M10command responsemessage based on saved plottersettings
	std::string getM11LimitResponseMessage(); //response for M11 limits query

private:
//	volatile int curX; //MAYBE NOT NEEDED!?
//	volatile int curY;//MAYBE NOT NEEDED!?

	int penUp; //saved values in memory for pencilservo, affect M10command responsemessage
	int penDown;

	bool xMotorClockwise; //by default should be true, so that motordirs are correct way setup (false xDirPin == right == increasingX == mDrawclockwise)
	bool yMotorClockwise; //by default should be true, so that  (false yDirPin == up == increasingY == mDrawclockwise)

	int height;//amount of actual mm
	int width;//amount of actual mm
	int speed; //percentage of speed [0, 100]

	DigitalIoPin *limit1p; //limitpointers needed to get the limitstatuses, for M11command query
	DigitalIoPin *limit2p;
	DigitalIoPin *limit3p;
	DigitalIoPin *limit4p;


};



#endif /* PLOTTERSETTINGS_H_ */
