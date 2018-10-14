/*
 * PlotterSettings.cpp
 *
 *  Created on: 12 Oct 2018
 *      Author: Lauri
 */

#include <PlotterSettings.h>



PlotterSettings::PlotterSettings(CommandStruct &cmd) {

	if (cmd.commandWord == CommandStruct::M5 && cmd.isLegal) {
		penUp = cmd.penUp; //saved values in memory for pencilservo, affect M10command responsemessage
		penDown = cmd.penDown;

		xMotorClockwise = cmd.xMotorClockwise; //by default should be true, so that motordirs are correct way setup (false xDirPin == right == increasingX == mDrawclockwise)
		yMotorClockwise = cmd.yMotorClockwise; //by default should be true, so that  (false yDirPin == up == increasingY == mDrawclockwise)

		height = cmd.height; //amount of actual mm
		width = cmd.width; //amount of actual mm
		speed = cmd.speed; //percentage of speed [0, 100]

	}

	limit1p = NULL;
	limit2p = NULL;
	limit3p = NULL;
	limit4p = NULL;
}

PlotterSettings::PlotterSettings( CommandStruct &cmd, DigitalIoPin*L1, DigitalIoPin*L2, DigitalIoPin*L3, DigitalIoPin*L4) {

	if (cmd.commandWord == CommandStruct::M5 && cmd.isLegal) {
		penUp = cmd.penUp; //saved values in memory for pencilservo, affect M10command responsemessage
		penDown = cmd.penDown;

		xMotorClockwise = cmd.xMotorClockwise; //by default should be true, so that motordirs are correct way setup (false xDirPin == right == increasingX == mDrawclockwise)
		yMotorClockwise = cmd.yMotorClockwise; //by default should be true, so that  (false yDirPin == up == increasingY == mDrawclockwise)

		height = cmd.height; //amount of actual mm
		width = cmd.width; //amount of actual mm
		speed = cmd.speed; //percentage of speed [0, 100]

	}


	limit1p = L1;
	limit2p = L2;
	limit3p = L3;
	limit4p = L4;
}


PlotterSettings::~PlotterSettings() {
	// TODO Auto-generated destructor stub
}



void PlotterSettings::setPenUp(int newupval){
	penUp = newupval;
}

void PlotterSettings::setPenDown(int newdownval){
	penDown = newdownval;
}

void PlotterSettings::setHeight(int newh){
	height = newh;
}
void PlotterSettings::setWidth(int neww){
	width = neww;
}
void PlotterSettings::setSpeed(int newS){
	speed = newS;
}
void PlotterSettings::setXMotorDir(bool isclockwise){
	xMotorClockwise = isclockwise;
}
void PlotterSettings::setYMotorDir(bool isclockwise){
	yMotorClockwise = isclockwise;
}


void PlotterSettings::updateM5Values(CommandStruct &cmd){

	if (cmd.commandWord == CommandStruct::M5 && cmd.isLegal) {

		xMotorClockwise = cmd.xMotorClockwise; //by default should be true, so that motordirs are correct way setup (false xDirPin == right == increasingX == mDrawclockwise)
		yMotorClockwise = cmd.yMotorClockwise; //by default should be true, so that  (false yDirPin == up == increasingY == mDrawclockwise)

		height = cmd.height; //amount of actual mm
		width = cmd.width; //amount of actual mm
		speed = cmd.speed; //percentage of speed [0, 100]

	}
}


void PlotterSettings::setLimitPointers(DigitalIoPin *L1, DigitalIoPin *L2, DigitalIoPin *L3, DigitalIoPin *L4){
	limit1p = L1;
	limit2p = L2;
	limit3p = L3;
	limit4p = L4;
}

std::string PlotterSettings::getM11LimitResponseMessage(){
	if (limit4p != NULL && limit3p != NULL && limit2p != NULL && limit1p != NULL) {
		std::string response("M11");
		if ( limit4p->read() ) //if limit is true => implies limitIsNotDepressed == open switch
			response += " 1";
		else
			response += " 0";


		if (limit4p->read())
			response += " 1";
		else
			response += " 0";


		if (limit2p->read())
			response += " 1 ";
		else
			response += " 0 ";

		if (limit1p->read())
			response += "1\r\n";
		else
			response += "0\r\n";

		response += "OK\r\n";

		return response;
	}
	else{
		return "nullpointerlimitP_in_PlotterSettings\r\n";
	}


}

std::string PlotterSettings::getM10ResponseMessage(){
	/*format the M10 replymessage based on saved plottersettings*/
	std::string response="M10 XY ";
	response += std::to_string(height);
	response += " ";
	response += std::to_string(width);
	response += " 0.00 0.00 ";

	if(xMotorClockwise)
		response += "A0 ";
	else
		response += "A1 ";

	if(yMotorClockwise)
		response += "B0 ";
	else
		response += "B1 ";

	response += "H0 ";

	response += "S";
	response += std::to_string(speed);
	response += " ";
	response += "U";
	response += std::to_string(penUp);
	response += " ";
	response += "D";
	response += std::to_string(penDown);

	response += "\r\nOK\r\n";

	return response;
}
