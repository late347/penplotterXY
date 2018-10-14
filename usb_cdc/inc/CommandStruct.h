#pragma once


struct CommandStruct {
	enum {
		M1,
		M4,
		M10,
		G1,
		G28,
		M5,
		M11,
		M28,
		M2,
		uninitialized

	} commandWord; //any  Gcode command has commandWord (essentially a basetype)
	int commandNumber; //pencilServoParameter, or laserParameter [0,255]

	int penUp; //parameter for M2 command
	int penDown; //parameter for M2 command

	int height; //parameter for M5 command
	int width; //parameter for M5 command
	int speed;  //parameter for M5 command

	bool xMotorClockwise; //parameter for M5 command
	bool yMotorClockwise; //parameter for M5 command

	bool isLegal; //any Gcode command has legality
	int xCoord; //coords for G1command HUNDREDTHS of mm
	int yCoord; //coords for G1command HUNDREDTHS of mm


	CommandStruct() {
		commandWord = uninitialized;
		commandNumber = -1;
		penUp = -1;
		penDown = -1;
		height = -1;
		width = -1;
		speed = -1;
		xMotorClockwise = true;
		yMotorClockwise = true;
		isLegal = false;
		xCoord = -1;
		yCoord = -1;
	}
};
