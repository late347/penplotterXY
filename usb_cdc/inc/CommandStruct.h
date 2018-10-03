#pragma once


typedef struct CommandStruct {
	enum {
		M1,
		M4,
		M10,
		G1,
		G28

	} commandWord;
	int commandNumber;
	bool isLegal;
	int xCoord;
	int yCoord;
};
