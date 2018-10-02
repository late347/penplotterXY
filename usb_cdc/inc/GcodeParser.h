#pragma once
#include <vector>
#include <string>
#include "CommandStruct.h"


//DONT SAY USING NAMESPACE STD; IN HEADERFILES
class GcodeParser {
public:
	//functions
	GcodeParser(){} //10 spaces in vector for words in the beginning EDITED::should create allocatin for 10 emptoes??
	virtual ~GcodeParser();

	/*parseCommand is the main parser function, which uses other helper functions inside it
	parseCommand creates temporary struct, and later returns struct by value
	first we tokenize
	secondly we parse
	thirdly we get results
	fourthly we clear tokensvec
	retrun struct*/

	CommandStruct parseCommand(const std::string &rawInput );
	
	/*tokenizes cppstring into tokensVec*/
	void tokenizeInput(const std::string &rawInput);

	/*parse<Cmdword> functions are helper functions for parseCommand main function*/
	bool parseM10(CommandStruct &cmdRef);
	bool parseG28(CommandStruct &cmdRef);
	bool assertNotLeadingZero(const char &letter);
	bool parseM4(CommandStruct &cmdRef);
	bool parseM1(CommandStruct &cmdRef);

	int getCoordsFromG1Parameter( const bool &isPositive, const std::string &coordsRef, const char &axisChar);
	
	
	bool parseG1ParameterELITELEVEL(const std::string &coordsRef, const char &axischar, int &returnableCoordInt);
	bool parseG1(CommandStruct &cmdRef);

	/*clear tokens from vec, in preparation for new inputRound*/
	void clearTokens() { tokensVec.clear(); }

	//public vector datamember, for easy access???!!!
	std::vector<std::string> tokensVec;

protected:
	//datamembers

	/*internal constant for parsing purposes*/
	const int maxCharAmount = 30;
	const char delimiter = ' ';//delimit for tokenizing
};

