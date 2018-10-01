#include "GcodeParser.h"
#include"CommandStruct.h"
#include "ITM_write.h"
using namespace std;


GcodeParser::~GcodeParser() {
}

CommandStruct GcodeParser::parseCommand(const std::string & rawInput) {
	//create temp struct and return it by value in the end after parsing is done
	CommandStruct cmd{ CommandStruct::M1, -2147483648, false, -2147483648, -2147483648 };

	
	bool isLegal = false;
	tokenizeInput(rawInput);
	isLegal = parseG28(cmd);
	if (!isLegal) {
		isLegal = parseM10(cmd);
	}
	if (!isLegal) {
		isLegal = parseM1(cmd);
	}
	if (!isLegal) {
		isLegal = parseM4(cmd);
	}
	if (!isLegal) {
		isLegal = parseG1(cmd);
	}
	
	(this->clearTokens());
	return cmd;
}


//tokenises input cppstring into datamember vector
void GcodeParser::tokenizeInput(const std::string &rawInput) {

	auto res = rawInput.find(delimiter);
	int searchInd = 0;
	int pos = 0;
	int size = rawInput.length();
	bool earlyReturn = false;
	if (size > maxCharAmount) {
		tokensVec.push_back("INVALID_COMMAND!");
		earlyReturn = true;
	}

	/*maybe legal maybe not!?, doesnt contain delimiter, but was gotten with getline
	=>implies one complete word ready to be parsed*/
	if (!earlyReturn && res == std::string::npos) {
		tokensVec.push_back(rawInput);
		earlyReturn = true;
	}

	else if (!earlyReturn && res != std::string::npos) {
		//get the tokens from inputstring into the vector as words
		while (pos != string::npos) {
			pos = rawInput.find(delimiter, searchInd);
			if (searchInd == pos) {
				tokensVec.push_back("INVALID_COMMAND!");
				++searchInd;
			}
			else if (pos != string::npos) {
				tokensVec.push_back(rawInput.substr(searchInd, pos - searchInd));
				searchInd = pos + 1;
			}
		}

		if (searchInd < size) {
			//get last valid token, when the rawstr doesnt end in the delimitin char
			tokensVec.push_back(rawInput.substr(searchInd, size - searchInd));
			earlyReturn = true;
		}
		else if (rawInput[size - 1] == delimiter) {
			tokensVec.push_back("INVALID_COMMAND!");
			earlyReturn = true;
		}
	}
}

bool GcodeParser::parseM10(CommandStruct & cmdRef) {
	//check how many words(or tokens) we got inside
	auto len = tokensVec.size();
	if (len == 1 && tokensVec[0] == "M10") {
		cmdRef.commandWord = CommandStruct::M10;
		cmdRef.commandNumber = 0;
		cmdRef.isLegal = true;
		cmdRef.xCoord = 0;
		cmdRef.yCoord = 0;
		return true;
	}
	else {
		return false;
	}	
}

bool GcodeParser::parseG28(CommandStruct & cmdRef) {
	

	auto size = tokensVec.size();
	if (size == 1 && tokensVec[0] == "G28") {
		cmdRef.commandWord = CommandStruct::G28;
		cmdRef.commandNumber = 0;
		cmdRef.isLegal = true;
		cmdRef.xCoord = 0;
		cmdRef.yCoord = 0;
		return true;
	}
	else {
		return false;
	}


}

bool GcodeParser::assertNotLeadingZero(const char &letter) {
	if (isdigit(letter) && letter != '0') {
		return true;
	}
	else { return false; }
}

bool GcodeParser::parseM4(CommandStruct & cmdRef) {

	auto size = tokensVec.size();
	int comNum;

	if (size == 2 && tokensVec[0]=="M4") {
		std::string numberParameter(tokensVec[1]);
		int digitcount = numberParameter.length();
		bool legalnumber = false;
		if ( digitcount <= 3 && digitcount >= 1) {
			if (digitcount==1) {
				legalnumber = isdigit(numberParameter[0]);
			}
			else if (digitcount==2) {
				legalnumber = (assertNotLeadingZero(numberParameter[0]) && isdigit(numberParameter[1]));
			}
			else if (digitcount==3) {
				legalnumber = (assertNotLeadingZero(numberParameter[0]) && isdigit(numberParameter[1]) && isdigit(numberParameter[2]));
			}
			if (legalnumber) {
				sscanf(numberParameter.c_str(), "%d", &comNum); //sscanf should be ok at this late stage when all is checked to be legal alrady
				cmdRef.commandWord = CommandStruct::M4;
				cmdRef.commandNumber = comNum;
				cmdRef.isLegal = true;
				cmdRef.xCoord = 0;
				cmdRef.yCoord = 0;
				return true;
			}
		
		}
	}



	return false;
}

bool GcodeParser::parseM1(CommandStruct & cmdRef) {
	auto size = tokensVec.size();
	int comNum; //size of number parameter of M1 command should be between 0-255 int value???
	if (size == 2 && tokensVec[0]=="M1") {
		string numberParameter(tokensVec[1]);
		auto digitcount = numberParameter.length();
		bool legalNumber = false;
		if (digitcount==1) {
			legalNumber = isdigit(numberParameter[0]);
		}
		else if (digitcount==2) {
			legalNumber = assertNotLeadingZero(numberParameter[0]) && isdigit(numberParameter[1]);
		}
		else if (digitcount==3) {
			legalNumber = assertNotLeadingZero(numberParameter[0]) && isdigit(numberParameter[1]) && isdigit(numberParameter[2]);
		}
		if (legalNumber) {
			sscanf(numberParameter.c_str(), "%d", &comNum); //size of number parameter of M1 command should be between 0-255 int value
			cmdRef.commandWord = CommandStruct::M1;
			cmdRef.commandNumber = comNum;
			cmdRef.isLegal = true;
			cmdRef.xCoord = 0;
			cmdRef.yCoord = 0;
			return true;
		}
	}


	return false;
}

int GcodeParser::getCoordsFromG1Parameter( const bool & isPositive, const std::string & coordsRef, const char & axisChar) {
	
	int temp1, temp2;
	temp1 = temp2 = 0;
	int returnvalue = 0;
	/*objective is to convert the coord values like X123.12 into "sadasosat" to int value
	sscanf to separate into two int variables 123*100 + 12 = 12312 sadasosaa
	and hence also in reverse...
	12312 /100 = 123.12 original result

	for negative numbers, you can do similar but remember to multiply with (-1) to negate and expect the minussign
	*/
	if (axisChar == 'X') {
		if (isPositive) {
			sscanf(coordsRef.c_str(), "X%d.%d", &temp1, &temp2);
			returnvalue = 100 * temp1 + temp2;
		}
		else { //is negative
			sscanf(coordsRef.c_str(), "X-%d.%d", &temp1, &temp2);
			returnvalue = (-1)* (100 * temp1 + temp2); //negate AFTER you count the unsigned values properly
		}
	}
	else if (axisChar == 'Y') {
		if (isPositive) {
			sscanf(coordsRef.c_str(), "Y%d.%d", &temp1, &temp2);
			returnvalue = 100 * temp1 + temp2;
		}
		else { //is negative
			sscanf(coordsRef.c_str(), "Y-%d.%d", &temp1, &temp2);
			returnvalue = (-1)* (100 * temp1 + temp2); //negate AFTER you count the unsigned values properly
		}
	}
	else {
		 //   should not return this normally, if you call with only x or y axes
	}

	return returnvalue;
}

bool GcodeParser::parseG1ParameterELITELEVEL(const string & coordsRef, const char & axischar, int & returnableCoordInt) {


	int loopInd = 1;
	int len = coordsRef.length();
	bool isLegalParam = false;
	bool isPos = true;

	//check if str too short for legality
	//minimum coordinates size is "X0.00"==len5
	if (len < 5) {
		return isLegalParam;
	}
	//check to find decimalpoint
	//in actuality decimalpoint must be third last char always.
	//furthermore there should only exist one decimalpoint, but later at the end we will check that 
	//there will not be any decimalpoints at the last two characters
	auto res = coordsRef.find('.', 0);
	if (res == (len-3) ) {
		//all is good
	}
	else {
		//didn't find decimalpoint at expected location
		return isLegalParam;
	}
	//check that you find axisChar at expected location
	if (coordsRef[0] != axischar) {
		return isLegalParam;
	}

	//check if 1st ind is minussign or digit
	if (coordsRef[1]=='-') {
		//neg number
		++loopInd;
		isPos = false;
	}
	else if (isdigit(coordsRef[1])) {
		//pos number
		isPos = true;
	}
	else {
		//illegal char found => false
		return isLegalParam;
	}

	//check if leadinzero present or not, when having applicable length string
	if (  (len>=6 && isPos) || (len>=7 && !isPos)  ) {
		if (assertNotLeadingZero(coordsRef[loopInd])==false ) {
			//leading zero present =>illegal
			return isLegalParam;
		}
		//otherwise leading zero check was fine
	}

	//check all chars upto the decimalpoint
	for (size_t i = loopInd; coordsRef[i] != '.'; i++) {
		if (  isdigit(coordsRef[i])  == false    ) {
			return isLegalParam;
		}
	}

	//check if last two chars are truedigits
	if (  isdigit(coordsRef[len - 1]) && isdigit(coordsRef[len - 2])     ) {
		isLegalParam = true;
	}
	


	//extract the int values from coords if legalfound
	if (isLegalParam) {
		returnableCoordInt = getCoordsFromG1Parameter(isPos, coordsRef, axischar);
		
	}
	return isLegalParam;

}

bool GcodeParser::parseG1(CommandStruct & cmdRef) {
	auto size = tokensVec.size();
	int possibleXCoord=0;
	int possibleYCoord=0;
	bool isLegal = false;
	char x = 'X';
	char y = 'Y';

	if (size==4) {
		if (tokensVec[0] == "G1") {
			if (parseG1ParameterELITELEVEL(tokensVec[1], x, possibleXCoord)  
				&& parseG1ParameterELITELEVEL(tokensVec[2], y, possibleYCoord)) {
				if (tokensVec[3]=="A0") {
					isLegal = true;
					cmdRef.commandWord = CommandStruct::G1;
					cmdRef.xCoord = possibleXCoord;
					cmdRef.yCoord = possibleYCoord;
					cmdRef.isLegal = true;
					cmdRef.commandNumber = 0;
				}

			}
		}

	}


	return isLegal;
}

