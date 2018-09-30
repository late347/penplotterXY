/*
 * LimitedCounter.cpp
 *
 *  Created on: 7 Mar 2018
 *      Author: Lauri
 */

#include "LimitedCounter.h"

LimitedCounter::~LimitedCounter() {
	// TODO Auto-generated destructor stub
}


LimitedCounter & LimitedCounter ::operator++(){
	if(count<max) {
		++count;
	}

	return *this;
}

LimitedCounter LimitedCounter::operator++(int){
	if(count<max){

		LimitedCounter old= *this;
		++count;
		return old;

	}
	else{
		return *this;
	}
}

LimitedCounter LimitedCounter::operator--(int){
	if(count>min){
		LimitedCounter old=*this;
		--count;
		return old;
	}
	else{
		return *this;
	}
}

LimitedCounter & LimitedCounter::operator--(){
	if(count>min){
		--count;
	}
	return *this;
}

LimitedCounter & LimitedCounter::operator=(const LimitedCounter & rhsref){
	if(this== &rhsref){
		return *this;
	}
	else{
		count=rhsref.count;
//		min=minvalue;
//		max=maxvalue;
		return *this;
	}
}

LimitedCounter LimitedCounter::operator+(const LimitedCounter &lcref){	// addition operator overloaded PREVENT ADDING TOGETHER BEYOND MAX

	if(count + lcref.count <= max ){
		LimitedCounter lc(count + lcref.count);
		return lc;
	}
	else{
		LimitedCounter lc2(max);
		return lc2;
	}

}

LimitedCounter LimitedCounter::operator-(const LimitedCounter &lcref){ //subtraction operator overloaded PREVENT SUBRATCTING BEYOND MIN
	if(count - lcref.count >= min){
	LimitedCounter lc1(count-lcref.count);
	return lc1;
	}
	else{
		LimitedCounter lc2(min);
		return lc2;
	}
}


bool LimitedCounter::operator==(const LimitedCounter & rhsref){
	if (count==rhsref.count){
		return true;
	}
	else{
		return false;
	}
}
bool LimitedCounter::operator>=(const LimitedCounter & rhsref){
	if(count>=rhsref.count){
		return true;
	}
	else{
		return false;
	}
}
bool LimitedCounter::operator<=(const LimitedCounter & rhsref){
	if(count<=rhsref.count){
		return true;
	}
	else{
		return false;
	}
}
bool LimitedCounter::operator<(const LimitedCounter & rhsref){
	if(count<rhsref.count){
		return true;
	}
	else{
		return false;
	}
}
bool LimitedCounter::operator>(const LimitedCounter & rhsref){
	if(count>rhsref.count){
		return true;
	}
	else{
		return false;
	}
}
