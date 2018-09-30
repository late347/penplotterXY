/*
 * LimitedCounter.h
 *
 *  Created on: 7 Mar 2018
 *      Author: Lauri
 */

#ifndef LIMITEDCOUNTER_H_
#define LIMITEDCOUNTER_H_

class LimitedCounter {
public:
	const int MAX_COLOR = 255;
	const int MIN_COLOR = 0;

	LimitedCounter(int curvalue=50) : count(curvalue), min(MIN_COLOR), max(MAX_COLOR) {}

	virtual ~LimitedCounter();

	LimitedCounter & operator++();	//prefix increment
	LimitedCounter operator++(int); //postfix incremnet

	LimitedCounter operator--(int); //postfix decrement
	LimitedCounter & operator--();	//prefix decrement

	LimitedCounter & operator=(const LimitedCounter & rhs); //assignment operator

	operator int() {return count;} // conversion operator

	/*boolean operators*/
	bool operator==(const LimitedCounter & rhs);
	bool operator<(const LimitedCounter & rhs);
	bool operator>(const LimitedCounter & rhs);
	bool operator<=(const LimitedCounter & rhs);
	bool operator>=(const LimitedCounter & rhs);

	LimitedCounter operator+(const LimitedCounter &lcref);
	LimitedCounter operator-(const LimitedCounter &lcref);

private:
	int count;
	int min;
	int max;
};

#endif /* LIMITEDCOUNTER_H_ */
