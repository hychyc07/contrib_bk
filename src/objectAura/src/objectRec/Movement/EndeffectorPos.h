/*
 * AMpos.h
 *
 *  Created on: Mar 9, 2010
 *      Author: kuka
 */

#ifndef AMPOS_H_
#define AMPOS_H_
#include <fstream>
using namespace std;

class EndeffectorPos {

	// x,y,z: meter
	// a,b,c: degree (euler)

private:

public:
	friend std::ostream& operator<<(std::ostream& out,
			const EndeffectorPos& spec);
	double x, y, z, a, b, c;
	EndeffectorPos() :
		a(0), b(0), c(0), x(0), y(0), z(0) {
	}
	EndeffectorPos(double x1, double y1, double z1, double a1, double b1,
			double c1) :
		x(x1), y(y1), z(z1), a(a1), b(b1), c(c1) {
	}
};

#endif /* AMPOS_H_ */
