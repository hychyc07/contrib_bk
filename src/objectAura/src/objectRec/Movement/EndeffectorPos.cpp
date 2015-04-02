#include "EndeffectorPos.h"

using namespace std;

ostream& operator<<(ostream& out, const EndeffectorPos& e) {
	out.precision(3);
	out << "EndeffectorPos[x=" << e.x << " y=" << e.y << " z=" << e.z << " a="
			<< e.a << " b=" << e.b << " c=" << e.c << "] ";
	return out;
}
