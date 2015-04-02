#ifndef PMP_UTILITIES_H
#define PMP_UTILITIES_H

#include <iostream>
#include <string>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

namespace iCub{
	namespace pmplib{

		class PmpPortWriter
		{
			public:
				PmpPortWriter():_isOpen(false), writerId(-1){};
				bool isOpen(){return _isOpen;};
				void init( const int &_id){id.addInt(_id);};
				void init( const string &name){id.addString(name.c_str());};

				bool openPort()
				{
					OutPort = new Port();
					string name = "/part/";
					name += id.get(0).asString().c_str();
					name += "/outstream";

					if(!OutPort->open(name.c_str()))
					{
						cout << "unable to open Port" << endl;
						return false;
					};

					_isOpen = true;

					return true;
				};
				void closePort()
				{OutPort->close(); _isOpen = false;};
				void interruptPort(){OutPort->interrupt();};
				void setWriterId(const int id){writerId = id;};
				int getWriterId(){return writerId;};
				int clearWriterId(){writerId = -1;};
				void write(PortWriter &writer, const int _writerId)
				{
					if (_writerId == writerId)
						OutPort->write(writer);
				};

			private:
				Bottle id;
				int writerId;
				Port *OutPort;
				bool _isOpen;
		};

		static string PropertyGroup2String(Bottle group)
		{
			Bottle bot;
			bot.clear();

			for (int i = 1; i < group.size(); i++)
			{
				bot.add(group.get(i));
			}

			string s = bot.toString().c_str();
			//cout << s << endl;
			return s;
		};

		static Value PropertyGroup2Bottle(Bottle group)
		{
			Bottle bot;
			bot.clear();

			for (int i = 1; i < group.size(); i++)
			{
				bot.add(group.get(i));
			}

			Value val;
			val.fromString(bot.toString());
			return val;
		};

		static Vector Bottle2Vector(Bottle Bot)
		{
			Vector v(Bot.size());
			for (int i = 0; i < Bot.size(); i++)
			{
				v(i) = Bot.get(i).asDouble();
			}

			return v;
		};

		static void Bottle2Matrix(Bottle Bot, Matrix &m)
		{
			int k=0;
			for(int i=0; i<m.rows(); i++)
			{
				for(int j=0; j<m.cols(); j++)
				{
					m(i,j) = Bot.get(k).asDouble();
					k++;
				}
			}
		};

		static Bottle Vector2Bottle(Vector v)
		{
			Bottle bot;
			for (unsigned int i=0; i<v.size(); i++)
			{
				bot.addDouble(v(i));
			}
			return bot;
		};

		static Bottle Matrix2Bottle(Matrix m)
		{
			Bottle bot;
			for (int i=0; i<m.rows(); i++)
			{
				Bottle &temp = bot.addList();
				for(int j=0; j<m.cols(); j++)
				{
					temp.addDouble(m(i,j));
				}
			}
			return bot;
		};
		static Matrix RPY2Rot (const Vector &RPY_deg)
		{
			Matrix R(3,3);

			double cr = cos(RPY_deg(0)*CTRL_DEG2RAD);
			double sr = sin(RPY_deg(0)*CTRL_DEG2RAD);
			double cp = cos(RPY_deg(1)*CTRL_DEG2RAD);
			double sp = sin(RPY_deg(1)*CTRL_DEG2RAD);
			double cy = cos(RPY_deg(2)*CTRL_DEG2RAD);
			double sy = sin(RPY_deg(2)*CTRL_DEG2RAD);

			R(0,0) = cr*cp;
			R(1,0) = sr*cp;
			R(2,0) = -sp;

			R(0,1) = cr*sp*sy - sr*cy;
			R(1,1) = sr*sp*sy + cr*cy;
			R(2,1) = cp*sy;

			R(0,2) = cr*sp*cy + sr*sy;
			R(1,2) = sr*sp*cy - cr*sy;
			R(2,2) = cp*cy;

			return R;
		}
	}
}


#endif