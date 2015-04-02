#ifndef __ICUB_SKINCALIBRATIONLOGGER_H__
#define __ICUB_SKINCALIBRATIONLOGGER_H__

#include <stdio.h>
#include <stdarg.h>
#include <string>

using namespace std;

namespace iCub
{	namespace skinCalibration
	{
		class SkinCalibrationLogger
		{

			static FILE * stream_pointer;
			static string context_label;

			SkinCalibrationLogger();

			static int baseWrite(FILE *stream, string format, va_list& args);

		public:

			static void setContextLabel(string);
			static void setStreamToStdOut();
			static void setStreamToStdErr();
			static void setStreamToFile(string);

			static bool isInitialized();

			static int writeMsg(string format, ...);
			static int writeToStdOut(string format, ...);
			static int writeToStdErr(string format, ...);
			static int writeToFile(string filename, string format, ...);

		};
	}
}

#endif //__ICUB_SKINCALIBRATIONLOGGER_H__