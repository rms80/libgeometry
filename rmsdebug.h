// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef __RMS_DEBUG_H__
#define __RMS_DEBUG_H__

#ifdef _WIN32
#include "windows.h"
#pragma warning(disable:4505)
#endif

#include <string>
using namespace std;

#include <stdio.h>
#include <stdarg.h>



static int _RMSInfo(const char * str, ...)
{	
	static char buf[1024];
	va_list args;

	va_start(args, str);
	//vsprintf_s(buf, 1024, str, args);
#ifdef _WIN32
	OutputDebugString(buf);
#else
	fprintf(stderr, "%s", buf);
#endif
	va_end(args);

	return 0;
}

static string _RMSInfoString(char * str, ...)
{	
	static char buf[1024];
	va_list args;

	va_start(args, str);
//vsprintf_s(buf, 1024, str, args);
	va_end(args);
	return string(buf);
}



#endif   // __RMS_DEBUG_H__
