// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)


// always build this file in debug mode, so that proper debug functions are defined.
// lgASSERT is enabled/disabled in config.h
//#ifndef _DEBUG
//#define _DEBUG
//#endif

#ifdef WIN32
#include <windows.h>
#include <crtdbg.h>
#endif

#include <cstdlib>

#include <stdio.h>

void lgBreakToDebugger()
{
#ifdef WIN32
	DebugBreak();
#else
	abort();
#endif
}


int lgAssertReport(const char * filename, int line, const char * message) 
{
#ifdef WIN32
	#ifdef DEBUG
//	return _CrtDbgReportW(_CRT_ASSERT, filename, line, NULL, message);
		return _CrtDbgReport(_CRT_ASSERT, filename, line, NULL, message);
	#else
		char buf[1024];
		_snprintf_s(buf, 1024, "lgASSERT FAILED - [%s:%d] - %s\n", filename, line, message);
		OutputDebugString(buf);
		lgBreakToDebugger();
		return 1;
	#endif
#else
        fprintf(stderr, "lgASSERT FAILED - [%s:%d] - %s\n", filename, line, message);
	lgBreakToDebugger();
	return 1;	// [RMS] is this right?
#endif
}
