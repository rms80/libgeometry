// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)


#ifdef _WIN32

#include <windows.h>

static LARGE_INTEGER _RMSTUNE_starts[16];
static LARGE_INTEGER _RMSTUNE_ends[16];

static LARGE_INTEGER _RMSTUNE_accums[16];

static void _RMSTUNE_start(int i)
{
	QueryPerformanceCounter(&_RMSTUNE_starts[i]);
}

static void _RMSTUNE_end(int i)
{
	QueryPerformanceCounter(&_RMSTUNE_ends[i]);
}

static void _RMSTUNE_accum_init(int i)
{
	memset(&_RMSTUNE_accums[i], 0, sizeof(LARGE_INTEGER));
}

static void _RMSTUNE_accum(int i)
{
	_RMSTUNE_accums[i].QuadPart += (_RMSTUNE_ends[i].QuadPart - _RMSTUNE_starts[i].QuadPart);
}

static double _RMSTUNE_time(int i)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return (double)( (_RMSTUNE_ends[i].QuadPart - _RMSTUNE_starts[i].QuadPart)) / (double)freq.QuadPart;
}

static double _RMSTUNE_accum_time(int i)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return (double)( _RMSTUNE_accums[i].QuadPart) / (double)freq.QuadPart;
}

static void _RMSTUNE_winprint(double timeval, char * str)
{
	char buf[256];
	sprintf_s(buf, 256, "%f \n", timeval);
	OutputDebugString(str);
	OutputDebugString(buf);
}


#else

// [TODO] portable version

#endif