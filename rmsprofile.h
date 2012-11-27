// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef RMS_PROFILE_H
#define RMS_PROFILE_H

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


#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

static timeval _RMSTUNE_starts[16];
static timeval _RMSTUNE_ends[16];
static long _RMSTUNE_accums[16];


static long msBetween(const timeval& start, const timeval& end){
  long seconds  = end.tv_sec  - start.tv_sec;
  long useconds = end.tv_usec - start.tv_usec;

  return  ((seconds) * 1000 + useconds/1000.0) + 0.5;
}


static void _RMSTUNE_start(int i){ gettimeofday(&_RMSTUNE_starts[i],NULL); }
static void _RMSTUNE_end(int i){ gettimeofday(&_RMSTUNE_ends[i],NULL); }

static void _RMSTUNE_accum_init(int i){ _RMSTUNE_accums[i] = 0;}

static void _RMSTUNE_accum(int i){_RMSTUNE_accums[i] += msBetween(_RMSTUNE_starts[i],_RMSTUNE_ends[i]);}

static double _RMSTUNE_time(int i){
  return msBetween(_RMSTUNE_starts[i],_RMSTUNE_ends[i]);
} // result in ms

static double _RMSTUNE_accum_time(int i){ return (double) (_RMSTUNE_accums[i] ); } // time in ms


static void _RMSTUNE_Print_Time(const std::string& msg, int i){
  std::cout << msg << ": " << _RMSTUNE_time(i) << " ms " << std::endl;
}


static void DebugBreak(){
  std::cout << "DebugBreak not implemented on Linux!" << std::endl;
}



#endif

#endif
