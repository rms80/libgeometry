// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

// [RMS] DLL build is currently useless because the following macro is missing for most classes

// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the RMSBLOBTREE_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// RMSBLOBTREE_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef LIBGEOMETRY_EXPORTS
#define LIBGEOMETRY_API __declspec(dllexport)
#else
#define LIBGEOMETRY_API __declspec(dllimport)
#endif



#ifndef WIN32
#include <cmath>
static bool _finite(double val){return std::isfinite(val);}
#endif



// defined in config.cpp
void lgBreakToDebugger();
int lgAssertReport(const char * filename, int line, const char * message);


// set up our own lgASSERT macros, based on win32 crt version

#define lg_ASSERT_EXPR(expr, msg) \
        (void) ((!!(expr)) || \
                (1 != lgAssertReport(__FILE__, __LINE__, msg)) || \
                (lgBreakToDebugger(), 0))

#ifdef DEBUG
//#define lglgASSERT(expr)   lg_ASSERT_EXPR((expr), NULL)
#define lgASSERT(expr)  lg_ASSERT_EXPR((expr), (#expr))
#else
#define lgASSERT(expr)
#endif







// [RMS] following is broken because I got lazy with using the macro-namespaces...

// hacky stuff so that files compile here and in shapeshop...

//#define IN_SHAPESHOP

//#ifdef IN_SHAPESHOP
//#define wmlNS Wml
//#define rms rmsmesh
//#include <Frame.h>
//#include "VFTriangleMesh.h"
//#else
//#define RMSIMPLICIT_API 
//#define wmlNS Wml
//#define rms rms
//#endif
