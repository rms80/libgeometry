// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

// Bit hacks are available at
//     http://graphics.stanford.edu/~seander/bithacks.html

#ifndef WM4BITHACKS_H
#define WM4BITHACKS_H

namespace Wm4
{

bool IsPowerOfTwo (unsigned int uiValue);
unsigned int Log2OfPowerOfTwo (unsigned int uiPowerOfTwo);

#include "Wm4BitHacks.inl"

}

#endif
