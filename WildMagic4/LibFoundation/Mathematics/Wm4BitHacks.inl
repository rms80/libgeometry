// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline bool IsPowerOfTwo (unsigned int uiValue)
{
    return (uiValue > 0) && ((uiValue & (uiValue - 1)) == 0);
}
//----------------------------------------------------------------------------
inline unsigned int Log2OfPowerOfTwo (unsigned int uiPowerOfTwo)
{
    unsigned int uiLog2 = (uiPowerOfTwo & 0xAAAAAAAA) != 0;
    uiLog2 |= ((uiPowerOfTwo & 0xFFFF0000) != 0) << 4;
    uiLog2 |= ((uiPowerOfTwo & 0xFF00FF00) != 0) << 3;
    uiLog2 |= ((uiPowerOfTwo & 0xF0F0F0F0) != 0) << 2;
    uiLog2 |= ((uiPowerOfTwo & 0xCCCCCCCC) != 0) << 1;
    return uiLog2;
}
//----------------------------------------------------------------------------
