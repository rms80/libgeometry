// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4TRIANGLEKEY_H
#define WM4TRIANGLEKEY_H

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

namespace Wm4
{

class WM4_FOUNDATION_ITEM TriangleKey
{
public:
    TriangleKey (int iV0 = -1, int iV1 = -1, int iV2 = -1);
    bool operator< (const TriangleKey& rkKey) const;
    operator size_t () const;
    int V[3];
};

#include "Wm4TriangleKey.inl"

}

#endif
