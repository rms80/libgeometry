// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4AXISALIGNEDBOX2_H
#define WM4AXISALIGNEDBOX2_H

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

namespace Wm4
{

template <class Real>
class AxisAlignedBox2
{
public:
    // construction
    AxisAlignedBox2 ();  // uninitialized

    // The caller must ensure that fXMin <= fXMax and fYMin <= fYMax.  The
    // class will not check for validity of the input.
    AxisAlignedBox2 (Real fXMin, Real fXMax, Real fYMin, Real fYMax);

    // Overlap testing is in the strict sense.  If the two boxes are just
    // touching along a common edge, the boxes are reported as overlapping.
    bool HasXOverlap (const AxisAlignedBox2& rkBox) const;
    bool HasYOverlap (const AxisAlignedBox2& rkBox) const;
    bool TestIntersection (const AxisAlignedBox2& rkBox) const;

    // The return value is 'true' if there is overlap.  In this case the
    // intersection is stored in rkIntr.  If the return value is 'false',
    // if there is no overlap.  In this case rkIntr is undefined.
    bool FindIntersection (const AxisAlignedBox2& rkBox,
        AxisAlignedBox2& rkIntr) const;

    Real Min[2], Max[2];
};

#include "Wm4AxisAlignedBox2.inl"

typedef AxisAlignedBox2<float> AxisAlignedBox2f;
typedef AxisAlignedBox2<double> AxisAlignedBox2d;

}

#endif
