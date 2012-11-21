// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CYLINDER3_H
#define WM4CYLINDER3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Segment3.h"

namespace Wm4
{

template <class Real>
class Cylinder3
{
public:
    // Cylinder line segment has end points C-(H/2)*D and C+(H/2)*D where
    // D is a unit-length vector.  H is infinity for infinite cylinder.

    // construction
    Cylinder3 ();  // uninitialized
    Cylinder3 (const Segment3<Real>& rkSegment, Real fHeight, Real fRadius);

    Segment3<Real> Segment;
    Real Height, Radius;
};

#include "Wm4Cylinder3.inl"

typedef Cylinder3<float> Cylinder3f;
typedef Cylinder3<double> Cylinder3d;

}

#endif
