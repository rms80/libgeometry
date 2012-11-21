// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CAPSULE3_H
#define WM4CAPSULE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Segment3.h"

namespace Wm4
{

template <class Real>
class Capsule3
{
public:
    // construction
    Capsule3 ();  // uninitialized
    Capsule3 (const Segment3<Real>& rkSegment, Real fRadius);

    Segment3<Real> Segment;
    Real Radius;
};

#include "Wm4Capsule3.inl"

typedef Capsule3<float> Capsule3f;
typedef Capsule3<double> Capsule3d;

}

#endif
