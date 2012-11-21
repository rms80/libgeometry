// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4TRIANGLE3_H
#define WM4TRIANGLE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real>
class Triangle3
{
public:
    // The triangle is represented as an array of three vertices, V0, V1,
    // and V2.

    // construction
    Triangle3 ();  // uninitialized
    Triangle3 (const Vector3<Real>& rkV0, const Vector3<Real>& rkV1,
        const Vector3<Real>& rkV2);
    Triangle3 (const Vector3<Real> akV[3]);

    // distance from the point Q to the triangle
    Real DistanceTo (const Vector3<Real>& rkQ) const;

    Vector3<Real> V[3];
};

#include "Wm4Triangle3.inl"

typedef Triangle3<float> Triangle3f;
typedef Triangle3<double> Triangle3d;

}

#endif
