// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4TRIANGLE2_H
#define WM4TRIANGLE2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"

namespace Wm4
{

template <class Real>
class Triangle2
{
public:
    // The triangle is represented as an array of three vertices, V0, V1,
    // and V2.

    // construction
    Triangle2 ();  // uninitialized
    Triangle2 (const Vector2<Real>& rkV0, const Vector2<Real>& rkV1,
        const Vector2<Real>& rkV2);
    Triangle2 (const Vector2<Real> akV[3]);

    // distance from the point Q to the triangle
    Real DistanceTo (const Vector2<Real>& rkQ) const;

    Vector2<Real> V[3];
};

#include "Wm4Triangle2.inl"

typedef Triangle2<float> Triangle2f;
typedef Triangle2<double> Triangle2d;

}

#endif
