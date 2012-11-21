// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4RAY2_H
#define WM4RAY2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"

namespace Wm4
{

template <class Real>
class Ray2
{
public:
    // The ray is represented as P+t*D, where P is the ray origin, D is a
    // unit-length direction vector, and t >= 0.  The user must ensure that
    // the direction vector satisfies this condition.

    // construction
    Ray2 ();  // uninitialized
    Ray2 (const Vector2<Real>& rkOrigin, const Vector2<Real>& rkDirection);

    Vector2<Real> Origin, Direction;
};

#include "Wm4Ray2.inl"

typedef Ray2<float> Ray2f;
typedef Ray2<double> Ray2d;

}

#endif
