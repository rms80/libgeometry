// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CIRCLE2_H
#define WM4CIRCLE2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"

namespace Wm4
{

template <class Real>
class Circle2
{
public:
    // construction
    Circle2 ();  // uninitialized
    Circle2 (const Vector2<Real>& rkCenter, Real fRadius);

    Vector2<Real> Center;
    Real Radius;
};

#include "Wm4Circle2.inl"

typedef Circle2<float> Circle2f;
typedef Circle2<double> Circle2d;

}

#endif
