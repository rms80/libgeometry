// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4APPRPLANEFIT3_H
#define WM4APPRPLANEFIT3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Plane3.h"

namespace Wm4
{

// Least-squares fit of a plane to (x,y,f(x,y)) data by using distance
// measurements in the z-direction.  The resulting plane is represented by
// z = A*x + B*y + C.  The return value is 'false' if the 3x3 coefficient
// matrix in the linear system that defines A, B, and C is (nearly) singular.
// In this case, A, B, and C are returned as MAX_REAL.
template <class Real> WM4_FOUNDATION_ITEM
bool HeightPlaneFit3 (int iQuantity, const Vector3<Real>* akPoint,
    Real& rfA, Real& rfB, Real& rfC);

// Least-squares fit of a plane to (x,y,z) data by using distance measurements
// orthogonal to the proposed plane.
template <class Real> WM4_FOUNDATION_ITEM
Plane3<Real> OrthogonalPlaneFit3 (int iQuantity,
    const Vector3<Real>* akPoint);

}

#endif
