// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4APPRLINEFIT2_H
#define WM4APPRLINEFIT2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Line2.h"

namespace Wm4
{

// Least-squares fit of a line to (x,f(x)) data by using distance
// measurements in the y-direction.  The resulting line is represented by
// y = A*x + B.  The return value is 'false' if the 2x2 coefficient matrix
// in the linear system that defines A and B is (nearly) singular.  In this
// case, A and B are returned as MAX_REAL.
template <class Real> WM4_FOUNDATION_ITEM
bool HeightLineFit2 (int iQuantity, const Vector2<Real>* akPoint, Real& rfA,
    Real& rfB);

// Least-squares fit of a line to (x,y,z) data by using distance measurements
// orthogonal to the proposed line.
template <class Real> WM4_FOUNDATION_ITEM
Line2<Real> OrthogonalLineFit2 (int iQuantity, const Vector2<Real>* akPoint);

}

#endif
