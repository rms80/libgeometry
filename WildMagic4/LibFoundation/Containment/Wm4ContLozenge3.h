// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CONTLOZENGE3_H
#define WM4CONTLOZENGE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Lozenge3.h"

namespace Wm4
{

// Compute plane of lozenge rectangle using least-squares fit.  Parallel
// planes are chosen close enough together so that all the data points lie
// between them.  The radius is half the distance between the two planes.
// The half-cylinder and quarter-cylinder side pieces are chosen using a
// method similar to that used for fitting by capsules.
template <class Real> WM4_FOUNDATION_ITEM
Lozenge3<Real> ContLozenge (int iQuantity, const Vector3<Real>* akPoint);

// Test for containment of a point x by a lozenge.
template <class Real> WM4_FOUNDATION_ITEM
bool InLozenge (const Vector3<Real>& rkPoint,
    const Lozenge3<Real>& rkLozenge);

}

#endif
