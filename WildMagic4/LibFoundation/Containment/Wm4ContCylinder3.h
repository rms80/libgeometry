// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CONTCYLINDER3_H
#define WM4CONTCYLINDER3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Cylinder3.h"

namespace Wm4
{

// Compute the cylinder axis segment using least-squares fit.  The radius is
// the maximum distance from points to the axis.  The height is determined by
// projection of points onto the axis and determining the containing interval.
template <class Real> WM4_FOUNDATION_ITEM
Cylinder3<Real> ContCylinder (int iQuantity, const Vector3<Real>* akPoint);

}

#endif
