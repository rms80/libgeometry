// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CONTCAPSULE3_H
#define WM4CONTCAPSULE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Capsule3.h"
#include "Wm4Sphere3.h"

namespace Wm4
{

// Compute axis of capsule segment using least-squares fit.  Radius is
// maximum distance from points to axis.  Hemispherical caps are chosen
// as close together as possible.
template <class Real> WM4_FOUNDATION_ITEM
Capsule3<Real> ContCapsule (int iQuantity, const Vector3<Real>* akPoint);

// Test for containment of a point by a capsule.
template <class Real> WM4_FOUNDATION_ITEM
bool InCapsule (const Vector3<Real>& rkPoint,
    const Capsule3<Real>& rkCapsule);

// Test for containment of a sphere by a capsule.
template <class Real> WM4_FOUNDATION_ITEM
bool InCapsule (const Sphere3<Real>& rkSphere,
    const Capsule3<Real>& rkCapsule);

// Test for containment of a capsule by a capsule.
template <class Real> WM4_FOUNDATION_ITEM
bool InCapsule (const Capsule3<Real>& rkTestCapsule,
    const Capsule3<Real>& rkCapsule);

// Compute a capsule that contains the input capsules.  The returned capsule
// is not necessarily the one of smallest volume that contains the inputs.
template <class Real> WM4_FOUNDATION_ITEM
Capsule3<Real> MergeCapsules (const Capsule3<Real>& rkCapsule0,
    const Capsule3<Real>& rkCapsule1);

}

#endif
