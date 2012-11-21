// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CONTSCRIBECIRCLE2_H
#define WM4CONTSCRIBECIRCLE2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Circle2.h"
#include "Wm4LinearSystem.h"

namespace Wm4
{

// All functions return 'true' if circle has been constructed, 'false'
// otherwise (input points are linearly dependent).

// circle containing three 2D points
template <class Real> WM4_FOUNDATION_ITEM
bool Circumscribe (const Vector2<Real>& rkV0, const Vector2<Real>& rkV1,
    const Vector2<Real>& rkV2, Circle2<Real>& rkCircle);

// circle inscribing triangle of three 2D points
template <class Real> WM4_FOUNDATION_ITEM
bool Inscribe (const Vector2<Real>& rkV0, const Vector2<Real>& rkV1,
    const Vector2<Real>& rkV2, Circle2<Real>& rkCircle);

}

#endif
