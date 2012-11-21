// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4APPRLINEFIT3_H
#define WM4APPRLINEFIT3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Line3.h"

namespace Wm4
{

// Least-squares fit of a line to (x,y,z) data by using distance measurements
// orthogonal to the proposed line.
template <class Real> WM4_FOUNDATION_ITEM
Line3<Real> OrthogonalLineFit3 (int iQuantity, const Vector3<Real>* akPoint);

}

#endif
