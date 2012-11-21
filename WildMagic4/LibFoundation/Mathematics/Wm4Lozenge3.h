// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4LOZENGE3_H
#define WM4LOZENGE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Rectangle3.h"

namespace Wm4
{

template <class Real>
class Lozenge3
{
public:
    // construction
    Lozenge3 ();  // uninitialized
    Lozenge3 (const Rectangle3<Real>& rkRectangle, Real fRadius);

    Rectangle3<Real> Rectangle;
    Real Radius;
};

#include "Wm4Lozenge3.inl"

typedef Lozenge3<float> Lozenge3f;
typedef Lozenge3<double> Lozenge3d;

}

#endif
