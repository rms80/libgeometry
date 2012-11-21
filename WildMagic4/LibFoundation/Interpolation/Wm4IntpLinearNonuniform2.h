// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPLINEARNONUNIFORM2_H
#define WM4INTPLINEARNONUNIFORM2_H

// Linear interpolation of a network of triangles whose vertices are of the
// form (x,y,f(x,y)).

#include "Wm4FoundationLIB.h"
#include "Wm4Delaunay2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpLinearNonuniform2
{
public:
    // Construction and destruction.  If you want IntpLinearNonuniform2 to
    // delete the input array during destruction, set bOwner to 'true'.
    // Otherwise, you own the array and must delete it yourself.
    IntpLinearNonuniform2 (const Delaunay2<Real>& rkDT, Real* afF,
        bool bOwner);

    ~IntpLinearNonuniform2 ();

    // Linear interpolation.  The return value is 'true' if and only if the
    // input point P is in the convex hull of the input vertices, in which
    // case the interpolation is valid.
    bool Evaluate (const Vector2<Real>& rkP, Real& rfF);

private:
    const Delaunay2<Real>* m_pkDT;
    Real* m_afF;
    bool m_bOwner;
};

typedef IntpLinearNonuniform2<float> IntpLinearNonuniform2f;
typedef IntpLinearNonuniform2<double> IntpLinearNonuniform2d;

}

#endif
