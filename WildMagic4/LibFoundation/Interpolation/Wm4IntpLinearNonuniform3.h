// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPLINEARNONUNIFORM3_H
#define WM4INTPLINEARNONUNIFORM3_H

// Linear interpolation of a network of triangles whose vertices are of the
// form (x,y,z,f(x,y,z)).

#include "Wm4FoundationLIB.h"
#include "Wm4Delaunay3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpLinearNonuniform3
{
public:
    // Construction and destruction.  If you want IntpLinearNonuniform3 to
    // delete the input array during destruction, set bOwner to 'true'.
    // Otherwise, you own the array and must delete it yourself.
    IntpLinearNonuniform3 (const Delaunay3<Real>& rkDT, Real* afF,
        bool bOwner);

    ~IntpLinearNonuniform3 ();

    // Linear interpolation.  The return value is 'true' if and only if the
    // input point is in the convex hull of the input vertices, in which case
    // the interpolation is valid.
    bool Evaluate (const Vector3<Real>& rkP, Real& rfF);

private:
    const Delaunay3<Real>* m_pkDT;
    Real* m_afF;
    bool m_bOwner;
};

typedef IntpLinearNonuniform3<float> IntpLinearNonuniform3f;
typedef IntpLinearNonuniform3<double> IntpLinearNonuniform3d;

}

#endif
