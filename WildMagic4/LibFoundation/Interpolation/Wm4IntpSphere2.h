// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPSPHERE2_H
#define WM4INTPSPHERE2_H

// Interpolation of a scalar-valued function defined on a sphere.  Although
// the sphere lives in 3D, the interpolation is a 2D method whose input
// points are angles (theta,phi) from spherical coordinates.  The domains of
// the angles are -PI <= theta <= PI and 0 <= phi <= PI.

#include "Wm4FoundationLIB.h"
#include "Wm4IntpQdrNonuniform2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpSphere2
{
public:
    // Construction and destruction.  If you want IntpSphere2 to delete the
    // input arrays during destruction, set bOwner to 'true'.  Otherwise, you
    // own the arrays and must delete them yourself.
    //
    // For complete spherical coverage, include the two antipodal (theta,phi)
    // points (-PI,0,F(-PI,0)) and (-PI,PI,F(-PI,PI)) in the input data.
    // These correspond to the sphere poles x = 0, y = 0, and |z| = 1.
    //
    // The computation type is for the Delaunay triangulation and should be
    // one of Query::{QT_INT64,QT_INTEGER,QT_RATIONAL,QT_REAL}.
    IntpSphere2 (int iQuantity, Real* afTheta, Real* afPhi, Real* afF,
        bool bOwner, Query::Type eQueryType);

    ~IntpSphere2 ();

    // Spherical coordinates are
    //   x = cos(theta)*sin(phi)
    //   y = sin(theta)*sin(phi)
    //   z = cos(phi)
    // for -PI <= theta <= PI, 0 <= phi <= PI.  The application can use this
    // function to convert unit length vectors (x,y,z) to (theta,phi).
    static void GetSphericalCoords (Real fX, Real fY, Real fZ,
        Real& rfTheta, Real& rfPhi);

    bool Evaluate (Real fTheta, Real fPhi, Real& rfF);

private:
    Delaunay2<Real>* m_pkDel;
    IntpQdrNonuniform2<Real>* m_pkInterp;
};

typedef IntpSphere2<float> IntpSphere2f;
typedef IntpSphere2<double> IntpSphere2d;

}

#endif
