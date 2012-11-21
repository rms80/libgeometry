// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CONE3_H
#define WM4CONE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real>
class Cone3
{
public:
    // An acute cone is Dot(A,X-V) = |X-V| cos(T) where V is the vertex, A
    // is the unit-length direction of the axis of the cone, and T is the
    // cone angle with 0 < T < PI/2.  The cone interior is defined by the
    // inequality Dot(A,X-V) >= |X-V| cos(T).  Since cos(T) > 0, we can avoid
    // computing square roots.  The solid cone is defined by the inequality
    // Dot(A,X-V)^2 >= Dot(X-V,X-V) cos(T)^2.

    // construction
    Cone3 ();  // uninitialized
    Cone3 (const Vector3<Real>& rkVertex, const Vector3<Real>& rkAxis,
        Real fAngle);
    Cone3 (const Vector3<Real>& rkVertex, const Vector3<Real>& rkAxis,
        Real fCosAngle, Real fSinAngle);

    Vector3<Real> Vertex;
    Vector3<Real> Axis;
    Real CosAngle, SinAngle;  // cos(T), sin(T)
};

#include "Wm4Cone3.inl"

typedef Cone3<float> Cone3f;
typedef Cone3<double> Cone3d;

}

#endif
