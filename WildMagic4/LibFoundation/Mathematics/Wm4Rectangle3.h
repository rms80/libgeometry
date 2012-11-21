// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4RECTANGLE3_H
#define WM4RECTANGLE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real>
class Rectangle3
{
public:
    // Points are R(s,t) = C+s0*U0+s1*U1, where C is the center of the
    // rectangle, U0 and U1 are unit-length and perpendicular axes.  The
    // parameters s0 and s1 are constrained by |s0| <= e0 and |s1| <= e1,
    // where e0 > 0 and e1 > 0 are called the extents of the rectangle.

    // construction
    Rectangle3 ();  // uninitialized
    Rectangle3 (const Vector3<Real>& rkCenter, const Vector3<Real>* akAxis,
        const Real* afExtent);
    Rectangle3 (const Vector3<Real>& rkCenter, const Vector3<Real>& rkAxis0,
        const Vector3<Real>& rkAxis1, Real fExtent0, Real fExtent1);

    void ComputeVertices (Vector3<Real> akVertex[4]) const;

    // corners
    Vector3<Real> GetPPCorner () const;  // C + e0*A0 + e1*A1
    Vector3<Real> GetPMCorner () const;  // C + e0*A0 - e1*A1
    Vector3<Real> GetMPCorner () const;  // C - e0*A0 + e1*A1
    Vector3<Real> GetMMCorner () const;  // C - e0*A0 - e1*A1

    Vector3<Real> Center;
    Vector3<Real> Axis[2];
    Real Extent[2];
};

#include "Wm4Rectangle3.inl"

typedef Rectangle3<float> Rectangle3f;
typedef Rectangle3<double> Rectangle3d;

}

#endif
