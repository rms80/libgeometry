// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4BOX3_H
#define WM4BOX3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real>
class Box3
{
public:
    // A box has center C, axis directions U[0], U[1], and U[2] (all
    // unit-length vectors), and extents e[0], e[1], and e[2] (all nonnegative
    // numbers).  A point X = C+y[0]*U[0]+y[1]*U[1]+y[2]*U[2] is inside or
    // on the box whenever |y[i]| <= e[i] for all i.

    // construction
    Box3 ();  // uninitialized
    Box3 (const Vector3<Real>& rkCenter, const Vector3<Real>* akAxis,
        const Real* afExtent);
    Box3 (const Vector3<Real>& rkCenter, const Vector3<Real>& rkAxis0,
        const Vector3<Real>& rkAxis1, const Vector3<Real>& rkAxis2,
        Real fExtent0, Real fExtent1, Real fExtent2);

    void ComputeVertices (Vector3<Real> akVertex[8]) const;

    Vector3<Real> Center;
    Vector3<Real> Axis[3];  // must be an orthonormal set of vectors
    Real Extent[3];         // must be nonnegative
};

#include "Wm4Box3.inl"

typedef Box3<float> Box3f;
typedef Box3<double> Box3d;

}

#endif
