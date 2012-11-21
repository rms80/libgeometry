// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRPLANE3TRIANGLE3_H
#define WM4INTRPLANE3TRIANGLE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Plane3.h"
#include "Wm4Triangle3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrPlane3Triangle3
    : public Intersector<Real,Vector3<Real> >
{
public:
    // If you want a fuzzy determination, set the epsilon value to a small
    // positive number.
    IntrPlane3Triangle3 (const Plane3<Real>& rkPlane,
        const Triangle3<Real>& rkTriangle, Real fEpsilon = (Real)0.0);

    // object access
    const Plane3<Real>& GetPlane () const;
    const Triangle3<Real>& GetTriangle () const;

    // static intersection queries
    virtual bool Test ();
    virtual bool Find ();

    // The intersection set, which is empty, a point, a line segment, or the
    // entire triangle (GetQuantity returns 0, 1, 2, or 3).
    int GetQuantity () const;
    const Vector3<Real>& GetPoint (int i) const;

protected:
    using Intersector<Real,Vector3<Real> >::IT_EMPTY;
    using Intersector<Real,Vector3<Real> >::IT_POINT;
    using Intersector<Real,Vector3<Real> >::IT_SEGMENT;
    using Intersector<Real,Vector3<Real> >::IT_POLYGON;
    using Intersector<Real,Vector3<Real> >::m_iIntersectionType;

    // the objects to intersect
    const Plane3<Real>* m_pkPlane;
    const Triangle3<Real>* m_pkTriangle;

    // information about the intersection set
    int m_iQuantity;
    Vector3<Real> m_akPoint[3];

    // for fuzzy arithmetic
    Real m_fEpsilon;
};

typedef IntrPlane3Triangle3<float> IntrPlane3Triangle3f;
typedef IntrPlane3Triangle3<double> IntrPlane3Triangle3d;

}

#endif
