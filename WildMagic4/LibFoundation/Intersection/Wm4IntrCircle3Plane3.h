// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRCIRCLE3PLANE3_H
#define WM4INTRCIRCLE3PLANE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Circle3.h"
#include "Wm4Plane3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrCircle3Plane3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrCircle3Plane3 (const Circle3<Real>& rkCircle,
        const Plane3<Real>& rkPlane);

    // object access
    const Circle3<Real>& GetCircle () const;
    const Plane3<Real>& GetPlane () const;

    // static intersection queries
    virtual bool Test ();
    virtual bool Find ();

    // Information about the intersection set.  Only get the specific object
    // of intersection corresponding to the intersection type.  If the type
    // is IT_POINT, use GetPoint(i).  If the type is IT_OTHER, the set is a
    // circle, so use GetIntersectionCircle(), which returns the circle
    // object.
    int GetQuantity () const;
    const Vector3<Real>& GetPoint (int i) const;
    const Circle3<Real>& GetIntersectionCircle () const;

protected:
    using Intersector<Real,Vector3<Real> >::IT_EMPTY;
    using Intersector<Real,Vector3<Real> >::IT_POINT;
    using Intersector<Real,Vector3<Real> >::IT_PLANE;
    using Intersector<Real,Vector3<Real> >::IT_OTHER;
    using Intersector<Real,Vector3<Real> >::m_iIntersectionType;

    // the objects to intersect
    const Circle3<Real>* m_pkCircle;
    const Plane3<Real>* m_pkPlane;

    // information about the intersection set
    int m_iQuantity;
    Vector3<Real> m_akPoint[2];
};

typedef IntrCircle3Plane3<float> IntrCircle3Plane3f;
typedef IntrCircle3Plane3<double> IntrCircle3Plane3d;

}

#endif
