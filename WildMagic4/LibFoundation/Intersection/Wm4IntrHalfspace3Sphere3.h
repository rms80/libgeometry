// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRHALFSPACE3SPHERE3_H
#define WM4INTRHALFSPACE3SPHERE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Plane3.h"
#include "Wm4Sphere3.h"

// A halfspace is the set of points on the side of a plane to which the plane
// normal points.  The queries here are for intersection of a sphere and a
// halfspace.  In the dynamice find query, if the sphere is already
// intersecting the halfspace, the return value is 'false'.  The idea is to
// find first time of contact, in which case there is a single point of
// contacct.

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrHalfspace3Sphere3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrHalfspace3Sphere3 (const Plane3<Real>& rkHalfspace,
        const Sphere3<Real>& rkSphere);

    // object access
    const Plane3<Real>& GetHalfspace () const;
    const Sphere3<Real>& GetSphere () const;

    // Static query.
    virtual bool Test ();

    // Dynamic queries.
    virtual bool Test (Real fTMax, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

    virtual bool Find (Real fTMax, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

    // The intersection set is empty or a point.  GetPoint() returns a
    // valid result only when Find(...) returns 'true'.
    const Vector3<Real>& GetPoint () const;

protected:
    using Intersector<Real,Vector3<Real> >::m_fContactTime;

    // The objects to intersect.
    const Plane3<Real>* m_pkHalfspace;
    const Sphere3<Real>* m_pkSphere;

    // Information about the intersection set.
    Vector3<Real> m_kPoint;
};

typedef IntrHalfspace3Sphere3<float> IntrHalfspace3Sphere3f;
typedef IntrHalfspace3Sphere3<double> IntrHalfspace3Sphere3d;

}

#endif
