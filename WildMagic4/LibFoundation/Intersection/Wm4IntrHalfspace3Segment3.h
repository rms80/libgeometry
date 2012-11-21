// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRHALFSPACE3SEGMENT3_H
#define WM4INTRHALFSPACE3SEGMENT3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Plane3.h"
#include "Wm4Segment3.h"

// A halfspace is the set of points on the side of a plane to which the plane
// normal points.  The queries here are for intersection of a line segment and
// a halfspace.  In the dynamice find query, if the segment is already
// intersecting the halfspace, the return value is 'false'.  The idea is to
// find first time of contact.

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrHalfspace3Segment3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrHalfspace3Segment3 (const Plane3<Real>& rkHalfspace,
        const Segment3<Real>& rkSegment);

    // object access
    const Plane3<Real>& GetHalfspace () const;
    const Segment3<Real>& GetSegment () const;

    // Static queries.
    virtual bool Test ();
    virtual bool Find ();

    // Dynamic queries.
    virtual bool Test (Real fTMax, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

    virtual bool Find (Real fTMax, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

    // The intersection set is empty, a point, or a segment.  The function
    // GetQuantity() returns 0, 1, or 2.
    int GetQuantity () const;
    const Vector3<Real>& GetPoint (int i) const;

protected:
    using Intersector<Real,Vector3<Real> >::m_fContactTime;

    // The objects to intersect.
    const Plane3<Real>* m_pkHalfspace;
    const Segment3<Real>* m_pkSegment;

    // Information about the intersection set.
    int m_iQuantity;
    Vector3<Real> m_akPoint[2];
};

typedef IntrHalfspace3Segment3<float> IntrHalfspace3Segment3f;
typedef IntrHalfspace3Segment3<double> IntrHalfspace3Segment3d;

}

#endif
