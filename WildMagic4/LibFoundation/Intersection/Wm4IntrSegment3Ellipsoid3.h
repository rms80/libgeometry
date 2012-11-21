// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRSEGMENT3ELLIPSOID3_H
#define WM4INTRSEGMENT3ELLIPSOID3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Segment3.h"
#include "Wm4Ellipsoid3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrSegment3Ellipsoid3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrSegment3Ellipsoid3 (const Segment3<Real>& rkSegment,
        const Ellipsoid3<Real>& rkEllipsoid);

    // object access
    const Segment3<Real>& GetSegment () const;
    const Ellipsoid3<Real>& GetEllipsoid () const;

    // static intersection queries
    virtual bool Test ();
    virtual bool Find ();

    // The intersection set.  The ellipsoid is considered a solid, so if the
    // segment is stricly inside the ellipsoid, the intersection type is
    // IT_SEGMENT and the function GetPoint(i) returns the segment end
    // points.  If you want the ellipsoid to be only the surface itself, and
    // you want a "no intersection" result when the segment is strictly
    // inside the ellipsoid, just compare the segment end points to the
    // values from GetPoint(i) whenever IT_SEGMENT is the type.  If they
    // are the same, consider this a "no intersection".
    int GetQuantity () const;
    const Vector3<Real>& GetPoint (int i) const;

private:
    using Intersector<Real,Vector3<Real> >::IT_EMPTY;
    using Intersector<Real,Vector3<Real> >::IT_POINT;
    using Intersector<Real,Vector3<Real> >::IT_SEGMENT;
    using Intersector<Real,Vector3<Real> >::m_iIntersectionType;

    // the objects to intersect
    const Segment3<Real>* m_pkSegment;
    const Ellipsoid3<Real>* m_pkEllipsoid;

    // information about the intersection set
    int m_iQuantity;
    Vector3<Real> m_akPoint[2];
};

typedef IntrSegment3Ellipsoid3<float> IntrSegment3Ellipsoid3f;
typedef IntrSegment3Ellipsoid3<double> IntrSegment3Ellipsoid3d;

}

#endif
