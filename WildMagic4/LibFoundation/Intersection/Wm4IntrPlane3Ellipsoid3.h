// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRPLANE3ELLIPSOID3_H
#define WM4INTRPLANE3ELLIPSOID3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Plane3.h"
#include "Wm4Ellipsoid3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrPlane3Ellipsoid3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrPlane3Ellipsoid3 (const Plane3<Real>& rkPlane,
        const Ellipsoid3<Real>& rkEllipsoid);

    // object access
    const Plane3<Real>& GetPlane () const;
    const Ellipsoid3<Real>& GetEllipsoid () const;

    // static intersection query
    virtual bool Test ();

    // Culling support.  The view frustum is assumed to be on the positive
    // side of the plane.  The ellipsoid is culled if it is on the negative
    // side of the plane.
    bool EllipsoidIsCulled () const;

protected:
    // the objects to intersect
    const Plane3<Real>* m_pkPlane;
    const Ellipsoid3<Real>* m_pkEllipsoid;
};

typedef IntrPlane3Ellipsoid3<float> IntrPlane3Ellipsoid3f;
typedef IntrPlane3Ellipsoid3<double> IntrPlane3Ellipsoid3d;

}

#endif
