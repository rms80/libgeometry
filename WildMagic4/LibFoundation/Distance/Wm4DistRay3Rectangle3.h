// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4DISTRAY3RECTANGLE3_H
#define WM4DISTRAY3RECTANGLE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Distance.h"
#include "Wm4Rectangle3.h"
#include "Wm4Ray3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM DistRay3Rectangle3
    : public Distance<Real,Vector3<Real> >
{
public:
    DistRay3Rectangle3 (const Ray3<Real>& rkRay,
        const Rectangle3<Real>& rkRectangle);

    // object access
    const Ray3<Real>& GetRay () const;
    const Rectangle3<Real>& GetRectangle () const;

    // static distance queries
    virtual Real Get ();
    virtual Real GetSquared ();

    // function calculations for dynamic distance queries
    virtual Real Get (Real fT, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);
    virtual Real GetSquared (Real fT, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

    // Information about the closest points.
    Real GetRayParameter () const;
    Real GetRectangleCoordinate (int i) const;

private:
    using Distance<Real,Vector3<Real> >::m_kClosestPoint0;
    using Distance<Real,Vector3<Real> >::m_kClosestPoint1;

    const Ray3<Real>* m_pkRay;
    const Rectangle3<Real>* m_pkRectangle;

    // Information about the closest points.
    Real m_fRayParameter;   // closest0 = ray.origin+param*ray.direction
    Real m_afRectCoord[2];  // closest1 = rect.center+param0*rect.dir0+
                            //            param1*rect.dir1
};

typedef DistRay3Rectangle3<float> DistRay3Rectangle3f;
typedef DistRay3Rectangle3<double> DistRay3Rectangle3d;

}

#endif
