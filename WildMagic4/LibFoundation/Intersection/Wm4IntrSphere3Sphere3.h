// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRSPHERE3SPHERE3_H
#define WM4INTRSPHERE3SPHERE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Sphere3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrSphere3Sphere3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrSphere3Sphere3 (const Sphere3<Real>& rkSphere0,
        const Sphere3<Real>& rkSphere1);

    // object access
    const Sphere3<Real>& GetSphere0 () const;
    const Sphere3<Real>& GetSphere1 () const;

    // static test-intersection query
    virtual bool Test ();

    // static find-intersection query
    virtual bool Find ();

    // intersection set for static find-intersection query
    const Vector3<Real>& GetCenter () const;
    const Vector3<Real>& GetUAxis () const;
    const Vector3<Real>& GetVAxis () const;
    const Vector3<Real>& GetNormal () const;
    Real GetRadius () const;

    // dynamic test-intersection query
    virtual bool Test (Real fTMax, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

    // dynamic find-intersection query
    virtual bool Find (Real fTMax, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

    // intersection set for dynamic find-intersection query
    const Vector3<Real>& GetContactPoint () const;

private:
    using Intersector<Real,Vector3<Real> >::m_fContactTime;

    // the objects to intersect
    const Sphere3<Real>* m_pkSphere0;
    const Sphere3<Real>* m_pkSphere1;

    // Circle of intersection for static spheres.  The center is C and lies
    // on a plane spanned by the unit-length, orthogonal vectors U and V.
    // The plane normal is a unit-length vector N.  The radius of the circle
    // in that plane is R.
    Vector3<Real> m_kCenter, m_kUAxis, m_kVAxis, m_kNormal;
    Real m_fRadius;

    // Point of intersection for dynamic spheres.
    Vector3<Real> m_kContactPoint;
};

typedef IntrSphere3Sphere3<float> IntrSphere3Sphere3f;
typedef IntrSphere3Sphere3<double> IntrSphere3Sphere3d;

}

#endif
