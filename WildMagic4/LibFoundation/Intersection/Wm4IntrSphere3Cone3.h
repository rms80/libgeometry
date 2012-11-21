// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRSPHERE3CONE3_H
#define WM4INTRSPHERE3CONE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Sphere3.h"
#include "Wm4Cone3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrSphere3Cone3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrSphere3Cone3 (const Sphere3<Real>& rkSphere,
        const Cone3<Real>& rkCone);

    // object access
    const Sphere3<Real>& GetSphere () const;
    const Cone3<Real>& GetCone () const;

    // static intersection queries
    virtual bool Test ();
    virtual bool Find ();

    // In the static find-intersection query, if an intersection occurs
    // between the sphere and cone, it is potentially an infinite set.  The
    // intersection point closest to the cone vertex is returned by this
    // function.
    const Vector3<Real>& GetPoint () const;

private:
    // the objects to intersect
    const Sphere3<Real>* m_pkSphere;
    const Cone3<Real>* m_pkCone;

    // closest intersection point to cone vertex
    Vector3<Real> m_kPoint;
};

typedef IntrSphere3Cone3<float> IntrSphere3Cone3f;
typedef IntrSphere3Cone3<double> IntrSphere3Cone3d;

}

#endif
