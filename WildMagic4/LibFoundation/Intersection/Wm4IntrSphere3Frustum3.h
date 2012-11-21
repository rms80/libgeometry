// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRSPHERE3FRUSTUM3_H
#define WM4INTRSPHERE3FRUSTUM3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Sphere3.h"
#include "Wm4Frustum3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrSphere3Frustum3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrSphere3Frustum3 (const Sphere3<Real>& rkSphere,
        const Frustum3<Real>& rkFrustum);

    // object access
    const Sphere3<Real>& GetSphere () const;
    const Frustum3<Real>& GetFrustum () const;

    // static intersection query
    virtual bool Test ();

private:
    // the objects to intersect
    const Sphere3<Real>* m_pkSphere;
    const Frustum3<Real>* m_pkFrustum;
};

typedef IntrSphere3Frustum3<float> IntrSphere3Frustum3f;
typedef IntrSphere3Frustum3<double> IntrSphere3Frustum3d;

}

#endif
