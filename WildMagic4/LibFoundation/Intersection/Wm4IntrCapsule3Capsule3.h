// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRCAPSULE3CAPSULE3_H
#define WM4INTRCAPSULE3CAPSULE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Capsule3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrCapsule3Capsule3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrCapsule3Capsule3 (const Capsule3<Real>& rkCapsule0,
        const Capsule3<Real>& rkCapsule1);

    // object access
    const Capsule3<Real>& GetCapsule0 () const;
    const Capsule3<Real>& GetCapsule1 () const;

    // static test-intersection query
    virtual bool Test ();

private:
    // the objects to intersect
    const Capsule3<Real>* m_pkCapsule0;
    const Capsule3<Real>* m_pkCapsule1;
};

typedef IntrCapsule3Capsule3<float> IntrCapsule3Capsule3f;
typedef IntrCapsule3Capsule3<double> IntrCapsule3Capsule3d;

}

#endif
