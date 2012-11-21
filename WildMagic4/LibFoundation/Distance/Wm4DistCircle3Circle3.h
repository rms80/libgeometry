// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4DISTCIRCLE3CIRCLE3_H
#define WM4DISTCIRCLE3CIRCLE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Distance.h"
#include "Wm4Circle3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM DistCircle3Circle3
    : public Distance<Real,Vector3<Real> >
{
public:
    DistCircle3Circle3 (const Circle3<Real>& rkCircle0,
        const Circle3<Real>& rkCircle1);

    // object access
    const Circle3<Real>& GetCircle0 () const;
    const Circle3<Real>& GetCircle1 () const;

    // static distance queries
    virtual Real Get ();
    virtual Real GetSquared ();

    // function calculations for dynamic distance queries
    virtual Real Get (Real fT, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);
    virtual Real GetSquared (Real fT, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

private:
    using Distance<Real,Vector3<Real> >::m_kClosestPoint0;
    using Distance<Real,Vector3<Real> >::m_kClosestPoint1;

    const Circle3<Real>* m_pkCircle0;
    const Circle3<Real>* m_pkCircle1;
};

typedef DistCircle3Circle3<float> DistCircle3Circle3f;
typedef DistCircle3Circle3<double> DistCircle3Circle3d;

}

#endif
