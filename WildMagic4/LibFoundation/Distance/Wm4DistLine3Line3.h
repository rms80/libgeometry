// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4DISTLINE3LINE3_H
#define WM4DISTLINE3LINE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Distance.h"
#include "Wm4Line3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM DistLine3Line3
    : public Distance<Real,Vector3<Real> >
{
public:
    DistLine3Line3 (const Line3<Real>& rkLine0, const Line3<Real>& rkLine1);

    // object access
    const Line3<Real>& GetLine0 () const;
    const Line3<Real>& GetLine1 () const;

    // static distance queries
    virtual Real Get ();
    virtual Real GetSquared ();

    // function calculations for dynamic distance queries
    virtual Real Get (Real fT, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);
    virtual Real GetSquared (Real fT, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

    // Information about the closest points.
    Real GetLine0Parameter () const;
    Real GetLine1Parameter () const;

private:
    using Distance<Real,Vector3<Real> >::m_kClosestPoint0;
    using Distance<Real,Vector3<Real> >::m_kClosestPoint1;

    const Line3<Real>* m_pkLine0;
    const Line3<Real>* m_pkLine1;

    // Information about the closest points.
    Real m_fLine0Parameter;  // closest0 = line0.origin+param*line0.direction
    Real m_fLine1Parameter;  // closest1 = line1.origin+param*line1.direction
};

typedef DistLine3Line3<float> DistLine3Line3f;
typedef DistLine3Line3<double> DistLine3Line3d;

}

#endif
