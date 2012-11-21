// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4DISTVECTOR3SEGMENT3_H
#define WM4DISTVECTOR3SEGMENT3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Distance.h"
#include "Wm4Segment3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM DistVector3Segment3
    : public Distance<Real,Vector3<Real> >
{
public:
    DistVector3Segment3 (const Vector3<Real>& rkVector,
        const Segment3<Real>& rkSegment);

    // object access
    const Vector3<Real>& GetVector () const;
    const Segment3<Real>& GetSegment () const;

    // static distance queries
    virtual Real Get ();
    virtual Real GetSquared ();

    // function calculations for dynamic distance queries
    virtual Real Get (Real fT, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);
    virtual Real GetSquared (Real fT, const Vector3<Real>& rkVelocity0,
        const Vector3<Real>& rkVelocity1);

    // Information about the closest line point.
    Real GetSegmentParameter () const;

private:
    using Distance<Real,Vector3<Real> >::m_kClosestPoint0;
    using Distance<Real,Vector3<Real> >::m_kClosestPoint1;

    const Vector3<Real>* m_pkVector;
    const Segment3<Real>* m_pkSegment;

    // Information about the closest segment point.
    Real m_fSegmentParameter;  // closest1 = seg.origin+param*seg.direction
};

typedef DistVector3Segment3<float> DistVector3Segment3f;
typedef DistVector3Segment3<double> DistVector3Segment3d;

}

#endif
