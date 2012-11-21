// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistRay3Triangle3.h"
#include "Wm4DistLine3Triangle3.h"
#include "Wm4DistVector3Triangle3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistRay3Triangle3<Real>::DistRay3Triangle3 (const Ray3<Real>& rkRay,
    const Triangle3<Real>& rkTriangle)
    :
    m_pkRay(&rkRay),
    m_pkTriangle(&rkTriangle)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Ray3<Real>& DistRay3Triangle3<Real>::GetRay () const
{
    return *m_pkRay;
}
//----------------------------------------------------------------------------
template <class Real>
const Triangle3<Real>& DistRay3Triangle3<Real>::GetTriangle () const
{
    return *m_pkTriangle;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Triangle3<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Triangle3<Real>::GetSquared ()
{
    DistLine3Triangle3<Real> kLTDist(Line3<Real>(m_pkRay->Origin,
        m_pkRay->Direction),*m_pkTriangle);
    Real fSqrDist = kLTDist.GetSquared();
    m_fRayParameter = kLTDist.GetLineParameter();

    if (m_fRayParameter >= (Real)0.0)
    {
        m_kClosestPoint0 = kLTDist.GetClosestPoint0();
        m_kClosestPoint1 = kLTDist.GetClosestPoint1();
        m_afTriangleBary[0] = kLTDist.GetTriangleBary(0);
        m_afTriangleBary[1] = kLTDist.GetTriangleBary(1);
        m_afTriangleBary[2] = kLTDist.GetTriangleBary(2);
    }
    else
    {
        m_kClosestPoint0 = m_pkRay->Origin;
        DistVector3Triangle3<Real> kVTDist(m_kClosestPoint0,*m_pkTriangle);
        fSqrDist = kVTDist.GetSquared();
        m_kClosestPoint1 = kVTDist.GetClosestPoint1();
        m_fRayParameter = (Real)0.0;
        m_afTriangleBary[0] = kVTDist.GetTriangleBary(0);
        m_afTriangleBary[1] = kVTDist.GetTriangleBary(1);
        m_afTriangleBary[2] = kVTDist.GetTriangleBary(2);
    }

    return fSqrDist;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Triangle3<Real>::Get (Real fT, const Vector3<Real>& rkVelocity0,
    const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMOrigin = m_pkRay->Origin + fT*rkVelocity0;
    Vector3<Real> kMV0 = m_pkTriangle->V[0] + fT*rkVelocity1;
    Vector3<Real> kMV1 = m_pkTriangle->V[1] + fT*rkVelocity1;
    Vector3<Real> kMV2 = m_pkTriangle->V[2] + fT*rkVelocity1;
    Ray3<Real> kMRay(kMOrigin,m_pkRay->Direction);
    Triangle3<Real> kMTriangle(kMV0,kMV1,kMV2);
    return DistRay3Triangle3<Real>(kMRay,kMTriangle).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Triangle3<Real>::GetSquared (Real fT,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMOrigin = m_pkRay->Origin + fT*rkVelocity0;
    Vector3<Real> kMV0 = m_pkTriangle->V[0] + fT*rkVelocity1;
    Vector3<Real> kMV1 = m_pkTriangle->V[1] + fT*rkVelocity1;
    Vector3<Real> kMV2 = m_pkTriangle->V[2] + fT*rkVelocity1;
    Ray3<Real> kMRay(kMOrigin,m_pkRay->Direction);
    Triangle3<Real> kMTriangle(kMV0,kMV1,kMV2);
    return DistRay3Triangle3<Real>(kMRay,kMTriangle).GetSquared();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Triangle3<Real>::GetRayParameter () const
{
    return m_fRayParameter;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Triangle3<Real>::GetTriangleBary (int i) const
{
    assert(0 <= i && i < 3);
    return m_afTriangleBary[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistRay3Triangle3<float>;

template WM4_FOUNDATION_ITEM
class DistRay3Triangle3<double>;
//----------------------------------------------------------------------------
}
