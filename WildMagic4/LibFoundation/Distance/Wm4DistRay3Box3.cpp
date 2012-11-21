// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistRay3Box3.h"
#include "Wm4DistVector3Box3.h"
#include "Wm4DistLine3Box3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistRay3Box3<Real>::DistRay3Box3 (const Ray3<Real>& rkRay,
    const Box3<Real>& rkBox)
    :
    m_pkRay(&rkRay),
    m_pkBox(&rkBox)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Ray3<Real>& DistRay3Box3<Real>::GetRay () const
{
    return *m_pkRay;
}
//----------------------------------------------------------------------------
template <class Real>
const Box3<Real>& DistRay3Box3<Real>::GetBox () const
{
    return *m_pkBox;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Box3<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Box3<Real>::GetSquared ()
{
    Line3<Real> kLine(m_pkRay->Origin,m_pkRay->Direction);
    DistLine3Box3<Real> kLBDist(kLine,*m_pkBox);
    Real fSqrDistance = kLBDist.GetSquared();
    Real fLParam = kLBDist.GetLineParameter();

    if (fLParam >= (Real)0.0)
    {
        m_kClosestPoint0 = kLBDist.GetClosestPoint0();
        m_kClosestPoint1 = kLBDist.GetClosestPoint1();
    }
    else
    {
        DistVector3Box3<Real> kVBDist(m_pkRay->Origin,*m_pkBox);
        fSqrDistance = kVBDist.GetSquared();
        m_kClosestPoint0 = kVBDist.GetClosestPoint0();
        m_kClosestPoint1 = kVBDist.GetClosestPoint1();
    }
    return fSqrDistance;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Box3<Real>::Get (Real fT, const Vector3<Real>& rkVelocity0,
    const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMOrigin = m_pkRay->Origin + fT*rkVelocity0;
    Vector3<Real> kMCenter = m_pkBox->Center + fT*rkVelocity1;
    Ray3<Real> kMRay(kMOrigin,m_pkRay->Direction);
    Box3<Real> kMBox(kMCenter,m_pkBox->Axis,m_pkBox->Extent);
    return DistRay3Box3<Real>(kMRay,kMBox).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistRay3Box3<Real>::GetSquared (Real fT,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMOrigin = m_pkRay->Origin + fT*rkVelocity0;
    Vector3<Real> kMCenter = m_pkBox->Center + fT*rkVelocity1;
    Ray3<Real> kMRay(kMOrigin,m_pkRay->Direction);
    Box3<Real> kMBox(kMCenter,m_pkBox->Axis,m_pkBox->Extent);
    return DistRay3Box3<Real>(kMRay,kMBox).GetSquared();
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistRay3Box3<float>;

template WM4_FOUNDATION_ITEM
class DistRay3Box3<double>;
//----------------------------------------------------------------------------
}
