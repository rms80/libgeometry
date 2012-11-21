// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistVector3Box3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistVector3Box3<Real>::DistVector3Box3 (const Vector3<Real>& rkVector,
    const Box3<Real>& rkBox)
    :
    m_pkVector(&rkVector),
    m_pkBox(&rkBox)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& DistVector3Box3<Real>::GetVector () const
{
    return *m_pkVector;
}
//----------------------------------------------------------------------------
template <class Real>
const Box3<Real>& DistVector3Box3<Real>::GetBox () const
{
    return *m_pkBox;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Box3<Real>::Get ()
{
    return Math<Real>::Sqrt(GetSquared());
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Box3<Real>::GetSquared ()
{
    // work in the box's coordinate system
    Vector3<Real> kDiff = *m_pkVector - m_pkBox->Center;

    // compute squared distance and closest point on box
    Real fSqrDistance = (Real)0.0, fDelta;
    Vector3<Real> kClosest;
    int i;
    for (i = 0; i < 3; i++)
    {
        kClosest[i] = kDiff.Dot(m_pkBox->Axis[i]);
        if (kClosest[i] < -m_pkBox->Extent[i])
        {
            fDelta = kClosest[i] + m_pkBox->Extent[i];
            fSqrDistance += fDelta*fDelta;
            kClosest[i] = -m_pkBox->Extent[i];
        }
        else if (kClosest[i] > m_pkBox->Extent[i])
        {
            fDelta = kClosest[i] - m_pkBox->Extent[i];
            fSqrDistance += fDelta*fDelta;
            kClosest[i] = m_pkBox->Extent[i];
        }
    }

    m_kClosestPoint0 = *m_pkVector;
    m_kClosestPoint1 = m_pkBox->Center;
    for (i = 0; i < 3; i++)
    {
        m_kClosestPoint1 += kClosest[i]*m_pkBox->Axis[i];
    }

    return fSqrDistance;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Box3<Real>::Get (Real fT, const Vector3<Real>& rkVelocity0,
    const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector3<Real> kMCenter = m_pkBox->Center + fT*rkVelocity1;
    Box3<Real> kMBox(kMCenter,m_pkBox->Axis,m_pkBox->Extent);
    return DistVector3Box3<Real>(kMVector,kMBox).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Box3<Real>::GetSquared (Real fT,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector3<Real> kMCenter = m_pkBox->Center + fT*rkVelocity1;
    Box3<Real> kMBox(kMCenter,m_pkBox->Axis,m_pkBox->Extent);
    return DistVector3Box3<Real>(kMVector,kMBox).GetSquared();
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistVector3Box3<float>;

template WM4_FOUNDATION_ITEM
class DistVector3Box3<double>;
//----------------------------------------------------------------------------
}
