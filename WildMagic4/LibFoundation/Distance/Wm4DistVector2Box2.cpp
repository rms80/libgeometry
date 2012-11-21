// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistVector2Box2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistVector2Box2<Real>::DistVector2Box2 (const Vector2<Real>& rkVector,
    const Box2<Real>& rkBox)
    :
    m_pkVector(&rkVector),
    m_pkBox(&rkBox)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& DistVector2Box2<Real>::GetVector () const
{
    return *m_pkVector;
}
//----------------------------------------------------------------------------
template <class Real>
const Box2<Real>& DistVector2Box2<Real>::GetBox () const
{
    return *m_pkBox;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Box2<Real>::Get ()
{
    return Math<Real>::Sqrt(GetSquared());
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Box2<Real>::GetSquared ()
{
    // work in the box's coordinate system
    Vector2<Real> kDiff = *m_pkVector - m_pkBox->Center;

    // compute squared distance and closest point on box
    Real fSqrDistance = (Real)0.0, fDelta;
    Vector2<Real> kClosest;
    int i;
    for (i = 0; i < 2; i++)
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
    for (i = 0; i < 2; i++)
    {
        m_kClosestPoint1 += kClosest[i]*m_pkBox->Axis[i];
    }

    return fSqrDistance;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Box2<Real>::Get (Real fT, const Vector2<Real>& rkVelocity0,
    const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector2<Real> kMCenter = m_pkBox->Center + fT*rkVelocity1;
    Box2<Real> kMBox(kMCenter,m_pkBox->Axis,m_pkBox->Extent);
    return DistVector2Box2<Real>(kMVector,kMBox).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Box2<Real>::GetSquared (Real fT,
    const Vector2<Real>& rkVelocity0, const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector2<Real> kMCenter = m_pkBox->Center + fT*rkVelocity1;
    Box2<Real> kMBox(kMCenter,m_pkBox->Axis,m_pkBox->Extent);
    return DistVector2Box2<Real>(kMVector,kMBox).GetSquared();
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistVector2Box2<float>;

template WM4_FOUNDATION_ITEM
class DistVector2Box2<double>;
//----------------------------------------------------------------------------
}
