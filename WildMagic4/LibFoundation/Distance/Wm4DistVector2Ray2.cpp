// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistVector2Ray2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistVector2Ray2<Real>::DistVector2Ray2 (const Vector2<Real>& rkVector,
    const Ray2<Real>& rkRay)
    :
    m_pkVector(&rkVector),
    m_pkRay(&rkRay)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& DistVector2Ray2<Real>::GetVector () const
{
    return *m_pkVector;
}
//----------------------------------------------------------------------------
template <class Real>
const Ray2<Real>& DistVector2Ray2<Real>::GetRay () const
{
    return *m_pkRay;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Ray2<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Ray2<Real>::GetSquared ()
{
    Vector2<Real> kDiff = *m_pkVector - m_pkRay->Origin;
    Real fParam = m_pkRay->Direction.Dot(kDiff);
    if (fParam > (Real)0.0)
    {
        m_kClosestPoint1 = m_pkRay->Origin + fParam*m_pkRay->Direction;
    }
    else
    {
        m_kClosestPoint1 = m_pkRay->Origin;
    }

    m_kClosestPoint0 = *m_pkVector;
    kDiff = m_kClosestPoint1 - m_kClosestPoint0;
    return kDiff.SquaredLength();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Ray2<Real>::Get (Real fT, const Vector2<Real>& rkVelocity0,
    const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector2<Real> kMOrigin = m_pkRay->Origin + fT*rkVelocity1;
    Ray2<Real> kMRay(kMOrigin,m_pkRay->Direction);
    return DistVector2Ray2<Real>(kMVector,kMRay).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Ray2<Real>::GetSquared (Real fT,
    const Vector2<Real>& rkVelocity0, const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector2<Real> kMOrigin = m_pkRay->Origin + fT*rkVelocity1;
    Ray2<Real> kMRay(kMOrigin,m_pkRay->Direction);
    return DistVector2Ray2<Real>(kMVector,kMRay).GetSquared();
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistVector2Ray2<float>;

template WM4_FOUNDATION_ITEM
class DistVector2Ray2<double>;
//----------------------------------------------------------------------------
}
