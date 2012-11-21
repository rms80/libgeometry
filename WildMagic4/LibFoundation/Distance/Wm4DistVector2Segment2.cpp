// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistVector2Segment2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistVector2Segment2<Real>::DistVector2Segment2 (const Vector2<Real>& rkVector,
    const Segment2<Real>& rkSegment)
    :
    m_pkVector(&rkVector),
    m_pkSegment(&rkSegment)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& DistVector2Segment2<Real>::GetVector () const
{
    return *m_pkVector;
}
//----------------------------------------------------------------------------
template <class Real>
const Segment2<Real>& DistVector2Segment2<Real>::GetSegment () const
{
    return *m_pkSegment;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Segment2<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Segment2<Real>::GetSquared ()
{
    Vector2<Real> kDiff = *m_pkVector - m_pkSegment->Origin;
    Real fParam = m_pkSegment->Direction.Dot(kDiff);

    if (-m_pkSegment->Extent < fParam)
    {
        if (fParam < m_pkSegment->Extent)
        {
            m_kClosestPoint1 = m_pkSegment->Origin +
                fParam*m_pkSegment->Direction;
        }
        else
        {
            m_kClosestPoint1 = m_pkSegment->Origin +
                m_pkSegment->Extent*m_pkSegment->Direction;
        }
    }
    else
    {
        m_kClosestPoint1 = m_pkSegment->Origin -
            m_pkSegment->Extent*m_pkSegment->Direction;
    }

    m_kClosestPoint0 = *m_pkVector;
    kDiff = m_kClosestPoint1 - m_kClosestPoint0;
    return kDiff.SquaredLength();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Segment2<Real>::Get (Real fT,
    const Vector2<Real>& rkVelocity0, const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector2<Real> kMOrigin = m_pkSegment->Origin + fT*rkVelocity1;
    Segment2<Real> kMSegment(kMOrigin,m_pkSegment->Direction,
        m_pkSegment->Extent);
    return DistVector2Segment2<Real>(kMVector,kMSegment).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Segment2<Real>::GetSquared (Real fT,
    const Vector2<Real>& rkVelocity0, const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector2<Real> kMOrigin = m_pkSegment->Origin + fT*rkVelocity1;
    Segment2<Real> kMSegment(kMOrigin,m_pkSegment->Direction,
        m_pkSegment->Extent);
    return DistVector2Segment2<Real>(kMVector,kMSegment).GetSquared();
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistVector2Segment2<float>;

template WM4_FOUNDATION_ITEM
class DistVector2Segment2<double>;
//----------------------------------------------------------------------------
}
