// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistVector2Line2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistVector2Line2<Real>::DistVector2Line2 (const Vector2<Real>& rkVector,
    const Line2<Real>& rkLine)
    :
    m_pkVector(&rkVector),
    m_pkLine(&rkLine)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& DistVector2Line2<Real>::GetVector () const
{
    return *m_pkVector;
}
//----------------------------------------------------------------------------
template <class Real>
const Line2<Real>& DistVector2Line2<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Line2<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Line2<Real>::GetSquared ()
{
    Vector2<Real> kDiff = *m_pkVector - m_pkLine->Origin;
    Real fParam = m_pkLine->Direction.Dot(kDiff);
    m_kClosestPoint0 = *m_pkVector;
    m_kClosestPoint1 = m_pkLine->Origin + fParam*m_pkLine->Direction;
    kDiff = m_kClosestPoint1 - m_kClosestPoint0;
    return kDiff.SquaredLength();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Line2<Real>::Get (Real fT, const Vector2<Real>& rkVelocity0,
    const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector2<Real> kMOrigin = m_pkLine->Origin + fT*rkVelocity1;
    Line2<Real> kMLine(kMOrigin,m_pkLine->Direction);
    return DistVector2Line2<Real>(kMVector,kMLine).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector2Line2<Real>::GetSquared (Real fT,
    const Vector2<Real>& rkVelocity0, const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector2<Real> kMOrigin = m_pkLine->Origin + fT*rkVelocity1;
    Line2<Real> kMLine(kMOrigin,m_pkLine->Direction);
    return DistVector2Line2<Real>(kMVector,kMLine).GetSquared();
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistVector2Line2<float>;

template WM4_FOUNDATION_ITEM
class DistVector2Line2<double>;
//----------------------------------------------------------------------------
}
