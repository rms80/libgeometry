// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistVector3Line3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistVector3Line3<Real>::DistVector3Line3 (const Vector3<Real>& rkVector,
    const Line3<Real>& rkLine)
    :
    m_pkVector(&rkVector),
    m_pkLine(&rkLine)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& DistVector3Line3<Real>::GetVector () const
{
    return *m_pkVector;
}
//----------------------------------------------------------------------------
template <class Real>
const Line3<Real>& DistVector3Line3<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Line3<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Line3<Real>::GetSquared ()
{
    Vector3<Real> kDiff = *m_pkVector - m_pkLine->Origin;
    m_fLineParameter = m_pkLine->Direction.Dot(kDiff);
    m_kClosestPoint0 = *m_pkVector;
    m_kClosestPoint1 = m_pkLine->Origin +
        m_fLineParameter*m_pkLine->Direction;
    kDiff = m_kClosestPoint1 - m_kClosestPoint0;
    return kDiff.SquaredLength();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Line3<Real>::Get (Real fT, const Vector3<Real>& rkVelocity0,
    const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector3<Real> kMOrigin = m_pkLine->Origin + fT*rkVelocity1;
    Line3<Real> kMLine(kMOrigin,m_pkLine->Direction);
    return DistVector3Line3<Real>(kMVector,kMLine).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Line3<Real>::GetSquared (Real fT,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector3<Real> kMOrigin = m_pkLine->Origin + fT*rkVelocity1;
    Line3<Real> kMLine(kMOrigin,m_pkLine->Direction);
    return DistVector3Line3<Real>(kMVector,kMLine).GetSquared();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Line3<Real>::GetLineParameter () const
{
    return m_fLineParameter;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistVector3Line3<float>;

template WM4_FOUNDATION_ITEM
class DistVector3Line3<double>;
//----------------------------------------------------------------------------
}
