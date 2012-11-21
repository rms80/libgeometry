// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistLine2Ray2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistLine2Ray2<Real>::DistLine2Ray2 (const Line2<Real>& rkLine,
    const Ray2<Real>& rkRay)
    :
    m_pkLine(&rkLine),
    m_pkRay(&rkRay)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Line2<Real>& DistLine2Ray2<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
const Ray2<Real>& DistLine2Ray2<Real>::GetRay () const
{
    return *m_pkRay;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine2Ray2<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine2Ray2<Real>::GetSquared ()
{
    Vector2<Real> kDiff = m_pkLine->Origin - m_pkRay->Origin;
    Real fA01 = -m_pkLine->Direction.Dot(m_pkRay->Direction);
    Real fB0 = kDiff.Dot(m_pkLine->Direction);
    Real fC = kDiff.SquaredLength();
    Real fDet = Math<Real>::FAbs((Real)1.0 - fA01*fA01);
    Real fB1, fS0, fS1, fSqrDist;

    if (fDet >= Math<Real>::ZERO_TOLERANCE)
    {
        fB1 = -kDiff.Dot(m_pkRay->Direction);
        fS1 = fA01*fB0-fB1;

        if (fS1 >= (Real)0.0)
        {
            // two interior points are closest, one on line and one on ray
            Real fInvDet = ((Real)1.0)/fDet;
            fS0 = (fA01*fB1-fB0)*fInvDet;
            fS1 *= fInvDet;
            fSqrDist = (Real)0.0;
        }
        else
        {
            // end point of ray and interior point of line are closest
            fS0 = -fB0;
            fS1 = (Real)0.0;
            fSqrDist = fB0*fS0+fC;
        }
    }
    else
    {
        // lines are parallel, closest pair with one point at ray origin
        fS0 = -fB0;
        fS1 = (Real)0.0;
        fSqrDist = fB0*fS0+fC;
    }

    m_kClosestPoint0 = m_pkLine->Origin + fS0*m_pkLine->Direction;
    m_kClosestPoint1 = m_pkRay->Origin + fS1*m_pkRay->Direction;
    return Math<Real>::FAbs(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine2Ray2<Real>::Get (Real fT, const Vector2<Real>& rkVelocity0,
    const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMOrigin0 = m_pkLine->Origin + fT*rkVelocity0;
    Vector2<Real> kMOrigin1 = m_pkRay->Origin + fT*rkVelocity1;
    Line2<Real> kMLine(kMOrigin0,m_pkLine->Direction);
    Ray2<Real> kMRay(kMOrigin1,m_pkRay->Direction);
    return DistLine2Ray2<Real>(kMLine,kMRay).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine2Ray2<Real>::GetSquared (Real fT,
    const Vector2<Real>& rkVelocity0, const Vector2<Real>& rkVelocity1)
{
    Vector2<Real> kMOrigin0 = m_pkLine->Origin + fT*rkVelocity0;
    Vector2<Real> kMOrigin1 = m_pkRay->Origin + fT*rkVelocity1;
    Line2<Real> kMLine(kMOrigin0,m_pkLine->Direction);
    Ray2<Real> kMRay(kMOrigin1,m_pkRay->Direction);
    return DistLine2Ray2<Real>(kMLine,kMRay).GetSquared();
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistLine2Ray2<float>;

template WM4_FOUNDATION_ITEM
class DistLine2Ray2<double>;
//----------------------------------------------------------------------------
}
