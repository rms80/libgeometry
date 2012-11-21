// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistLine3Line3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistLine3Line3<Real>::DistLine3Line3 (const Line3<Real>& rkLine0,
    const Line3<Real>& rkLine1)
    :
    m_pkLine0(&rkLine0),
    m_pkLine1(&rkLine1)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Line3<Real>& DistLine3Line3<Real>::GetLine0 () const
{
    return *m_pkLine0;
}
//----------------------------------------------------------------------------
template <class Real>
const Line3<Real>& DistLine3Line3<Real>::GetLine1 () const
{
    return *m_pkLine1;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine3Line3<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine3Line3<Real>::GetSquared ()
{
    Vector3<Real> kDiff = m_pkLine0->Origin - m_pkLine1->Origin;
    Real fA01 = -m_pkLine0->Direction.Dot(m_pkLine1->Direction);
    Real fB0 = kDiff.Dot(m_pkLine0->Direction);
    Real fC = kDiff.SquaredLength();
    Real fDet = Math<Real>::FAbs((Real)1.0 - fA01*fA01);
    Real fB1, fS0, fS1, fSqrDist;

    if (fDet >= Math<Real>::ZERO_TOLERANCE)
    {
        // lines are not parallel
        fB1 = -kDiff.Dot(m_pkLine1->Direction);
        Real fInvDet = ((Real)1.0)/fDet;
        fS0 = (fA01*fB1-fB0)*fInvDet;
        fS1 = (fA01*fB0-fB1)*fInvDet;
        fSqrDist = fS0*(fS0+fA01*fS1+((Real)2.0)*fB0) +
            fS1*(fA01*fS0+fS1+((Real)2.0)*fB1)+fC;
    }
    else
    {
        // lines are parallel, select any closest pair of points
        fS0 = -fB0;
        fS1 = (Real)0.0;
        fSqrDist = fB0*fS0+fC;
    }

    m_kClosestPoint0 = m_pkLine0->Origin + fS0*m_pkLine0->Direction;
    m_kClosestPoint1 = m_pkLine1->Origin + fS1*m_pkLine1->Direction;
    m_fLine0Parameter = fS0;
    m_fLine1Parameter = fS1;
    return Math<Real>::FAbs(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine3Line3<Real>::Get (Real fT, const Vector3<Real>& rkVelocity0,
    const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMOrigin0 = m_pkLine0->Origin + fT*rkVelocity0;
    Vector3<Real> kMOrigin1 = m_pkLine1->Origin + fT*rkVelocity1;
    Line3<Real> kMLine0(kMOrigin0,m_pkLine0->Direction);
    Line3<Real> kMLine1(kMOrigin1,m_pkLine1->Direction);
    return DistLine3Line3<Real>(kMLine0,kMLine1).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine3Line3<Real>::GetSquared (Real fT,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMOrigin0 = m_pkLine0->Origin + fT*rkVelocity0;
    Vector3<Real> kMOrigin1 = m_pkLine1->Origin + fT*rkVelocity1;
    Line3<Real> kMLine0(kMOrigin0,m_pkLine0->Direction);
    Line3<Real> kMLine1(kMOrigin1,m_pkLine1->Direction);
    return DistLine3Line3<Real>(kMLine0,kMLine1).GetSquared();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine3Line3<Real>::GetLine0Parameter () const
{
    return m_fLine0Parameter;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistLine3Line3<Real>::GetLine1Parameter () const
{
    return m_fLine1Parameter;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistLine3Line3<float>;

template WM4_FOUNDATION_ITEM
class DistLine3Line3<double>;
//----------------------------------------------------------------------------
}
