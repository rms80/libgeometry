// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4EllipsoidGeodesic.h"
#include "Wm4Math.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
EllipsoidGeodesic<Real>::EllipsoidGeodesic (Real fXExtent, Real fYExtent,
    Real fZExtent)
    :
    RiemannianGeodesic<Real>(2)
{
    m_fXExtent = fXExtent;
    m_fYExtent = fYExtent;
    m_fZExtent = fZExtent;
}
//----------------------------------------------------------------------------
template <class Real>
EllipsoidGeodesic<Real>::~EllipsoidGeodesic ()
{
}
//----------------------------------------------------------------------------
template <class Real>
Vector3<Real> EllipsoidGeodesic<Real>::ComputePosition (
    const GVector<Real>& rkPoint)
{
    Real fCos0 = Math<Real>::Cos(rkPoint[0]);
    Real fSin0 = Math<Real>::Sin(rkPoint[0]);
    Real fCos1 = Math<Real>::Cos(rkPoint[1]);
    Real fSin1 = Math<Real>::Sin(rkPoint[1]);

    return Vector3<Real>(
        m_fXExtent*fCos0*fSin1,
        m_fYExtent*fSin0*fSin1,
        m_fZExtent*fCos1
    );
}
//----------------------------------------------------------------------------
template <class Real>
void EllipsoidGeodesic<Real>::ComputeMetric (const GVector<Real>& rkPoint)
{
    Real fCos0 = Math<Real>::Cos(rkPoint[0]);
    Real fSin0 = Math<Real>::Sin(rkPoint[0]);
    Real fCos1 = Math<Real>::Cos(rkPoint[1]);
    Real fSin1 = Math<Real>::Sin(rkPoint[1]);

    Vector3<Real> kDer0(
        -m_fXExtent*fSin0*fSin1,
        m_fYExtent*fCos0*fSin1,
        (Real)0.0);

    Vector3<Real> kDer1(
        m_fXExtent*fCos0*fCos1,
        m_fYExtent*fSin0*fCos1,
        -m_fZExtent*fSin1);

    m_kMetric[0][0] = kDer0.Dot(kDer0);
    m_kMetric[0][1] = kDer0.Dot(kDer1);
    m_kMetric[1][0] = m_kMetric[0][1];
    m_kMetric[1][1] = kDer1.Dot(kDer1);
}
//----------------------------------------------------------------------------
template <class Real>
void EllipsoidGeodesic<Real>::ComputeChristoffel1 (
    const GVector<Real>& rkPoint)
{
    Real fCos0 = Math<Real>::Cos(rkPoint[0]);
    Real fSin0 = Math<Real>::Sin(rkPoint[0]);
    Real fCos1 = Math<Real>::Cos(rkPoint[1]);
    Real fSin1 = Math<Real>::Sin(rkPoint[1]);

    Vector3<Real> kDer0(
        -m_fXExtent*fSin0*fSin1,
        m_fYExtent*fCos0*fSin1,
        (Real)0.0);

    Vector3<Real> kDer1(
        m_fXExtent*fCos0*fCos1,
        m_fYExtent*fSin0*fCos1,
        -m_fZExtent*fSin1);

    Vector3<Real> kDer00(
        -m_fXExtent*fCos0*fSin1,
        -m_fYExtent*fSin0*fSin1,
        (Real)0.0);

    Vector3<Real> kDer01(
        -m_fXExtent*fSin0*fCos1,
        m_fYExtent*fCos0*fCos1,
        (Real)0.0);

    Vector3<Real> kDer11(
        -m_fXExtent*fCos0*fSin1,
        -m_fYExtent*fSin0*fSin1,
        -m_fZExtent*fCos1);

    m_akChristoffel1[0][0][0] = kDer00.Dot(kDer0);
    m_akChristoffel1[0][0][1] = kDer01.Dot(kDer0);
    m_akChristoffel1[0][1][0] = m_akChristoffel1[0][0][1];
    m_akChristoffel1[0][1][1] = kDer11.Dot(kDer0);

    m_akChristoffel1[1][0][0] = kDer00.Dot(kDer1);
    m_akChristoffel1[1][0][1] = kDer01.Dot(kDer1);
    m_akChristoffel1[1][1][0] = m_akChristoffel1[1][0][1];
    m_akChristoffel1[1][1][1] = kDer11.Dot(kDer1);
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class EllipsoidGeodesic<float>;

template WM4_FOUNDATION_ITEM
class EllipsoidGeodesic<double>;
}
//----------------------------------------------------------------------------
