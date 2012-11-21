// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistTriangle3Triangle3.h"
#include "Wm4DistSegment3Triangle3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistTriangle3Triangle3<Real>::DistTriangle3Triangle3 (
    const Triangle3<Real>& rkTriangle0, const Triangle3<Real>& rkTriangle1)
    :
    m_pkTriangle0(&rkTriangle0),
    m_pkTriangle1(&rkTriangle1)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Triangle3<Real>& DistTriangle3Triangle3<Real>::GetTriangle0 () const
{
    return *m_pkTriangle0;
}
//----------------------------------------------------------------------------
template <class Real>
const Triangle3<Real>& DistTriangle3Triangle3<Real>::GetTriangle1 () const
{
    return *m_pkTriangle1;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistTriangle3Triangle3<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistTriangle3Triangle3<Real>::GetSquared ()
{
    // compare edges of triangle0 to the interior of triangle1
    Real fSqrDist = Math<Real>::MAX_REAL, fSqrDistTmp;
    Segment3<Real> kSeg;
    Real fRatio;
    int i0, i1;
    for (i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
    {
        kSeg.Origin = ((Real)0.5)*(m_pkTriangle0->V[i0]+m_pkTriangle0->V[i1]);
        kSeg.Direction = m_pkTriangle0->V[i1] - m_pkTriangle0->V[i0];
        kSeg.Extent = ((Real)0.5)*kSeg.Direction.Normalize();
        DistSegment3Triangle3<Real> kSTDist(kSeg,*m_pkTriangle1);
        fSqrDistTmp = kSTDist.GetSquared();
        if (fSqrDistTmp < fSqrDist)
        {
            m_kClosestPoint0 = kSTDist.GetClosestPoint0();
            m_kClosestPoint1 = kSTDist.GetClosestPoint1();
            fSqrDist = fSqrDistTmp;

            fRatio = kSTDist.GetSegmentParameter()/kSeg.Extent;
            m_afTriangleBary0[i0] = ((Real)0.5)*((Real)1.0 - fRatio);
            m_afTriangleBary0[i1] = (Real)1.0 - m_afTriangleBary0[i0];
            m_afTriangleBary0[3-i0-i1] = (Real)0.0;
            m_afTriangleBary1[0] = kSTDist.GetTriangleBary(0);
            m_afTriangleBary1[1] = kSTDist.GetTriangleBary(1);
            m_afTriangleBary1[2] = kSTDist.GetTriangleBary(2);

            if (fSqrDist <= Math<Real>::ZERO_TOLERANCE)
            {
                return (Real)0.0;
            }
        }
    }

    // compare edges of triangle1 to the interior of triangle0
    for (i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
    {
        kSeg.Origin = ((Real)0.5)*(m_pkTriangle1->V[i0]+m_pkTriangle1->V[i1]);
        kSeg.Direction = m_pkTriangle1->V[i1] - m_pkTriangle1->V[i0];
        kSeg.Extent = ((Real)0.5)*kSeg.Direction.Normalize();
        DistSegment3Triangle3<Real> kSTDist(kSeg,*m_pkTriangle0);
        fSqrDistTmp = kSTDist.GetSquared();
        if (fSqrDistTmp < fSqrDist)
        {
            m_kClosestPoint0 = kSTDist.GetClosestPoint0();
            m_kClosestPoint1 = kSTDist.GetClosestPoint1();
            fSqrDist = fSqrDistTmp;

            fRatio = kSTDist.GetSegmentParameter()/kSeg.Extent;
            m_afTriangleBary1[i0] = ((Real)0.5)*((Real)1.0 - fRatio);
            m_afTriangleBary1[i1] = (Real)1.0 - m_afTriangleBary1[i0];
            m_afTriangleBary1[3-i0-i1] = (Real)0.0;
            m_afTriangleBary0[0] = kSTDist.GetTriangleBary(0);
            m_afTriangleBary0[1] = kSTDist.GetTriangleBary(1);
            m_afTriangleBary0[2] = kSTDist.GetTriangleBary(2);

            if (fSqrDist <= Math<Real>::ZERO_TOLERANCE)
            {
                return (Real)0.0;
            }
        }
    }

    return fSqrDist;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistTriangle3Triangle3<Real>::Get (Real fT,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMV00 = m_pkTriangle0->V[0] + fT*rkVelocity0;
    Vector3<Real> kMV01 = m_pkTriangle0->V[1] + fT*rkVelocity0;
    Vector3<Real> kMV02 = m_pkTriangle0->V[2] + fT*rkVelocity0;
    Vector3<Real> kMV10 = m_pkTriangle1->V[0] + fT*rkVelocity1;
    Vector3<Real> kMV11 = m_pkTriangle1->V[1] + fT*rkVelocity1;
    Vector3<Real> kMV12 = m_pkTriangle1->V[2] + fT*rkVelocity1;
    Triangle3<Real> kMTriangle0(kMV00,kMV01,kMV02);
    Triangle3<Real> kMTriangle1(kMV10,kMV11,kMV12);
    return DistTriangle3Triangle3<Real>(kMTriangle0,kMTriangle1).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistTriangle3Triangle3<Real>::GetSquared (Real fT,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMV00 = m_pkTriangle0->V[0] + fT*rkVelocity0;
    Vector3<Real> kMV01 = m_pkTriangle0->V[1] + fT*rkVelocity0;
    Vector3<Real> kMV02 = m_pkTriangle0->V[2] + fT*rkVelocity0;
    Vector3<Real> kMV10 = m_pkTriangle1->V[0] + fT*rkVelocity1;
    Vector3<Real> kMV11 = m_pkTriangle1->V[1] + fT*rkVelocity1;
    Vector3<Real> kMV12 = m_pkTriangle1->V[2] + fT*rkVelocity1;
    Triangle3<Real> kMTriangle0(kMV00,kMV01,kMV02);
    Triangle3<Real> kMTriangle1(kMV10,kMV11,kMV12);
    return DistTriangle3Triangle3<Real>(kMTriangle0,kMTriangle1).GetSquared();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistTriangle3Triangle3<Real>::GetTriangleBary0 (int i) const
{
    assert(0 <= i && i < 3);
    return m_afTriangleBary0[i];
}
//----------------------------------------------------------------------------
template <class Real>
Real DistTriangle3Triangle3<Real>::GetTriangleBary1 (int i) const
{
    assert(0 <= i && i < 3);
    return m_afTriangleBary1[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistTriangle3Triangle3<float>;

template WM4_FOUNDATION_ITEM
class DistTriangle3Triangle3<double>;
//----------------------------------------------------------------------------
}
