// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrSegment3Sphere3.h"
#include "Wm4IntrSegment3Capsule3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrSegment3Sphere3<Real>::IntrSegment3Sphere3 (
    const Segment3<Real>& rkSegment, const Sphere3<Real>& rkSphere)
    :
    m_pkSegment(&rkSegment),
    m_pkSphere(&rkSphere)
{
    m_iQuantity = 0;
    ZeroThreshold = Math<Real>::ZERO_TOLERANCE;
}
//----------------------------------------------------------------------------
template <class Real>
const Segment3<Real>& IntrSegment3Sphere3<Real>::GetSegment () const
{
    return *m_pkSegment;
}
//----------------------------------------------------------------------------
template <class Real>
const Sphere3<Real>& IntrSegment3Sphere3<Real>::GetSphere () const
{
    return *m_pkSphere;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSegment3Sphere3<Real>::Test ()
{
    Vector3<Real> kDiff = m_pkSegment->Origin - m_pkSphere->Center;
    Real fA0 = kDiff.Dot(kDiff) - m_pkSphere->Radius*m_pkSphere->Radius;
    Real fA1 = m_pkSegment->Direction.Dot(kDiff);
    Real fDiscr = fA1*fA1 - fA0;
    if (fDiscr < (Real)0.0)
    {
        return false;
    }

    Real fTmp0 = m_pkSegment->Extent*m_pkSegment->Extent + fA0;
    Real fTmp1 = ((Real)2.0)*fA1*m_pkSegment->Extent;
    Real fQM = fTmp0 - fTmp1;
    Real fQP = fTmp0 + fTmp1;
    if (fQM*fQP <= (Real)0.0)
    {
        return true;
    }

    return fQM > (Real)0.0 && Math<Real>::FAbs(fA1) < m_pkSegment->Extent;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSegment3Sphere3<Real>::Find ()
{
    Vector3<Real> kDiff = m_pkSegment->Origin - m_pkSphere->Center;
    Real fA0 = kDiff.Dot(kDiff) - m_pkSphere->Radius*m_pkSphere->Radius;
    Real fA1 = m_pkSegment->Direction.Dot(kDiff);
    Real fDiscr = fA1*fA1 - fA0;
    if (fDiscr < (Real)0.0)
    {
        m_iQuantity = 0;
        m_iIntersectionType = IT_EMPTY;
        return false;
    }

    Real fTmp0 = m_pkSegment->Extent*m_pkSegment->Extent + fA0;
    Real fTmp1 = ((Real)2.0)*fA1*m_pkSegment->Extent;
    Real fQM = fTmp0 - fTmp1;
    Real fQP = fTmp0 + fTmp1;
    Real fRoot;
    if (fQM*fQP <= (Real)0.0)
    {
        fRoot = Math<Real>::Sqrt(fDiscr);
        m_afSegmentT[0] = (fQM > (Real)0.0 ? -fA1 - fRoot : -fA1 + fRoot);
        m_akPoint[0] = m_pkSegment->Origin + m_afSegmentT[0] *
            m_pkSegment->Direction;
        m_iQuantity = 1;
        m_iIntersectionType = IT_POINT;
        return true;
    }

    if (fQM > (Real)0.0 && Math<Real>::FAbs(fA1) < m_pkSegment->Extent)
    {
        if (fDiscr >= ZeroThreshold)
        {
            fRoot = Math<Real>::Sqrt(fDiscr);
            m_afSegmentT[0] = -fA1 - fRoot;
            m_afSegmentT[1] = -fA1 + fRoot;
            m_akPoint[0] = m_pkSegment->Origin + m_afSegmentT[0] *
                m_pkSegment->Direction;
            m_akPoint[1] = m_pkSegment->Origin + m_afSegmentT[1] *
                m_pkSegment->Direction;
            m_iQuantity = 2;
            m_iIntersectionType = IT_SEGMENT;
        }
        else
        {
            m_afSegmentT[0] = -fA1;
            m_akPoint[0] = m_pkSegment->Origin + m_afSegmentT[0] *
                m_pkSegment->Direction;
            m_iQuantity = 1;
            m_iIntersectionType = IT_POINT;
        }
    }
    else
    {
        m_iQuantity = 0;
        m_iIntersectionType = IT_EMPTY;
    }

    return m_iQuantity > 0;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSegment3Sphere3<Real>::Test (Real fTMax,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    // check if initially intersecting
    if (Test())
    {
        return true;
    }

    // Substract the segment velocity from the sphere velocity so that
    // the calculations are based in the coordinate system of the segment.
    // In this system, the line is of course stationary.  The sphere spans
    // a capsule, but instead we will "grow" the segment by the sphere radius
    // and shrink the sphere to its center.  The problem is now to detect
    // the first time the moving center intersects the capsule formed by
    // the line segment and sphere radius.

    Capsule3<Real> kCapsule;
    kCapsule.Segment = *m_pkSegment;
    kCapsule.Radius = m_pkSphere->Radius;

    Vector3<Real> kRelativeVelocity = rkVelocity1 - rkVelocity0;
    Real fRelativeSpeed = kRelativeVelocity.Normalize();
    Segment3<Real> kPath;
    kPath.Extent = ((Real)0.5)*fTMax*fRelativeSpeed;
    kPath.Direction = kRelativeVelocity;  // unit-length vector
    kPath.Origin = m_pkSphere->Center + kPath.Extent*kPath.Direction;

    return IntrSegment3Capsule3<Real>(kPath,kCapsule).Test();
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSegment3Sphere3<Real>::Find (Real fTMax,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    // check if initially intersecting
    if (Find())
    {
        m_fContactTime = (Real)0.0;
        m_iIntersectionType = IT_OTHER;
        return true;
    }

    // Substract the segment velocity from the sphere velocity so that
    // the calculations are based in the coordinate system of the segment.
    // In this system, the line is of course stationary.  The sphere spans
    // a capsule, but instead we will "grow" the segment by the sphere radius
    // and shrink the sphere to its center.  The problem is now to detect
    // the first time the moving center intersects the capsule formed by
    // the line segment and sphere radius.

    Capsule3<Real> kCapsule;
    kCapsule.Segment = *m_pkSegment;
    kCapsule.Radius = m_pkSphere->Radius;

    Vector3<Real> kRelativeVelocity = rkVelocity1 - rkVelocity0;
    Real fRelativeSpeed = kRelativeVelocity.Normalize();
    Segment3<Real> kPath;
    kPath.Extent = ((Real)0.5)*fTMax*fRelativeSpeed;
    kPath.Direction = kRelativeVelocity;  // unit-length vector
    kPath.Origin = m_pkSphere->Center + kPath.Extent*kPath.Direction;

    IntrSegment3Capsule3<Real> kIntr(kPath,kCapsule);
    if (!kIntr.Find())
    {
        m_iIntersectionType = IT_EMPTY;
        return false;
    }

    // We now know the sphere will intersect the segment.  This can happen
    // either at a segment end point or at a segment interior point.  We
    // need to determine which.
    m_fContactTime = (kIntr.GetParameter(0) + kPath.Extent)/fRelativeSpeed;
    m_iQuantity = 1;

    Vector3<Real> kMCenter = m_pkSphere->Center + 
        m_fContactTime*rkVelocity1;

    Vector3<Real> kMOrigin = m_pkSegment->Origin +
        m_fContactTime*rkVelocity0;

    Real fOrigin = m_pkSegment->Direction.Dot(kMOrigin);
    Real fNegEnd = fOrigin - m_pkSegment->Extent;
    Real fPosEnd = fOrigin + m_pkSegment->Extent;
    Real fCenter = m_pkSegment->Direction.Dot(kMCenter);

    if (fCenter < fNegEnd)
    {
        // intersection at segment end point P-e*D
        m_akPoint[0] = kMOrigin - m_pkSegment->Extent*m_pkSegment->Direction;
    }
    else if (fCenter > fPosEnd)
    {
        // intersection at segment end point P+e*D
        m_akPoint[0] = kMOrigin + m_pkSegment->Extent*m_pkSegment->Direction;
    }
    else
    {
        // Intersection with interior point on edge.  Use the projection
        // along direction axis to find which point that is.
        m_akPoint[0] = kMOrigin + (fCenter - fOrigin)*m_pkSegment->Direction;
    }

    return true;
}
//----------------------------------------------------------------------------
template <class Real>
int IntrSegment3Sphere3<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& IntrSegment3Sphere3<Real>::GetPoint (int i) const
{
    assert(0 <= i && i < m_iQuantity);
    return m_akPoint[i];
}
//----------------------------------------------------------------------------
template <class Real>
Real IntrSegment3Sphere3<Real>::GetSegmentT (int i) const
{
    assert(0 <= i && i < m_iQuantity);
    return m_afSegmentT[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrSegment3Sphere3<float>;

template WM4_FOUNDATION_ITEM
class IntrSegment3Sphere3<double>;
//----------------------------------------------------------------------------
}
