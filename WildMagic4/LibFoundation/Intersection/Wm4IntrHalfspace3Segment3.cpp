// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrHalfspace3Segment3.h"
#include "Wm4IntrUtility3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrHalfspace3Segment3<Real>::IntrHalfspace3Segment3 (
    const Plane3<Real>& rkHalfspace, const Segment3<Real>& rkSegment)
    :
    m_pkHalfspace(&rkHalfspace),
    m_pkSegment(&rkSegment)
{
    m_iQuantity = 0;
}
//----------------------------------------------------------------------------
template <class Real>
const Plane3<Real>& IntrHalfspace3Segment3<Real>::GetHalfspace () const
{
    return *m_pkHalfspace;
}
//----------------------------------------------------------------------------
template <class Real>
const Segment3<Real>& IntrHalfspace3Segment3<Real>::GetSegment () const
{
    return *m_pkSegment;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Segment3<Real>::Test ()
{
    Vector3<Real> akSegment[2] =
    {
        m_pkSegment->GetNegEnd(),
        m_pkSegment->GetPosEnd(),
    };

    Real fMin, fMax;
    IntrAxis<Real>::GetProjection(m_pkHalfspace->Normal,akSegment,fMin,fMax);
    return fMin <= m_pkHalfspace->Constant;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Segment3<Real>::Find ()
{
    // Start with the segment and clip it against the plane.
    m_iQuantity = 2;
    m_akPoint[0] = m_pkSegment->GetNegEnd();
    m_akPoint[1] = m_pkSegment->GetPosEnd();

    ClipConvexPolygonAgainstPlane<Real>(-m_pkHalfspace->Normal,
        -m_pkHalfspace->Constant,m_iQuantity,m_akPoint);
    
    return m_iQuantity > 0;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Segment3<Real>::Test (Real fTMax,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    m_fContactTime = (Real)0;
    Real fTLast = Math<Real>::MAX_REAL;
    Vector3<Real> kVelocity = rkVelocity1 - rkVelocity0;

    Vector3<Real> akSegment[2] =
    {
        m_pkSegment->GetNegEnd(),
        m_pkSegment->GetPosEnd(),
    };

    Real fMin, fMax;
    IntrAxis<Real>::GetProjection(m_pkHalfspace->Normal,akSegment,fMin,fMax);

    return IntrAxis<Real>::Test(m_pkHalfspace->Normal,kVelocity,
        -Math<Real>::MAX_REAL,m_pkHalfspace->Constant,fMin,fMax,fTMax,
        m_fContactTime,fTLast);
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Segment3<Real>::Find (Real fTMax,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    m_fContactTime = (Real)0;
    Real fTLast = Math<Real>::MAX_REAL;
    Vector3<Real> kVelocity = rkVelocity1 - rkVelocity0;

    Vector3<Real> akSegment[2] =
    {
        m_pkSegment->GetNegEnd(),
        m_pkSegment->GetPosEnd(),
    };

    IntrConfiguration<Real> kCfg;
    IntrAxis<Real>::GetConfiguration(m_pkHalfspace->Normal,akSegment,kCfg);

    if (!IntrAxis<Real>::Test(m_pkHalfspace->Normal,kVelocity,
        -Math<Real>::MAX_REAL,m_pkHalfspace->Constant,kCfg.Min,kCfg.Max,
        fTMax,m_fContactTime,fTLast))
    {
        // never intersecting
        return false;
    }

    if (m_fContactTime == (Real)0)
    {
        // intersecting now
        return false;
    }

    // line on positive side (right)
    if (kCfg.Map == IntrConfiguration<Real>::m11)
    {
        m_iQuantity = 1;
        m_akPoint[0] = akSegment[kCfg.Index[0]];
    }
    else // kCfg.Map == IntrConfiguration<Real>::m2
    {
        m_iQuantity = 2;
        m_akPoint[0] = akSegment[kCfg.Index[0]];
        m_akPoint[1] = akSegment[kCfg.Index[1]];
    }

    // adjust points to the correct place in time, as well
    Vector3<Real> kDiff = m_fContactTime*rkVelocity1;
    for (int i = 0; i < m_iQuantity; i++)
    {
        m_akPoint[i] += kDiff;
    }

    return true;
}
//----------------------------------------------------------------------------
template <class Real>
int IntrHalfspace3Segment3<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& IntrHalfspace3Segment3<Real>::GetPoint (int i) const
{
    assert(0 <= i && i < m_iQuantity);
    return m_akPoint[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrHalfspace3Segment3<float>;

template WM4_FOUNDATION_ITEM
class IntrHalfspace3Segment3<double>;
//----------------------------------------------------------------------------
}
