// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrHalfspace3Triangle3.h"
#include "Wm4IntrUtility3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrHalfspace3Triangle3<Real>::IntrHalfspace3Triangle3 (
    const Plane3<Real>& rkHalfspace, const Triangle3<Real>& rkTriangle)
    :
    m_pkHalfspace(&rkHalfspace),
    m_pkTriangle(&rkTriangle)
{
    m_iQuantity = 0;
}
//----------------------------------------------------------------------------
template <class Real>
const Plane3<Real>& IntrHalfspace3Triangle3<Real>::GetHalfspace () const
{
    return *m_pkHalfspace;
}
//----------------------------------------------------------------------------
template <class Real>
const Triangle3<Real>& IntrHalfspace3Triangle3<Real>::GetTriangle () const
{
    return *m_pkTriangle;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Triangle3<Real>::Test ()
{
    Real fMin, fMax;
    IntrAxis<Real>::GetProjection(m_pkHalfspace->Normal,*m_pkTriangle,
        fMin,fMax);

    return fMin <= m_pkHalfspace->Constant;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Triangle3<Real>::Find ()
{
    // Start with the triangle and then clip it against the plane.
    m_iQuantity = 3;
    for (int i = 0; i < 3; i++)
    {
        m_akPoint[i] = m_pkTriangle->V[i];
    }

    ClipConvexPolygonAgainstPlane<Real>(-m_pkHalfspace->Normal,
        -m_pkHalfspace->Constant,m_iQuantity,m_akPoint);
    
    return m_iQuantity > 0;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Triangle3<Real>::Test (Real fTMax,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    m_fContactTime = (Real)0;
    Real fTLast = Math<Real>::MAX_REAL;
    Vector3<Real> kVelocity = rkVelocity1 - rkVelocity0;

    Real fMin, fMax;
    IntrAxis<Real>::GetProjection(m_pkHalfspace->Normal,*m_pkTriangle,
        fMin,fMax);

    return IntrAxis<Real>::Test(m_pkHalfspace->Normal,kVelocity,
        -Math<Real>::MAX_REAL,m_pkHalfspace->Constant,fMin,fMax,fTMax,
        m_fContactTime,fTLast);
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Triangle3<Real>::Find (Real fTMax,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    m_fContactTime = (Real)0;
    Real fTLast = Math<Real>::MAX_REAL;
    Vector3<Real> kVelocity = rkVelocity1 - rkVelocity0;

    IntrConfiguration<Real> kCfg;
    IntrAxis<Real>::GetConfiguration(m_pkHalfspace->Normal,*m_pkTriangle,
        kCfg);

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

    // tri on positive side (right)
    if (kCfg.Map == IntrConfiguration<Real>::m12
    ||  kCfg.Map == IntrConfiguration<Real>::m111)
    {
        // point
        m_iQuantity = 1;
        m_akPoint[0] = m_pkTriangle->V[kCfg.Index[0]];
    }
    else if (kCfg.Map == IntrConfiguration<Real>::m21)
    {
        // segment
        m_iQuantity = 2;
        m_akPoint[0] = m_pkTriangle->V[kCfg.Index[0]];
        m_akPoint[1] = m_pkTriangle->V[kCfg.Index[1]];
    }
    else
    {
        // face
        m_iQuantity = 3;
        for (int i = 0; i < 3; i++)
        {
            m_akPoint[i] = m_pkTriangle->V[i];
        }
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
int IntrHalfspace3Triangle3<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& IntrHalfspace3Triangle3<Real>::GetPoint (int i) const
{
    assert(0 <= i && i < m_iQuantity);
    return m_akPoint[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrHalfspace3Triangle3<float>;

template WM4_FOUNDATION_ITEM
class IntrHalfspace3Triangle3<double>;
//----------------------------------------------------------------------------
}
