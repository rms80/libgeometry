// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrHalfspace3Sphere3.h"
#include "Wm4IntrUtility3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrHalfspace3Sphere3<Real>::IntrHalfspace3Sphere3 (
    const Plane3<Real>& rkHalfspace, const Sphere3<Real>& rkSphere)
    :
    m_pkHalfspace(&rkHalfspace),
    m_pkSphere(&rkSphere)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Plane3<Real>& IntrHalfspace3Sphere3<Real>::GetHalfspace () const
{
    return *m_pkHalfspace;
}
//----------------------------------------------------------------------------
template <class Real>
const Sphere3<Real>& IntrHalfspace3Sphere3<Real>::GetSphere () const
{
    return *m_pkSphere;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Sphere3<Real>::Test ()
{
    Real fDistance = m_pkHalfspace->Normal.Dot(m_pkSphere->Center);
    return fDistance <= m_pkHalfspace->Constant + m_pkSphere->Radius;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Sphere3<Real>::Test (Real fTMax,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    m_fContactTime = (Real)0;
    Real fTLast = Math<Real>::MAX_REAL;
    Vector3<Real> kVelocity = rkVelocity1 - rkVelocity0;
    Real fDistance = m_pkHalfspace->Normal.Dot(m_pkSphere->Center);

    return IntrAxis<Real>::Test(m_pkHalfspace->Normal,kVelocity,
        -Math<Real>::MAX_REAL,m_pkHalfspace->Constant,
        fDistance-m_pkSphere->Radius, fDistance+m_pkSphere->Radius,fTMax,
        m_fContactTime,fTLast);
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrHalfspace3Sphere3<Real>::Find (Real fTMax,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    m_fContactTime = (Real)0;
    Real fTLast = Math<Real>::MAX_REAL;
    Vector3<Real> kVelocity = rkVelocity1 - rkVelocity0;
    Real fDistance = m_pkHalfspace->Normal.Dot(m_pkSphere->Center);

    if (!IntrAxis<Real>::Test(m_pkHalfspace->Normal,kVelocity,
        -Math<Real>::MAX_REAL,m_pkHalfspace->Constant,
        fDistance-m_pkSphere->Radius, fDistance+m_pkSphere->Radius,fTMax,
        m_fContactTime,fTLast))
    {
        // never intersecting
        return false;
    }

    if (m_fContactTime == (Real)0)
    {
        // intersecting now
        return false;
    }

    m_kPoint = m_pkSphere->Center + m_fContactTime*rkVelocity1 -
        m_pkSphere->Radius*m_pkHalfspace->Normal;

    return true;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& IntrHalfspace3Sphere3<Real>::GetPoint () const
{
    return m_kPoint;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrHalfspace3Sphere3<float>;

template WM4_FOUNDATION_ITEM
class IntrHalfspace3Sphere3<double>;
//----------------------------------------------------------------------------
}
