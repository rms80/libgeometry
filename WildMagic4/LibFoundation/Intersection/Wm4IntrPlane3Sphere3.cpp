// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrPlane3Sphere3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrPlane3Sphere3<Real>::IntrPlane3Sphere3 (const Plane3<Real>& rkPlane,
    const Sphere3<Real>& rkSphere)
    :
    m_pkPlane(&rkPlane),
    m_pkSphere(&rkSphere)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Plane3<Real>& IntrPlane3Sphere3<Real>::GetPlane () const
{
    return *m_pkPlane;
}
//----------------------------------------------------------------------------
template <class Real>
const Sphere3<Real>& IntrPlane3Sphere3<Real>::GetSphere () const
{
    return *m_pkSphere;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrPlane3Sphere3<Real>::Test ()
{
    Real fSignedDistance = m_pkPlane->DistanceTo(m_pkSphere->Center);
    return Math<Real>::FAbs(fSignedDistance) <= m_pkSphere->Radius;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrPlane3Sphere3<Real>::Find ()
{
    Real fSignedDistance = m_pkPlane->DistanceTo(m_pkSphere->Center);
    Real fDistance = Math<Real>::FAbs(fSignedDistance);
    m_kCircle.Center = m_pkSphere->Center - fSignedDistance*m_pkPlane->Normal;
    m_kCircle.N = m_pkPlane->Normal;
    if (fDistance <= m_pkSphere->Radius)
    {
        // The sphere intersects the plane in a circle.  The circle is
        // degenerate when fDistance is equal to m_pkSphere->Radius, in which
        // case the circle radius is zero.
        m_kCircle.Radius = Math<Real>::Sqrt(Math<Real>::FAbs(
            m_pkSphere->Radius*m_pkSphere->Radius - fDistance*fDistance));
        return true;
    }

    // Additional indication that the circle is invalid.
    m_kCircle.Radius = (Real)-1;
    return false;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrPlane3Sphere3<Real>::SphereIsCulled () const
{
    Real fSignedDistance = m_pkPlane->DistanceTo(m_pkSphere->Center);
    return fSignedDistance <= -m_pkSphere->Radius;
}
//----------------------------------------------------------------------------
template <class Real>
const Circle3<Real>& IntrPlane3Sphere3<Real>::GetCircle () const
{
    return m_kCircle;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrPlane3Sphere3<float>;

template WM4_FOUNDATION_ITEM
class IntrPlane3Sphere3<double>;
//----------------------------------------------------------------------------
}
