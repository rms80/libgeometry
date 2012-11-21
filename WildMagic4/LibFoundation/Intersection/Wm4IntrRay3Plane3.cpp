// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrRay3Plane3.h"
#include "Wm4IntrLine3Plane3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrRay3Plane3<Real>::IntrRay3Plane3 (const Ray3<Real>& rkRay,
    const Plane3<Real>& rkPlane)
    :
    m_pkRay(&rkRay),
    m_pkPlane(&rkPlane)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Ray3<Real>& IntrRay3Plane3<Real>::GetRay () const
{
    return *m_pkRay;
}
//----------------------------------------------------------------------------
template <class Real>
const Plane3<Real>& IntrRay3Plane3<Real>::GetPlane () const
{
    return *m_pkPlane;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrRay3Plane3<Real>::Test ()
{
    return Find();
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrRay3Plane3<Real>::Find ()
{
    Line3<Real> kLine(m_pkRay->Origin,m_pkRay->Direction);
    IntrLine3Plane3<Real> kIntr(kLine,*m_pkPlane);
    if (kIntr.Find())
    {
        // The line intersects the plane, but possibly at a point that is
        // not on the ray.
        m_iIntersectionType = kIntr.GetIntersectionType();
        m_fRayT = kIntr.GetLineT();
        return m_fRayT >= (Real)0.0;
    }

    m_iIntersectionType = IT_EMPTY;
    return false;
}
//----------------------------------------------------------------------------
template <class Real>
Real IntrRay3Plane3<Real>::GetRayT () const
{
    return m_fRayT;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrRay3Plane3<float>;

template WM4_FOUNDATION_ITEM
class IntrRay3Plane3<double>;
//----------------------------------------------------------------------------
}
