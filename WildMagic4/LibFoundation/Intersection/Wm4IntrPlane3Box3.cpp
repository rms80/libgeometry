// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrPlane3Box3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrPlane3Box3<Real>::IntrPlane3Box3 (const Plane3<Real>& rkPlane,
    const Box3<Real>& rkBox)
    :
    m_pkPlane(&rkPlane),
    m_pkBox(&rkBox)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Plane3<Real>& IntrPlane3Box3<Real>::GetPlane () const
{
    return *m_pkPlane;
}
//----------------------------------------------------------------------------
template <class Real>
const Box3<Real>& IntrPlane3Box3<Real>::GetBox () const
{
    return *m_pkBox;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrPlane3Box3<Real>::Test ()
{
    Real afTmp[3] =
    {
        m_pkBox->Extent[0]*(m_pkPlane->Normal.Dot(m_pkBox->Axis[0])),
        m_pkBox->Extent[1]*(m_pkPlane->Normal.Dot(m_pkBox->Axis[1])),
        m_pkBox->Extent[2]*(m_pkPlane->Normal.Dot(m_pkBox->Axis[2]))
    };

    Real fRadius = Math<Real>::FAbs(afTmp[0]) + Math<Real>::FAbs(afTmp[1]) +
        Math<Real>::FAbs(afTmp[2]);

    Real fSignedDistance = m_pkPlane->DistanceTo(m_pkBox->Center);
    return Math<Real>::FAbs(fSignedDistance) <= fRadius;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrPlane3Box3<Real>::BoxIsCulled () const
{
    Real afTmp[3] =
    {
        m_pkBox->Extent[0]*(m_pkPlane->Normal.Dot(m_pkBox->Axis[0])),
        m_pkBox->Extent[1]*(m_pkPlane->Normal.Dot(m_pkBox->Axis[1])),
        m_pkBox->Extent[2]*(m_pkPlane->Normal.Dot(m_pkBox->Axis[2]))
    };

    Real fRadius = Math<Real>::FAbs(afTmp[0]) + Math<Real>::FAbs(afTmp[1]) +
        Math<Real>::FAbs(afTmp[2]);

    Real fSignedDistance = m_pkPlane->DistanceTo(m_pkBox->Center);
    return fSignedDistance <= -fRadius;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrPlane3Box3<float>;

template WM4_FOUNDATION_ITEM
class IntrPlane3Box3<double>;
//----------------------------------------------------------------------------
}
