// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrCapsule3Capsule3.h"
#include "Wm4DistSegment3Segment3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrCapsule3Capsule3<Real>::IntrCapsule3Capsule3 (
    const Capsule3<Real>& rkCapsule0, const Capsule3<Real>& rkCapsule1)
    :
    m_pkCapsule0(&rkCapsule0),
    m_pkCapsule1(&rkCapsule1)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Capsule3<Real>& IntrCapsule3Capsule3<Real>::GetCapsule0 () const
{
    return *m_pkCapsule0;
}
//----------------------------------------------------------------------------
template <class Real>
const Capsule3<Real>& IntrCapsule3Capsule3<Real>::GetCapsule1 () const
{
    return *m_pkCapsule1;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrCapsule3Capsule3<Real>::Test ()
{
    Real fDistance = DistSegment3Segment3<Real>(m_pkCapsule0->Segment,
        m_pkCapsule1->Segment).Get();
    Real fRSum = m_pkCapsule0->Radius + m_pkCapsule1->Radius;
    return fDistance <= fRSum;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrCapsule3Capsule3<float>;

template WM4_FOUNDATION_ITEM
class IntrCapsule3Capsule3<double>;
//----------------------------------------------------------------------------
}
