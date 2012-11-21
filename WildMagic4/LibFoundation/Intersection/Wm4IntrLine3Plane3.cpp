// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrLine3Plane3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrLine3Plane3<Real>::IntrLine3Plane3 (const Line3<Real>& rkLine,
    const Plane3<Real>& rkPlane)
    :
    m_pkLine(&rkLine),
    m_pkPlane(&rkPlane)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Line3<Real>& IntrLine3Plane3<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
const Plane3<Real>& IntrLine3Plane3<Real>::GetPlane () const
{
    return *m_pkPlane;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Plane3<Real>::Test ()
{
    Real fDdN = m_pkLine->Direction.Dot(m_pkPlane->Normal);
    if (Math<Real>::FAbs(fDdN) > Math<Real>::ZERO_TOLERANCE)
    {
        // The line is not parallel to the plane, so they must intersect.
        // The line parameter is *not* set, since this is a test-intersection
        // query.
        m_iIntersectionType = IT_POINT;
        return true;
    }

    // The line and plane are parallel.  Determine if they are numerically
    // close enough to be coincident.
    Real fSDistance = m_pkPlane->DistanceTo(m_pkLine->Origin);
    if (Math<Real>::FAbs(fSDistance) <= Math<Real>::ZERO_TOLERANCE)
    {
        m_iIntersectionType = IT_LINE;
        return true;
    }

    m_iIntersectionType = IT_EMPTY;
    return false;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Plane3<Real>::Find ()
{
    Real fDdN = m_pkLine->Direction.Dot(m_pkPlane->Normal);
    Real fSDistance = m_pkPlane->DistanceTo(m_pkLine->Origin);
    if (Math<Real>::FAbs(fDdN) > Math<Real>::ZERO_TOLERANCE)
    {
        // The line is not parallel to the plane, so they must intersect.
        m_fLineT = -fSDistance/fDdN;
        m_iIntersectionType = IT_POINT;
        return true;
    }

    // The Line and plane are parallel.  Determine if they are numerically
    // close enough to be coincident.
    if (Math<Real>::FAbs(fSDistance) <= Math<Real>::ZERO_TOLERANCE)
    {
        // The line is coincident with the plane, so choose t = 0 for the
        // parameter.
        m_fLineT = (Real)0.0;
        m_iIntersectionType = IT_LINE;
        return true;
    }

    m_iIntersectionType = IT_EMPTY;
    return false;
}
//----------------------------------------------------------------------------
template <class Real>
Real IntrLine3Plane3<Real>::GetLineT () const
{
    return m_fLineT;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrLine3Plane3<float>;

template WM4_FOUNDATION_ITEM
class IntrLine3Plane3<double>;
//----------------------------------------------------------------------------
}
