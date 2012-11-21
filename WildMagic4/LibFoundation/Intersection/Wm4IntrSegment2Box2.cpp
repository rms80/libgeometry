// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrSegment2Box2.h"
#include "Wm4IntrLine2Box2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrSegment2Box2<Real>::IntrSegment2Box2 (const Segment2<Real>& rkSegment,
    const Box2<Real>& rkBox, bool bSolid)
    :
    m_pkSegment(&rkSegment),
    m_pkBox(&rkBox)
{
    m_bSolid = bSolid;
}
//----------------------------------------------------------------------------
template <class Real>
const Segment2<Real>& IntrSegment2Box2<Real>::GetSegment () const
{
    return *m_pkSegment;
}
//----------------------------------------------------------------------------
template <class Real>
const Box2<Real>& IntrSegment2Box2<Real>::GetBox () const
{
    return *m_pkBox;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSegment2Box2<Real>::Test ()
{
    Vector2<Real> kDiff = m_pkSegment->Origin - m_pkBox->Center;

    Real fAWdU[2], fADdU[2], fRhs;
    fAWdU[0] = Math<Real>::FAbs(m_pkSegment->Direction.Dot(m_pkBox->Axis[0]));
    fADdU[0] = Math<Real>::FAbs(kDiff.Dot(m_pkBox->Axis[0]));
    fRhs = m_pkBox->Extent[0] + m_pkSegment->Extent*fAWdU[0];
    if (fADdU[0] > fRhs)
    {
        return false;
    }

    fAWdU[1] = Math<Real>::FAbs(m_pkSegment->Direction.Dot(m_pkBox->Axis[1]));
    fADdU[1] = Math<Real>::FAbs(kDiff.Dot(m_pkBox->Axis[1]));
    fRhs = m_pkBox->Extent[1] + m_pkSegment->Extent*fAWdU[1];
    if (fADdU[1] > fRhs)
    {
        return false;
    }

    Vector2<Real> kPerp = m_pkSegment->Direction.Perp();
    Real fLhs = Math<Real>::FAbs(kPerp.Dot(kDiff));
    Real fPart0 = Math<Real>::FAbs(kPerp.Dot(m_pkBox->Axis[0]));
    Real fPart1 = Math<Real>::FAbs(kPerp.Dot(m_pkBox->Axis[1]));
    fRhs = m_pkBox->Extent[0]*fPart0 + m_pkBox->Extent[1]*fPart1;
    return fLhs <= fRhs;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSegment2Box2<Real>::Find ()
{
    Real fT0 = -m_pkSegment->Extent, fT1 = m_pkSegment->Extent;
    return IntrLine2Box2<Real>::DoClipping(fT0,fT1,m_pkSegment->Origin,
        m_pkSegment->Direction,*m_pkBox,m_bSolid,m_iQuantity,m_akPoint,
        m_iIntersectionType);
}
//----------------------------------------------------------------------------
template <class Real>
int IntrSegment2Box2<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& IntrSegment2Box2<Real>::GetPoint (int i) const
{
    assert(0 <= i && i < m_iQuantity);
    return m_akPoint[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrSegment2Box2<float>;

template WM4_FOUNDATION_ITEM
class IntrSegment2Box2<double>;
//----------------------------------------------------------------------------
}
