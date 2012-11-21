// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrSegment2Circle2.h"
#include "Wm4IntrLine2Circle2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrSegment2Circle2<Real>::IntrSegment2Circle2 (
    const Segment2<Real>& rkSegment,
    const Circle2<Real>& rkCircle)
    :
    m_pkSegment(&rkSegment),
    m_pkCircle(&rkCircle)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Segment2<Real>& IntrSegment2Circle2<Real>::GetSegment () const
{
    return *m_pkSegment;
}
//----------------------------------------------------------------------------
template <class Real>
const Circle2<Real>& IntrSegment2Circle2<Real>::GetCircle () const
{
    return *m_pkCircle;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSegment2Circle2<Real>::Find ()
{
    Real afT[2];
    bool bIntersects = IntrLine2Circle2<Real>::Find(m_pkSegment->Origin,
        m_pkSegment->Direction,m_pkCircle->Center,m_pkCircle->Radius,m_iQuantity,
        afT);

    if (bIntersects)
    {
        // reduce root count if line-circle intersections are not on segment
        if (m_iQuantity == 1)
        {
            if (Math<Real>::FAbs(afT[0]) > m_pkSegment->Extent)
            {
                m_iQuantity = 0;
            }
        }
        else
        {
            if (afT[1] < -m_pkSegment->Extent || afT[0] >  m_pkSegment->Extent)
            {
                m_iQuantity = 0;
            }
            else
            {
                if (afT[1] <= m_pkSegment->Extent)
                {
                    if (afT[0] < -m_pkSegment->Extent)
                    {
                        m_iQuantity = 1;
                        afT[0] = afT[1];
                    }
                }
                else
                {
                    m_iQuantity = (afT[0] >= -m_pkSegment->Extent ? 1 : 0);
                }
            }
        }

        for (int i = 0; i < m_iQuantity; i++)
        {
            m_akPoint[i] = m_pkSegment->Origin + afT[i]*m_pkSegment->Direction;
        }
    }

    m_iIntersectionType = (m_iQuantity > 0 ? IT_POINT : IT_EMPTY);
    return m_iIntersectionType != IT_EMPTY;
}
//----------------------------------------------------------------------------
template <class Real>
int IntrSegment2Circle2<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& IntrSegment2Circle2<Real>::GetPoint (int i)
    const
{
    assert(0 <= i && i < m_iQuantity);
    return m_akPoint[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrSegment2Circle2<float>;

template WM4_FOUNDATION_ITEM
class IntrSegment2Circle2<double>;
//----------------------------------------------------------------------------
}
