// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrRay2Triangle2.h"
#include "Wm4Intersector1.h"
#include "Wm4IntrLine2Triangle2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrRay2Triangle2<Real>::IntrRay2Triangle2 (const Ray2<Real>& rkRay,
    const Triangle2<Real>& rkTriangle)
    :
    m_pkRay(&rkRay),
    m_pkTriangle(&rkTriangle)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Ray2<Real>& IntrRay2Triangle2<Real>::GetRay () const
{
    return *m_pkRay;
}
//----------------------------------------------------------------------------
template <class Real>
const Triangle2<Real>& IntrRay2Triangle2<Real>::GetTriangle () const
{
    return *m_pkTriangle;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrRay2Triangle2<Real>::Test ()
{
    Real afDist[3];
    int aiSign[3], iPositive, iNegative, iZero;
    IntrLine2Triangle2<Real>::TriangleLineRelations(m_pkRay->Origin,
        m_pkRay->Direction,*m_pkTriangle,afDist,aiSign,iPositive,iNegative,
        iZero);

    if (iPositive == 3 || iNegative == 3)
    {
        m_iIntersectionType = IT_EMPTY;
    }
    else
    {
        Real afParam[2];
        IntrLine2Triangle2<Real>::GetInterval(m_pkRay->Origin,
            m_pkRay->Direction,*m_pkTriangle,afDist,aiSign,afParam);

        Intersector1<Real> kIntr(afParam[0],afParam[1],(Real)0,
            Math<Real>::MAX_REAL);

        kIntr.Find();

        m_iQuantity = kIntr.GetQuantity();
        if (m_iQuantity == 2)
        {
            m_iIntersectionType = IT_SEGMENT;
        }
        else if (m_iQuantity == 1)
        {
            m_iIntersectionType = IT_POINT;
        }
        else
        {
            m_iIntersectionType = IT_EMPTY;
        }
    }

    return m_iIntersectionType != IT_EMPTY;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrRay2Triangle2<Real>::Find ()
{
    Real afDist[3];
    int aiSign[3], iPositive, iNegative, iZero;
    IntrLine2Triangle2<Real>::TriangleLineRelations(m_pkRay->Origin,
        m_pkRay->Direction,*m_pkTriangle,afDist,aiSign,iPositive,iNegative,
        iZero);

    if (iPositive == 3 || iNegative == 3)
    {
        // No intersections.
        m_iQuantity = 0;
        m_iIntersectionType = IT_EMPTY;
    }
    else
    {
        Real afParam[2];
        IntrLine2Triangle2<Real>::GetInterval(m_pkRay->Origin,
            m_pkRay->Direction,*m_pkTriangle,afDist,aiSign,afParam);

        Intersector1<Real> kIntr(afParam[0],afParam[1],(Real)0,
            Math<Real>::MAX_REAL);

        kIntr.Find();

        m_iQuantity = kIntr.GetQuantity();
        if (m_iQuantity == 2)
        {
            // Segment intersection.
            m_iIntersectionType = IT_SEGMENT;
            m_akPoint[0] = m_pkRay->Origin + kIntr.GetOverlap(0)*
                m_pkRay->Direction;
            m_akPoint[1] = m_pkRay->Origin + kIntr.GetOverlap(1)*
                m_pkRay->Direction;
        }
        else if (m_iQuantity == 1)
        {
            // Point intersection.
            m_iIntersectionType = IT_POINT;
            m_akPoint[0] = m_pkRay->Origin + kIntr.GetOverlap(0)*
                m_pkRay->Direction;
        }
        else
        {
            // No intersections.
            m_iIntersectionType = IT_EMPTY;
        }
    }

    return m_iIntersectionType != IT_EMPTY;
}
//----------------------------------------------------------------------------
template <class Real>
int IntrRay2Triangle2<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& IntrRay2Triangle2<Real>::GetPoint (int i) const
{
    return m_akPoint[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrRay2Triangle2<float>;

template WM4_FOUNDATION_ITEM
class IntrRay2Triangle2<double>;
//----------------------------------------------------------------------------
}
