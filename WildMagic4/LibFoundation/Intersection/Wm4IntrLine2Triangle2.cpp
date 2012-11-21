// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrLine2Triangle2.h"
#include "Wm4Intersector1.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrLine2Triangle2<Real>::IntrLine2Triangle2 (const Line2<Real>& rkLine,
    const Triangle2<Real>& rkTriangle)
    :
    m_pkLine(&rkLine),
    m_pkTriangle(&rkTriangle)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Line2<Real>& IntrLine2Triangle2<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
const Triangle2<Real>& IntrLine2Triangle2<Real>::GetTriangle () const
{
    return *m_pkTriangle;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine2Triangle2<Real>::Test ()
{
    Real afDist[3];
    int aiSign[3], iPositive, iNegative, iZero;
    TriangleLineRelations(m_pkLine->Origin,m_pkLine->Direction,*m_pkTriangle,
        afDist,aiSign,iPositive,iNegative,iZero);

    if (iPositive == 3 || iNegative == 3)
    {
        m_iIntersectionType = IT_EMPTY;
    }
    else
    {
        Real afParam[2];
        GetInterval(m_pkLine->Origin,m_pkLine->Direction,*m_pkTriangle,afDist,
            aiSign,afParam);

        Intersector1<Real> kIntr(afParam[0],afParam[1],
            -Math<Real>::MAX_REAL,+Math<Real>::MAX_REAL);

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
bool IntrLine2Triangle2<Real>::Find ()
{
    Real afDist[3];
    int aiSign[3], iPositive, iNegative, iZero;
    TriangleLineRelations(m_pkLine->Origin,m_pkLine->Direction,*m_pkTriangle,
        afDist,aiSign,iPositive,iNegative,iZero);

    if (iPositive == 3 || iNegative == 3)
    {
        // No intersections.
        m_iQuantity = 0;
        m_iIntersectionType = IT_EMPTY;
    }
    else
    {
        Real afParam[2];
        GetInterval(m_pkLine->Origin,m_pkLine->Direction,*m_pkTriangle,afDist,
            aiSign,afParam);

        Intersector1<Real> kIntr(afParam[0],afParam[1],
            -Math<Real>::MAX_REAL,+Math<Real>::MAX_REAL);

        kIntr.Find();

        m_iQuantity = kIntr.GetQuantity();
        if (m_iQuantity == 2)
        {
            // Segment intersection.
            m_iIntersectionType = IT_SEGMENT;
            m_akPoint[0] = m_pkLine->Origin + kIntr.GetOverlap(0)*
                m_pkLine->Direction;
            m_akPoint[1] = m_pkLine->Origin + kIntr.GetOverlap(1)*
                m_pkLine->Direction;
        }
        else if (m_iQuantity == 1)
        {
            // Point intersection.
            m_iIntersectionType = IT_POINT;
            m_akPoint[0] = m_pkLine->Origin + kIntr.GetOverlap(0)*
                m_pkLine->Direction;
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
int IntrLine2Triangle2<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& IntrLine2Triangle2<Real>::GetPoint (int i) const
{
    return m_akPoint[i];
}
//----------------------------------------------------------------------------
template <class Real>
void IntrLine2Triangle2<Real>::TriangleLineRelations (
    const Vector2<Real>& rkOrigin, const Vector2<Real>& rkDirection,
    const Triangle2<Real>& rkTriangle, Real afDist[3], int aiSign[3],
    int& riPositive, int& riNegative, int& riZero)
{
    riPositive = 0;
    riNegative = 0;
    riZero = 0;
    for (int i = 0; i < 3; i++)
    {
        Vector2<Real> kDiff = rkTriangle.V[i] - rkOrigin;
        afDist[i] = kDiff.DotPerp(rkDirection);
        if (afDist[i] > Math<Real>::ZERO_TOLERANCE)
        {
            aiSign[i] = 1;
            riPositive++;
        }
        else if (afDist[i] < -Math<Real>::ZERO_TOLERANCE)
        {
            aiSign[i] = -1;
            riNegative++;
        }
        else
        {
            afDist[i] = (Real)0.0;
            aiSign[i] = 0;
            riZero++;
        }
    }
}
//----------------------------------------------------------------------------
template <class Real>
void IntrLine2Triangle2<Real>::GetInterval (const Vector2<Real>& rkOrigin,
    const Vector2<Real>& rkDirection, const Triangle2<Real>& rkTriangle,
    const Real afDist[3], const int aiSign[3], Real afParam[2])
{
    // Project triangle onto line.
    Real afProj[3];
    int i;
    for (i = 0; i < 3; i++)
    {
        Vector2<Real> kDiff = rkTriangle.V[i] - rkOrigin;
        afProj[i] = rkDirection.Dot(kDiff);
    }

    // Compute transverse intersections of triangle edges with line.
    Real fNumer, fDenom;
    int i0, i1, i2;
    int iQuantity = 0;
    for (i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
    {
        if (aiSign[i0]*aiSign[i1] < 0)
        {
            assert(iQuantity < 2);
            fNumer = afDist[i0]*afProj[i1] - afDist[i1]*afProj[i0];
            fDenom = afDist[i0] - afDist[i1];
            afParam[iQuantity++] = fNumer/fDenom;
        }
    }

    // Check for grazing contact.
    if (iQuantity < 2)
    {
        for (i0 = 1, i1 = 2, i2 = 0; i2 < 3; i0 = i1, i1 = i2++)
        {
            if (aiSign[i2] == 0)
            {
                assert(iQuantity < 2);
                afParam[iQuantity++] = afProj[i2];
            }
        }
    }

    // Sort.
    assert(iQuantity >= 1);
    if (iQuantity == 2)
    {
        if (afParam[0] > afParam[1])
        {
            Real fSave = afParam[0];
            afParam[0] = afParam[1];
            afParam[1] = fSave;
        }
    }
    else
    {
        afParam[1] = afParam[0];
    }
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrLine2Triangle2<float>;

template WM4_FOUNDATION_ITEM
class IntrLine2Triangle2<double>;
//----------------------------------------------------------------------------
}
