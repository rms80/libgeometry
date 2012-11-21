// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrLine2Box2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrLine2Box2<Real>::IntrLine2Box2 (const Line2<Real>& rkLine,
    const Box2<Real>& rkBox)
    :
    m_pkLine(&rkLine),
    m_pkBox(&rkBox)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Line2<Real>& IntrLine2Box2<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
const Box2<Real>& IntrLine2Box2<Real>::GetBox () const
{
    return *m_pkBox;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine2Box2<Real>::Test ()
{
    Vector2<Real> kDiff = m_pkLine->Origin - m_pkBox->Center;
    Vector2<Real> kPerp = m_pkLine->Direction.Perp();
    Real fLhs = Math<Real>::FAbs(kPerp.Dot(kDiff));
    Real fPart0 = Math<Real>::FAbs(kPerp.Dot(m_pkBox->Axis[0]));
    Real fPart1 = Math<Real>::FAbs(kPerp.Dot(m_pkBox->Axis[1]));
    Real fRhs = m_pkBox->Extent[0]*fPart0 + m_pkBox->Extent[1]*fPart1;
    return fLhs <= fRhs;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine2Box2<Real>::Find ()
{
    Real fT0 = -Math<Real>::MAX_REAL, fT1 = Math<Real>::MAX_REAL;
    return DoClipping(fT0,fT1,m_pkLine->Origin,m_pkLine->Direction,*m_pkBox,
        true,m_iQuantity,m_akPoint,m_iIntersectionType);
}
//----------------------------------------------------------------------------
template <class Real>
int IntrLine2Box2<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& IntrLine2Box2<Real>::GetPoint (int i) const
{
    assert(0 <= i && i < m_iQuantity);
    return m_akPoint[i];
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine2Box2<Real>::DoClipping (Real fT0, Real fT1,
    const Vector2<Real>& rkOrigin, const Vector2<Real>& rkDirection,
    const Box2<Real>& rkBox, bool bSolid, int& riQuantity,
    Vector2<Real> akPoint[2], int& riIntrType)
{
    assert(fT0 < fT1);

    // convert linear component to box coordinates
    Vector2<Real> kDiff = rkOrigin - rkBox.Center;
    Vector2<Real> kBOrigin(
        kDiff.Dot(rkBox.Axis[0]),
        kDiff.Dot(rkBox.Axis[1])
    );
    Vector2<Real> kBDirection(
        rkDirection.Dot(rkBox.Axis[0]),
        rkDirection.Dot(rkBox.Axis[1])
    );

    Real fSaveT0 = fT0, fSaveT1 = fT1;
    bool bNotAllClipped =
        Clip(+kBDirection.X(),-kBOrigin.X()-rkBox.Extent[0],fT0,fT1) &&
        Clip(-kBDirection.X(),+kBOrigin.X()-rkBox.Extent[0],fT0,fT1) &&
        Clip(+kBDirection.Y(),-kBOrigin.Y()-rkBox.Extent[1],fT0,fT1) &&
        Clip(-kBDirection.Y(),+kBOrigin.Y()-rkBox.Extent[1],fT0,fT1);

    if (bNotAllClipped && (bSolid || fT0 != fSaveT0 || fT1 != fSaveT1))
    {
        if (fT1 > fT0)
        {
            riIntrType = IT_SEGMENT;
            riQuantity = 2;
            akPoint[0] = rkOrigin + fT0*rkDirection;
            akPoint[1] = rkOrigin + fT1*rkDirection;
        }
        else
        {
            riIntrType = IT_POINT;
            riQuantity = 1;
            akPoint[0] = rkOrigin + fT0*rkDirection;
        }
    }
    else
    {
        riIntrType = IT_EMPTY;
        riQuantity = 0;
    }

    return riIntrType != IT_EMPTY;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine2Box2<Real>::Clip (Real fDenom, Real fNumer, Real& rfT0,
    Real& rfT1)
{
    // Return value is 'true' if line segment intersects the current test
    // plane.  Otherwise 'false' is returned in which case the line segment
    // is entirely clipped.

    if (fDenom > (Real)0.0)
    {
        if (fNumer > fDenom*rfT1)
        {
            return false;
        }
        if (fNumer > fDenom*rfT0)
        {
            rfT0 = fNumer/fDenom;
        }
        return true;
    }
    else if (fDenom < (Real)0.0)
    {
        if (fNumer > fDenom*rfT0)
        {
            return false;
        }
        if (fNumer > fDenom*rfT1)
        {
            rfT1 = fNumer/fDenom;
        }
        return true;
    }
    else
    {
        return fNumer <= (Real)0.0;
    }
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrLine2Box2<float>;

template WM4_FOUNDATION_ITEM
class IntrLine2Box2<double>;
//----------------------------------------------------------------------------
}
