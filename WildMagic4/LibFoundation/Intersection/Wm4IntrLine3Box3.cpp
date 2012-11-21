// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrLine3Box3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrLine3Box3<Real>::IntrLine3Box3 (const Line3<Real>& rkLine,
    const Box3<Real>& rkBox)
    :
    m_pkLine(&rkLine),
    m_pkBox(&rkBox)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Line3<Real>& IntrLine3Box3<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
const Box3<Real>& IntrLine3Box3<Real>::GetBox () const
{
    return *m_pkBox;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Box3<Real>::Test ()
{
    Real afAWdU[3], afAWxDdU[3], fRhs;

    Vector3<Real> kDiff = m_pkLine->Origin - m_pkBox->Center;
    Vector3<Real> kWxD = m_pkLine->Direction.Cross(kDiff);

    afAWdU[1] = Math<Real>::FAbs(m_pkLine->Direction.Dot(m_pkBox->Axis[1]));
    afAWdU[2] = Math<Real>::FAbs(m_pkLine->Direction.Dot(m_pkBox->Axis[2]));
    afAWxDdU[0] = Math<Real>::FAbs(kWxD.Dot(m_pkBox->Axis[0]));
    fRhs = m_pkBox->Extent[1]*afAWdU[2] + m_pkBox->Extent[2]*afAWdU[1];
    if (afAWxDdU[0] > fRhs)
    {
        return false;
    }

    afAWdU[0] = Math<Real>::FAbs(m_pkLine->Direction.Dot(m_pkBox->Axis[0]));
    afAWxDdU[1] = Math<Real>::FAbs(kWxD.Dot(m_pkBox->Axis[1]));
    fRhs = m_pkBox->Extent[0]*afAWdU[2] + m_pkBox->Extent[2]*afAWdU[0];
    if (afAWxDdU[1] > fRhs)
    {
        return false;
    }

    afAWxDdU[2] = Math<Real>::FAbs(kWxD.Dot(m_pkBox->Axis[2]));
    fRhs = m_pkBox->Extent[0]*afAWdU[1] + m_pkBox->Extent[1]*afAWdU[0];
    if (afAWxDdU[2] > fRhs)
    {
        return false;
    }

    return true;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Box3<Real>::Find ()
{
    Real fT0 = -Math<Real>::MAX_REAL, fT1 = Math<Real>::MAX_REAL;
    return DoClipping(fT0,fT1,m_pkLine->Origin,m_pkLine->Direction,*m_pkBox,
        true,m_iQuantity,m_akPoint,m_iIntersectionType);
}
//----------------------------------------------------------------------------
template <class Real>
int IntrLine3Box3<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& IntrLine3Box3<Real>::GetPoint (int i) const
{
    assert(0 <= i && i < m_iQuantity);
    return m_akPoint[i];
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Box3<Real>::DoClipping (Real fT0, Real fT1,
    const Vector3<Real>& rkOrigin, const Vector3<Real>& rkDirection,
    const Box3<Real>& rkBox, bool bSolid, int& riQuantity,
    Vector3<Real> akPoint[2], int& riIntrType)
{
    assert(fT0 < fT1);

    // convert linear component to box coordinates
    Vector3<Real> kDiff = rkOrigin - rkBox.Center;
    Vector3<Real> kBOrigin(
        kDiff.Dot(rkBox.Axis[0]),
        kDiff.Dot(rkBox.Axis[1]),
        kDiff.Dot(rkBox.Axis[2])
    );
    Vector3<Real> kBDirection(
        rkDirection.Dot(rkBox.Axis[0]),
        rkDirection.Dot(rkBox.Axis[1]),
        rkDirection.Dot(rkBox.Axis[2])
    );

    Real fSaveT0 = fT0, fSaveT1 = fT1;
    bool bNotAllClipped =
        Clip(+kBDirection.X(),-kBOrigin.X()-rkBox.Extent[0],fT0,fT1) &&
        Clip(-kBDirection.X(),+kBOrigin.X()-rkBox.Extent[0],fT0,fT1) &&
        Clip(+kBDirection.Y(),-kBOrigin.Y()-rkBox.Extent[1],fT0,fT1) &&
        Clip(-kBDirection.Y(),+kBOrigin.Y()-rkBox.Extent[1],fT0,fT1) &&
        Clip(+kBDirection.Z(),-kBOrigin.Z()-rkBox.Extent[2],fT0,fT1) &&
        Clip(-kBDirection.Z(),+kBOrigin.Z()-rkBox.Extent[2],fT0,fT1);

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
        riQuantity = 0;
        riIntrType = IT_EMPTY;
    }

    return riIntrType != IT_EMPTY;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Box3<Real>::Clip (Real fDenom, Real fNumer, Real& rfT0,
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
class IntrLine3Box3<float>;

template WM4_FOUNDATION_ITEM
class IntrLine3Box3<double>;
//----------------------------------------------------------------------------
}
