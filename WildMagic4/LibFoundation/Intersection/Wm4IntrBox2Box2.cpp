// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrBox2Box2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrBox2Box2<Real>::IntrBox2Box2 (const Box2<Real>& rkBox0,
    const Box2<Real>& rkBox1)
    :
    m_pkBox0(&rkBox0),
    m_pkBox1(&rkBox1)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Box2<Real>& IntrBox2Box2<Real>::GetBox0 () const
{
    return *m_pkBox0;
}
//----------------------------------------------------------------------------
template <class Real>
const Box2<Real>& IntrBox2Box2<Real>::GetBox1 () const
{
    return *m_pkBox1;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrBox2Box2<Real>::Test ()
{
    // convenience variables
    const Vector2<Real>* akA = m_pkBox0->Axis;
    const Vector2<Real>* akB = m_pkBox1->Axis;
    const Real* afEA = m_pkBox0->Extent;
    const Real* afEB = m_pkBox1->Extent;

    // compute difference of box centers, D = C1-C0
    Vector2<Real> kD = m_pkBox1->Center - m_pkBox0->Center;

    Real aafAbsAdB[2][2], fAbsAdD, fRSum;
    
    // axis C0+t*A0
    aafAbsAdB[0][0] = Math<Real>::FAbs(akA[0].Dot(akB[0]));
    aafAbsAdB[0][1] = Math<Real>::FAbs(akA[0].Dot(akB[1]));
    fAbsAdD = Math<Real>::FAbs(akA[0].Dot(kD));
    fRSum = afEA[0] + afEB[0]*aafAbsAdB[0][0] + afEB[1]*aafAbsAdB[0][1];
    if (fAbsAdD > fRSum)
    {
        return false;
    }

    // axis C0+t*A1
    aafAbsAdB[1][0] = Math<Real>::FAbs(akA[1].Dot(akB[0]));
    aafAbsAdB[1][1] = Math<Real>::FAbs(akA[1].Dot(akB[1]));
    fAbsAdD = Math<Real>::FAbs(akA[1].Dot(kD));
    fRSum = afEA[1] + afEB[0]*aafAbsAdB[1][0] + afEB[1]*aafAbsAdB[1][1];
    if (fAbsAdD > fRSum)
    {
        return false;
    }

    // axis C0+t*B0
    fAbsAdD = Math<Real>::FAbs(akB[0].Dot(kD));
    fRSum = afEB[0] + afEA[0]*aafAbsAdB[0][0] + afEA[1]*aafAbsAdB[1][0];
    if (fAbsAdD > fRSum)
    {
        return false;
    }

    // axis C0+t*B1
    fAbsAdD = Math<Real>::FAbs(akB[1].Dot(kD));
    fRSum = afEB[1] + afEA[0]*aafAbsAdB[0][1] + afEA[1]*aafAbsAdB[1][1];
    if (fAbsAdD > fRSum)
    {
        return false;
    }

    return true;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrBox2Box2<float>;

template WM4_FOUNDATION_ITEM
class IntrBox2Box2<double>;
//----------------------------------------------------------------------------
}
