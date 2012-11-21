// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrLine3Sphere3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrLine3Sphere3<Real>::IntrLine3Sphere3 (const Line3<Real>& rkLine,
    const Sphere3<Real>& rkSphere)
    :
    m_pkLine(&rkLine),
    m_pkSphere(&rkSphere)
{
    m_iQuantity = 0;
}
//----------------------------------------------------------------------------
template <class Real>
const Line3<Real>& IntrLine3Sphere3<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
const Sphere3<Real>& IntrLine3Sphere3<Real>::GetSphere () const
{
    return *m_pkSphere;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Sphere3<Real>::Test ()
{
    Vector3<Real> kDiff = m_pkLine->Origin - m_pkSphere->Center;
    Real fA0 = kDiff.Dot(kDiff) - m_pkSphere->Radius*m_pkSphere->Radius;
    Real fA1 = m_pkLine->Direction.Dot(kDiff);
    Real fDiscr = fA1*fA1 - fA0;
    return fDiscr >= (Real)0.0;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Sphere3<Real>::Find ()
{
    Vector3<Real> kDiff = m_pkLine->Origin - m_pkSphere->Center;
    Real fA0 = kDiff.Dot(kDiff) - m_pkSphere->Radius*m_pkSphere->Radius;
    Real fA1 = m_pkLine->Direction.Dot(kDiff);
    Real fDiscr = fA1*fA1 - fA0;

    if (fDiscr < (Real)0.0)
    {
        m_iIntersectionType = IT_EMPTY;
        m_iQuantity = 0;
    }
    else if (fDiscr >= Math<Real>::ZERO_TOLERANCE)
    {
        Real fRoot = Math<Real>::Sqrt(fDiscr);
        m_afLineT[0] = -fA1 - fRoot;
        m_afLineT[1] = -fA1 + fRoot;
        m_akPoint[0] = m_pkLine->Origin + m_afLineT[0]*m_pkLine->Direction;
        m_akPoint[1] = m_pkLine->Origin + m_afLineT[1]*m_pkLine->Direction;
        m_iIntersectionType = IT_SEGMENT;
        m_iQuantity = 2;
    }
    else
    {
        m_afLineT[0] = -fA1;
        m_akPoint[0] = m_pkLine->Origin + m_afLineT[0]*m_pkLine->Direction;
        m_iQuantity = 1;
        m_iIntersectionType = IT_POINT;
    }

    return m_iQuantity > 0;
}
//----------------------------------------------------------------------------
template <class Real>
int IntrLine3Sphere3<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& IntrLine3Sphere3<Real>::GetPoint (int i) const
{
    assert(0 <= i && i < m_iQuantity);
    return m_akPoint[i];
}
//----------------------------------------------------------------------------
template <class Real>
Real IntrLine3Sphere3<Real>::GetLineT (int i) const
{
    assert( 0 <= i && i < m_iQuantity );
    return m_afLineT[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrLine3Sphere3<float>;

template WM4_FOUNDATION_ITEM
class IntrLine3Sphere3<double>;
//----------------------------------------------------------------------------
}
