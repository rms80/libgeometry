// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntpAkima1.h"
#include "Wm4Math.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntpAkima1<Real>::IntpAkima1 (int iQuantity, Real* afF)
{
    // At least three data points are needed to construct the estimates of
    // the boundary derivatives.
    assert(iQuantity >= 3 && afF);

    m_iQuantity = iQuantity;
    m_afF = afF;
    m_akPoly = WM4_NEW Polynomial[iQuantity-1];
}
//----------------------------------------------------------------------------
template <class Real>
IntpAkima1<Real>::~IntpAkima1 ()
{
    WM4_DELETE[] m_akPoly;
}
//----------------------------------------------------------------------------
template <class Real>
int IntpAkima1<Real>::GetQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
const Real* IntpAkima1<Real>::GetF () const
{
    return m_afF;
}
//----------------------------------------------------------------------------
template <class Real>
const typename IntpAkima1<Real>::Polynomial*
IntpAkima1<Real>::GetPolynomials () const
{
    return m_akPoly;
}
//----------------------------------------------------------------------------
template <class Real>
const typename IntpAkima1<Real>::Polynomial&
IntpAkima1<Real>::GetPolynomial (int i) const
{
    assert(0 <= i && i < m_iQuantity-1);
    return m_akPoly[i];
}
//----------------------------------------------------------------------------
template <class Real>
Real IntpAkima1<Real>::ComputeDerivative (Real* afSlope) const
{
    if (afSlope[1] != afSlope[2])
    {
        if (afSlope[0] != afSlope[1])
        {
            if (afSlope[2] != afSlope[3])
            {
                Real fAD0 = Math<Real>::FAbs(afSlope[3] - afSlope[2]);
                Real fAD1 = Math<Real>::FAbs(afSlope[0] - afSlope[1]);
                return (fAD0*afSlope[1]+fAD1*afSlope[2])/(fAD0+fAD1);
            }
            else
            {
                return afSlope[2];
            }
        }
        else
        {
            if (afSlope[2] != afSlope[3])
            {
                return afSlope[1];
            }
            else
            {
                return ((Real)0.5)*(afSlope[1]+afSlope[2]);
            }
        }
    }
    else
    {
        return afSlope[1];
    }
}
//----------------------------------------------------------------------------
template <class Real>
Real IntpAkima1<Real>::operator() (Real fX) const
{
    int iIndex;
    Real fDX;

    if (Lookup(fX,iIndex,fDX))
    {
        return m_akPoly[iIndex](fDX);
    }

    return Math<Real>::MAX_REAL;
}
//----------------------------------------------------------------------------
template <class Real>
Real IntpAkima1<Real>::operator() (int iOrder, Real fX) const
{
    int iIndex;
    Real fDX;

    if (Lookup(fX,iIndex,fDX))
    {
        return m_akPoly[iIndex](iOrder,fDX);
    }

    return Math<Real>::MAX_REAL;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntpAkima1<float>;

template WM4_FOUNDATION_ITEM
class IntpAkima1<double>;
//----------------------------------------------------------------------------
}
