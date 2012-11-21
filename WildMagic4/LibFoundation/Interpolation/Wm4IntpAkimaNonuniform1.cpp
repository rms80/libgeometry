// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntpAkimaNonuniform1.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntpAkimaNonuniform1<Real>::IntpAkimaNonuniform1 (int iQuantity, Real* afX,
    Real* afF)
    :
    IntpAkima1<Real>(iQuantity,afF)
{
    m_afX = afX;

    // compute slopes
    Real* afSlope = WM4_NEW Real[iQuantity+3];
    int i, iP1, iP2;
    for (i = 0, iP1 = 1, iP2 = 2; i < iQuantity-1; i++, iP1++, iP2++)
    {
        Real fDX = afX[iP1] - afX[i];
        Real fDF = afF[iP1] - afF[i];
        afSlope[iP2] = fDF/fDX;
    }

    afSlope[1] = ((Real)2.0)*afSlope[2] - afSlope[3];
    afSlope[0] = ((Real)2.0)*afSlope[1] - afSlope[2];
    afSlope[iQuantity+1] = ((Real)2.0)*afSlope[iQuantity] -
        afSlope[iQuantity-1];
    afSlope[iQuantity+2] = ((Real)2.0)*afSlope[iQuantity+1] -
        afSlope[iQuantity];

    // construct derivatives
    Real* afFDer = WM4_NEW Real[iQuantity];
    for (i = 0; i < iQuantity; i++)
    {
        afFDer[i] = ComputeDerivative(afSlope+i);
    }

    // construct polynomials
    for (i = 0, iP1 = 1; i < iQuantity-1; i++, iP1++)
    {
        typename IntpAkima1<Real>::Polynomial& rkPoly = m_akPoly[i];

        Real fF0 = afF[i], fF1 = afF[iP1];
        Real fFDer0 = afFDer[i], fFDer1 = afFDer[iP1];
        Real fDF = fF1 - fF0;
        Real fDX = afX[iP1] - afX[i];
        Real fDX2 = fDX*fDX, fDX3 = fDX2*fDX;

        rkPoly[0] = fF0;
        rkPoly[1] = fFDer0;
        rkPoly[2] = (((Real)3.0)*fDF-fDX*(fFDer1+((Real)2.0)*fFDer0))/fDX2;
        rkPoly[3] = (fDX*(fFDer0 + fFDer1)-((Real)2.0)*fDF)/fDX3;
    }

    WM4_DELETE[] afSlope;
    WM4_DELETE[] afFDer;
}
//----------------------------------------------------------------------------
template <class Real>
IntpAkimaNonuniform1<Real>::~IntpAkimaNonuniform1 ()
{
}
//----------------------------------------------------------------------------
template <class Real>
const Real* IntpAkimaNonuniform1<Real>::GetX () const
{
    return m_afX;
}
//----------------------------------------------------------------------------
template <class Real>
Real IntpAkimaNonuniform1<Real>::GetXMin () const
{
    return m_afX[0];
}
//----------------------------------------------------------------------------
template <class Real>
Real IntpAkimaNonuniform1<Real>::GetXMax () const
{
    return m_afX[m_iQuantity-1];
}
//----------------------------------------------------------------------------
template <class Real>
bool IntpAkimaNonuniform1<Real>::Lookup (Real fX, int& riIndex, Real& rfDX)
    const
{
    if (fX >= m_afX[0])
    {
        if (fX <= m_afX[m_iQuantity-1])
        {
            for (riIndex = 0; riIndex+1 < m_iQuantity; riIndex++)
            {
                if (fX < m_afX[riIndex+1])
                {
                    rfDX = fX - m_afX[riIndex];
                    return true;
                }
            }

            riIndex--;
            rfDX = fX - m_afX[riIndex];
            return true;
        }
    }

    return false;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntpAkimaNonuniform1<float>;

template WM4_FOUNDATION_ITEM
class IntpAkimaNonuniform1<double>;
//----------------------------------------------------------------------------
}
