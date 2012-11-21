// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4OdeImplicitEuler.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
OdeImplicitEuler<Real>::OdeImplicitEuler (int iDim, Real fStep,
    typename OdeSolver<Real>::Function oFunction,
    DerivativeFunction oDFunction, void* pvData)
    :
    OdeSolver<Real>(iDim,fStep,oFunction,pvData),
    m_kDF(iDim,iDim),
    m_kF(iDim),
    m_kIdentity(iDim,iDim)
{
    m_oDFunction = oDFunction;
    for (int i = 0; i < iDim; i++)
    {
        m_kIdentity[i][i] = (Real)1.0;
    }
}
//----------------------------------------------------------------------------
template <class Real>
OdeImplicitEuler<Real>::~OdeImplicitEuler ()
{
}
//----------------------------------------------------------------------------
template <class Real>
void OdeImplicitEuler<Real>::Update (Real fTIn, Real* afXIn, Real& rfTOut,
    Real* afXOut)
{
    m_oFunction(fTIn,afXIn,m_pvData,m_kF);
    m_oDFunction(fTIn,afXIn,m_pvData,m_kDF);
    GMatrix<Real> kDG = m_kIdentity - m_fStep*m_kDF;
    GMatrix<Real> kDGInverse(m_iDim,m_iDim);
    bool bInvertible = kDG.GetInverse(kDGInverse);

    if (bInvertible)
    {
        m_kF = kDGInverse*m_kF;
        for (int i = 0; i < m_iDim; i++)
        {
            afXOut[i] = afXIn[i] + m_fStep*m_kF[i];
        }
    }
    else
    {
        size_t uiSize = m_iDim*sizeof(Real);
        System::Memcpy(afXOut,uiSize,afXIn,uiSize);
    }

    rfTOut = fTIn + m_fStep;
}
//----------------------------------------------------------------------------
template <class Real>
void OdeImplicitEuler<Real>::SetStepSize (Real fStep)
{
    m_fStep = fStep;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class OdeImplicitEuler<float>;

template WM4_FOUNDATION_ITEM
class OdeImplicitEuler<double>;
//----------------------------------------------------------------------------
}
