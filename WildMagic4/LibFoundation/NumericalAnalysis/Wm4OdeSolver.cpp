// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4OdeSolver.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
OdeSolver<Real>::OdeSolver (int iDim, Real fStep, Function oFunction,
    void* pvData)
{
    m_iDim = iDim;
    m_fStep = fStep;
    m_oFunction = oFunction;
    m_pvData = pvData;
    m_afFValue = WM4_NEW Real[iDim];
}
//----------------------------------------------------------------------------
template <class Real>
OdeSolver<Real>::~OdeSolver ()
{
    WM4_DELETE[] m_afFValue;
}
//----------------------------------------------------------------------------
template <class Real>
Real OdeSolver<Real>::GetStepSize () const
{
    return m_fStep;
}
//----------------------------------------------------------------------------
template <class Real>
void OdeSolver<Real>::SetData (void* pvData)
{
    m_pvData = pvData;
}
//----------------------------------------------------------------------------
template <class Real>
void* OdeSolver<Real>::GetData () const
{
    return m_pvData;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class OdeSolver<float>;

template WM4_FOUNDATION_ITEM
class OdeSolver<double>;
//----------------------------------------------------------------------------
}
