// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4RUNGEKUTTA4_H
#define WM4RUNGEKUTTA4_H

#include "Wm4FoundationLIB.h"
#include "Wm4OdeSolver.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM OdeRungeKutta4 : public OdeSolver<Real>
{
public:
    OdeRungeKutta4 (int iDim, Real fStep,
        typename OdeSolver<Real>::Function oFunction, void* pvData = 0);

    virtual ~OdeRungeKutta4 ();

    virtual void Update (Real fTIn, Real* afXIn, Real& rfTOut,
        Real* afXOut);

    virtual void SetStepSize (Real fStep);

protected:
    using OdeSolver<Real>::m_iDim;
    using OdeSolver<Real>::m_fStep;
    using OdeSolver<Real>::m_pvData;
    using OdeSolver<Real>::m_afFValue;

    Real m_fHalfStep, m_fSixthStep;
    Real* m_afTemp1;
    Real* m_afTemp2;
    Real* m_afTemp3;
    Real* m_afTemp4;
    Real* m_afXTemp;
};

typedef OdeRungeKutta4<float> OdeRungeKutta4f;
typedef OdeRungeKutta4<double> OdeRungeKutta4d;

}

#endif
