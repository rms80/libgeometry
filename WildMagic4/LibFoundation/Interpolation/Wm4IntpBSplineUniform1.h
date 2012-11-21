// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPBSPLINEUNIFORM1_H
#define WM4INTPBSPLINEUNIFORM1_H

#include "Wm4FoundationLIB.h"
#include "Wm4IntpBSplineUniform.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpBSplineUniform1
    : public IntpBSplineUniform<Real>
{
public:
    // Construction.  IntpBSplineUniform1 accepts responsibility for
    // deleting the input array afData.
    IntpBSplineUniform1 (int iDegree, int iDim, Real* afData);

    // spline evaluation for function interpolation (no derivatives)
    Real operator() (Real fX);
    virtual Real operator() (Real* afX);

    // spline evaluation, derivative count given in iDx and aiDx[]
    Real operator() (int iDx, Real fX);
    virtual Real operator() (int* aiDx, Real* afX);

private:
    using IntpBSplineUniform<Real>::m_iDegree;
    using IntpBSplineUniform<Real>::m_afData;
    using IntpBSplineUniform<Real>::m_aiGridMin;
    using IntpBSplineUniform<Real>::m_aiGridMax;
    using IntpBSplineUniform<Real>::m_aiBase;
    using IntpBSplineUniform<Real>::m_aiOldBase;
    using IntpBSplineUniform<Real>::m_aafMatrix;
    using IntpBSplineUniform<Real>::m_afInter;
    using IntpBSplineUniform<Real>::m_aafPoly;
    using IntpBSplineUniform<Real>::m_oEvaluateCallback;

    void EvaluateUnknownData ();
    void ComputeIntermediate ();
};

typedef IntpBSplineUniform1<float> IntpBSplineUniform1f;
typedef IntpBSplineUniform1<double> IntpBSplineUniform1d;

}

#endif
