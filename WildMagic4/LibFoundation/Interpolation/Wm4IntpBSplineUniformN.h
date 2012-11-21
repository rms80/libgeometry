// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPBSPLINEUNIFORMN_H
#define WM4INTPBSPLINEUNIFORMN_H

#include "Wm4FoundationLIB.h"
#include "Wm4IntpBSplineUniform.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpBSplineUniformN
    : public IntpBSplineUniform<Real>
{
public:
    // Construction and destruction.  IntpBSplineUniformN accepts
    // responsibility for deleting the input array afData.  The input array
    // aiDim is copied.
    IntpBSplineUniformN (int iDims, int iDegree, const int* aiDim,
        Real* afData);
    virtual ~IntpBSplineUniformN ();

    int Index (int* aiI) const;

    // spline evaluation for function interpolation (no derivatives)
    virtual Real operator() (Real* afX);

    // spline evaluation, derivative counts given in aiDx[]
    virtual Real operator() (int* aiDx, Real* afX);

private:
    using IntpBSplineUniform<Real>::m_iDims;
    using IntpBSplineUniform<Real>::m_iDegree;
    using IntpBSplineUniform<Real>::m_iDp1;
    using IntpBSplineUniform<Real>::m_iDp1ToN;
    using IntpBSplineUniform<Real>::m_aiDim;
    using IntpBSplineUniform<Real>::m_afData;
    using IntpBSplineUniform<Real>::m_aiGridMin;
    using IntpBSplineUniform<Real>::m_aiGridMax;
    using IntpBSplineUniform<Real>::m_aiBase;
    using IntpBSplineUniform<Real>::m_aiOldBase;
    using IntpBSplineUniform<Real>::m_aafMatrix;
    using IntpBSplineUniform<Real>::m_afCache;
    using IntpBSplineUniform<Real>::m_afInter;
    using IntpBSplineUniform<Real>::m_aafPoly;
    using IntpBSplineUniform<Real>::m_afProduct;
    using IntpBSplineUniform<Real>::m_aiSkip;
    using IntpBSplineUniform<Real>::m_oEvaluateCallback;

    int* m_aiEvI;
    int* m_aiCiLoop;
    int* m_aiCiDelta;
    int* m_aiOpI;
    int* m_aiOpJ;
    int* m_aiOpDelta;

    void EvaluateUnknownData ();
    void ComputeIntermediate ();
};

typedef IntpBSplineUniformN<float> IntpBSplineUniformNf;
typedef IntpBSplineUniformN<double> IntpBSplineUniformNd;

}

#endif
