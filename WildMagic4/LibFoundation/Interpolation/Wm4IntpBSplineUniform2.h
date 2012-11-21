// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPBSPLINEUNIFORM2_H
#define WM4INTPBSPLINEUNIFORM2_H

#include "Wm4FoundationLIB.h"
#include "Wm4IntpBSplineUniform.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpBSplineUniform2
    : public IntpBSplineUniform<Real>
{
public:
    // Construction.  IntpBSplineUniform2 accepts responsibility for
    // deleting the input array afData.
    IntpBSplineUniform2 (int iDegree, const int* aiDim, Real* afData);

    int Index (int iX, int iY) const;

    // spline evaluation for function interpolation (no derivatives)
    Real operator() (Real fX, Real fY);
    virtual Real operator() (Real* afX);

    // spline evaluation, derivative counts given in iDx, iDy, aiDx[]
    Real operator() (int iDx, int iDy, Real fX, Real fY);
    virtual Real operator() (int* aiDx, Real* afX);

private:
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

    void EvaluateUnknownData ();
    void ComputeIntermediate ();
};

typedef IntpBSplineUniform2<float> IntpBSplineUniform2f;
typedef IntpBSplineUniform2<double> IntpBSplineUniform2d;

}

#endif
