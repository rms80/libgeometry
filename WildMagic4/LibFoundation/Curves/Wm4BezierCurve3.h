// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4BEZIERCURVE3_H
#define WM4BEZIERCURVE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4SingleCurve3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM BezierCurve3 : public SingleCurve3<Real>
{
public:
    // Construction and destruction.  BezierCurve3 accepts responsibility for
    // deleting the input arrays.
    BezierCurve3 (int iDegree, Vector3<Real>* akCtrlPoint);
    virtual ~BezierCurve3 ();

    int GetDegree () const;
    const Vector3<Real>* GetControlPoints () const;

    virtual Vector3<Real> GetPosition (Real fTime) const;
    virtual Vector3<Real> GetFirstDerivative (Real fTime) const;
    virtual Vector3<Real> GetSecondDerivative (Real fTime) const;
    virtual Vector3<Real> GetThirdDerivative (Real fTime) const;

    virtual Real GetVariation (Real fT0, Real fT1,
        const Vector3<Real>* pkP0 = 0, const Vector3<Real>* pkP1 = 0) const;

protected:
    int m_iDegree;
    int m_iNumCtrlPoints;
    Vector3<Real>* m_akCtrlPoint;
    Vector3<Real>* m_akDer1CtrlPoint;
    Vector3<Real>* m_akDer2CtrlPoint;
    Vector3<Real>* m_akDer3CtrlPoint;
    Real** m_aafChoose;

    // variation support
    int m_iTwoDegree;
    int m_iTwoDegreePlusOne;
    Real* m_afSigma;
    Real* m_afRecip;
    Real* m_afPowT0;
    Real* m_afPowOmT0;
    Real* m_afPowT1;
    Real* m_afPowOmT1;
};

typedef BezierCurve3<float> BezierCurve3f;
typedef BezierCurve3<double> BezierCurve3d;

}

#endif
