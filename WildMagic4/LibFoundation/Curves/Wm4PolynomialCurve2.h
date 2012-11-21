// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4POLYNOMIALCURVE2_H
#define WM4POLYNOMIALCURVE2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Polynomial1.h"
#include "Wm4SingleCurve2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM PolynomialCurve2 : public SingleCurve2<Real>
{
public:
    // Construction and destruction.  PolynomialCurve2 accepts responsibility
    // for deleting the input polynomials.
    PolynomialCurve2 (Polynomial1<Real>* pkXPoly, Polynomial1<Real>* pkYPoly);
    virtual ~PolynomialCurve2 ();

    int GetDegree () const;
    const Polynomial1<Real>* GetXPolynomial () const;
    const Polynomial1<Real>* GetYPolynomial () const;

    virtual Vector2<Real> GetPosition (Real fTime) const;
    virtual Vector2<Real> GetFirstDerivative (Real fTime) const;
    virtual Vector2<Real> GetSecondDerivative (Real fTime) const;
    virtual Vector2<Real> GetThirdDerivative (Real fTime) const;

    virtual Real GetVariation (Real fT0, Real fT1,
        const Vector2<Real>* pkP0 = 0, const Vector2<Real>* pkP1 = 0) const;

protected:
    using SingleCurve2<Real>::m_fTMin;
    using SingleCurve2<Real>::m_fTMax;

    Polynomial1<Real>* m_pkXPoly;
    Polynomial1<Real>* m_pkYPoly;
    Polynomial1<Real> m_kXDer1, m_kYDer1;
    Polynomial1<Real> m_kXDer2, m_kYDer2;
    Polynomial1<Real> m_kXDer3, m_kYDer3;
};

typedef PolynomialCurve2<float> PolynomialCurve2f;
typedef PolynomialCurve2<double> PolynomialCurve2d;

}

#endif
