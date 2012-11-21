// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4POLYNOMIALCURVE3_H
#define WM4POLYNOMIALCURVE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Polynomial1.h"
#include "Wm4SingleCurve3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM PolynomialCurve3 : public SingleCurve3<Real>
{
public:
    // Construction and destruction.  PolynomialCurve3 accepts responsibility
    // for deleting the input polynomials.
    PolynomialCurve3 (Polynomial1<Real>* pkXPoly, Polynomial1<Real>* pkYPoly,
        Polynomial1<Real>* pkZPoly);

    virtual ~PolynomialCurve3 ();

    int GetDegree () const;
    const Polynomial1<Real>* GetXPolynomial () const;
    const Polynomial1<Real>* GetYPolynomial () const;
    const Polynomial1<Real>* GetZPolynomial () const;

    virtual Vector3<Real> GetPosition (Real fTime) const;
    virtual Vector3<Real> GetFirstDerivative (Real fTime) const;
    virtual Vector3<Real> GetSecondDerivative (Real fTime) const;
    virtual Vector3<Real> GetThirdDerivative (Real fTime) const;

    virtual Real GetVariation (Real fT0, Real fT1,
        const Vector3<Real>* pkP0 = 0, const Vector3<Real>* pkP1 = 0) const;

protected:
    using SingleCurve3<Real>::m_fTMin;
    using SingleCurve3<Real>::m_fTMax;

    Polynomial1<Real>* m_pkXPoly;
    Polynomial1<Real>* m_pkYPoly;
    Polynomial1<Real>* m_pkZPoly;
    Polynomial1<Real> m_kXDer1, m_kYDer1, m_kZDer1;
    Polynomial1<Real> m_kXDer2, m_kYDer2, m_kZDer2;
    Polynomial1<Real> m_kXDer3, m_kYDer3, m_kZDer3;
};

typedef PolynomialCurve3<float> PolynomialCurve3f;
typedef PolynomialCurve3<double> PolynomialCurve3d;

}

#endif
