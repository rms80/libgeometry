// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4TCBSPLINE3_H
#define WM4TCBSPLINE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4MultipleCurve3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM TCBSpline3 : public MultipleCurve3<Real>
{
public:
    // Construction and destruction.  TCBSpline3 accepts responsibility for
    // deleting the input arrays.
    TCBSpline3 (int iSegments, Real* afTime, Vector3<Real>* akPoint,
        Real* afTension, Real* afContinuity, Real* afBias);

    virtual ~TCBSpline3 ();

    const Vector3<Real>* GetPoints () const;
    const Real* GetTensions () const;
    const Real* GetContinuities () const;
    const Real* GetBiases () const;

    virtual Vector3<Real> GetPosition (Real fTime) const;
    virtual Vector3<Real> GetFirstDerivative (Real fTime) const;
    virtual Vector3<Real> GetSecondDerivative (Real fTime) const;
    virtual Vector3<Real> GetThirdDerivative (Real fTime) const;

protected:
    using MultipleCurve3<Real>::m_iSegments;
    using MultipleCurve3<Real>::m_afTime;
    using MultipleCurve3<Real>::GetSpeedWithData;

    void ComputePoly (int i0, int i1, int i2, int i3);

    virtual Real GetSpeedKey (int iKey, Real fTime) const;
    virtual Real GetLengthKey (int iKey, Real fT0, Real fT1) const;
    virtual Real GetVariationKey (int iKey, Real fT0, Real fT1,
        const Vector3<Real>& rkA, const Vector3<Real>& rkB) const;

    Vector3<Real>* m_akPoint;
    Real* m_afTension;
    Real* m_afContinuity;
    Real* m_afBias;
    Vector3<Real>* m_akA;
    Vector3<Real>* m_akB;
    Vector3<Real>* m_akC;
    Vector3<Real>* m_akD;

    class WM4_FOUNDATION_ITEM ThisPlusKey
    {
    public:
        ThisPlusKey (const TCBSpline3* pkThis, int iKey)
            :
            This(pkThis),
            Key(iKey)
        {
        }

        const TCBSpline3* This;
        int Key;
    };
};

typedef TCBSpline3<float> TCBSpline3f;
typedef TCBSpline3<double> TCBSpline3d;

}

#endif
