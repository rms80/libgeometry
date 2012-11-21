// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4MULTIPLECURVE3_H
#define WM4MULTIPLECURVE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Curve3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM MultipleCurve3 : public Curve3<Real>
{
public:
    // Construction and destruction for abstract base class.  MultipleCurve3
    // accepts responsibility for deleting the input array.
    MultipleCurve3 (int iSegments, Real* afTime);
    virtual ~MultipleCurve3 ();

    // member access
    int GetSegments () const;
    const Real* GetTimes () const;

    // length-from-time and time-from-length
    virtual Real GetLength (Real fT0, Real fT1) const;
    virtual Real GetTime (Real fLength, int iIterations = 32,
        Real fTolerance = (Real)1e-06) const;

    // support for subdivision
    virtual Real GetVariation (Real fT0, Real fT1,
        const Vector3<Real>* pkP0 = 0, const Vector3<Real>* pkP1 = 0) const;

protected:
    using Curve3<Real>::m_fTMin;
    using Curve3<Real>::m_fTMax;

    int m_iSegments;
    Real* m_afTime;

    // These quantities are allocated by GetLength when they are needed the
    // first time.  The allocations occur in InitializeLength (called by
    // GetLength), so this member function must be 'const'. In order to
    // allocate the arrays in a 'const' function, they must be declared as
    // 'mutable'.
    mutable Real* m_afLength;
    mutable Real* m_afAccumLength;

    void GetKeyInfo (Real fTime, int& riKey, Real& rfDt) const;

    void InitializeLength () const;
    virtual Real GetSpeedKey (int iKey, Real fTime) const = 0;
    virtual Real GetLengthKey (int iKey, Real fT0, Real fT1) const = 0;
    virtual Real GetVariationKey (int iKey, Real fT0, Real fT1,
        const Vector3<Real>& rkA, const Vector3<Real>& rkB) const = 0;

    static Real GetSpeedWithData (Real fTime, void* pvData);
};

typedef MultipleCurve3<float> MultipleCurve3f;
typedef MultipleCurve3<double> MultipleCurve3d;

}

#endif
