// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4GVECTOR_H
#define WM4GVECTOR_H

#include "Wm4FoundationLIB.h"
#include "Wm4Math.h"

namespace Wm4
{

template <class Real>
class GVector
{
public:
    // construction
    GVector (int iSize = 0);
    GVector (int iSize, const Real* afTuple);
    GVector (const GVector& rkV);
    ~GVector ();

    // coordinate access
    void SetSize (int iSize);
    int GetSize () const;
    operator const Real* () const;
    operator Real* ();
    Real operator[] (int i) const;
    Real& operator[] (int i);

    // assignment
    GVector& operator= (const GVector& rkV);

    // comparison
    bool operator== (const GVector& rkV) const;
    bool operator!= (const GVector& rkV) const;
    bool operator<  (const GVector& rkV) const;
    bool operator<= (const GVector& rkV) const;
    bool operator>  (const GVector& rkV) const;
    bool operator>= (const GVector& rkV) const;

    // arithmetic operations
    GVector operator+ (const GVector& rkV) const;
    GVector operator- (const GVector& rkV) const;
    GVector operator* (Real fScalar) const;
    GVector operator/ (Real fScalar) const;
    GVector operator- () const;

    // arithmetic updates
    GVector& operator+= (const GVector& rkV);
    GVector& operator-= (const GVector& rkV);
    GVector& operator*= (Real fScalar);
    GVector& operator/= (Real fScalar);

    // vector operations
    Real Length () const;
    Real SquaredLength () const;
    Real Dot (const GVector& rkV) const;
    Real Normalize ();

protected:
    // support for comparisons
    int CompareArrays (const GVector& rkV) const;

    int m_iSize;
    Real* m_afTuple;
};

template <class Real>
GVector<Real> operator* (Real fScalar, const GVector<Real>& rkV);

#include "Wm4GVector.inl"

typedef GVector<float> GVectorf;
typedef GVector<double> GVectord;

}

#endif
