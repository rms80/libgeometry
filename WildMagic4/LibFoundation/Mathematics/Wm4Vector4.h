// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4VECTOR4_H
#define WM4VECTOR4_H

#include "Wm4FoundationLIB.h"
#include "Wm4Math.h"

namespace Wm4
{

template <class Real>
class Vector4
{
public:
    // construction
    Vector4 ();  // uninitialized
    Vector4 (Real fX, Real fY, Real fZ, Real fW);
    Vector4 (const Real* afTuple);
    Vector4 (const Vector4& rkV);

    // coordinate access
    inline operator const Real* () const;
    inline operator Real* ();
    inline Real operator[] (int i) const;
    inline Real& operator[] (int i);
    inline Real X () const;
    inline Real& X ();
    inline Real Y () const;
    inline Real& Y ();
    inline Real Z () const;
    inline Real& Z ();
    inline Real W () const;
    inline Real& W ();

    // assignment
    inline Vector4& operator= (const Vector4& rkV);

    // comparison
    bool operator== (const Vector4& rkV) const;
    bool operator!= (const Vector4& rkV) const;
    bool operator<  (const Vector4& rkV) const;
    bool operator<= (const Vector4& rkV) const;
    bool operator>  (const Vector4& rkV) const;
    bool operator>= (const Vector4& rkV) const;

    // arithmetic operations
    inline Vector4 operator+ (const Vector4& rkV) const;
    inline Vector4 operator- (const Vector4& rkV) const;
    inline Vector4 operator* (Real fScalar) const;
    inline Vector4 operator/ (Real fScalar) const;
    inline Vector4 operator- () const;

    // arithmetic updates
    inline Vector4& operator+= (const Vector4& rkV);
    inline Vector4& operator-= (const Vector4& rkV);
    inline Vector4& operator*= (Real fScalar);
    inline Vector4& operator/= (Real fScalar);

    // vector operations
    inline Real Length () const;
    inline Real SquaredLength () const;
    inline Real Dot (const Vector4& rkV) const;
    inline Real Normalize ();

    // special vectors
    WM4_FOUNDATION_ITEM static const Vector4 ZERO;
    WM4_FOUNDATION_ITEM static const Vector4 UNIT_X;  // (1,0,0,0)
    WM4_FOUNDATION_ITEM static const Vector4 UNIT_Y;  // (0,1,0,0)
    WM4_FOUNDATION_ITEM static const Vector4 UNIT_Z;  // (0,0,1,0)
    WM4_FOUNDATION_ITEM static const Vector4 UNIT_W;  // (0,0,0,1)
    WM4_FOUNDATION_ITEM static const Vector4 ONE;     // (1,1,1,1)

private:
    // support for comparisons
    int CompareArrays (const Vector4& rkV) const;

    Real m_afTuple[4];
};

// arithmetic operations
template <class Real>
Vector4<Real> operator* (Real fScalar, const Vector4<Real>& rkV);

// debugging output
template <class Real>
std::ostream& operator<< (std::ostream& rkOStr, const Vector4<Real>& rkV);

#include "Wm4Vector4.inl"

typedef Vector4<float> Vector4f;
typedef Vector4<double> Vector4d;

}

#endif
