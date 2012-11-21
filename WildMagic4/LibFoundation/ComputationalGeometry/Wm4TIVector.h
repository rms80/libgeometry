// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4TIVECTOR_H
#define WM4TIVECTOR_H

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

namespace Wm4
{

template <int VSIZE>
class TIVector
{
public:
    // construction
    TIVector ();
    TIVector (const TIVector& rkV);

    // coordinate access
    operator const Integer64* () const;
    operator Integer64* ();
    Integer64 operator[] (int i) const;
    Integer64& operator[] (int i);

    // assignment
    TIVector& operator= (const TIVector& rkV);

    // comparison
    bool operator== (const TIVector& rkV) const;
    bool operator!= (const TIVector& rkV) const;
    bool operator<  (const TIVector& rkV) const;
    bool operator<= (const TIVector& rkV) const;
    bool operator>  (const TIVector& rkV) const;
    bool operator>= (const TIVector& rkV) const;

    // arithmetic operations
    TIVector operator+ (const TIVector& rkV) const;
    TIVector operator- (const TIVector& rkV) const;
    TIVector operator* (const Integer64& riI) const;
    TIVector operator/ (const Integer64& riI) const;
    TIVector operator- () const;

    // arithmetic updates
    TIVector& operator+= (const TIVector& rkV);
    TIVector& operator-= (const TIVector& rkV);
    TIVector& operator*= (const Integer64& riI);
    TIVector& operator/= (const Integer64& riI);

    // vector operations
    Integer64 SquaredLength () const;
    Integer64 Dot (const TIVector& rkV) const;

protected:
    // support for comparisons
    int CompareArrays (const TIVector& rkV) const;

    Integer64 m_aiTuple[VSIZE];
};

template <int VSIZE>
TIVector<VSIZE> operator* (const Integer64& riI, const TIVector<VSIZE>& rkV);

#include "Wm4TIVector.inl"

}

#endif
