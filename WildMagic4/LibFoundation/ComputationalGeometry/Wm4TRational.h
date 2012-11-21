// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4TRATIONAL_H
#define WM4TRATIONAL_H

#include "Wm4FoundationLIB.h"
#include "Wm4TInteger.h"

namespace Wm4
{

// N is the number of 32-bit words per TInteger numerator/denominator
template <int N>
class TRational
{
public:
    // construction
    TRational ();  // default rational is 0/1
    TRational (const TRational& rkR);
    TRational (const TInteger<N>& rkNumer);
    TRational (const TInteger<N>& rkNumer, const TInteger<N>& rkDenom);

    // construction converters
    TRational (int iNumer);
    TRational (int iNumer, int iDenom);
    TRational (float fValue);
    TRational (double dValue);

    // member access
    TInteger<N>& Numer ();
    TInteger<N>& Denom ();
    const TInteger<N>& Numer () const;
    const TInteger<N>& Denom () const;

    // assignment
    TRational& operator= (const TRational& rkR);

    // comparison
    bool operator== (const TRational& rkR) const;
    bool operator!= (const TRational& rkR) const;
    bool operator<= (const TRational& rkR) const;
    bool operator<  (const TRational& rkR) const;
    bool operator>= (const TRational& rkR) const;
    bool operator>  (const TRational& rkR) const;

    // arithmetic operations
    TRational operator+ (const TRational& rkR) const;
    TRational operator- (const TRational& rkR) const;
    TRational operator* (const TRational& rkR) const;
    TRational operator/ (const TRational& rkR) const;
    TRational operator- () const;

    // arithmetic updates
    TRational& operator+= (const TRational& rkR);
    TRational& operator-= (const TRational& rkR);
    TRational& operator*= (const TRational& rkR);
    TRational& operator/= (const TRational& rkR);

    // conversions to float and double
    void ConvertTo (float& rfValue) const;
    void ConvertTo (double& rdValue) const;

    // compute the absolute value of the rational number
    TRational Abs () const;

private:
    // miscellaneous utilities
    void EliminatePowersOfTwo ();

    static void GetPositiveFloat (const TInteger<N>& rkDenom,
        TInteger<N>& rkQuo, TInteger<N>& rkRem, int iBlock,
        unsigned int& ruiExponent, unsigned int& ruiMantissa);

    static void GetPositiveDouble (const TInteger<N>& rkDenom,
        TInteger<N>& rkQuo, TInteger<N>& rkRem, int iBlock,
        unsigned int& ruiExponent, unsigned int& ruiMantissaHi,
        unsigned int& ruiMantissaLo);

    TInteger<N> m_kNumer, m_kDenom;
};

template <int N>
TRational<N> operator+ (const TInteger<N>& rkI, const TRational<N>& rkR);

template <int N>
TRational<N> operator- (const TInteger<N>& rkI, const TRational<N>& rkR);

template <int N>
TRational<N> operator* (const TInteger<N>& rkI, const TRational<N>& rkR);

template <int N>
TRational<N> operator/ (const TInteger<N>& rkI, const TRational<N>& rkR);

#include "Wm4TRational.inl"

}

#endif
