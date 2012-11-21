// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4QUADRATIC2_H
#define WM4QUADRATIC2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Matrix2.h"

namespace Wm4
{

template <class Real>
class Quadratic2
{
public:
    // A quadratic function of two variables x and y is
    //   Q(x,y) = a0 + a1*x + a2*y + a3*x*x + a4*x*y + a5*y*y
    // This class is a simple wrapper for storing the coefficients and for
    // evaluating the function.

    // Construction and destruction.  The default constructor initializes the
    // coefficients to zero.
    Quadratic2 ();
    Quadratic2 (Real fA0, Real fA1, Real fA2, Real fA3, Real fA4, Real fA5);
    Quadratic2 (const Real afA[6]);

    // Access coefficients as an array.
    operator const Real* () const;
    operator Real* ();
    Real operator[] (int i) const;
    Real& operator[] (int i);

    // Access coefficients by variable name.
    Real Constant() const;  // a0
    Real& Constant();
    Real X() const;         // a1
    Real& X();
    Real Y() const;         // a2
    Real& Y();
    Real XX() const;        // a3
    Real& XX();
    Real XY() const;        // a4
    Real& XY();
    Real YY() const;        // a5
    Real& YY();

    // Access coefficients by variable powers.  The valid powers are
    // (xorder,yorder) in {(0,0),(1,0),(0,1),(2,0),(1,1),(0,2)}.  The Set
    // method ignores an invalid pair.  The Get method returns zero on an
    // invalid pair.
    void Set (int iXOrder, int iYOrder, Real fCoeff);
    Real Get (int iXOrder, int iYOrder) const;

    // Evaluate Q(x,y).
    Real operator() (Real fX, Real fY) const;
    Real operator() (const Vector2<Real>& rkP) const;

    // Transformations intended to modify the level sets of Q(X) = 0.  A
    // level set point X may be transformed by Y = R*S*X+T, where S is a
    // diagonal matrix of positive scales, R is a rotation matrix, and T is
    // a translation vector.  The new level set has points Y which are the
    // solution to P(Y) = 0, where
    //   P(Y) = Q(X) = Q(Inverse(S)*Transpose(R)*(Y-T))
    // The following functions compute the coefficients for P(Y), where the
    // scale matrix is represented as a 2-tuple.

    // Compute P(Y) = Q(Y-T).
    Quadratic2 Translate (const Vector2<Real>& rkTrn) const;

    // Compute P(Y) = Q(Transpose(R)*Y).
    Quadratic2 Rotate (const Matrix2<Real>& rkRot) const;

    // Compute P(Y) = Q(Inverse(S)*Y).
    Quadratic2 Scale (const Vector2<Real>& rkScale) const;

    // TO DO.  Add classification code to decide what type of set is defined
    // by Q(x,y) = 0.  Add factorization code.

private:
    Real m_afCoeff[6];
};

#include "Wm4Quadratic2.inl"

typedef Quadratic2<float> Quadratic2f;
typedef Quadratic2<double> Quadratic2d;

}

#endif
