// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4QUADRATIC3_H
#define WM4QUADRATIC3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Matrix3.h"

namespace Wm4
{

template <class Real>
class Quadratic3
{
public:
    // A quadratic function of three variables x, y, and z is
    //   Q(x,y,z) = a0 + a1*x + a2*y + a3*z + a4*x*x + a5*x*y + a6*x*z +
    //              a7*y*y + a8*y*z + a9*z*z
    // This class is a simple wrapper for storing the coefficients and for
    // evaluating the function.

    // Construction and destruction.  The default constructor initializes the
    // coefficients to zero.
    Quadratic3 ();
    Quadratic3 (Real fA0, Real fA1, Real fA2, Real fA3, Real fA4, Real fA5,
        Real fA6, Real fA7, Real fA8, Real fA9);
    Quadratic3 (const Real afA[10]);

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
    Real Z() const;         // a3
    Real& Z();
    Real XX() const;        // a4
    Real& XX();
    Real XY() const;        // a5
    Real& XY();
    Real XZ() const;        // a6
    Real& XZ();
    Real YY() const;        // a7
    Real& YY();
    Real YZ() const;        // a8
    Real& YZ();
    Real ZZ() const;        // a9
    Real& ZZ();

    // Access coefficients by variable powers.  The valid powers are
    // (xorder,yorder,zorder) in {(0,0,0),(1,0,0),(0,1,0),(0,0,1),(2,0,0),
    // (1,1,0),(1,0,1),(0,2,0),(0,1,1),(0,0,2)}.  The Set method ignores an
    // invalid triple.  The Get method returns zero on an invalid triple.
    void Set (int iXOrder, int iYOrder, int iZOrder, Real fCoeff);
    Real Get (int iXOrder, int iYOrder, int iZOrder) const;

    // Evaluate Q(x,y,z).
    Real operator() (Real fX, Real fY, Real fZ) const;
    Real operator() (const Vector3<Real>& rkP) const;

    // Transformations intended to modify the level sets of Q(X) = 0.  A
    // level set point X may be transformed by Y = R*S*X+T, where S is a
    // diagonal matrix of positive scales, R is a rotation matrix, and T is
    // a translation vector.  The new level set has points Y which are the
    // solution to P(Y) = 0, where
    //   P(Y) = Q(X) = Q(Inverse(S)*Transpose(R)*(Y-T))
    // The following functions compute the coefficients for P(Y), where the
    // scale matrix is represented as a 3-tuple.

    // Compute P(Y) = Q(Y-T).
    Quadratic3 Translate (const Vector3<Real>& rkTrn) const;

    // Compute P(Y) = Q(Transpose(R)*Y).
    Quadratic3 Rotate (const Matrix3<Real>& rkRot) const;

    // Compute P(Y) = Q(Inverse(S)*Y).
    Quadratic3 Scale (const Vector3<Real>& rkScale) const;

    // TO DO.  Add classification code to decide what type of set is defined
    // by Q(x,y,z) = 0.  Add factorization code.

private:
    Real m_afCoeff[10];
};

#include "Wm4Quadratic3.inl"

typedef Quadratic3<float> Quadratic3f;
typedef Quadratic3<double> Quadratic3d;

}

#endif
