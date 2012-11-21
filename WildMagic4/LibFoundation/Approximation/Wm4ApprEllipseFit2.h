// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4APPRELLIPSEFIT2_H
#define WM4APPRELLIPSEFIT2_H

// The ellipse in general form is  X^t A X + B^t X + C = 0 where A is a
// positive definite 2x2 matrix, B is a 2x1 vector, C is a scalar, and X is
// a 2x1 vector X.  Completing the square, (X-U)^t A (X-U) = U^t A U - C
// where U = -0.5 A^{-1} B.  Define M = A/(U^t A U - C).  The ellipse is
// (X-U)^t M (X-U) = 1.  Factor M = R^t D R where R is orthonormal and D is
// diagonal with positive diagonal terms.  The ellipse in factored form is
// (X-U)^t R^t D^t R (X-U) = 1.
//
// Find the least squares fit of a set of N points P[0] through P[N-1].
// Return value is the least-squares energy function at (U,R,D).

#include "Wm4FoundationLIB.h"
#include "Wm4Matrix2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM EllipseFit2
{
public:
    EllipseFit2 (int iQuantity, const Vector2<Real>* akPoint,
        Vector2<Real>& rkU, Matrix2<Real>& rkR, Real afD[2], Real& rfError);

private:
    static void InitialGuess (int iQuantity, const Vector2<Real>* akPoint,
        Vector2<Real>& rkU, Matrix2<Real>& rkR, Real afD[2]);

    static Real Energy (const Real* afV, void* pvData);

    int m_iQuantity;
    const Vector2<Real>* m_akPoint;
    Vector2<Real>* m_akTemp;
};

typedef EllipseFit2<float> EllipseFit2f;
typedef EllipseFit2<double> EllipseFit2d;

}

#endif
