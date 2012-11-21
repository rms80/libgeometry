// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4QUADTOQUADTRANSFORMS_H
#define WM4QUADTOQUADTRANSFORMS_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"
#include "Wm4Matrix2.h"

namespace Wm4
{

//----------------------------------------------------------------------------
// Homogeneous mapping of quadrilateral <p00,p10,p11,p01> to square [0,1]^2.
// The quadrilateral points are ordered counterclockwise and map onto the
// corners (0,0), (1,0), (1,1), and (0,1), respectively.

template <class Real>
class WM4_FOUNDATION_ITEM HmQuadToSqr
{
public:
    HmQuadToSqr (const Vector2<Real>& rkP00, const Vector2<Real>& rkP10,
        const Vector2<Real>& rkP11, const Vector2<Real>& rkP01);

    Vector2<Real> Transform (const Vector2<Real>& rkP);

protected:
    Vector2<Real> m_kT, m_kG, m_kD;
    Matrix2<Real> m_kM;
};

//----------------------------------------------------------------------------
// Homogeneous mapping of square [0,1]^2 to quadrilateral <p00,p10,p11,p01>.
// The quadrilateral points are ordered counterclockwise and map onto the
// corners (0,0), (1,0), (1,1), and (0,1), respectively.

template <class Real>
class WM4_FOUNDATION_ITEM HmSqrToQuad
{
public:
    HmSqrToQuad (const Vector2<Real>& rkP00, const Vector2<Real>& rkP10,
        const Vector2<Real>& rkP11, const Vector2<Real>& rkP01);

    Vector2<Real> Transform (const Vector2<Real>& rkP);

protected:
    Vector2<Real> m_kT, m_kG, m_kD;
    Matrix2<Real> m_kM;
};

//----------------------------------------------------------------------------
// Bilinear mapping of quadrilateral <p00,p10,p11,p01> to square [0,1]^2.
// The quadrilateral points are ordered counterclockwise and map onto the
// corners (0,0), (1,0), (1,1), and (0,1), respectively.
//
// If p is strictly inside the quadrilateral, then
//   p = (1-t)*[(1-s)*p00+s*p10]+t*[(1-s)*p01+s*p11]
//     = p00 + s*(p10-p00) + t*(p01-p00) + s*t*(p11+p00-p01-p10)
//   (0,0) = (p00-p) + s*(p10-p00) + t*(p01-p00) + s*t*(p11+p00-p10-p01)
//         = A + s*B + t*C + s*t*D (this equation defines A, B, C, D)
//
// Define K((x1,y1),(x2,y2)) = x1*y2-x2*y1.  Note that K(U,V) = -K(V,U).
//   0 = K(A,C) + s*K(B,C) + s*t*K(D,C) = ac + bc*s - cd*s*t
//   0 = K(A,B) + t*K(C,B) + s*t*K(D,B) = ab - bc*t - bd*s*t
// where ac = K(A,C), bc = K(B,C), cd = K(C,D), ab = K(A,B), and bd = K(B,D).
// Also, bc is not zero.  If bc is zero (nearly zero), then B and C are
// parallel (nearly parallel) and the quadrilateral is degenerate (nearly
// degenerate).
//
// The second equation is solved for
//   t = ab/(bc + bd*s)
// Replace in the first equation to obtain
//   0 = ac + bc*s - cd*s*(ab/(bc+bd*s))
// Multiply by bc+bd*s to obtain the quadratic equation
//   0 = (ac+bc*s)*(bc+bd*s)-ab*cd*s
//     = ac*bc+(bc^2+ac*bd-ab*cd)*s+bc*bd*s^2

template <class Real>
class WM4_FOUNDATION_ITEM BiQuadToSqr
{
public:
    BiQuadToSqr (const Vector2<Real>& rkP00, const Vector2<Real>& rkP10,
        const Vector2<Real>& rkP11, const Vector2<Real>& rkP01);

    Vector2<Real> Transform (const Vector2<Real>& rkP);

protected:
    static Real Deviation (const Vector2<Real>& rkSPoint);

    Vector2<Real> m_kP00, m_kB, m_kC, m_kD;
    Real m_fBC, m_fBD, m_fCD;
};

//----------------------------------------------------------------------------
// Bilinear mapping of square [0,1]^2 to quadrilateral <p00,p10,p11,p01>.
// The quadrilateral points are ordered counterclockwise and map onto the
// corners (0,0), (1,0), (1,1), and (0,1), respectively.
//
// Let be in the square.  The corresponding quadrilateral point is
// p = (1-t)*[(1-s)*p00+s*p10]+t*[(1-s)*p01+s*p11].

template <class Real>
class WM4_FOUNDATION_ITEM BiSqrToQuad
{
public:
    BiSqrToQuad (const Vector2<Real>& rkP00, const Vector2<Real>& rkP10,
        const Vector2<Real>& rkP11, const Vector2<Real>& rkP01);

    Vector2<Real> Transform (const Vector2<Real>& rkP);

protected:
    Vector2<Real> m_kS00, m_kS01, m_kS10, m_kS11;
};

typedef HmQuadToSqr<float> HmQuadToSqrf;
typedef HmQuadToSqr<double> HmQuadToSqrd;
typedef HmSqrToQuad<float> HmSqrToQuadf;
typedef HmSqrToQuad<double> HmSqrToQuadd;
typedef BiQuadToSqr<float> BiQuadToSqrf;
typedef BiQuadToSqr<double> BiQuadToSqrd;
typedef BiSqrToQuad<float> BiSqrToQuadf;
typedef BiSqrToQuad<double> BiSqrToQuadd;

}

#endif
