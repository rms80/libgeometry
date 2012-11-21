// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPTHINPLATESPLINE2_H
#define WM4INTPTHINPLATESPLINE2_H

// WARNING.  This code maps the inputs (x,y) to the unit square.  The thin
// plate spline function evaluation maps the input to the unit square and
// performs the interpolation in that space.  The idea is to keep the floating
// point numbers to order 1 for numerical stability of the algorithm. Some
// folks are not excited about this preprocessing step as it is not part of
// the classical thin plate spline algorithm.  It is easy enough for you to
// remove the remappings from the code.

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpThinPlateSpline2
{
public:
    // Construction and destruction.  Data points are of form (x,y,f(x,y)).
    // The smoothing parameter must be nonnegative.  If you want
    // IntpThinPlateSpline2 to delete the input arrays during destruction, set
    // bOwner to 'true'.  Otherwise, you own the arrays and must delete them
    // yourself.
    IntpThinPlateSpline2 (int iQuantity, Real* afX, Real* afY,
        Real* afF, Real fSmooth, bool bOwner);

    ~IntpThinPlateSpline2 ();

    // Check this after the constructor call to see if the thin plate spline
    // coefficients were successfully computed.  If so, then calls to
    // operator()(Real,Real) will work properly.
    bool IsInitialized () const;

    // Evaluate the thin plate spline interpolator.  If IsInitialized()
    // returns 'false', this operator will always return MAX_REAL.
    Real operator() (Real fX, Real fY);

private:
    static Real Kernel (Real fT);

    bool m_bInitialized;
    int m_iQuantity;

    // input data mapped to unit cube
    Real* m_afX;
    Real* m_afY;

    // thin plate spline coefficients
    Real* m_afA;
    Real m_afB[3];

    // extent of input data
    Real m_fXMin, m_fXMax, m_fXInvRange;
    Real m_fYMin, m_fYMax, m_fYInvRange;
};

typedef IntpThinPlateSpline2<float> IntpThinPlateSpline2f;
typedef IntpThinPlateSpline2<double> IntpThinPlateSpline2d;

}

#endif
