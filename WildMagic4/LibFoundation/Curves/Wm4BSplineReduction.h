// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4BSPLINEREDUCTION_H
#define WM4BSPLINEREDUCTION_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real, class TVector>
class WM4_FOUNDATION_ITEM BSplineReduction
{
public:
    // The input quantity iQuantity must be 2 or larger.  The caller is
    // responsible for deleting the input array akCtrl and the output array
    // akCtrlOut.  The degree iDegree of the input curve must satisfy the
    // condition 1 <= iDegree <= iQuantity-1.  The degree of the output
    // curve is the same as that of the input curve.  The value fFraction
    // must be in [0,1].  If the fraction is 1, the output curve is identical
    // to the input curve.  If the fraction is too small to produce a valid
    // number of control points, the output control quantity is chosen to be
    // riQuantityOut = iDegree+1.

    BSplineReduction (int iQuantity, const TVector* akCtrl, int iDegree,
        Real fFraction, int& riQuantityOut, TVector*& rakCtrlOut);

    ~BSplineReduction ();

private:
    Real MinSupport (int iBasis, int i) const;
    Real MaxSupport (int iBasis, int i) const;
    Real F (int iBasis, int i, int j, Real fTime);
    static Real Integrand (Real fTime, void* pvThis);

    int m_iDegree;
    int m_aiQuantity[2];
    int m_aiNumKnots[2];  // N+D+2
    Real* m_aafKnot[2];

    // for the integration-based least-squares fitting
    int m_aiBasis[2], m_aiIndex[2];
};

typedef BSplineReduction<float,Vector2f> BSplineReduction2f;
typedef BSplineReduction<double,Vector2d> BSplineReduction2d;
typedef BSplineReduction<float,Vector3f> BSplineReduction3f;
typedef BSplineReduction<double,Vector3d> BSplineReduction3d;

}

#endif
