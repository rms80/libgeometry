// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4ApprCircleFit2.h"
#include "Wm4ApprQuadraticFit2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
bool CircleFit2 (int iQuantity, const Vector2<Real>* akPoint,
    int iMaxIterations, Circle2<Real>& rkCircle, bool bInitialCenterIsAverage)
{
    // compute the average of the data points
    Vector2<Real> kAverage = akPoint[0];
    int i0;
    for (i0 = 1; i0 < iQuantity; i0++)
    {
        kAverage += akPoint[i0];
    }
    Real fInvQuantity = ((Real)1.0)/(Real)iQuantity;
    kAverage *= fInvQuantity;

    // initial guess
    if (bInitialCenterIsAverage)
    {
        rkCircle.Center = kAverage;
    }
    else
    {
        QuadraticCircleFit2<Real>(iQuantity,akPoint,rkCircle.Center,
            rkCircle.Radius);
    }


    int i1;
    for (i1 = 0; i1 < iMaxIterations; i1++)
    {
        // update the iterates
        Vector2<Real> kCurrent = rkCircle.Center;

        // compute average L, dL/da, dL/db
        Real fLAverage = (Real)0.0;
        Vector2<Real> kDerLAverage = Vector2<Real>::ZERO;
        for (i0 = 0; i0 < iQuantity; i0++)
        {
            Vector2<Real> kDiff = akPoint[i0] - rkCircle.Center;
            Real fLength = kDiff.Length();
            if (fLength > Math<Real>::ZERO_TOLERANCE)
            {
                fLAverage += fLength;
                Real fInvLength = ((Real)1.0)/fLength;
                kDerLAverage -= fInvLength*kDiff;
            }
        }
        fLAverage *= fInvQuantity;
        kDerLAverage *= fInvQuantity;

        rkCircle.Center = kAverage + fLAverage*kDerLAverage;
        rkCircle.Radius = fLAverage;

        Vector2<Real> kDiff = rkCircle.Center - kCurrent;
        if (Math<Real>::FAbs(kDiff.X()) <= Math<Real>::ZERO_TOLERANCE
        &&  Math<Real>::FAbs(kDiff.Y()) <= Math<Real>::ZERO_TOLERANCE)
        {
            break;
        }
    }

    return i1 < iMaxIterations;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
bool CircleFit2<float> (int, const Vector2<float>*, int, Circle2<float>&,
    bool);

template WM4_FOUNDATION_ITEM
bool CircleFit2<double> (int, const Vector2<double>*, int, Circle2<double>&,
    bool);
//----------------------------------------------------------------------------
}
