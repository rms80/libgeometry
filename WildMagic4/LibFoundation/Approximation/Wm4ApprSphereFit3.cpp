// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4ApprSphereFit3.h"
#include "Wm4ApprQuadraticFit3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
bool SphereFit3 (int iQuantity, const Vector3<Real>* akPoint,
    int iMaxIterations, Sphere3<Real>& rkSphere,
    bool bInitialCenterIsAverage)
{
    // compute the average of the data points
    Vector3<Real> kAverage = akPoint[0];
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
        rkSphere.Center = kAverage;
    }
    else
    {
        QuadraticSphereFit3<Real>(iQuantity,akPoint,rkSphere.Center,
            rkSphere.Radius);
    }

    int i1;
    for (i1 = 0; i1 < iMaxIterations; i1++)
    {
        // update the iterates
        Vector3<Real> kCurrent = rkSphere.Center;

        // compute average L, dL/da, dL/db, dL/dc
        Real fLAverage = (Real)0.0;
        Vector3<Real> kDerLAverage = Vector3<Real>::ZERO;
        for (i0 = 0; i0 < iQuantity; i0++)
        {
            Vector3<Real> kDiff = akPoint[i0] - rkSphere.Center;
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

        rkSphere.Center = kAverage + fLAverage*kDerLAverage;
        rkSphere.Radius = fLAverage;

        Vector3<Real> kDiff = rkSphere.Center - kCurrent;
        if (Math<Real>::FAbs(kDiff.X()) <= Math<Real>::ZERO_TOLERANCE
        &&  Math<Real>::FAbs(kDiff.Y()) <= Math<Real>::ZERO_TOLERANCE
        &&  Math<Real>::FAbs(kDiff.Z()) <= Math<Real>::ZERO_TOLERANCE)
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
bool SphereFit3<float> (int, const Vector3<float>*, int, Sphere3<float>&,
    bool);

template WM4_FOUNDATION_ITEM
bool SphereFit3<double> (int, const Vector3<double>*, int, Sphere3<double>&,
    bool);
//----------------------------------------------------------------------------
}
