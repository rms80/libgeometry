// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4ApprPolyFit2.h"
#include "Wm4LinearSystem.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
Real* PolyFit2 (int iSamples, const Real* afX, const Real* afW, int iXDegree)
{
    int iQuantity = iXDegree + 1;
    Real* afCoeff = WM4_NEW Real[iQuantity];

    int i0, i1, iS;

    // powers of x
    Real** aafXP;
    Allocate<Real>(2*iXDegree+1,iSamples,aafXP);
    for (iS = 0; iS < iSamples; iS++)
    {
        aafXP[iS][0] = (Real)1.0;
        for (i0 = 1; i0 <= 2*iXDegree; i0++)
        {
            aafXP[iS][i0] = afX[iS]*aafXP[iS][i0-1];
        }
    }

    // Vandermonde matrix and right-hand side of linear system
    GMatrix<Real> kA(iQuantity,iQuantity);
    Real* afB = WM4_NEW Real[iQuantity];

    for (i0 = 0; i0 <= iXDegree; i0++)
    {
        Real fSum = (Real)0.0;
        for (iS = 0; iS < iSamples; iS++)
        {
            fSum += afW[iS]*aafXP[iS][i0];
        }

        afB[i0] = fSum;

        for (i1 = 0; i1 <= iXDegree; i1++)
        {
            fSum = (Real)0.0;
            for (iS = 0; iS < iSamples; iS++)
            {
                fSum += aafXP[iS][i0+i1];
            }

            kA(i0,i1) = fSum;
        }
    }

    // solve for the polynomial coefficients
    bool bHasSolution = LinearSystem<Real>().Solve(kA,afB,afCoeff);
    assert(bHasSolution);
    (void)bHasSolution;  // avoid compiler warning in release build
    WM4_DELETE[] afB;
    Deallocate<Real>(aafXP);

    return afCoeff;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
float* PolyFit2<float> (int, const float*, const float*, int);

template WM4_FOUNDATION_ITEM
double* PolyFit2<double> (int, const double*, const double*, int);
//----------------------------------------------------------------------------
}
