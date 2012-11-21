// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4ApprPolyFit4.h"
#include "Wm4LinearSystem.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
Real* PolyFit4 (int iSamples, const Real* afX, const Real* afY,
    const Real* afZ, const Real* afW, int iXDegree, int iYDegree,
    int iZDegree)
{
    int iXBound = iXDegree + 1;
    int iYBound = iYDegree + 1;
    int iZBound = iZDegree + 1;
    int iQuantity = iXBound*iYBound*iZBound;
    Real* afCoeff = WM4_NEW Real[iQuantity];

    int i0, j0, k0, i1, j1, k1, iS;

    // powers of x, y, z
    Real** aafXP;
    Real** aafYP;
    Real** aafZP;
    Allocate<Real>(2*iXDegree+1,iSamples,aafXP);
    Allocate<Real>(2*iYDegree+1,iSamples,aafYP);
    Allocate<Real>(2*iZDegree+1,iSamples,aafZP);
    for (iS = 0; iS < iSamples; iS++)
    {
        aafXP[iS][0] = (Real)1.0;
        for (i0 = 1; i0 <= 2*iXDegree; i0++)
        {
            aafXP[iS][i0] = afX[iS]*aafXP[iS][i0-1];
        }

        aafYP[iS][0] = (Real)1.0;
        for (j0 = 1; j0 <= 2*iYDegree; j0++)
        {
            aafYP[iS][j0] = afY[iS]*aafYP[iS][j0-1];
        }

        aafZP[iS][0] = (Real)1.0;
        for (k0 = 1; k0 <= 2*iZDegree; k0++)
        {
            aafZP[iS][k0] = afZ[iS]*aafZP[iS][k0-1];
        }
    }

    // Vandermonde matrix and right-hand side of linear system
    GMatrix<Real> kA(iQuantity,iQuantity);
    Real* afB = WM4_NEW Real[iQuantity];

    for (k0 = 0; k0 <= iZDegree; k0++)
    {
        for (j0 = 0; j0 <= iYDegree; j0++)
        {
            for (i0 = 0; i0 <= iXDegree; i0++)
            {
                int iIndex0 = i0+iXBound*(j0+iYBound*k0);
                Real fSum = (Real)0.0;
                for (iS = 0; iS < iSamples; iS++)
                {
                    fSum += afW[iS] * aafXP[iS][i0] * aafYP[iS][j0] *
                        aafZP[iS][k0];
                }
                afB[iIndex0] = fSum;

                for (k1 = 0; k1 <= iZDegree; k1++)
                {
                    for (j1 = 0; j1 <= iYDegree; j1++)
                    {
                        for (i1 = 0; i1 <= iXDegree; i1++)
                        {
                            int iIndex1 = i1+iXBound*(j1+iYBound*k1);
                            fSum = (Real)0.0;
                            for (iS = 0; iS < iSamples; iS++)
                            {
                                fSum += aafXP[iS][i0+i1] * aafYP[iS][j0+j1] *
                                    aafZP[iS][k0+k1];
                            }
                            kA(iIndex0,iIndex1) = fSum;
                        }
                    }
                }
            }
        }
    }

    // solve for the polynomial coefficients
    bool bHasSolution = LinearSystem<Real>().Solve(kA,afB,afCoeff);
    assert(bHasSolution);
    (void)bHasSolution;  // avoid compiler warning in release build
    WM4_DELETE[] afB;
    Deallocate<Real>(aafXP);
    Deallocate<Real>(aafYP);
    Deallocate<Real>(aafZP);

    return afCoeff;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
float* PolyFit4<float> (int, const float*, const float*, const float*,
    const float*, int, int, int);

template WM4_FOUNDATION_ITEM
double* PolyFit4<double> (int, const double*, const double*, const double*,
    const double*, int, int, int);
//----------------------------------------------------------------------------
}
