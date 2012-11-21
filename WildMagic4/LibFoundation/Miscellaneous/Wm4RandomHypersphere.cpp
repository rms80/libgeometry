// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4RandomHypersphere.h"
#include "Wm4Math.h"
using namespace Wm4;

//----------------------------------------------------------------------------
static void RecurseRandomPoint (int iDimension, double* adPoint)
{
    // select random point on circle
    double dAngle = Mathd::IntervalRandom(0.0,Mathd::TWO_PI);
    double dCos = Mathd::Cos(dAngle);
    double dSin = Mathd::Sin(dAngle);

    if (iDimension > 3)
    {
        // split components into two sets and adjust values
        int i, iHalf = iDimension/2;
        for (i = 0; i < iHalf; i++)
        {
            adPoint[i] *= dCos;
        }
        for (i = iHalf; i < iDimension; i++)
        {
            adPoint[i] *= dSin;
        }

        // recurse on each half of the components
        RecurseRandomPoint(iHalf,adPoint);
        RecurseRandomPoint(iDimension-iHalf,adPoint+iHalf);
    }
    else if (iDimension == 3)
    {
        double dValue = Mathd::SymmetricRandom();
        double dComplement = Mathd::Sqrt(Mathd::FAbs(1.0-dValue*dValue));
        adPoint[0] *= dValue;
        adPoint[1] *= dComplement*dCos;
        adPoint[2] *= dComplement*dSin;
    }
    else if (iDimension == 2)
    {
        adPoint[0] *= dCos;
        adPoint[1] *= dSin;
    }
}
//----------------------------------------------------------------------------
void Wm4::RandomPointOnHypersphere (int iDimension, double* adPoint)
{
    for (int i = 0; i < iDimension; i++)
    {
        adPoint[i] = 1.0;
    }

    RecurseRandomPoint(iDimension,adPoint);
}
//----------------------------------------------------------------------------
void Wm4::Histogram (int iDimension, double dAngle, int iQuantity,
    double** aadPoint, int* aiHistogram)
{
    // Count the number of points located in the cone of specified angle
    // about each of the samples.
    double dCos = Mathd::Cos(dAngle);

    for (int i = 0; i < iQuantity; i++)
    {
        aiHistogram[i] = 0;
        for (int j = 0; j < iQuantity; j++)
        {
            // compute dot product between points P[i] and P[j]
            double dDot = 0;
            for (int k = 0; k < iDimension; k++)
            {
                dDot += aadPoint[i][k]*aadPoint[j][k];
            }
            if (dDot >= dCos)
            {
                aiHistogram[i]++;
            }
        }
    }
}
//----------------------------------------------------------------------------
