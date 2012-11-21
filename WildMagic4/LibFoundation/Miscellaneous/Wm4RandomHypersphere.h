// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4RANDOMHYPERSPHERE_H
#define WM4RANDOMHYPERSPHERE_H

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

namespace Wm4
{

// Generate N random points on the hypersphere in D-space,
//
//     x_1^2 + ... + x_D^2 = 1
//
// The function selects a random angle in [0,2*pi) and partitions
// the equation into
//
//     x_1^2 + ... + x_{D/2}^2 = (cos(A))^2
//
// and
//
//     x_{D/2+1}^2 + ... + x_D^2 = (sin(A))^2
//
// The function initializes all components of P to 1.  The partitioned
// components are updated as x_i *= cos(A) for 0 <= i < D/2 and
// x_i *= sin(A) for D/2 <= i < D.  The function is recursively called
// on the partitioned components.

WM4_FOUNDATION_ITEM void RandomPointOnHypersphere (int iDimension,
    double* adPoint);

// An attempt to determine the uniformity of N randomly generated
// points P[0] through P[N-1] on the hypersphere.  Select a positive
// angle.  For each point P[i] count the number H[i] of random points
// P[j] which lie in the cone with axis P[i] and specified angle.  For
// N suitably large, H[i] should be constant for all i.  To be really
// sure of the uniformity, you should look at other cones whose axes
// are not the sample points.  However, this requires generating random
// points to get the axes and, well, you can see the problem...

WM4_FOUNDATION_ITEM void Histogram (int iDimension, double dAngle,
    int iQuantity, double** aadPoint, int* aiHistogram);

//An example of how to use these functions.
//
//void TestRandomHyperspherePoints ()
//{
//    const int iQuantity = 4096;
//    const int iDimension = 3;
//    int i;
//
//    double** aadPoint = WM4_NEW double*[iQuantity];
//    aadPoint[0] = WM4_NEW double[iQuantity*iDimension];
//    for (i = 1; i < iQuantity; i++)
//        aadPoint[i] = &aadPoint[0][iDimension*i];
//
//    for (i = 0; i < iQuantity; i++)
//        RandomPointOnHypersphere(iDimension,aadPoint[i]);
//
//    int* aiHistogram = WM4_NEW int[iQuantity];
//    double dAngle = 0.5;
//    Histogram(iDimension,dAngle,iQuantity,aadPoint,aiHistogram);
//
//    ofstream ostr("histo.txt");
//    for (i = 0; i < iQuantity; i++)
//        ostr << i << ' ' << aiHistogram[i] << endl;
//
//    WM4_DELETE[] aiHistogram;
//    WM4_DELETE[] aadPoint[0];
//    WM4_DELETE[] aadPoint;
//}
}

#endif
