// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CONTELLIPSOID3MINCR_H
#define WM4CONTELLIPSOID3MINCR_H

#include "Wm4FoundationLIB.h"
#include "Wm4Matrix3.h"

namespace Wm4
{

// Compute the minimum-volume ellipsoid, (X-C)^T R D R^T (X-C) = 1, given the
// center C and orientation matrix R.  The columns of R are the axes of the
// ellipsoid.  The algorithm computes the diagonal matrix D.  The minimum
// volume is (4*pi/3)/sqrt(D[0]*D[1]*D[2]), where D = diag(D[0],D[1],D[2]).
// The problem is equivalent to maximizing the product D[0]*D[1]*D[2] for a
// given C and R, and subject to the constraints
//   (P[i]-C)^T R D R^T (P[i]-C) <= 1
// for all input points P[i] with 0 <= i < N.  Each constraint has the form
//   A[0]*D[0] + A[1]*D[1] + A[2]*D[2] <= 1
// where A[0] >= 0, A[1] >= 0, and A[2] >= 0.

template <class Real>
class WM4_FOUNDATION_ITEM ContEllipsoid3MinCR
{
public:
    ContEllipsoid3MinCR (int iQuantity, const Vector3<Real>* akPoint,
        const Vector3<Real>& rkC, const Matrix3<Real>& rkR, Real afD[3]);

private:
    void FindEdgeMax (int iQuantity, Real* afA, Real* afB, Real* afC,
        int& riPlane0, int& riPlane1, Real& rfX0, Real& rfY0, Real& rfZ0);

    void FindFacetMax (int iQuantity, Real* afA, Real* afB, Real* afC,
        int& riPlane0, Real& rfX0, Real& rfY0, Real& rfZ0);

    void MaxProduct (int iQuantity, Real* afA, Real* afB, Real* afC,
        Real& rfX, Real& rfY, Real& rfZ);
};

typedef ContEllipsoid3MinCR<float> ContEllipsoid3MinCRf;
typedef ContEllipsoid3MinCR<double> ContEllipsoid3MinCRd;

}

#endif
