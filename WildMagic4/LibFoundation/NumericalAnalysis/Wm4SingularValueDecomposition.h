// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4SINGULARVALUEDECOMPOSITION
#define WM4SINGULARVALUEDECOMPOSITION

#include "Wm4FoundationLIB.h"
#include "Wm4GMatrix.h"

namespace Wm4
{
template <class Real>
class WM4_FOUNDATION_ITEM SingularValueDecomposition
{
public:
    // Singular value decomposition, M = L*D*Transpose(R), where L and R are
    // orthogonal and D is a diagonal matrix whose diagonal entries are
    // nonnegative.  Observe that M is m-by-n with m >= n, L is m-by-m, R is
    // n-by-n, and D is m-by-n; that is, M and D are the same size and not
    // necessarily square.
    SingularValueDecomposition (const GMatrix<Real>& rkM, GMatrix<Real>& rkL,
        GMatrix<Real>& rkD, GMatrix<Real>& rkRTranspose);

    ~SingularValueDecomposition ();

private:
    // === Matrix Computations Algorithm 5.1.1 (house)
    // Compute V so that V[0] = 1, and (I-2*V*V^T/V^T*V)*X is zero in all
    // but the first component.  Return V.
    static GVector<Real> HouseholderVector (const GVector<Real>& rkX);

    // === Matrix Computations Algorithm 5.1.2 (row.house)
    // Overwrite A with (I-2*V*V^T/V^T*V)*A.
    static void HouseholderPremultiply (const GVector<Real>& rkV,
        GMatrix<Real>& rkA);

    // === Matrix Computations Algorithm 5.1.3 (col.house)
    // Overwrite A with A*(I-2*V*V^T/V^T*V).
    static void HouseholderPostmultiply (const GVector<Real>& rkV,
        GMatrix<Real>& rkA);

    // === Matrix Computations Algorithm 5.2.1
    // Factor A = Q*R with Q orthogonal and R upper triangular.
    static void HouseholderQR (const GMatrix<Real>& rkA,
        GMatrix<Real>& rkQ, GMatrix<Real>& rkR);
};

typedef SingularValueDecomposition<float> SingularValueDecompositionf;
typedef SingularValueDecomposition<double> SingularValueDecompositiond;

}

#endif
