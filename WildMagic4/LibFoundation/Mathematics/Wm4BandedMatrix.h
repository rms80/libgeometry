// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4BANDEDMATRIX_H
#define WM4BANDEDMATRIX_H

#include "Wm4FoundationLIB.h"
#include "Wm4Math.h"

namespace Wm4
{

template <class Real>
class BandedMatrix
{
public:
    BandedMatrix (int iSize, int iLBands, int iUBands);
    BandedMatrix (const BandedMatrix& rkM);
    ~BandedMatrix ();

    BandedMatrix& operator= (const BandedMatrix& rkM);

    int GetSize () const;
    int GetLBands () const;
    int GetUBands () const;

    Real* GetDBand ();
    const Real* GetDBand () const;

    int GetLBandMax (int i) const;  // LBand(i):  0 <= index < LBandMax
    Real* GetLBand (int i);
    const Real* GetLBand (int i) const;

    int GetUBandMax (int i) const;  // UBand(i):  0 <= index < UBandMax
    Real* GetUBand (int i);
    const Real* GetUBand (int i) const;

    Real& operator() (int iRow, int iCol);
    Real operator() (int iRow, int iCol) const;

    void SetZero ();
    void SetIdentity ();

    // Factor the square banded matrix A into A = L*L^T, where L is a
    // lower-triangular matrix (L^T is an upper-triangular matrix).
    // This is an LU decomposition that allows for stable inversion
    // of A to solve A*X = B.  The return value is 'true' iff the
    // factorizing is successful (L is invertible).  If successful, A
    // contains the Cholesky factorization (L in the lower-triangular part
    // of A and/ L^T in the upper-triangular part of A).
    bool CholeskyFactor ();

    // Solve the linear system A*X = B, where A is an NxN banded matrix and
    // B is an Nx1 vector.  The unknown X is also Nx1.  The input to this
    // function is B.  The output X is computed and stored in B.  The return
    // value is 'true' iff the system has a solution.  The matrix A and the
    // vector B are both modified by this function.  If successful, A contains
    // the Cholesky factorization (L in the lower-triangular part of A and
    // L^T in the upper-triangular part of A).
    bool SolveSystem (Real* afB);

    // Solve the linear system A*X = B, where A is an NxN banded matrix and
    // B is an NxM matrix.  The unknown X is also NxM.  The input to this
    // function is B.  The output X is computed and stored in B.  The return
    // value is 'true' iff the system has a solution.  The matrix A and the
    // vector B are both modified by this function.  If successful, A contains
    // the Cholesky factorization (L in the lower-triangular part of A and
    // L^T in the upper-triangular part of A).
    bool SolveSystem (Real** aafB, int iNumBColumns);

private:
    // The linear system is L*U*X = B, where A = L*U and U = L^T,  Reduce this
    // to U*X = L^{-1}*B.  The return value is 'true' iff the operation is
    // successful.
    bool SolveLower (Real* afData) const;

    // The linear system is U*X = L^{-1}*B.  Reduce this to
    // X = U^{-1}*L^{-1}*B.  The return value is 'true' iff the operation is
    // successful.
    bool SolveUpper (Real* afData) const;

    // The linear system is L*U*X = B, where A = L*U and U = L^T,  Reduce this
    // to U*X = L^{-1}*B.  The return value is 'true' iff the operation is
    // successful.
    bool SolveLower (Real** aafData, int iNumBColumns) const;

    // The linear system is U*X = L^{-1}*B.  Reduce this to
    // X = U^{-1}*L^{-1}*B.  The return value is 'true' iff the operation is
    // successful.
    bool SolveUpper (Real** aafData, int iNumBColumns) const;

    void Allocate ();
    void Deallocate ();

    int m_iSize, m_iLBands, m_iUBands;
    Real* m_afDBand;
    Real** m_aafLBand;
    Real** m_aafUBand;
};

#include "Wm4BandedMatrix.inl"

typedef BandedMatrix<float> BandedMatrixf;
typedef BandedMatrix<double> BandedMatrixd;

}

#endif
