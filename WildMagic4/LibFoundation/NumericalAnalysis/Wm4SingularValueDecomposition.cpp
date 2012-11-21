// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4SingularValueDecomposition.h"
#include "Wm4Eigen.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
SingularValueDecomposition<Real>::SingularValueDecomposition (
    const GMatrix<Real>& rkM, GMatrix<Real>& rkL, GMatrix<Real>& rkD,
    GMatrix<Real>& rkRTranspose)
{
    // TODO.  Implement other QR factorizations and SVD code from "Matrix
    // Computations", and then give the user the ability to specify which
    // methods are used here.

    int iNumRows = rkM.GetRows();
    int iNumCols = rkM.GetColumns();
    assert(iNumRows >= iNumCols);

    rkL.SetSize(iNumRows,iNumRows);
    rkD.SetSize(iNumRows,iNumCols);
    rkRTranspose.SetSize(iNumCols,iNumCols);

    GMatrix<Real> kMTM = rkM.TransposeTimes(rkM);
    Eigen<Real> es(kMTM);
    es.DecrSortEigenStuff();
    GMatrix<Real> kV = es.GetEigenvectors();
    GMatrix<Real> kMV = rkM*kV;
    HouseholderQR(kMV,rkL,rkD);
    rkRTranspose = kV.Transpose();

    // Because Householder transformations are used, some of the diagonal
    // entries might be negative.  Make them positive and negate the signs
    // of the corresponding columns of L.
    for (int iCol = 0; iCol < iNumCols; iCol++)
    {
        if (rkD[iCol][iCol] < (Real)0)
        {
            rkD[iCol][iCol] = -rkD[iCol][iCol];
            for (int iRow = 0; iRow < iNumRows; iRow++)
            {
                rkL[iRow][iCol] = -rkL[iRow][iCol];
            }
        }
    }
}
//----------------------------------------------------------------------------
template <class Real>
SingularValueDecomposition<Real>::~SingularValueDecomposition ()
{
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> SingularValueDecomposition<Real>::HouseholderVector (
    const GVector<Real>& rkX)
{
    GVector<Real> kV = rkX;
    Real fLength = kV.Length();
    Real fBeta = rkX[0] + Math<Real>::Sign(rkX[0])*fLength;
    if (fBeta != (Real)0)
    {
        Real fInvBeta = ((Real)1)/fBeta;
        for (int i = 1; i < kV.GetSize(); i++)
        {
            kV[i] *= fInvBeta;
        }
    }
    kV[0] = (Real)1;

    return kV;
}
//----------------------------------------------------------------------------
template <class Real>
void SingularValueDecomposition<Real>::HouseholderPremultiply (
    const GVector<Real>& rkV, GMatrix<Real>& rkA)
{
    GVector<Real> kW = (((Real)-2)/rkV.SquaredLength())*(rkV*rkA);
    int iNumRows = rkA.GetRows();
    int iNumCols = rkA.GetColumns();
    for (int iRow = 0; iRow < iNumRows; iRow++)
    {
        for (int iCol = 0; iCol < iNumCols; iCol++)
        {
            rkA[iRow][iCol] += rkV[iRow]*kW[iCol];
        }
    }
}
//----------------------------------------------------------------------------
template <class Real>
void SingularValueDecomposition<Real>::HouseholderPostmultiply (
    const GVector<Real>& rkV, GMatrix<Real>& rkA)
{
    GVector<Real> kW = (((Real)-2)/rkV.SquaredLength())*(rkA*rkV);
    int iNumRows = rkA.GetRows();
    int iNumCols = rkA.GetColumns();
    for (int iRow = 0; iRow < iNumRows; iRow++)
    {
        for (int iCol = 0; iCol < iNumCols; iCol++)
        {
            rkA[iRow][iCol] += kW[iRow]*rkV[iCol];
        }
    }
}
//----------------------------------------------------------------------------
template <class Real>
void SingularValueDecomposition<Real>::HouseholderQR (
    const GMatrix<Real>& rkA, GMatrix<Real>& rkQ, GMatrix<Real>& rkR)
{
    // The matrix R gets a copy of A, and is then overwritten during the
    // algorithm with the correct entries to be upper triangular.
    rkR = rkA;
    int iNumRows = rkR.GetRows();
    int iNumCols = rkR.GetColumns();
    assert(iNumRows >= iNumCols);
    int iRow, iCol;
    GVector<Real> kV(iNumRows);
    std::vector<GVector<Real> > kVSave;
    for (iCol = 0; iCol < iNumCols; iCol++)
    {
        // Create the Householder vector for the partial column of A.
        for (iRow = 0; iRow < iCol; iRow++)
        {
            kV[iRow] = (Real)0;
        }
        Real fLength = (Real)0;
        for (iRow = iCol; iRow < iNumRows; iRow++)
        {
            kV[iRow] = rkR[iRow][iCol];
            fLength += kV[iRow]*kV[iRow];
        }
        fLength = Math<Real>::Sqrt(fLength);
        Real fBeta = kV[iCol] + Math<Real>::Sign(kV[iCol])*fLength;
        if (fBeta != (Real)0)
        {
            Real fInvBeta = ((Real)1)/fBeta;
            for (int i = iCol+1; i < iNumRows; i++)
            {
                kV[i] *= fInvBeta;
            }
        }
        kV[iCol] = (Real)1;

        // Premultiply A by the V-reflection matrix.
        HouseholderPremultiply(kV,rkR);

        // Save the Householder vectors.
        kVSave.push_back(kV);
    }

    // First, make Q the identity.  Second, extract the Householder vectors
    // and premultiply by the V-reflections to build Q.
    memset((Real*)rkQ,0,rkQ.GetQuantity()*sizeof(Real));
    for (iRow = 0; iRow < iNumRows; iRow++)
    {
        rkQ[iRow][iRow] = (Real)1;
    }

    for (iCol = iNumCols - 1; iCol >= 0; iCol--)
    {
        // Get the Householder vector.
        kV = kVSave[iCol];

        // Premultiply Q by the V-reflection matrix.
        HouseholderPremultiply(kV,rkQ);
    }
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class SingularValueDecomposition<float>;

template WM4_FOUNDATION_ITEM
class SingularValueDecomposition<double>;
//----------------------------------------------------------------------------
}
