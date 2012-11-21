// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4GridGraph2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
GridGraph2<Real>::GridGraph2 (int iXSize, int iYSize)
{
    assert(iXSize > 0 && iYSize > 0);

    m_iXSize = iXSize;
    m_iYSize = iYSize;
    m_iVertexQuantity = m_iXSize*m_iYSize;
    m_akVertex = WM4_NEW Vertex[m_iVertexQuantity];

    m_iPathQuantity = 0;
    m_aiPath = WM4_NEW int[m_iVertexQuantity];
    m_aiPending = WM4_NEW int[m_iVertexQuantity];

    RelaxationCallback = 0;
    m_iNumProcessed = 0;
}
//----------------------------------------------------------------------------
template <class Real>
GridGraph2<Real>::~GridGraph2 ()
{
    WM4_DELETE[] m_akVertex;
    WM4_DELETE[] m_aiPath;
    WM4_DELETE[] m_aiPending;
}
//----------------------------------------------------------------------------
template <class Real>
int GridGraph2<Real>::GetXSize () const
{
    return m_iXSize;
}
//----------------------------------------------------------------------------
template <class Real>
int GridGraph2<Real>::GetYSize () const
{
    return m_iYSize;
}
//----------------------------------------------------------------------------
template <class Real>
int GridGraph2<Real>::GetVertexQuantity () const
{
    return m_iVertexQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
void GridGraph2<Real>::SetWeight (int iX, int iY, int iDX, int iDY,
    Real fWeight)
{
    assert (0 <= iX && iX < m_iXSize && 0 <= iY && iY < m_iYSize);
    assert (abs(iDX) <= 1 && abs(iDY) <= 1);
    m_akVertex[GetIndex(iX,iY)].SetWeight(iDX,iDY,fWeight);
}
//----------------------------------------------------------------------------
template <class Real>
Real GridGraph2<Real>::GetWeight (int iX, int iY, int iDX, int iDY) const
{
    assert (0 <= iX && iX < m_iXSize && 0 <= iY && iY < m_iYSize);
    assert (abs(iDX) <= 1 && abs(iDY) <= 1);
    return m_akVertex[GetIndex(iX,iY)].GetWeight(iDX,iDY);
}
//----------------------------------------------------------------------------
template <class Real>
void GridGraph2<Real>::ComputeMinimumWeightPath (int iX0, int iY0, int iX1,
    int iY1)
{
    m_iPathQuantity = 0;

    if (iX0 < 0 || iX0 >= m_iXSize || iY0 < 0 || iY0 >= m_iYSize
    ||  iX1 < 0 || iX1 >= m_iXSize || iY1 < 0 || iY1 >= m_iYSize)
    {
        return;
    }

    // Initialize the minimum weight estimates and the predecessors for the
    // graph vertices.
    int iPendingQuantity = m_iVertexQuantity;
    int i;
    for (i = 0; i < m_iVertexQuantity; i++)
    {
        Vertex& rkVertex = m_akVertex[i];
        rkVertex.Estimate = Math<Real>::MAX_REAL;
        rkVertex.Predecessor = -1;
        m_aiPending[i] = i;
    }
    m_akVertex[GetIndex(iX0,iY0)].Estimate = (Real)0.0;

    m_iNumProcessed = 0;
    if (RelaxationCallback)
    {
        RelaxationCallback();
    }

    while (iPendingQuantity > 0)
    {
        // ***** TO DO.  Replace this by an efficient priority queue.
        // Find the vertex with minimum estimate.
        Real fMinimum = m_akVertex[m_aiPending[0]].Estimate;
        int iMinIndex = 0;
        for (i = 1; i < iPendingQuantity; i++)
        {
            Real fValue = m_akVertex[m_aiPending[i]].Estimate;
            if (fValue < fMinimum)
            {
                fMinimum = fValue;
                iMinIndex = i;
            }
        }

        // Remove the minimum-estimate vertex from the pending array.  Keep
        // the array elements contiguous.
        int iMinVertex = m_aiPending[iMinIndex];
        iPendingQuantity--;
        if (iMinIndex != iPendingQuantity)
        {
            m_aiPending[iMinIndex] = m_aiPending[iPendingQuantity];
        }
        // ***** END TO DO.

        // Relax the paths from the minimum-estimate vertex.
        Vertex& rkMinVertex = m_akVertex[iMinVertex];
        int iXMin = GetX(iMinVertex);
        int iYMin = GetY(iMinVertex);
        for (int iDY = -1; iDY <= 1; iDY++)
        {
            for (int iDX = -1; iDX <= 1; iDX++)
            {
                if (iDX != 0 || iDY != 0)
                {
                    int iXAdj = iXMin + iDX;
                    int iYAdj = iYMin + iDY;
                    if (0 <= iXAdj && iXAdj < m_iXSize
                    &&  0 <= iYAdj && iYAdj < m_iYSize)
                    {
                        Vertex& rkAdjacent =
                            m_akVertex[GetIndex(iXAdj,iYAdj)];

                        Real fWeight = GetWeight(iXMin,iYMin,iDX,iDY);
                        Real fNewEstimate = rkMinVertex.Estimate + fWeight;
                        if (rkAdjacent.Estimate > fNewEstimate)
                        {
                            rkAdjacent.Estimate = fNewEstimate;
                            rkAdjacent.Predecessor = iMinVertex;
                        }
                    }
                }
            }
        }

        m_iNumProcessed++;
        if (RelaxationCallback)
        {
            RelaxationCallback();
        }
    }

    // Construct the path.
    int* piIndex = m_aiPath;
    i = GetIndex(iX1,iY1);
    while (i >= 0)
    {
        m_iPathQuantity++;
        *piIndex++ = i;
        i = m_akVertex[i].Predecessor;
    }
}
//----------------------------------------------------------------------------
template <class Real>
int GridGraph2<Real>::GetPathQuantity () const
{
    return m_iPathQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
void GridGraph2<Real>::GetPathPoint (int i, int& riX, int& riY) const
{
    if (0 <= i && i < m_iPathQuantity)
    {
        riX = GetX(m_aiPath[i]);
        riY = GetY(m_aiPath[i]);
    }
}
//----------------------------------------------------------------------------
template <class Real>
int GridGraph2<Real>::GetNumProcessed () const
{
    return m_iNumProcessed;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class GridGraph2<float>;

template<>
const int GridGraph2<float>::ms_aaiIndex[3][3] =
{
    {0,1,2}, {3,-1,4}, {5,6,7}
};

template WM4_FOUNDATION_ITEM
class GridGraph2<double>;

template<>
const int GridGraph2<double>::ms_aaiIndex[3][3] =
{
    {0,1,2}, {3,-1,4}, {5,6,7}
};
//----------------------------------------------------------------------------
}
