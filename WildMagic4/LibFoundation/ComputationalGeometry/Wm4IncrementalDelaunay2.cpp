// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IncrementalDelaunay2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <typename Real>
IncrementalDelaunay2<Real>::IncrementalDelaunay2 (Real fXMin, Real fYMin,
    Real fXMax, Real fYMax, Real fUncertainty)
    :
    m_fXMin(fXMin),
    m_fXMax(fXMax),
    m_fYMin(fYMin),
    m_fYMax(fYMax),
    m_fUncertainty(fUncertainty),
    m_iNumTriangles(0),
    m_aiIndex(0),
    m_aiAdjacent(0),
    m_iPathLast(-1),
    m_aiPath(0),
    m_iLastEdgeV0(-1),
    m_iLastEdgeV1(-1),
    m_iLastEdgeOpposite(-1),
    m_iLastEdgeOppositeIndex(-1)
{
    assert(m_fXMin < m_fXMax && m_fYMin < m_fYMax);
    assert((Real)0 <= m_fUncertainty && m_fUncertainty <= (Real)1);

    if (m_fUncertainty > (Real)0)
    {
        m_pkRatVertexPool = WM4_NEW std::vector<RVector>();
        m_pkRatVertexEvaluated = WM4_NEW std::vector<bool>();
    }
    else
    {
        m_pkRatVertexPool = 0;
        m_pkRatVertexEvaluated = 0;
    }

    // Create a supertriangle that contains the input rectangle.
    Real fX0 = ((Real)2)*fXMin - fXMax;
    Real fY0 = ((Real)2)*fYMin - fYMax;
    Real fX1 = ((Real)2)*fXMax - fXMin + ((Real)3)*(fYMax - fYMin);
    Real fY1 = fY0;
    Real fX2 = fX0;
    Real fY2 = ((Real)2)*fYMax - fYMin + ((Real)3)*(fXMax - fXMin);
    Vector2<Real> kSuperVertex0(fX0,fY0);
    Vector2<Real> kSuperVertex1(fX1,fY1);
    Vector2<Real> kSuperVertex2(fX2,fY2);

    // Insert the supertriangle vertices into the vertex storage.
    m_kVMap[kSuperVertex0] = 0;
    m_kVMap[kSuperVertex1] = 1;
    m_kVMap[kSuperVertex2] = 2;
    m_kVertexPool.push_back(kSuperVertex0);
    m_kVertexPool.push_back(kSuperVertex1);
    m_kVertexPool.push_back(kSuperVertex2);

    // Inert the supertriangle into the mesh.
    m_kTriangle.insert(WM4_NEW Triangle(0,1,2));
}
//----------------------------------------------------------------------------
template <typename Real>
IncrementalDelaunay2<Real>::~IncrementalDelaunay2 ()
{
    WM4_DELETE m_pkRatVertexPool;
    WM4_DELETE m_pkRatVertexEvaluated;
    WM4_DELETE[] m_aiIndex;
    WM4_DELETE[] m_aiAdjacent;
    WM4_DELETE[] m_aiPath;

    typename std::set<Triangle*>::iterator pkTIter = m_kTriangle.begin();
    for (/**/; pkTIter != m_kTriangle.end(); ++pkTIter)
    {
        WM4_DELETE *pkTIter;
    }
}
//----------------------------------------------------------------------------
template <typename Real>
int IncrementalDelaunay2<Real>::Insert (const Vector2<Real>& rkPosition)
{
    if (rkPosition[0] < m_fXMin || rkPosition[0] > m_fXMax
    ||  rkPosition[1] < m_fYMin || rkPosition[1] > m_fYMax)
    {
        // The vertex is outside the domain specified in the constructor.
        return -1;
    }

    typename VertexMap::iterator iter = m_kVMap.find(rkPosition);
    if (iter != m_kVMap.end())
    {
        // The vertex already exists, so just return its index.
        return iter->second;
    }

    // Store the position in the various pools.
    int iPosIndex = (int)m_kVertexPool.size();
    m_kVMap[rkPosition] = iPosIndex;
    m_kVertexPool.push_back(rkPosition);
    if (m_fUncertainty > (Real)0)
    {
        (*m_pkRatVertexPool).push_back(RVector());
        (*m_pkRatVertexEvaluated).push_back(false);
    }

    Triangle* pkTri = GetContainingTriangleInternal(rkPosition);
    if (!pkTri)
    {
        // All points must lie in the supertriangle, so each point must have
        // a containing triangle.
        assert(false);
        return -1;
    }

    // Locate and remove the triangles forming the insertion polygon.
    std::stack<Triangle*> kStack;
    VEManifoldMesh kPolygon(0,Edge::ECreator);
    kStack.push(pkTri);
    pkTri->OnStack = true;
    int j, iV0, iV1;
    Edge* pkEdge;
    while (!kStack.empty())
    {
        pkTri = kStack.top();
        kStack.pop();
        pkTri->OnStack = false;
        for (j = 0; j < 3; j++)
        {
            Triangle* pkAdj = pkTri->A[j];
            if (pkAdj)
            {
                // Detach triangle and adjacent triangle from each other.
                int iNullIndex = pkTri->DetachFrom(j,pkAdj);

                if (pkAdj->IsInsertionComponent(iPosIndex,rkPosition,pkTri,
                    this))
                {
                    if (!pkAdj->OnStack)
                    {
                        // Adjacent triangle inside insertion polygon.
                        kStack.push(pkAdj);
                        pkAdj->OnStack = true;
                    }
                }
                else
                {
                    // Adjacent triangle outside insertion polygon.
                    iV0 = pkTri->V[j];
                    iV1 = pkTri->V[(j+1)%3];
                    pkEdge = (Edge*)kPolygon.InsertEdge(iV0,iV1);
                    pkEdge->NullIndex = iNullIndex;
                    pkEdge->Tri = pkAdj;
                }
            }
            else
            {
                // The triangle is in the insertion polygon, but the adjacent
                // one does not exist.  This means one of two things:
                // (1) We are at an edge of the supertriangle, and that edge
                //     is part of the insertion polygon.
                // (2) We are at an edge that was recently shared by the
                //     triangle and the adjacent, but we detached those
                //     triangles from each other.  These edges should be
                //     ignored.
                iV0 = pkTri->V[j];
                if (0 <= iV0 && iV0 <= 2)
                {
                    // iV0 is a supervertex index
                    iV1 = pkTri->V[(j+1)%3];
                    if (0 <= iV1 && iV1 <= 2)
                    {
                        // iV1 is a supervertex index
                        pkEdge = (Edge*)kPolygon.InsertEdge(iV0,iV1);
                        pkEdge->NullIndex = -1;
                        pkEdge->Tri = 0;
                    }
                }
            }
        }
        m_kTriangle.erase(pkTri);
        WM4_DELETE pkTri;
    }

    // Insert the new triangles formed by the input point and the edges of
    // the insertion polygon.
    const VEManifoldMesh::EMap& rkEMap = kPolygon.GetEdges();
    assert(rkEMap.size() >= 3 && kPolygon.IsClosed());
    typename VEManifoldMesh::EMapCIterator pkEIter;
    for (pkEIter = rkEMap.begin(); pkEIter != rkEMap.end(); pkEIter++)
    {
        pkEdge = (Edge*)pkEIter->second;

        // Create and insert the new triangle.
        pkTri = WM4_NEW Triangle(iPosIndex,pkEdge->V[0],pkEdge->V[1]);
        m_kTriangle.insert(pkTri);

        // Establish the adjacency links across the polygon edge.
        pkTri->A[1] = pkEdge->Tri;
        if (pkEdge->Tri)
        {
            pkEdge->Tri->A[pkEdge->NullIndex] = pkTri;
        }

        // Update the edge's triangle pointer to point to the newly created
        // triangle.  This information is used later to establish the links
        // between the new triangles.
        pkEdge->Tri = pkTri;
    }

    // Establish the adjacency links between the new triangles.
    Edge* pkAdjEdge;
    for (pkEIter = rkEMap.begin(); pkEIter != rkEMap.end(); pkEIter++)
    {
        pkEdge = (Edge*)pkEIter->second;
        pkAdjEdge = (Edge*)pkEdge->E[0];
        pkEdge->Tri->A[0] = pkAdjEdge->Tri;
        pkAdjEdge = (Edge*)pkEdge->E[1];
        pkEdge->Tri->A[2] = pkAdjEdge->Tri;
    }

    return iPosIndex;
}
//----------------------------------------------------------------------------
template <typename Real>
int IncrementalDelaunay2<Real>::Remove (const Vector2<Real>& rkPosition)
{
    typename VertexMap::iterator iter = m_kVMap.find(rkPosition);
    if (iter == m_kVMap.end())
    {
        // The vertex does not exists, so return an invalid index.
        return -1;
    }
    int iPosIndex = iter->second;

    Triangle* pkInitialTri = GetContainingTriangleInternal(rkPosition);
    if (!pkInitialTri)
    {
        // All points must lie in the supertriangle, so each point must have
        // a containing triangle.  Moreover, in the Remove operation, the
        // point must be a vertex of a triangle.
        assert(false);
        return -1;
    }

    // Construct the removal polygon.
    std::vector<RPVertex> kPolygon;
    Triangle* pkTri = pkInitialTri;
    do
    {
        // Locate the vertex for the removal point.  The opposite edge is an
        // edge of the removal polygon.
        int i;
        for (i = 0; i < 3; i++)
        {
            if (pkTri->V[i] == iPosIndex)
            {
                break;
            }
        }
        if (i == 3)
        {
            // The removal point must be a vertex of the triangle.
            assert(false);
            return -1;
        }

        // The removal point is P = Tri.V[i].  The edge of the removal polygon
        // is <V1,V2>, where V1 = Tri.V[(i+1)%3] and V2 = Tri.V[(i+2)%3)].
        // The edge <P,V1> is shared by Tri and Adj = Tri.A[i].
        kPolygon.push_back(RPVertex(pkTri->V[(i+1)%3], pkTri, pkTri->A[i]));
        pkTri = pkTri->A[(i+2)%3];
    }
    while (pkTri != pkInitialTri);

    // Triangulate the removal polygon.
    Triangulate(kPolygon,iPosIndex,this);

    m_kVMap.erase(iter);
    return iPosIndex;
}
//----------------------------------------------------------------------------
template <typename Real>
void IncrementalDelaunay2<Real>::GetAllTriangles (int& riNumTriangles,
    int*& raiIndices)
{
    riNumTriangles = (int)m_kTriangle.size();
    raiIndices = WM4_NEW int[3*riNumTriangles];

    int* piIndices = raiIndices;
    typename std::set<Triangle*>::iterator pkTIter = m_kTriangle.begin();
    for (/**/; pkTIter != m_kTriangle.end(); pkTIter++)
    {
        Triangle* pkTri = *pkTIter;
        for (int i = 0; i < 3; i++)
        {
            *piIndices++ = pkTri->V[i];
        }
    }
}
//----------------------------------------------------------------------------
template <typename Real>
void IncrementalDelaunay2<Real>::GenerateRepresentation ()
{
    WM4_DELETE[] m_aiIndex;
    m_aiIndex = 0;
    WM4_DELETE[] m_aiAdjacent;
    m_aiAdjacent = 0;
    WM4_DELETE[] m_aiPath;
    m_aiPath = 0;

    // Assign integer values to the triangles for use by the caller.
    std::map<Triangle*,int> kPermute;
    typename std::set<Triangle*>::iterator pkTIter = m_kTriangle.begin();
    m_iNumTriangles = (int)m_kTriangle.size();
    Triangle* pkTri;
    int i;
    for (i = 0; pkTIter != m_kTriangle.end(); pkTIter++)
    {
        pkTri = *pkTIter;

        // Skip triangles that share a supervertex.
        if ((0 <= pkTri->V[0] && pkTri->V[0] <= 2)
        ||  (0 <= pkTri->V[1] && pkTri->V[1] <= 2)
        ||  (0 <= pkTri->V[2] && pkTri->V[2] <= 2))
        {
            m_iNumTriangles--;
            continue;
        }

        kPermute[pkTri] = i++;
    }
    kPermute[0] = -1;

    // Put Delaunay triangles into an array (vertices and adjacency info).
    if (m_iNumTriangles > 0)
    {
        m_aiIndex = WM4_NEW int[3*m_iNumTriangles];
        m_aiAdjacent = WM4_NEW int[3*m_iNumTriangles];
        i = 0;
        pkTIter = m_kTriangle.begin();
        for (/**/; pkTIter != m_kTriangle.end(); pkTIter++)
        {
            pkTri = *pkTIter;

            // Skip triangles that share a supervertex.
            if ((0 <= pkTri->V[0] && pkTri->V[0] <= 2)
            ||  (0 <= pkTri->V[1] && pkTri->V[1] <= 2)
            ||  (0 <= pkTri->V[2] && pkTri->V[2] <= 2))
            {
                continue;
            }

            m_aiIndex[i] = pkTri->V[0];
            if (ContainsSupervertex(pkTri->A[0]))
            {
                m_aiAdjacent[i++] = -1;
            }
            else
            {
                m_aiAdjacent[i++] = kPermute[pkTri->A[0]];
            }

            m_aiIndex[i] = pkTri->V[1];
            if (ContainsSupervertex(pkTri->A[1]))
            {
                m_aiAdjacent[i++] = -1;
            }
            else
            {
                m_aiAdjacent[i++] = kPermute[pkTri->A[1]];
            }

            m_aiIndex[i] = pkTri->V[2];
            if (ContainsSupervertex(pkTri->A[2]))
            {
                m_aiAdjacent[i++] = -1;
            }
            else
            {
                m_aiAdjacent[i++] = kPermute[pkTri->A[2]];
            }
        }
        assert(i == 3*m_iNumTriangles);

        m_iPathLast = -1;
        m_aiPath = WM4_NEW int[m_iNumTriangles+1];
    }
}
//----------------------------------------------------------------------------
template <typename Real>
int IncrementalDelaunay2<Real>::GetNumTriangles () const
{
    return m_iNumTriangles;
}
//----------------------------------------------------------------------------
template <typename Real>
const int* IncrementalDelaunay2<Real>::GetIndices () const
{
    return m_aiIndex;
}
//----------------------------------------------------------------------------
template <typename Real>
const int* IncrementalDelaunay2<Real>::GetAdjacencies () const
{
    return m_aiAdjacent;
}
//----------------------------------------------------------------------------
template <typename Real>
const std::vector<Vector2<Real> >&
IncrementalDelaunay2<Real>::GetVertices () const
{
    return m_kVertexPool;
}
//----------------------------------------------------------------------------
template <typename Real>
const std::map<Vector2<Real>,int>&
IncrementalDelaunay2<Real>::GetUniqueVertices () const
{
    return m_kVMap;
}
//----------------------------------------------------------------------------
template <class Real>
bool IncrementalDelaunay2<Real>::GetHull (int& riEQuantity, int*& raiIndex)
{
    riEQuantity = 0;
    raiIndex = 0;

    // Count the number of edges that are shared by triangles containing a
    // supervertex.
    int i, iAdjQuantity = 3*m_iNumTriangles;
    for (i = 0; i < iAdjQuantity; i++)
    {
        if (m_aiAdjacent[i] == -1)
        {
            riEQuantity++;
        }
    }
    assert(riEQuantity > 0);
    if (riEQuantity == 0)
    {
        return false;
    }

    // Enumerate the edges.
    raiIndex = WM4_NEW int[2*riEQuantity];
    int* piIndex = raiIndex;
    for (i = 0; i < iAdjQuantity; i++)
    {
        if (m_aiAdjacent[i] == -1)
        {
            int iTri = i/3, j = i%3;
            *piIndex++ = m_aiIndex[3*iTri+j];
            *piIndex++ = m_aiIndex[3*iTri+((j+1)%3)];
        }
    }

    return true;
}
//----------------------------------------------------------------------------
template <class Real>
int IncrementalDelaunay2<Real>::GetContainingTriangle (
    const Vector2<Real>& rkTest) const
{
    // The mesh might not have any triangles (only collinear points were
    // inserted).
    if (!m_aiPath)
    {
        return -1;
    }

    // Start at first triangle in mesh.
    int iIndex = (m_iPathLast >= 0 ? m_aiPath[m_iPathLast] : 0);
    m_iPathLast = -1;
    m_iLastEdgeV0 = -1;
    m_iLastEdgeV1 = -1;
    m_iLastEdgeOpposite = -1;
    m_iLastEdgeOppositeIndex = -1;

    // Use triangle edges as binary separating lines.
    for (int i = 0; i < m_iNumTriangles; i++)
    {
        m_aiPath[++m_iPathLast] = iIndex;

        int* aiV = &m_aiIndex[3*iIndex];

        if (ToLine(rkTest,aiV[0],aiV[1]) > 0)
        {
            iIndex = m_aiAdjacent[3*iIndex];
            if (iIndex == -1)
            {
                m_iLastEdgeV0 = aiV[0];
                m_iLastEdgeV1 = aiV[1];
                m_iLastEdgeOpposite = aiV[2];
                m_iLastEdgeOppositeIndex = 2;
                return -1;
            }
            continue;
        }

        if (ToLine(rkTest,aiV[1],aiV[2]) > 0)
        {
            iIndex = m_aiAdjacent[3*iIndex+1];
            if (iIndex == -1)
            {
                m_iLastEdgeV0 = aiV[1];
                m_iLastEdgeV1 = aiV[2];
                m_iLastEdgeOpposite = aiV[0];
                m_iLastEdgeOppositeIndex = 0;
                return -1;
            }
            continue;
        }

        if (ToLine(rkTest,aiV[2],aiV[0]) > 0)
        {
            iIndex = m_aiAdjacent[3*iIndex+2];
            if (iIndex == -1)
            {
                m_iLastEdgeV0 = aiV[2];
                m_iLastEdgeV1 = aiV[0];
                m_iLastEdgeOpposite = aiV[1];
                m_iLastEdgeOppositeIndex = 1;
                return -1;
            }
            continue;
        }

        m_iLastEdgeV0 = -1;
        m_iLastEdgeV1 = -1;
        m_iLastEdgeOpposite = -1;
        m_iLastEdgeOppositeIndex = -1;
        return iIndex;
    }

    return -1;
}
//----------------------------------------------------------------------------
template <class Real>
int IncrementalDelaunay2<Real>::GetPathLast () const
{
    return m_iPathLast;
}
//----------------------------------------------------------------------------
template <class Real>
const int* IncrementalDelaunay2<Real>::GetPath () const
{
    return m_aiPath;
}
//----------------------------------------------------------------------------
template <class Real>
int IncrementalDelaunay2<Real>::GetLastEdge (int& riV0, int& riV1, int& riV2)
    const
{
    riV0 = m_iLastEdgeV0;
    riV1 = m_iLastEdgeV1;
    riV2 = m_iLastEdgeOpposite;
    return m_iLastEdgeOppositeIndex;
}
//----------------------------------------------------------------------------
template <class Real>
bool IncrementalDelaunay2<Real>::GetVertexSet (int i, Vector2<Real> akV[3])
    const
{
    if (0 <= i && i < m_iNumTriangles)
    {
        akV[0] = m_kVertexPool[m_aiIndex[3*i  ]];
        akV[1] = m_kVertexPool[m_aiIndex[3*i+1]];
        akV[2] = m_kVertexPool[m_aiIndex[3*i+2]];
        return true;
    }

    return false;
}
//----------------------------------------------------------------------------
template <class Real>
bool IncrementalDelaunay2<Real>::GetIndexSet (int i, int aiIndex[3]) const
{
    if (0 <= i && i < m_iNumTriangles)
    {
        aiIndex[0] = m_aiIndex[3*i  ];
        aiIndex[1] = m_aiIndex[3*i+1];
        aiIndex[2] = m_aiIndex[3*i+2];
        return true;
    }

    return false;
}
//----------------------------------------------------------------------------
template <class Real>
bool IncrementalDelaunay2<Real>::GetAdjacentSet (int i, int aiAdjacent[3])
    const
{
    if (0 <= i && i < m_iNumTriangles)
    {
        aiAdjacent[0] = m_aiAdjacent[3*i  ];
        aiAdjacent[1] = m_aiAdjacent[3*i+1];
        aiAdjacent[2] = m_aiAdjacent[3*i+2];
        return true;
    }

    return false;
}
//----------------------------------------------------------------------------
template <class Real>
bool IncrementalDelaunay2<Real>::GetBarycentricSet (int i,
    const Vector2<Real>& rkTest, Real afBary[3]) const
{
    if (0 <= i && i < m_iNumTriangles)
    {
        Vector2<Real> kV0 = m_kVertexPool[m_aiIndex[3*i  ]];
        Vector2<Real> kV1 = m_kVertexPool[m_aiIndex[3*i+1]];
        Vector2<Real> kV2 = m_kVertexPool[m_aiIndex[3*i+2]];
        rkTest.GetBarycentrics(kV0,kV1,kV2,afBary);
        return true;
    }

    return false;
}
//----------------------------------------------------------------------------
template <typename Real>
int IncrementalDelaunay2<Real>::ToLine (const Vector2<Real>& rkTest, int iV0,
    int iV1) const
{
    if (m_fUncertainty < (Real)1)
    {
        // Order the points so that ToLine(test,v0,v1) and ToLine(test,v1,v0)
        // return the same geometric result.
        Vector2<Real> kV0 = m_kVertexPool[iV0];
        Vector2<Real> kV1 = m_kVertexPool[iV1];
        bool bPositive;
        if (kV0 < kV1)
        {
            bPositive = true;
        }
        else
        {
            Vector2<Real> kSave = kV0;
            kV0 = kV1;
            kV1 = kSave;
            bPositive = false;
        }

        Real fX0 = rkTest[0] - kV0[0];
        Real fY0 = rkTest[1] - kV0[1];
        Real fX1 = kV1[0] - kV0[0];
        Real fY1 = kV1[1] - kV0[1];
        Real fDet = fX0*fY1 - fX1*fY0;
        if (!bPositive)
        {
            fDet = -fDet;
        }

        if (m_fUncertainty == (Real)0)
        {
            // Compute the sign test using floating-point arithmetic.
            return (fDet > (Real)0 ? +1 : (fDet < (Real)0 ? -1 : 0));
        }

        // Use filtered predicates.
        Real fLen0 = Math<Real>::Sqrt(fX0*fX0 + fY0*fY0);
        Real fLen1 = Math<Real>::Sqrt(fX1*fX1 + fY1*fY1);
        Real fScaledUncertainty = m_fUncertainty*fLen0*fLen1;
        if (Math<Real>::FAbs(fDet) >= fScaledUncertainty)
        {
            // The floating-point sign test is deemed to be certain.
            return (fDet > (Real)0 ? +1 : (fDet < (Real)0 ? -1 : 0));
        }
    }

    // Compute the determinant using exact rational arithmetic.
    RVector kRatTest;
    kRatTest[0] = Rational(rkTest[0]);
    kRatTest[1] = Rational(rkTest[1]);
    int aiIndex[2] = { iV0, iV1 };
    for (int i = 0; i < 2; i++)
    {
        int j = aiIndex[i];
        if (!(*m_pkRatVertexEvaluated)[j])
        {
            (*m_pkRatVertexEvaluated)[j] = true;
            (*m_pkRatVertexPool)[j][0] = Rational(m_kVertexPool[j][0]);
            (*m_pkRatVertexPool)[j][1] = Rational(m_kVertexPool[j][1]);
        }
    }

    // Compute the sign test using rational arithmetic.
    const RVector& rkRatV0 = (*m_pkRatVertexPool)[iV0];
    const RVector& rkRatV1 = (*m_pkRatVertexPool)[iV1];
    Rational kRatX0 = kRatTest[0] - rkRatV0[0];
    Rational kRatY0 = kRatTest[1] - rkRatV0[1];
    Rational kRatX1 = rkRatV1[0] - rkRatV0[0];
    Rational kRatY1 = rkRatV1[1] - rkRatV0[1];
    Rational kRatDet = kRatX0*kRatY1 - kRatX1*kRatY0;
    return (kRatDet > 0 ? +1 : (kRatDet < 0 ? -1 : 0));
}
//----------------------------------------------------------------------------
template <typename Real>
int IncrementalDelaunay2<Real>::ToTriangle (const Vector2<Real>& rkTest,
    int iV0, int iV1, int iV2) const
{
    int iSign0 = ToLine(rkTest,iV1,iV2);
    if (iSign0 > 0)
    {
        return +1;
    }

    int iSign1 = ToLine(rkTest,iV0,iV2);
    if (iSign1 < 0)
    {
        return +1;
    }

    int iSign2 = ToLine(rkTest,iV0,iV1);
    if (iSign2 > 0)
    {
        return +1;
    }

    return ((iSign0 && iSign1 && iSign2) ? -1 : 0);
}
//----------------------------------------------------------------------------
template <typename Real>
int IncrementalDelaunay2<Real>::ToCircumcircle (const Vector2<Real>& rkTest,
    int iV0, int iV1, int iV2) const
{
    if (m_fUncertainty < (Real)1)
    {
        // Order the points so that ToCircumcircle(test,u0,u1,u2) returns the
        // same containment result for any permutation (u0,u1,u2) of
        // (v0,v1,v2).
        Vector2<Real> kV0 = m_kVertexPool[iV0];
        Vector2<Real> kV1 = m_kVertexPool[iV1];
        Vector2<Real> kV2 = m_kVertexPool[iV2];
        Vector2<Real> kSave;
        bool bPositive;
        if (kV0 < kV1)
        {
            if (kV2 < kV0)
            {
                // (2,0,1)
                kSave = kV2;
                kV2 = kV1;
                kV1 = kV0;
                kV0 = kSave;
                bPositive = true;
            }
            else if (kV2 < kV1)
            {
                // (0,2,1)
                kSave = kV1;
                kV1 = kV2;
                kV2 = kSave;
                bPositive = false;
            }
            else
            {
                // (0,1,2)
                bPositive = true;
            }
        }
        else
        {
            if (kV2 < kV1)
            {
                // (2,1,0)
                kSave = kV0;
                kV0 = kV2;
                kV2 = kSave;
                bPositive = false;
            }
            else if (kV2 < kV0)
            {
                // (1,2,0)
                kSave = kV0;
                kV0 = kV1;
                kV1 = kV2;
                kV2 = kSave;
                bPositive = true;
            }
            else
            {
                // (1,0,2)
                kSave = kV0;
                kV0 = kV1;
                kV1 = kSave;
                bPositive = false;
            }
        }

        Real fS0x = kV0[0] + rkTest[0];
        Real fD0x = kV0[0] - rkTest[0];
        Real fS0y = kV0[1] + rkTest[1];
        Real fD0y = kV0[1] - rkTest[1];
        Real fS1x = kV1[0] + rkTest[0];
        Real fD1x = kV1[0] - rkTest[0];
        Real fS1y = kV1[1] + rkTest[1];
        Real fD1y = kV1[1] - rkTest[1];
        Real fS2x = kV2[0] + rkTest[0];
        Real fD2x = kV2[0] - rkTest[0];
        Real fS2y = kV2[1] + rkTest[1];
        Real fD2y = kV2[1] - rkTest[1];
        Real fZ0 = fS0x*fD0x + fS0y*fD0y;
        Real fZ1 = fS1x*fD1x + fS1y*fD1y;
        Real fZ2 = fS2x*fD2x + fS2y*fD2y;
        Real fC00 = fD1y*fZ2 - fD2y*fZ1;
        Real fC01 = fD2y*fZ0 - fD0y*fZ2;
        Real fC02 = fD0y*fZ1 - fD1y*fZ0;
        Real fDet = fD0x*fC00 + fD1x*fC01 + fD2x*fC02;
        if (!bPositive)
        {
            fDet = -fDet;
        }

        if (m_fUncertainty == (Real)0)
        {
            // Compute the sign test using floating-point arithmetic.
            return (fDet < (Real)0 ? +1 : (fDet > (Real)0 ? -1 : 0));
        }

        // Use filtered predicates.
        Real fLen0 = Math<Real>::Sqrt(fD0x*fD0x + fD0y*fD0y + fZ0*fZ0);
        Real fLen1 = Math<Real>::Sqrt(fD1x*fD1x + fD1y*fD1y + fZ1*fZ1);
        Real fLen2 = Math<Real>::Sqrt(fD2x*fD2x + fD2y*fD2y + fZ2*fZ2);
        Real fScaledUncertainty = m_fUncertainty*fLen0*fLen1*fLen2;
        if (Math<Real>::FAbs(fDet) >= fScaledUncertainty)
        {
            return (fDet < (Real)0 ? 1 : (fDet > (Real)0 ? -1 : 0));
        }
    }

    // Compute the sign test using rational arithmetic.
    RVector kRatTest;
    kRatTest[0] = Rational(rkTest[0]);
    kRatTest[1] = Rational(rkTest[1]);
    int aiIndex[3] = { iV0, iV1, iV2 };
    for (int i = 0; i < 3; i++)
    {
        int j = aiIndex[i];
        if (!(*m_pkRatVertexEvaluated)[j])
        {
            (*m_pkRatVertexEvaluated)[j] = true;
            (*m_pkRatVertexPool)[j][0] = Rational(m_kVertexPool[j][0]);
            (*m_pkRatVertexPool)[j][1] = Rational(m_kVertexPool[j][1]);
        }
    }

    RVector& rkRatV0 = (*m_pkRatVertexPool)[iV0];
    RVector& rkRatV1 = (*m_pkRatVertexPool)[iV1];
    RVector& rkRatV2 = (*m_pkRatVertexPool)[iV2];
    Rational kRatS0x = rkRatV0[0] + kRatTest[0];
    Rational kRatD0x = rkRatV0[0] - kRatTest[0];
    Rational kRatS0y = rkRatV0[1] + kRatTest[1];
    Rational kRatD0y = rkRatV0[1] - kRatTest[1];
    Rational kRatS1x = rkRatV1[0] + kRatTest[0];
    Rational kRatD1x = rkRatV1[0] - kRatTest[0];
    Rational kRatS1y = rkRatV1[1] + kRatTest[1];
    Rational kRatD1y = rkRatV1[1] - kRatTest[1];
    Rational kRatS2x = rkRatV2[0] + kRatTest[0];
    Rational kRatD2x = rkRatV2[0] - kRatTest[0];
    Rational kRatS2y = rkRatV2[1] + kRatTest[1];
    Rational kRatD2y = rkRatV2[1] - kRatTest[1];
    Rational kRatZ0 = kRatS0x*kRatD0x + kRatS0y*kRatD0y;
    Rational kRatZ1 = kRatS1x*kRatD1x + kRatS1y*kRatD1y;
    Rational kRatZ2 = kRatS2x*kRatD2x + kRatS2y*kRatD2y;
    Rational kRatC00 = kRatD1y*kRatZ2 - kRatD2y*kRatZ1;
    Rational kRatC01 = kRatD2y*kRatZ0 - kRatD0y*kRatZ2;
    Rational kRatC02 = kRatD0y*kRatZ1 - kRatD1y*kRatZ0;
    Rational kRatDet = kRatD0x*kRatC00 + kRatD1x*kRatC01 + kRatD2x*kRatC02;
    return (kRatDet < 0 ? +1 : (kRatDet > 0 ? -1 : 0));
}
//----------------------------------------------------------------------------
template <typename Real>
typename IncrementalDelaunay2<Real>::Triangle*
IncrementalDelaunay2<Real>::GetContainingTriangleInternal (
    const Vector2<Real>& rkPosition) const
{
    // Locate which triangle in the current mesh contains vertex i.  By
    // construction, there must be such a triangle (the vertex cannot be
    // outside the supertriangle).

    Triangle* pkTri = *m_kTriangle.begin();
    int iTQuantity = (int)m_kTriangle.size();
    for (int iT = 0; iT < iTQuantity; iT++)
    {
        int* aiV = pkTri->V;

        if (ToLine(rkPosition,aiV[0],aiV[1]) > 0)
        {
            pkTri = pkTri->A[0];
            if (!pkTri)
            {
                break;
            }
            continue;
        }

        if (ToLine(rkPosition,aiV[1],aiV[2]) > 0)
        {
            pkTri = pkTri->A[1];
            if (!pkTri)
            {
                break;
            }
            continue;
        }

        if (ToLine(rkPosition,aiV[2],aiV[0]) > 0)
        {
            pkTri = pkTri->A[2];
            if (!pkTri)
            {
                break;
            }
            continue;
        }

        return pkTri;
    }

    assert(false);
    return 0;
}
//----------------------------------------------------------------------------
template <typename Real>
bool IncrementalDelaunay2<Real>::ContainsSupervertex (Triangle* pkTri) const
{
    for (int i = 0; i < 3; i++)
    {
        if (0 <= pkTri->V[i] && pkTri->V[i] <= 2)
        {
            return true;
        }
    }
    return false;
}
//----------------------------------------------------------------------------
template <typename Real>
void IncrementalDelaunay2<Real>::SwapEdge (Triangle* pkTri0, Triangle* pkTri1)
{
    int i0, i0p1, i0p2, i1, i1p1, i1p2, j;
    Triangle* pkAdj;

    // Locate the indices of the shared edge.
    for (i0 = 0; i0 < 3; i0++)
    {
        if (pkTri1 == pkTri0->A[i0])
        {
            break;
        }
    }
    if (i0 == 3)
    {
        assert(false);
        return;
    }
    i0p1 = (i0+1)%3;
    i0p2 = (i0+2)%3;

    for (i1 = 0; i1 < 3; i1++)
    {
        if (pkTri0 == pkTri1->A[i1])
        {
            break;
        }
    }
    if (i1 == 3)
    {
        assert(false);
        return;
    }
    i1p1 = (i1+1)%3;
    i1p2 = (i1+2)%3;

    pkTri0->V[i0p1] = pkTri1->V[i1p2];
    pkTri1->V[i1p1] = pkTri0->V[i0p2];

    pkAdj = pkTri1->A[i1p1];
    pkTri0->A[i0] = pkAdj;
    if (pkAdj)
    {
        for (j = 0; j < 3; j++)
        {
            if (pkAdj->A[j] == pkTri1)
            {
                pkAdj->A[j] = pkTri0;
                break;
            }
        }
        if (j == 3)
        {
            assert(false);
            return;
        }
    }

    pkAdj = pkTri0->A[i0p1];
    pkTri1->A[i1] = pkAdj;
    if (pkAdj)
    {
        for (j = 0; j < 3; j++)
        {
            if (pkAdj->A[j] == pkTri0)
            {
                pkAdj->A[j] = pkTri1;
                break;
            }
        }
        if (j == 3)
        {
            assert(false);
            return;
        }
    }

    pkTri0->A[i0p1] = pkTri1;
    pkTri1->A[i1p1] = pkTri0;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// IncrementalDelaunay2::Triangle
//----------------------------------------------------------------------------
template <class Real>
IncrementalDelaunay2<Real>::Triangle::Triangle (int iV0, int iV1, int iV2)
{
    V[0] = iV0;
    V[1] = iV1;
    V[2] = iV2;
    A[0] = 0;
    A[1] = 0;
    A[2] = 0;
    Time = -1;
    IsComponent = false;
    OnStack = false;
}
//----------------------------------------------------------------------------
template <class Real>
bool IncrementalDelaunay2<Real>::Triangle::IsInsertionComponent (
    int iPosIndex, const Vector2<Real>& rkTest, Triangle* pkAdj,
    const IncrementalDelaunay2* pkDelaunay)
{
    if (iPosIndex != Time)
    {
        Time = iPosIndex;

        // Determine the number of vertices in common with the supertriangle.
        // The supertriangle vertices have indices VQ-3, VQ-2, and VQ-1, where
        // VQ is the quantity of input vertices.
        int iCommon = 0, iSVIndex = -1, j;
        for (j = 0; j < 3; j++)
        {
            // The supervertices are at indices 0, 1, and 2, so loop counter
            // 'k' is the index into the supervertices.
            for (int k = 0; k < 3; k++)
            {
                if (V[j] == k)
                {
                    iCommon++;
                    iSVIndex = j;
                }
            }
        }

        int iRelation;
        if (iCommon == 0)
        {
            // The classic case is that a point is in the mesh formed only by
            // the input vertices, in which case we only test for containment
            // in the circumcircle of the triangle.
            iRelation = pkDelaunay->ToCircumcircle(rkTest,V[0],V[1],V[2]);
        }
        else
        {
            // The classic problem is that points outside the mesh formed
            // only by the input vertices must be handled from a visibility
            // perspective rather than using circumcircles (compare with
            // convex hull construction).  By not doing this, you can run into
            // the pitfall that has snared many folks--the boundary edges of
            // the final triangulation do not form a convex polygon.
            int iV0, iV1;
            if (iCommon == 1)
            {
                iV0 = V[(iSVIndex+1)%3];
                iV1 = V[(iSVIndex+2)%3];
            }
            else  // iCommon == 2
            {
                for (j = 0; j < 3; j++)
                {
                    if (A[j] != 0 && A[j] != pkAdj)
                    {
                        break;
                    }
                }
                iV0 = V[j];
                iV1 = V[(j+1)%3];
            }
            iRelation = pkDelaunay->ToLine(rkTest,iV0,iV1);
        }

        IsComponent = (iRelation < 0 ? true : false);
    }

    return IsComponent;
}
//----------------------------------------------------------------------------
template <class Real>
int IncrementalDelaunay2<Real>::Triangle::DetachFrom (int iAdj,
    Triangle* pkAdj)
{
    assert(0 <= iAdj && iAdj < 3 && A[iAdj] == pkAdj);
    A[iAdj] = 0;
    for (int i = 0; i < 3; i++)
    {
        if (pkAdj->A[i] == this)
        {
            pkAdj->A[i] = 0;
            return i;
        }
    }
    return -1;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// IncrementalDelaunay2::Edge
//----------------------------------------------------------------------------
template <class Real>
IncrementalDelaunay2<Real>::Edge::Edge (int iV0, int iV1, int iNullIndex,
    Triangle* pkTri)
    :
    VEManifoldMesh::Edge(iV0,iV1)
{
    NullIndex = iNullIndex;
    Tri = pkTri;
}
//----------------------------------------------------------------------------
template <class Real>
VEManifoldMesh::EPtr IncrementalDelaunay2<Real>::Edge::ECreator (int iV0,
    int iV1)
{
    return WM4_NEW Edge(iV0,iV1,0,0);
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// IncrementalDelaunay2::RPVertex
//----------------------------------------------------------------------------
template <class Real>
IncrementalDelaunay2<Real>::RPVertex::RPVertex (int iIndex, Triangle* pkTri,
    Triangle* pkAdj)
{
    Index = iIndex;
    Tri = pkTri;
    Adj = pkAdj;
    IsConvex = false;
    IsEarTip = false;
    IsSuperVertex = false;
    Weight = Math<Real>::MAX_REAL;
    VPrev = -1;
    VNext = -1;
    SPrev = -1;
    SNext = -1;
    EarRecord = 0;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// IncrementalDelaunay2::Triangulate
//----------------------------------------------------------------------------
template <class Real>
IncrementalDelaunay2<Real>::Triangulate::Triangulate (
    std::vector<RPVertex>& rkPolygon, int iRemoval,
    IncrementalDelaunay2* pkDelaunay)
    :
    m_rkPolygon(rkPolygon),
    m_iNumVertices((int)rkPolygon.size()),
    m_pkDelaunay(pkDelaunay),
    m_iCFirst(-1),
    m_iCLast(-1),
    m_iRFirst(-1),
    m_iRLast(-1),
    m_kEHeap((int)rkPolygon.size(),1)
{
    // Create a circular list of the polygon vertices for dynamic removal of
    // vertices.
    int iVQm1 = m_iNumVertices - 1;
    int i;
    for (i = 0; i <= iVQm1; i++)
    {
        RPVertex& rkV = V(i);
        rkV.VPrev = (i > 0 ? i-1 : iVQm1);
        rkV.VNext = (i < iVQm1 ? i+1 : 0);
    }

    // Create a circular list of the polygon vertices for dynamic removal of
    // vertices.  Keep track of two linear sublists, one for the convex
    // vertices and one for the reflex vertices.  This is an O(N) process
    // where N is the number of polygon vertices.
    for (i = 0; i <= iVQm1; i++)
    {
        if (IsConvex(i))
        {
            InsertAfterC(i);
        }
        else
        {
            InsertAfterR(i);
        }
    }

    // Identify the ear tips and build a circular list of them.  Let V0, V1,
    // and/ V2 be consecutive vertices forming a triangle T (the ear).  The
    // vertex V1 is an ear tip if no other vertices of the polygon lie inside
    // T.  Although it is enough to show that V1 is not an ear by finding at
    // least one other vertex inside T, it is sufficient to search only the
    // reflex vertices.  This is an O(C*R) process, where C is the number of
    // convex vertices and R is the number of reflex vertices with N = C+R.
    // The order is O(N^2), for example when C = R = N/2.
    Real fWeight;
    int iVPrev, iVNext;
    for (i = m_iCFirst; i != -1; i = V(i).SNext)
    {
        if (IsEarTip(i))
        {
            fWeight = ComputeWeight(i,iRemoval);
            V(i).EarRecord = m_kEHeap.Insert(i,fWeight);
            V(i).IsEarTip = true;
        }
    }

    // Remove the ears, one at a time.
    while (m_iNumVertices >= 3)
    {
        if (m_iNumVertices == 3)
        {
            // Only one triangle remains.  Erase the three subtriangles
            // linked to removal point P and then insert the remaining
            // triangle.
            assert(m_kEHeap.GetQuantity() == 3);

            m_kEHeap.Remove(i,fWeight);
            RPVertex& rkV0 = V(i);
            iVPrev = rkV0.VPrev;
            iVNext = rkV0.VNext;
            RPVertex& rkVp = V(iVPrev);
            RPVertex& rkVn = V(iVNext);
            Triangle* pkTri0 = rkV0.Tri;
            Triangle* pkTriP = rkVp.Tri;
            Triangle* pkTriN = rkVn.Tri;

            int i0;
            for (i0 = 0; i0 < 3; i0++)
            {
                if (pkTri0->V[i0] == iRemoval)
                {
                    break;
                }
            }
            if (i0 == 3)
            {
                assert(false);
                break;
            }
            pkTri0->V[i0] = rkVp.Index;

            int ip;
            for (ip = 0; ip < 3; ip++)
            {
                if (pkTriP->V[ip] == rkVp.Index)
                {
                    break;
                }
            }
            if (ip == 3)
            {
                assert(false);
                break;
            }
            Triangle* pkAdj = pkTriP->A[ip];
            pkTri0->A[i0] = pkAdj;
            if (pkAdj)
            {
                for (i = 0; i < 3; i++)
                {
                    if (pkAdj->V[i] == rkV0.Index)
                    {
                        pkAdj->A[i] = pkTri0;
                        break;
                    }
                }
                if (i == 3)
                {
                    assert(false);
                    break;
                }
            }


            int in;
            for (in = 0; in < 3; in++)
            {
                if (pkTriN->V[in] == rkVn.Index)
                {
                    break;
                }
            }
            if (in == 3)
            {
                assert(false);
                break;
            }
            pkAdj = pkTriN->A[in];
            pkTri0->A[(i0+2)%3] = pkAdj;
            if (pkAdj)
            {
                for (i = 0; i < 3; i++)
                {
                    if (pkAdj->V[i] == rkVp.Index)
                    {
                        pkAdj->A[i] = pkTri0;
                        break;
                    }
                }
                if (i == 3)
                {
                    assert(false);
                    break;
                }
            }

            m_pkDelaunay->m_kTriangle.erase(pkTriP);
            m_pkDelaunay->m_kTriangle.erase(pkTriN);
            break;
        }

        m_kEHeap.Remove(i,fWeight);
        iVPrev = V(i).VPrev;
        iVNext = V(i).VNext;
        m_pkDelaunay->SwapEdge(V(i).Adj,V(i).Tri);
        V(iVPrev).Tri = V(i).Tri;
        RemoveV(i);

        // Removal of the ear can cause an adjacent vertex to become an ear
        // or to stop being an ear.
        RPVertex& rkVPrev = V(iVPrev);
        if (rkVPrev.IsEarTip)
        {
            if (!IsEarTip(iVPrev))
            {
                m_kEHeap.Update(V(iVPrev).EarRecord,(Real)-1);
                m_kEHeap.Remove(i,fWeight);
                assert(i == iVPrev && fWeight == (Real)-1);
            }
        }
        else
        {
            bool bWasReflex = !rkVPrev.IsConvex;
            if (IsConvex(iVPrev))
            {
                if (bWasReflex)
                {
                    RemoveR(iVPrev);
                }

                if (IsEarTip(iVPrev))
                {
                    fWeight = ComputeWeight(iVPrev,iRemoval);
                    V(iVPrev).EarRecord = m_kEHeap.Insert(iVPrev,fWeight);
                    V(iVPrev).IsEarTip = true;
                }
            }
        }

        RPVertex& rkVNext = V(iVNext);
        if (rkVNext.IsEarTip)
        {
            if (!IsEarTip(iVNext))
            {
                m_kEHeap.Update(V(iVNext).EarRecord,(Real)-1);
                m_kEHeap.Remove(i,fWeight);
                assert(i == iVNext && fWeight == (Real)-1);
            }
        }
        else
        {
            bool bWasReflex = !rkVNext.IsConvex;
            if (IsConvex(iVNext))
            {
                if (bWasReflex)
                {
                    RemoveR(iVNext);
                }

                if (IsEarTip(iVNext))
                {
                    fWeight = ComputeWeight(iVNext,iRemoval);
                    V(iVNext).EarRecord = m_kEHeap.Insert(iVNext,fWeight);
                    V(iVNext).IsEarTip = true;
                }
            }
        }
    }
}
//----------------------------------------------------------------------------
template <class Real>
typename IncrementalDelaunay2<Real>::RPVertex&
IncrementalDelaunay2<Real>::Triangulate::V (int i)
{
    return m_rkPolygon[i];
}
//----------------------------------------------------------------------------
template <class Real>
bool IncrementalDelaunay2<Real>::Triangulate::IsConvex (int i)
{
    RPVertex& rkV = V(i);
    Vector2<Real> kCurr = m_pkDelaunay->GetVertices()[rkV.Index];
    int iPrev = V(rkV.VPrev).Index;
    int iNext = V(rkV.VNext).Index;
    rkV.IsConvex = (m_pkDelaunay->ToLine(kCurr,iPrev,iNext) > 0);
    return rkV.IsConvex;
}
//----------------------------------------------------------------------------
template <class Real>
bool IncrementalDelaunay2<Real>::Triangulate::IsEarTip (int i)
{
    RPVertex& rkV = V(i);

    if (m_iRFirst == -1)
    {
        // The remaining polygon is convex.
        rkV.IsEarTip = true;
        return true;
    }

    // Search the reflex vertices and test if any are in the triangle
    // <V[prev],V[curr],V[next]>.
    int iPrev = V(rkV.VPrev).Index;
    int iCurr = rkV.Index;
    int iNext = V(rkV.VNext).Index;
    rkV.IsEarTip = true;
    for (int j = m_iRFirst; j != -1; j = V(j).SNext)
    {
        // Check if the test vertex is already one of the triangle vertices.
        if (j == rkV.VPrev || j == i || j == rkV.VNext)
        {
            continue;
        }

        // Test if the vertex is inside or on the triangle.  When it is, it
        // causes V[curr] not to be an ear.
        Vector2<Real> kTest = m_pkDelaunay->GetVertices()[V(j).Index];
        if (m_pkDelaunay->ToTriangle(kTest,iPrev,iCurr,iNext) <= 0)
        {
            rkV.IsEarTip = false;
            break;
        }
    }

    return rkV.IsEarTip;
}
//----------------------------------------------------------------------------
template <class Real>
void IncrementalDelaunay2<Real>::Triangulate::InsertAfterC (int i)
{
    if (m_iCFirst == -1)
    {
        // add first convex vertex
        m_iCFirst = i;
    }
    else
    {
        V(m_iCLast).SNext = i;
        V(i).SPrev = m_iCLast;
    }
    m_iCLast = i;
}
//----------------------------------------------------------------------------
template <class Real>
void IncrementalDelaunay2<Real>::Triangulate::InsertAfterR (int i)
{
    if (m_iRFirst == -1)
    {
        // add first reflex vertex
        m_iRFirst = i;
    }
    else
    {
        V(m_iRLast).SNext = i;
        V(i).SPrev = m_iRLast;
    }
    m_iRLast = i;
}
//----------------------------------------------------------------------------
template <class Real>
void IncrementalDelaunay2<Real>::Triangulate::RemoveV (int i)
{
    int iCurrVPrev = V(i).VPrev;
    int iCurrVNext = V(i).VNext;
    V(iCurrVPrev).VNext = iCurrVNext;
    V(iCurrVNext).VPrev = iCurrVPrev;
    m_iNumVertices--;
}
//----------------------------------------------------------------------------
template <class Real>
void IncrementalDelaunay2<Real>::Triangulate::RemoveR (int i)
{
    assert(m_iRFirst != -1 && m_iRLast != -1);

    if (i == m_iRFirst)
    {
        m_iRFirst = V(i).SNext;
        if (m_iRFirst != -1)
        {
            V(m_iRFirst).SPrev = -1;
        }
        V(i).SNext = -1;
    }
    else if (i == m_iRLast)
    {
        m_iRLast = V(i).SPrev;
        if (m_iRLast != -1)
        {
            V(m_iRLast).SNext = -1;
        }
        V(i).SPrev = -1;
    }
    else
    {
        int iCurrSPrev = V(i).SPrev;
        int iCurrSNext = V(i).SNext;
        V(iCurrSPrev).SNext = iCurrSNext;
        V(iCurrSNext).SPrev = iCurrSPrev;
        V(i).SNext = -1;
        V(i).SPrev = -1;
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Real IncrementalDelaunay2<Real>::Triangulate::ComputeWeight (int iV0, int iP)
{
    RPVertex& rkV0 = V(iV0);
    assert(rkV0.IsEarTip);
    if (0 <= rkV0.Index && rkV0.Index <= 2)
    {
        // The vertex is a supervertex.  Return infinite weight so that the
        // supervertices are processed last.
        rkV0.IsSuperVertex = true;
        rkV0.Weight = Math<Real>::MAX_REAL;
        return rkV0.Weight;
    }

    // Get the adjacent vertices.
    int iVp = rkV0.VPrev;
    int iVn = rkV0.VNext;
    RPVertex& rkVp = V(iVp);
    RPVertex& rkVn = V(iVn);

    const Vector2<Real>& rkPosp = m_pkDelaunay->GetVertices()[rkVp.Index];
    const Vector2<Real>& rkPos0 = m_pkDelaunay->GetVertices()[rkV0.Index];
    const Vector2<Real>& rkPosn = m_pkDelaunay->GetVertices()[rkVn.Index];
    const Vector2<Real>& rkPosr = m_pkDelaunay->GetVertices()[iP];

    // Compute D.
    Real fX0 = rkPos0[0] - rkPosp[0];
    Real fY0 = rkPos0[1] - rkPosp[1];
    Real fX1 = rkPosn[0] - rkPosp[0];
    Real fY1 = rkPosn[1] - rkPosp[1];
    Real fDenom = fX0*fY1 - fX1*fY0;

    // Compute H.
    Real fS0x = rkPosp[0] + rkPosr[0];
    Real fD0x = rkPosp[0] - rkPosr[0];
    Real fS0y = rkPosp[1] + rkPosr[1];
    Real fD0y = rkPosp[1] - rkPosr[1];
    Real fS1x = rkPos0[0] + rkPosr[0];
    Real fD1x = rkPos0[0] - rkPosr[0];
    Real fS1y = rkPos0[1] + rkPosr[1];
    Real fD1y = rkPos0[1] - rkPosr[1];
    Real fS2x = rkPosn[0] + rkPosr[0];
    Real fD2x = rkPosn[0] - rkPosr[0];
    Real fS2y = rkPosn[1] + rkPosr[1];
    Real fD2y = rkPosn[1] - rkPosr[1];
    Real fZ0 = fS0x*fD0x + fS0y*fD0y;
    Real fZ1 = fS1x*fD1x + fS1y*fD1y;
    Real fZ2 = fS2x*fD2x + fS2y*fD2y;
    Real fC00 = fD1y*fZ2 - fD2y*fZ1;
    Real fC01 = fD2y*fZ0 - fD0y*fZ2;
    Real fC02 = fD0y*fZ1 - fD1y*fZ0;
    Real fNumer = fD0x*fC00 + fD1x*fC01 + fD2x*fC02;

    rkV0.Weight = fNumer/fDenom;
    return rkV0.Weight;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IncrementalDelaunay2<float>;

template WM4_FOUNDATION_ITEM
class IncrementalDelaunay2<double>;
//----------------------------------------------------------------------------
}
