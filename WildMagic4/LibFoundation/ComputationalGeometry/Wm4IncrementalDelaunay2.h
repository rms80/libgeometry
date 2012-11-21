// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INCREMENTALDELAUNAY2_H
#define WM4INCREMENTALDELAUNAY2_H

#include "Wm4FoundationLIB.h"
#include "Wm4RVector2.h"
#include "Wm4TMinHeap.h"
#include "Wm4Vector2.h"
#include "Wm4VEManifoldMesh.h"

namespace Wm4
{

template <typename Real>
class WM4_FOUNDATION_ITEM IncrementalDelaunay2
{
public:
    // Construction and destruction.  The bounding rectangle for the data
    // points must be specified.  Each (x,y) must satisfy xmin <= x <= xmax
    // and ymin <= y <= ymax.  If 'uncertainty' is set to 0, then the
    // geometric computations involve only floating-point arithmetic.  If
    // 'uncertainty' is in (0,1), then filtered predicates are used to
    // compute the signs of quantities of interest.  If 'uncertainty' is
    // set to 1, then exact rational arithmetic is used.
    IncrementalDelaunay2 (Real fXMin, Real fYMin, Real fXMax, Real fYMax,
        Real fUncertainty = (Real)0);

    ~IncrementalDelaunay2 ();

    // Insert a point into the triangulation.  The return value is the index
    // associated with the vertex in the vertex map.  The supertriangle
    // vertices are at indices 0, 1, and 2.  If the input point already
    // exists, its vertex-map index is simply returned.  If the position
    // is outside the domain specified in the constructor, the return value
    // is -1.
    int Insert (const Vector2<Real>& rkPosition);

    // Remove a point from the triangulation.  The return value is the index
    // associated with the vertex in the vertex map when that vertex exists.
    // If the vertex does not exist, the return value is -1.
    int Remove (const Vector2<Real>& rkPosition);

    // Support for debugging.  Return an index array of all the triangles,
    // including those that connect to vertices of the supertriangle.  The
    // caller is responsible for deleting the raiIndices output.
    void GetAllTriangles (int& riNumTriangles, int*& raiIndices);

    //========================================================================
    // Generate a compactified representation of the triangulation.  After
    // this call, the functions following this one are valid to call.  Also,
    // if you call Insert and/or Remove after calling GenerateRepresentation,
    // the functions following this one are invalid to call, and if you are
    // hanging onto pointers and/or references produced by these functions,
    // they are now invalid.  You may, however, call GenerateRepresentation
    // again at which time the functions following this one are once again
    // valid to call.
    void GenerateRepresentation ();

    // N = GetNumTriangles() is the number of triangles in the mesh.  The
    // array returned by I = GetIndices() contains N tuples, each tuple
    // having 3 elements and representing a triangle.  An index I[*] is
    // relative to the vertex array V.  The array returned by
    // A = GetAdjacencies() contains N tuples, each tuple having 3 elements
    // and representing those triangles adjacent to the 3 edges of a triangle.
    // An index A[*] is relative to the index array I.
    int GetNumTriangles () const;
    const int* GetIndices () const;
    const int* GetAdjacencies () const;

    // The input vertex array.  The array includes all unique points passed
    // to Insert, even if Remove was called later for any inserted points.
    // The points at indices 0, 1, and 2 are always the vertices of the
    // supertriangle.
    const std::vector<Vector2<Real> >& GetVertices () const;

    // The unique vertices processed.  These are the actual vertices in the
    // triangulation.  The 'int' value is the index associated with the
    // vertex.
    const std::map<Vector2<Real>,int>& GetUniqueVertices () const;

    // Locate those triangle edges that do not share other triangles.  The
    // returned quantity is the number of edges in the hull.  The returned
    // array has 2*quantity indices, each pair representing an edge.  The
    // edges are not ordered, but the pair of vertices for an edge is ordered
    // so that they conform to a counterclockwise traversal of the hull.  The
    // return value is 'true' iff the dimension is 2.
    bool GetHull (int& riEQuantity, int*& raiIndex);

    // Support for searching the triangulation for a triangle that contains
    // a point.  If there is a containing triangle, the returned value is a
    // triangle index i with 0 <= i < GetNumTriangles().  If there is not a
    // containing triangle, -1 is returned.
    int GetContainingTriangle (const Vector2<Real>& rkTest) const;

    // If GetContainingTriangle returns a nonnegative value, the path of
    // triangles searched for the containing triangles is stored in an array.
    // The last index of the array is returned by GetPathLast; it is one
    // less than the number of array elements.  The array itself is returned
    // by GetPath.
    int GetPathLast () const;
    const int* GetPath () const;

    // If GetContainingTriangle returns -1, the path of triangles searched
    // may be obtained by GetPathLast and GetPath.  The input point is outside
    // an edge of the last triangle in the path.  This function returns the
    // vertex indices <v0,v1> of the edge, listed in counterclockwise order
    // relative to the convex hull of the data points.  The final output is
    // the index of the vertex v2 opposite the edge.  The return value of
    // the function is the index of the triple of vertex indices; the value
    // is 0, 1, or 2.
    int GetLastEdge (int& riV0, int& riV1, int& riV2) const;

    // Get the vertices for triangle i.  The function returns 'true' if i is
    // a valid triangle index, in which case the vertices are valid.
    // Otherwise, the function returns 'false' and the vertices are invalid.
    bool GetVertexSet (int i, Vector2<Real> akV[3]) const;

    // Get the vertex indices for triangle i.  The function returns 'true' if
    // i is a valid triangle index, in which case the vertices are valid.
    // Otherwise, the function returns 'false' and the vertices are invalid.
    bool GetIndexSet (int i, int aiIndex[3]) const;

    // Get the indices for triangles adjacent to triangle i.  The function
    // returns 'true' if i is a valid triangle index, in which case the
    // adjacencies are valid.  Otherwise, the function returns 'false' and
    // the adjacencies are invalid.
    bool GetAdjacentSet (int i, int aiAdjacent[3]) const;

    // Compute the barycentric coordinates of P with respect to triangle i.
    // The function returns 'true' if i is a valid triangle index, in which
    // case the coordinates are valid.  Otherwise, the function returns
    // 'false' and the coordinate array is invalid.
    bool GetBarycentricSet (int i, const Vector2<Real>& rkTest,
        Real afBary[3]) const;
    //========================================================================

private:
    // Convenient type definitions.
    typedef std::map<Vector2<Real>,int> VertexMap;
    typedef TRational<4*sizeof(Real)> Rational;
    typedef RVector2<4*sizeof(Real)> RVector;

    class WM4_FOUNDATION_ITEM Triangle
    {
    public:
        Triangle (int iV0, int iV1, int iV2);

        bool IsInsertionComponent (int iPosIndex, const Vector2<Real>& rkTest,
            Triangle* pkAdj, const IncrementalDelaunay2* pkDelaunay);

        int DetachFrom (int iAdj, Triangle* pkAdj);

        int V[3];
        Triangle* A[3];
        int Time;
        bool IsComponent;
        bool OnStack;
    };

    class WM4_FOUNDATION_ITEM Edge : public VEManifoldMesh::Edge
    {
    public:
        Edge (int iV0 = -1, int iV1 = -1, int iNullIndex = -1,
            Triangle* pkTri = 0);

        static VEManifoldMesh::EPtr ECreator (int iV0, int iV1);

        int NullIndex;
        Triangle* Tri;
    };

    // Support for the removal polygon and its triangulation.
    class WM4_FOUNDATION_ITEM RPVertex
    {
    public:
        RPVertex (int iIndex = -1, Triangle* pkTri = 0, Triangle* pkAdj = 0);

        // The index into the vertex pool of the position.
        int Index;

        // The triangle sharing edge <Index,NextIndex> and inside the
        // removal polygon.
        Triangle* Tri;

        // The triangle sharing edge <PrevIndex,Index> and inside the
        // removal polygon.
        Triangle* Adj;

        // A vertex is either convex or reflex.  Its condition is stored by
        // the following member.
        bool IsConvex;

        // A convex vertex is either an ear tip or it is not.  Its condition
        // is stored by the following member.
        bool IsEarTip;

        // The removal polygon will contain supervertices when the removal
        // point is on the boundary of the convex hull of the triangulation.
        bool IsSuperVertex;

        // Let V0 be the position of 'this' vertex.  If V0 is a supervertex or
        // is not an ear, its weight is +INFINITY.  Otherwise, let Vp be its
        // predecessor and let Vn be its successor when traversing the polygon
        // counterclockwise.  Let P be the removal point.  The weight is the
        // ratio
        //   Weight = H(Vp,V0,Vn,P)/D(Vp,V0,Vn)
        // where
        //           +              -+
        //   D = det | Vp.x  Vp.y  1 |
        //           | V0.x  V0.y  1 |
        //           | Vn.x  Vn.y  1 |
        //           +-             -+
        // and
        //           +-                            -+
        //   H = det | Vp.x  Vp.y  Vp.x^2+Vp.y^2  1 |
        //           | V0.x  V0.y  V0.x^2+V0.y^2  1 |
        //           | Vn.x  Vn.y  Vn.x^2+Vn.y^2  1 |
        //           | P.x   P.y   P.x^2+P.y^2    1 |
        //           +-                            -+
        Real Weight;

        // Vertex links for polygon.
        int VPrev, VNext;

        // Convex/reflex vertex links (disjoint lists).
        int SPrev, SNext;

        // Ear tip record.
        const TMinHeapRecord<int,Real>* EarRecord;
    };

    class WM4_FOUNDATION_ITEM Triangulate
    {
    public:
        Triangulate (std::vector<RPVertex>& rkPolygon, int iRemoval,
            IncrementalDelaunay2* pkDelaunay);

    private:
        // Prevent MSVC warning C4512 (assignment operator could not be
        // generated).  No assignment is needed by this class.
        Triangulate& operator= (const Triangulate&) { return *this; }

        RPVertex& V (int i);
        bool IsConvex (int i);
        bool IsEarTip (int i);
        void InsertAfterC (int i);   // insert convex vertex
        void InsertAfterR (int i);   // insert reflex vertesx
        void RemoveV (int i);        // remove vertex
        void RemoveR (int i);        // remove reflex vertex
        Real ComputeWeight (int iV0, int iP);

        std::vector<RPVertex>& m_rkPolygon;
        int m_iNumVertices;
        IncrementalDelaunay2* m_pkDelaunay;
        int m_iCFirst, m_iCLast;  // linear list of convex vertices
        int m_iRFirst, m_iRLast;  // linear list of reflex vertices
        TMinHeap<int,Real> m_kEHeap;  // priority queue of ear tips
    };

    // The directed line is <V0,V1>.  The return value is +1 when 'test' is
    // to the right of the line, -1 when 'test' is to the left of the line,
    // or 0 when 'test' is exactly on the line.
    int ToLine (const Vector2<Real>& rkTest, int iV0, int iV1) const;

    // The triangle is <V0,V1,V2>.  The return value is +1 when 'test' is
    // outside the triangle, -1 when 'test' is inside the triangle, or 0 when
    // 'test' is exactly on the triangle.
    int ToTriangle (const Vector2<Real>& rkTest, int iV0, int iV1, int iV2)
        const;

    // The triangle of the circumcircle is <V0,V1,V2>.  The return value is
    // +1 when 'test' is outside the circle, -1 when 'test' is inside the
    // circle, or 0 when 'test' is exactly on the circle.
    int ToCircumcircle (const Vector2<Real>& rkTest, int iV0, int iV1,
        int iV2) const;

    // Use a linear walk to find the triangle containing the point.
    Triangle* GetContainingTriangleInternal (const Vector2<Real>& rkPosition)
        const;

    // Return 'true' iff the specified triangle contains a supervertex.  This
    // function is used by GenerateRepresentation.
    bool ContainsSupervertex (Triangle* pkTri) const;

    // Swap the shared edge with the other diagonal of the quadrilateral
    // union of the two triangles.
    void SwapEdge (Triangle* pkTri0, Triangle* pkTri1);

private:
    // The rectangular domain in which all input points live.
    Real m_fXMin, m_fXMax, m_fYMin, m_fYMax;

    // The vertices of the triangulation.  The vertex pool stores the unique
    // positions that were passed to the Insert function.  This allows for a
    // fast look-up of vertices by the GetContainingTriangle function.
    VertexMap m_kVMap;
    std::vector<Vector2<Real> > m_kVertexPool;

    // This member is used to decide whether or not to accept the results of
    // ToLine when computed using floating-point arithmetic.  The test
    // involves a determinant sign.  When the determinant is sufficiently
    // small, the result is uncertain and the determinant is recomputed using
    // exact rational arithmetic.
    Real m_fUncertainty;

    // When the uncertainty is positive, filtered predicate queries are used
    // and storage is needed for rational vectors to minimize the computation
    // of such vectors.
    mutable std::vector<RVector>* m_pkRatVertexPool;
    mutable std::vector<bool>* m_pkRatVertexEvaluated;

    // The current triangulation.
    std::set<Triangle*> m_kTriangle;

    // Compacted informatoin about the triangulation.
    //   N = number of triangles
    //   I = Array of 3-tuples of indices into V that represent the
    //       triangles (3*N total elements).
    //   A = Array of 3-tuples of indices into I that represent the
    //       adjacent triangles (3*N total elements).
    // The i-th triangle has vertices
    //   vertex[0] = V[I[3*i+0]]
    //   vertex[1] = V[I[3*i+1]]
    //   vertex[2] = V[I[3*i+2]]
    // and edge index pairs
    //   edge[0] = <I[3*i+0],I[3*i+1]>
    //   edge[1] = <I[3*i+1],I[3*i+2]>
    //   edge[2] = <I[3*i+2],I[3*i+0]>
    // The triangles adjacent to these edges have indices
    //   adjacent[0] = A[3*i+0] is the triangle sharing edge[0]
    //   adjacent[1] = A[3*i+1] is the triangle sharing edge[1]
    //   adjacent[2] = A[3*i+2] is the triangle sharing edge[2]
    // If there is no adjacent triangle, the A[*] value is set to -1.  The
    // triangle adjacent to edge[j] has vertices
    //   adjvertex[0] = V[I[3*adjacent[j]+0]]
    //   adjvertex[1] = V[I[3*adjacent[j]+1]]
    //   adjvertex[2] = V[I[3*adjacent[j]+2]]
    int m_iNumTriangles;
    int* m_aiIndex;
    int* m_aiAdjacent;

    // Store the path of triangles visited in a GetContainingTriangle
    // function call.
    mutable int m_iPathLast;
    mutable int* m_aiPath;

    // If a query point is not in the convex hull of the input points, the
    // point is outside an edge of the last triangle in the search path.
    // These are the vertex indices for that edge.
    mutable int m_iLastEdgeV0, m_iLastEdgeV1;
    mutable int m_iLastEdgeOpposite, m_iLastEdgeOppositeIndex;
};

typedef IncrementalDelaunay2<float> IncrementalDelaunay2f;
typedef IncrementalDelaunay2<double> IncrementalDelaunay2d;

}

#endif
