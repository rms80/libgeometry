// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4PLANARGRAPH_H
#define WM4PLANARGRAPH_H

#include "Wm4FoundationLIB.h"
#include "Wm4EdgeKey.h"

// The Point2 template class must represent a 2-tuple, each component of some
// scalar type Numeric.  Point2 must define the following member functions.
//     Point2::Point2 ();
//     Point2::(Numeric, Numeric);
//     Point2::~Point2 ();
//     Point2& Point2::operator= (const Point2&);
//     Point2 Point2::operator- (const Point2&) const;
//     Numeric Point2::operator[] (int i) const;

namespace Wm4
{

template <class Point2>
class PlanarGraph
{
public:
    PlanarGraph ();
    virtual ~PlanarGraph ();

    class Vertex
    {
    public:
        Vertex (const Point2& rkPosition, int iIndex)
            :
            Position(rkPosition)
        {
            Index = iIndex;
        }

        ~Vertex ()
        {
        }

        void Insert (Vertex* pkAdjacent)
        {
            Adjacent.push_back(pkAdjacent);
        }

        void Remove (Vertex* pkAdjacent)
        {
            // Maintain a compact array.
            int iQuantity = (int)Adjacent.size();
            for (int i = 0; i < iQuantity; i++)
            {
                if (pkAdjacent == Adjacent[i])
                {
                    // Maintain a compact array.
                    iQuantity--;
                    if (i < iQuantity)
                    {
                        Adjacent[i] = Adjacent[iQuantity];
                    }
                    Adjacent.pop_back();
                    return;
                }
            }
        }

        // The planar position for the vertex.
        Point2 Position;

        // A unique identifier for the vertex.
        int Index;

        // The adjacent vertices.
        std::vector<Vertex*> Adjacent;
    };

    typedef std::map<int,Vertex*> Vertices;
    typedef std::map<EdgeKey,bool> Edges;

    const Vertices& GetVertices () const;
    const Vertex* GetVertex (int iIndex) const;
    bool InsertVertex (const Point2& rkPosition, int iIndex);
    bool RemoveVertex (int iIndex);

    const Edges& GetEdges () const;
    bool InsertEdge (int iIndex0, int iIndex1);
    bool RemoveEdge (int iIndex0, int iIndex1);

    // Traverse the graph and extract the isolated vertices, filaments, and
    // minimal cycles.  See MinimalCycleBasis.pdf for the details.

    enum PrimitiveType
    {
        PT_ISOLATED_VERTEX,
        PT_FILAMENT,
        PT_MINIMAL_CYCLE
    };

    class Primitive
    {
    public:
        Primitive (PrimitiveType eType)
        {
            Type = eType;
        }

        PrimitiveType Type;
        std::vector<std::pair<Point2,int> > Sequence;
    };

    // The extraction of primitives destroys the graph.  If you need the
    // graph to persist, make a copy of it and call this function from the
    // copy.
    void ExtractPrimitives (std::vector<Primitive*>& rkPrimitives);

protected:
    // For sorting of the heap of vertex pointers.
    class VertexPtr
    {
    public:
        VertexPtr (Vertex* pkVertex)
        {
            m_pkVertex = pkVertex;
        }

        operator Vertex* ()
        {
            return m_pkVertex;
        }

        // Lexicographical ordering of vertices.  The query (x0,y0) < (x1,y1)
        // is true iff ((x0 < x1) || ((x0 == x1) && (y0 < y1))).
        bool operator< (const VertexPtr& rkVertexPtr) const
        {
            if (m_pkVertex->Position[0] < rkVertexPtr.m_pkVertex->Position[0])
            {
                return true;
            }

            if (m_pkVertex->Position[0] > rkVertexPtr.m_pkVertex->Position[0])
            {
                return false;
            }

            return m_pkVertex->Position[1] <
                rkVertexPtr.m_pkVertex->Position[1];
        }


    private:
        Vertex* m_pkVertex;
    };

    void SetCycleEdge (int iIndex0, int iIndex1, bool bCycleEdge);
    bool GetCycleEdge (int iIndex0, int iIndex1) const;

    void ExtractIsolatedVertex (Vertex* pkV0, std::set<VertexPtr>& rkHeap,
        std::vector<Primitive*>& rkPrimitives);

    void ExtractFilament (Vertex* pkV0, Vertex* pkV1,
        std::set<VertexPtr>& rkHeap, std::vector<Primitive*>& rkPrimitives);

    void ExtractPrimitive (Vertex* pkV0, std::set<VertexPtr>& rkHeap,
        std::vector<Primitive*>& rkPrimitives);

    Vertex* GetClockwiseMost (Vertex* pkVPrev, Vertex* pkVCurr);
    Vertex* GetCounterclockwiseMost (Vertex* pkVPrev, Vertex* pkVCurr);

    Vertices m_kVertices;
    Edges m_kEdges;
};

#include "Wm4PlanarGraph.inl"

}

#endif
