// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4ETNONMANIFOLDMESH_H
#define WM4ETNONMANIFOLDMESH_H

#include "Wm4FoundationLIB.h"
#include "Wm4EdgeKey.h"
#include "Wm4TriangleKey.h"

namespace Wm4
{

class WM4_FOUNDATION_ITEM ETNonmanifoldMesh
{
public:
    // edge data types
    class Edge;
    typedef Edge* EPtr;
    typedef const Edge* ECPtr;
    typedef EPtr (*ECreator)(int,int);
    typedef std::map<EdgeKey,Edge*> EMap;
    typedef EMap::iterator EMapIterator;
    typedef EMap::const_iterator EMapCIterator;

    // triangle data types
    class Triangle;
    typedef Triangle* TPtr;
    typedef const Triangle* TCPtr;
    typedef TPtr (*TCreator)(int,int,int);
    typedef std::map<TriangleKey,Triangle*> TMap;
    typedef TMap::iterator TMapIterator;
    typedef TMap::const_iterator TMapCIterator;

    // edge object
    class WM4_FOUNDATION_ITEM Edge
    {
    public:
        Edge (int iV0, int iV1);
        virtual ~Edge ();

        int V[2];
        std::set<Triangle*> T;
    };

    class WM4_FOUNDATION_ITEM Triangle
    {
    public:
        Triangle (int iV0, int iV1, int iV2);
        virtual ~Triangle ();

        // vertices (V[0],V[1],V[2])
        int V[3];

        // adjacent edges
        // E[0] represents edge (V[0],V[1])
        // E[1] represents edge (V[1],V[2])
        // E[2] represents edge (V[2],V[0])
        EPtr E[3];
    };


    ETNonmanifoldMesh (ECreator oECreator = 0, TCreator oTCreator = 0);
    virtual ~ETNonmanifoldMesh ();

    // member access
    int GetEdgeQuantity () const;
    const EMap& GetEdges () const;
    int GetTriangleQuantity () const;
    const TMap& GetTriangles () const;

    // mesh manipulation
    TPtr InsertTriangle (int iV0, int iV1, int iV2);
    bool RemoveTriangle (int iV0, int iV1, int iV2);

    // topological operations
    bool IsManifold () const;
    bool IsClosed () const;
    bool IsConnected () const;

    // Extract a connected component from the mesh and remove all the
    // triangles of the component from the mesh.  This is useful for computing
    // the components in a very large mesh that uses a lot of memory.  The
    // intention is that the function is called until all components are
    // found.  The typical code is
    //
    //     ETNonmanifoldMesh kMesh = <some mesh>;
    //     int iITotalQuantity = 3*kMesh.GetTriangleQuantity();
    //     int* aiConnect = WM4_NEW int[iITotalQuantity];
    //     for (int iIQuantity = 0; iIQuantity < iITotalQuantity; /**/ )
    //     {
    //         int iCurrentIQuantity;
    //         int* aiCurrentConnect = aiConnect + iIQuantity;
    //         kMesh.RemoveComponent(iCurrentIQuantity,aiCurrentConnect);
    //         iIQuantity += iCurrentIQuantity;
    //     }
    void RemoveComponent (int& riIQuantity, int* aiConnect);

    void Print (const char* acFilename);

protected:
    static EPtr CreateEdge (int iV0, int iV1);
    ECreator m_oECreator;
    EMap m_kEMap;

    static TPtr CreateTriangle (int iV0, int iV1, int iV2);
    TCreator m_oTCreator;
    TMap m_kTMap;
};

#include "Wm4ETNonmanifoldMesh.inl"

}

#endif
