// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline bool BasicMesh::IsValid () const
{
    return m_bIsValid;
}
//----------------------------------------------------------------------------
inline int BasicMesh::GetVQuantity () const
{
    return m_iVQuantity;
}
//----------------------------------------------------------------------------
inline int BasicMesh::GetEQuantity () const
{
    return m_iEQuantity;
}
//----------------------------------------------------------------------------
inline int BasicMesh::GetTQuantity () const
{
    return m_iTQuantity;
}
//----------------------------------------------------------------------------
inline const void* BasicMesh::GetPoints () const
{
    return m_akPoint;
}
//----------------------------------------------------------------------------
inline const int* BasicMesh::GetIndices () const
{
    return m_aiIndex;
}
//----------------------------------------------------------------------------
inline const BasicMesh::Vertex* BasicMesh::GetVertices () const
{
    return m_akVertex;
}
//----------------------------------------------------------------------------
inline const BasicMesh::Edge* BasicMesh::GetEdges () const
{
    return m_akEdge;
}
//----------------------------------------------------------------------------
inline const BasicMesh::Triangle* BasicMesh::GetTriangles () const
{
    return m_akTriangle;
}
//----------------------------------------------------------------------------
