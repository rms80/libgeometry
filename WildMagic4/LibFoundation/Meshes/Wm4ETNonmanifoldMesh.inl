// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline int ETNonmanifoldMesh::GetEdgeQuantity () const
{
    return (int)m_kEMap.size();
}
//----------------------------------------------------------------------------
inline const ETNonmanifoldMesh::EMap& ETNonmanifoldMesh::GetEdges () const
{
    return m_kEMap;
}
//----------------------------------------------------------------------------
inline int ETNonmanifoldMesh::GetTriangleQuantity () const
{
    return (int)m_kTMap.size();
}
//----------------------------------------------------------------------------
inline const ETNonmanifoldMesh::TMap& ETNonmanifoldMesh::GetTriangles () const
{
    return m_kTMap;
}
//----------------------------------------------------------------------------
