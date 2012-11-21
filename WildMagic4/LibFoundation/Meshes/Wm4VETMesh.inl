// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline int VETMesh::GetVertexQuantity () const
{
    return (int)m_kVMap.size();
}
//----------------------------------------------------------------------------
inline int VETMesh::GetEdgeQuantity () const
{
    return (int)m_kEMap.size();
}
//----------------------------------------------------------------------------
inline int VETMesh::GetTriangleQuantity () const
{
    return (int)m_kTMap.size();
}
//----------------------------------------------------------------------------
inline VETMesh* VETMesh::Create () const
{
    return new VETMesh;
}
//----------------------------------------------------------------------------
inline void VETMesh::OnVertexInsert (int,bool,void*&)
{
}
//----------------------------------------------------------------------------
inline void VETMesh::OnVertexRemove (int,bool,void*)
{
}
//----------------------------------------------------------------------------
inline void VETMesh::OnEdgeInsert (const Edge&,bool,void*&)
{
}
//----------------------------------------------------------------------------
inline void VETMesh::OnEdgeRemove (const Edge&,bool,void*)
{
}
//----------------------------------------------------------------------------
inline void VETMesh::OnTriangleInsert (const Triangle&,bool,void*&)
{
}
//----------------------------------------------------------------------------
inline void VETMesh::OnTriangleRemove (const Triangle&,bool,void*)
{
}
//----------------------------------------------------------------------------
inline const VETMesh::VMap& VETMesh::GetVertexMap () const
{
    return m_kVMap;
}
//----------------------------------------------------------------------------
inline const VETMesh::EMap& VETMesh::GetEdgeMap () const
{
    return m_kEMap;
}
//----------------------------------------------------------------------------
inline const VETMesh::TMap& VETMesh::GetTriangleMap () const
{
    return m_kTMap;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// VETMesh::Edge
//----------------------------------------------------------------------------
inline VETMesh::Edge::Edge ()
{
}
//----------------------------------------------------------------------------
inline VETMesh::Edge::Edge (int iV0, int iV1)
{
    if (iV0 < iV1)
    {
        // v0 is minimum.
        V[0] = iV0;
        V[1] = iV1;
    }
    else
    {
        // v1 is minimum.
        V[0] = iV1;
        V[1] = iV0;
    }
}
//----------------------------------------------------------------------------
inline bool VETMesh::Edge::operator< (const Edge& rkE) const
{
    if (V[1] < rkE.V[1])
    {
        return true;
    }

    if (V[1] == rkE.V[1])
    {
        return V[0] < rkE.V[0];
    }

    return false;
}
//----------------------------------------------------------------------------
inline bool VETMesh::Edge::operator== (const Edge& rkE) const
{
    return V[0] == rkE.V[0] && V[1] == rkE.V[1];
}
//----------------------------------------------------------------------------
inline bool VETMesh::Edge::operator!= (const Edge& rkE) const
{
    return !operator==(rkE);
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// VETMesh::Triangle
//----------------------------------------------------------------------------
inline VETMesh::Triangle::Triangle ()
{
}
//----------------------------------------------------------------------------
inline VETMesh::Triangle::Triangle (int iV0, int iV1, int iV2)
{
    if (iV0 < iV1)
    {
        if (iV0 < iV2)
        {
            // v0 is minimum.
            V[0] = iV0;
            V[1] = iV1;
            V[2] = iV2;
        }
        else
        {
            // v2 is minimum.
            V[0] = iV2;
            V[1] = iV0;
            V[2] = iV1;
        }
    }
    else
    {
        if ( iV1 < iV2 )
        {
            // v1 is minimum.
            V[0] = iV1;
            V[1] = iV2;
            V[2] = iV0;
        }
        else
        {
            // v2 is minimum.
            V[0] = iV2;
            V[1] = iV0;
            V[2] = iV1;
        }
    }
}
//----------------------------------------------------------------------------
inline bool VETMesh::Triangle::operator< (const Triangle& rkT) const
{
    if (V[2] < rkT.V[2])
    {
        return true;
    }

    if (V[2] == rkT.V[2])
    {
        if (V[1] < rkT.V[1])
        {
            return true;
        }

        if (V[1] == rkT.V[1])
        {
            return V[0] < rkT.V[0];
        }
    }

    return false;
}
//----------------------------------------------------------------------------
inline bool VETMesh::Triangle::operator== (const Triangle& rkT) const
{
    return (V[0] == rkT.V[0]) &&
          ((V[1] == rkT.V[1] && V[2] == rkT.V[2]) ||
           (V[1] == rkT.V[2] && V[2] == rkT.V[1]));
}
//----------------------------------------------------------------------------
inline bool VETMesh::Triangle::operator!= (const Triangle& rkT) const
{
    return !operator==(rkT);
}
//----------------------------------------------------------------------------
