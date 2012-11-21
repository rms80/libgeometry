// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "VFMeshMerge.h"
#include <MeshUtils.h>
#include "rmsdebug.h"

using namespace rms;

VFMeshMerge::VFMeshMerge( ) 
{
	m_pBaseMesh = NULL;
	m_pBaseMask = NULL;
	m_pMergeMesh = NULL;
	UpdateMaxIDs();
}


void VFMeshMerge::Initialize( IMesh * pBaseMesh, VFMeshMask * pMask, IMesh * pMergeMesh,
							  VertexPair * vPairs, unsigned int nPairs )
{
	m_pBaseMesh = pBaseMesh;
	m_pBaseMask = pMask;
	m_pMergeMesh = pMergeMesh;
	UpdateMaxIDs();

	for ( unsigned int i = 0; i < nPairs; ++i ) {
		m_vBaseToMerge.set( vPairs[i].baseVID, vPairs[i].mergeVID );
		m_vMergeToBase.set( vPairs[i].mergeVID, vPairs[i].baseVID );
	}
}


void VFMeshMerge::UpdateMaxIDs()
{
	if ( m_pBaseMesh ) {
		m_nMaxBaseTID = m_pBaseMesh->GetTriangleCount();
		m_nMaxBaseVID = m_pBaseMesh->GetVertexCount();	
	} else {
		m_nMaxBaseTID = 0;
		m_nMaxBaseVID = 0;
	}
	if ( m_pMergeMesh ) {
		m_nMaxMergeTID = m_pMergeMesh->GetTriangleCount();
		m_nMaxMergeVID = m_pMergeMesh->GetVertexCount();	
	} else {
		m_nMaxMergeTID = 0;
		m_nMaxMergeVID = 0;
	}
	m_nMaxTID = m_nMaxBaseTID + m_nMaxMergeTID;
	m_nMaxVID = m_nMaxBaseVID + m_nMaxMergeVID;
	m_vBaseToMerge.clear(false);  m_vBaseToMerge.resize( m_nMaxBaseVID );
	m_vMergeToBase.clear(false);  m_vMergeToBase.resize( m_nMaxMergeVID );
}


void VFMeshMerge::GetVertex( VertexID vID, Wml::Vector3f & vVertex, Wml::Vector3f * pNormal ) const 
{ 
	if ( IsMergeVID(vID) ) {
		m_pMergeMesh->GetVertex( ToMergeVID(vID), vVertex, pNormal );
	} else {
		m_pBaseMesh->GetVertex( vID, vVertex, pNormal );
	}
}
void VFMeshMerge::GetNormal( VertexID vID, Wml::Vector3f & vNormal ) const
{ 
	if ( IsMergeVID(vID) ) {
		m_pMergeMesh->GetNormal( ToMergeVID(vID), vNormal );
	} else {
		m_pBaseMesh->GetNormal( vID, vNormal );
	}
}

void VFMeshMerge::GetTriangle( TriangleID tID, VertexID vTriangle[3]  ) const
{
	if ( IsMergeTID(tID) ) {
		m_pMergeMesh->GetTriangle( ToMergeTID(tID), vTriangle );

		// re-write boundary vertices
		for ( int j = 0; j < 3; ++j ) {
			if ( m_vMergeToBase.has( vTriangle[j] ) )
				vTriangle[j] = m_vMergeToBase.get( vTriangle[j] );
			else
				vTriangle[j] = FromMergeVID( vTriangle[j] );
		}

	} else {
		m_pBaseMesh->GetTriangle( tID, vTriangle );
	}
}

void VFMeshMerge::GetTriangle( TriangleID tID, Wml::Vector3f vTriangle[3], Wml::Vector3f * pNormals ) const
{
	if ( IsMergeTID(tID) ) {
		VertexID nTri[3];
		m_pMergeMesh->GetTriangle( ToMergeTID(tID), nTri );
		for ( int j = 0; j < 3; ++j ) {
			if ( m_vMergeToBase.has( nTri[j] ) )   // rewrite boundary vertices
				nTri[j] = m_vMergeToBase.get( nTri[j] );
			else
				nTri[j] = FromMergeVID( nTri[j] );
			GetVertex( nTri[j], vTriangle[j], (pNormals) ? &pNormals[j] : NULL );
		}

	} else {
		m_pBaseMesh->GetTriangle( tID, vTriangle, pNormals );
	}
}


static const unsigned int BASE_ONLY = 0;
static const unsigned int MERGE_ONLY = 1;
static const unsigned int BOTH_BASE = 2;
static const unsigned int BOTH_MERGE = 3;

//FUCK FUCK FUCK HAVE TO TRACK STATE A DIFFFERENT WAY!!! FLAG
//IS OVERWRITTEN BY SECOND MERGE ITERATION!!


//! initialize vertex neighbour iteration
void VFMeshMerge::BeginVtxTriangles( VtxNbrItr & v ) const
{
	bool bIsJoinVtx = (IsMergeVID(v.vID) == false) && m_vBaseToMerge.has(v.vID);
	if ( ! bIsJoinVtx ) {
		if ( ! IsMergeVID(v.vID) ) {
			m_pBaseMesh->BeginVtxTriangles( v );
			v.nData[1] = BASE_ONLY;
		} else {
			v.vID = ToMergeVID(v.vID);
			m_pMergeMesh->BeginVtxTriangles( v );
			v.vID = FromMergeVID(v.vID);
			v.nData[1] = MERGE_ONLY;
		}
	} else {
		m_pBaseMask->BeginVtxTriangles(v);
		v.nData[1] = BOTH_BASE;
	}
}

//! (possibly) un-ordered iteration around one-ring of a vertex. Returns InvalidID when done
IMesh::TriangleID VFMeshMerge::GetNextVtxTriangle( VtxNbrItr & v ) const
{
	switch ( v.nData[1] ) {
		case BASE_ONLY:
			return m_pBaseMesh->GetNextVtxTriangle( v );
		case BOTH_MERGE:
		case MERGE_ONLY: 
		{
			v.vID = ToMergeVID(v.vID);
			TriangleID tID = m_pMergeMesh->GetNextVtxTriangle( v );
			v.vID = FromMergeVID(v.vID);
			if (tID == IMesh::InvalidID)
				return IMesh::InvalidID;
			else
				return FromMergeTID( tID );
		}

		case BOTH_BASE: 
		{
			TriangleID tID = m_pBaseMask->GetNextVtxTriangle( v );
			if ( tID == InvalidID ) {

				VertexID vTmp = v.vID;
				v.vID = m_vBaseToMerge[v.vID];
				m_pMergeMesh->BeginVtxTriangles( v );
				v.vID = FromMergeVID(v.vID);
				v.nData[1] = BOTH_MERGE;

				return GetNextVtxTriangle(v);
			} else {
				return tID;
			}
		}

		default:
			lgASSERT(0);
			DebugBreak();
	}
	return InvalidID;
}



class MergeMeshRewriteCallback : public IMesh::NeighborTriCallback
{
public:
	MergeMeshRewriteCallback( IMesh::NeighborTriCallback * pPassThru, VFMeshMerge * pMergeMesh )
		{ m_pPassThru = pPassThru; m_pMergeMesh = pMergeMesh; }

	virtual void NextTriangle( IMesh::TriangleID tID )
		{ m_pPassThru->NextTriangle(  m_pMergeMesh->FromMergeTID(tID)   ); }

protected:
	IMesh::NeighborTriCallback * m_pPassThru;
	VFMeshMerge * m_pMergeMesh;
};



void VFMeshMerge::NeighbourIteration( VertexID vID, NeighborTriCallback * pCallback )
{
	bool bIsJoinVtx = (IsMergeVID(vID) == false) && m_vBaseToMerge.has(vID);
	if ( ! bIsJoinVtx ) {
		if ( ! IsMergeVID(vID) ) {
			m_pBaseMesh->NeighbourIteration(vID, pCallback);
		} else {
			vID = ToMergeVID(vID);
			MergeMeshRewriteCallback rewriter(pCallback, this);
			pCallback->BeginTriangles();
			m_pMergeMesh->NeighbourIteration(vID, &rewriter);
			pCallback->EndTriangles();
		}
	} else {
		pCallback->BeginTriangles();

		// iterate over mask triangles
		NeighborTriPassThruCallback passThru1(pCallback);
		m_pBaseMask->NeighbourIteration(vID, &passThru1);

		VertexID vMergeID = m_vBaseToMerge[vID];
		MergeMeshRewriteCallback rewriter(pCallback, this);
		m_pMergeMesh->NeighbourIteration(vMergeID, &rewriter);

		pCallback->EndTriangles();
	}

}



//! determine if a vertex is on the mesh boundary (if there is one)  (depends on GetNextVtxTriangle)
bool VFMeshMerge::IsBoundaryVertex( VertexID vID ) const
{
	// this version seems buggy, because BaseToMerge is only as large as BaseMesh,
	// but vID may not be in the base mesh...
#if 0 
	bool bIsJoinVtx = m_vBaseToMerge.has(vID);
	if ( ! bIsJoinVtx ) {
		if ( ! IsMergeVID(vID) )
			return m_pBaseMesh->IsBoundaryVertex(vID);
		 else
			return m_pMergeMesh->IsBoundaryVertex( ToMergeVID(vID) );
	} else
		return false;	// assuming here that boundary vertices will always be connected...
						// alternately could call IMesh::IsBoundaryVertex...
#endif

	if ( ! IsMergeVID(vID) ) {
		bool bIsJoinVtx = m_vBaseToMerge.has(vID);
		if ( ! bIsJoinVtx )
			return m_pBaseMesh->IsBoundaryVertex(vID);
	} else {
		return m_pMergeMesh->IsBoundaryVertex( ToMergeVID(vID) );
	}
	return false;
}

