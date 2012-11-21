// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include ".\meshselection.h"

#include <map>

using namespace rms;

MeshSelection::MeshSelection(const VFTriangleMesh * pMesh)
{
	m_pMesh = pMesh;
}

MeshSelection::~MeshSelection(void)
{
}


void MeshSelection::ClearAll()
{
	m_vTriangles.clear();
	m_vVertices.clear();
	m_vVertexDelta.clear();
}


void MeshSelection::SelectVertex( IMesh::VertexID tID )
{
	m_vVertices.insert(tID);
}

void MeshSelection::SelectVertices( IMesh::TriangleID tID )
{
	IMesh::VertexID nTri[3];
	m_pMesh->GetTriangle(tID, nTri);
	for ( int j = 0; j < 3; ++j )
		m_vVertices.insert(nTri[j]);
}


void MeshSelection::SelectFace( IMesh::TriangleID vID )
{
	m_vTriangles.insert(vID);
}

void MeshSelection::SelectFaces( IMesh::VertexID vID )
{
	VFTriangleMesh::VtxNbrItr itr(vID);
	m_pMesh->BeginVtxTriangles(itr);
	IMesh::TriangleID tID = m_pMesh->GetNextVtxTriangle(itr);
	while ( tID != IMesh::InvalidID ) {
		m_vTriangles.insert(tID);
		tID = m_pMesh->GetNextVtxTriangle(itr);
	}
}

void MeshSelection::SelectFaces( BitSet & vFaces )
{
	size_t nCount = vFaces.size();
	for ( unsigned int k = 0; k < nCount; ++k ) {
		if ( vFaces[k] )
			m_vTriangles.insert(k);
	}
}


void MeshSelection::SelectFaces( const std::vector<IMesh::VertexID> & vVerts )
{
	size_t nCount = vVerts.size();
	for ( unsigned int i = 0; i < nCount; ++i )
		SelectFaces( vVerts[i] );
}


void MeshSelection::SelectAllFaces()
{
	VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
	while ( curt != endt ) 
		m_vTriangles.insert( *curt++ );
}



void MeshSelection::GrowFaces( bool bComputeVertDelta )
{
	std::vector<IMesh::TriangleID> vCurTriangles(m_vTriangles.begin(), m_vTriangles.end());
	size_t nCount = vCurTriangles.size();
	IMesh::VertexID nTri[3];

	// save list of old selected verts if pNewVerts are requested
	std::set<IMesh::VertexID> vCurVerts;
	if ( bComputeVertDelta ) {
		for ( unsigned int i = 0; i < nCount; ++i ) {
			m_pMesh->GetTriangle( vCurTriangles[i], nTri );
			for ( int j = 0; j < 3; ++j )
				vCurVerts.insert( nTri[j] );
		}
	}

	for ( unsigned int i = 0; i < nCount; ++i ) {
		m_pMesh->GetTriangle( vCurTriangles[i], nTri );
		for ( int j = 0; j < 3; ++j )
			SelectFaces( nTri[j] );
	}

	if ( bComputeVertDelta ) {
		std::set<IMesh::TriangleID>::iterator curt(m_vTriangles.begin()), endt(m_vTriangles.end());
		while ( curt != endt ) {
			m_pMesh->GetTriangle( *curt++, nTri );
			for ( int j = 0; j < 3; ++j ) {
				if ( vCurVerts.find( nTri[j] ) == vCurVerts.end() )
					m_vVertexDelta.insert( nTri[j] );
			}
		}
	}
}


void MeshSelection::GrowVertices( bool bComputeVertDelta, int nIters )
{
	if ( bComputeVertDelta )
		m_vVertexDelta.clear();

	// [TODO] this would be a lot faster if we used a sparse bitset to represent selection

	for ( int n = 0; n < nIters; ++n ) {
		std::set<IMesh::VertexID> vCurVertices = m_vVertices;
		std::set<IMesh::VertexID>::iterator curv(vCurVertices.begin()), endv(vCurVertices.end());
		while ( curv != endv ) { 
			IMesh::VertexID vID = *curv++;

			IMesh::VtxNbrItr itr(vID);
			m_pMesh->BeginVtxEdges(itr);
			IMesh::EdgeID eID = m_pMesh->GetNextVtxEdges(itr);
			while ( eID != IMesh::InvalidID ) {
				IMesh::VertexID edgeV[2];  IMesh::EdgeID edgeT[2];
				m_pMesh->GetEdge(eID, edgeV, edgeT);

				IMesh::VertexID vNbr = (edgeV[0] == vID) ? edgeV[1] : edgeV[0];
				if ( bComputeVertDelta && vCurVertices.find(vNbr) == vCurVertices.end() )
					m_vVertexDelta.insert(vNbr);
				m_vVertices.insert(vNbr);

				eID = m_pMesh->GetNextVtxEdges(itr);
			}
		}
	}
}



bool MeshSelection::HasVertex(IMesh::VertexID vID) const
{
	return m_vVertices.find(vID) != m_vVertices.end();
}
bool MeshSelection::HasFace(IMesh::VertexID vID) const
{
	return m_vTriangles.find(vID) != m_vTriangles.end();
}


void MeshSelection::SelectFaceVertices()
{
	std::set<IMesh::TriangleID>::iterator curt(m_vTriangles.begin()), endt(m_vTriangles.end());
	while ( curt != endt )
		SelectVertices( *curt++ );
}


//! select one-ring of this vertex
void MeshSelection::SelectVertexFaces( IMesh::VertexID vID )
{
	IMesh::VtxNbrItr itr(vID);
	m_pMesh->BeginVtxTriangles(itr);
	IMesh::TriangleID tID = m_pMesh->GetNextVtxTriangle(itr);
	while ( tID != IMesh::InvalidID ) {
		SelectFace(tID);
		tID = m_pMesh->GetNextVtxTriangle(itr);
	}
}


void MeshSelection::SelectVertexFaces(bool bFullOnly)
{
	VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		IMesh::VertexID nTri[3];
		m_pMesh->GetTriangle(tID, nTri);
		if ( bFullOnly ) {
			if ( HasVertex(nTri[0]) && HasVertex(nTri[1]) && HasVertex(nTri[2]) )
				SelectFace( tID );
		} else {
			if ( HasVertex(nTri[0]) || HasVertex(nTri[1]) || HasVertex(nTri[2]) )
				SelectFace( tID );
		}
	}
}


//! flood-fill selection of all vertices connected to vID
void MeshSelection::FloodSelectVertices( IMesh::VertexID vSeedID )
{
	std::vector<IMesh::VertexID> vStack;
	vStack.push_back(vSeedID);
	while (! vStack.empty() ) {
		IMesh::VertexID vID = vStack.back();
		vStack.pop_back();
		m_vVertices.insert(vID);

		IMesh::VtxNbrItr itr(vID);
		m_pMesh->BeginVtxEdges(itr);
		IMesh::EdgeID eID = m_pMesh->GetNextVtxEdges(itr);
		while ( eID != IMesh::InvalidID ) {
			IMesh::VertexID edgeV[2];  IMesh::TriangleID edgeT[2];
			m_pMesh->GetEdge(eID, edgeV, edgeT);

			IMesh::VertexID vOther = (edgeV[0] == vID) ? edgeV[1] : edgeV[0];
			if ( m_vVertices.find(vOther) == m_vVertices.end() )
				vStack.push_back(vOther);

			eID = m_pMesh->GetNextVtxEdges(itr);
		}
	}
}




//! convert to bitset
void MeshSelection::SelectedVertices( rms::BitSet & bits ) const
{
	bits.resize(0);
	bits.resize( m_pMesh->GetMaxVertexID() );
	std::set<IMesh::VertexID>::const_iterator curv(m_vVertices.begin()), endv(m_vVertices.end());
	while ( curv != endv )
		bits.set( *curv++, true );
}




void MeshSelection::GetSelectionMesh( rms::VFTriangleMesh & mesh )
{
	mesh.Clear(false);

	std::set<IMesh::VertexID> vOldVerts = m_vVertices;
	m_vVertices.clear();
	SelectFaceVertices();
	
	std::map<IMesh::VertexID, IMesh::VertexID> vMap;
	std::set<IMesh::VertexID>::iterator curv(m_vVertices.begin()), endv(m_vVertices.end());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		Wml::Vector3f vVertex, vNormal;
		m_pMesh->GetVertex( vID, vVertex, &vNormal );
		vMap[vID] = mesh.AppendVertex( vVertex, &vNormal );
	}

	std::set<IMesh::TriangleID>::iterator curt(m_vTriangles.begin()), endt(m_vTriangles.end());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		IMesh::VertexID nTri[3];
		m_pMesh->GetTriangle(tID, nTri);
		mesh.AppendTriangle( vMap[nTri[0]], vMap[nTri[1]], vMap[nTri[2]] );
	}

	m_vVertices = vOldVerts;
}


