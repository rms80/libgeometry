// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "MeshInsertion.h"
#include "VectorUtil.h"
#include "MeshUtils.h"

#include "rmsdebug.h"

using namespace rms;

MeshInsertion::MeshInsertion()
{
	m_pInsertMesh = NULL;
	m_pDestMesh = NULL;
	m_pProjector = NULL;

	m_bUseHoleHack = false;
	m_nUseSingleInsertionLoop = -1;
	m_fInsertionScale = 1.0f;
}


bool MeshInsertion::SetDestMesh( rms::VFTriangleMesh * pDestMesh )
{
	m_pDestMesh = pDestMesh;

	m_vDestLoops.resize(0);
	MeshUtils::FindBoundaryLoops( *m_pDestMesh, m_vDestLoops );
	if ( m_vDestLoops.size() != 1 )
		return false;
	return true;
}

void MeshInsertion::SetInsertMesh( rms::VFTriangleMesh * pInsertMesh )
{
	m_pInsertMesh = pInsertMesh;

	m_vInsertLoops.resize(0);
	MeshUtils::FindBoundaryLoops( *m_pInsertMesh, m_vInsertLoops );
}

void MeshInsertion::SetInsertionScale( float fScale, const Wml::Vector2f & vInsertionOrigin )
{
	m_fInsertionScale = fScale;
	m_vInsertionOrigin = vInsertionOrigin;
}

void MeshInsertion::Set3DProjector( ISurfaceProjector * pProjector )
{
	m_pProjector = pProjector;
}




bool MeshInsertion::Compute()
{
	m_MergedMesh.Clear(false);
	m_vInsertToMergeVMap.Clear();
	m_vInsertToMergeTMap.Clear();
	m_vInsertBdryToMergeMap.Clear();
	m_vDestToMergeMap.Clear();

	// insert source UV mesh into dest UV mesh via delaunay
	rms::Triangulator2D trigen;

	// add all points from current dest mesh
	int nDestOffset = 10;
	VFTriangleMesh::vertex_iterator curd(m_pDestMesh->BeginVertices()), endd(m_pDestMesh->EndVertices());	
	while ( curd != endd ) {
		IMesh::VertexID vID = *curd++;
		Wml::Vector2f vUV;
		if (! m_pDestMesh->GetUV(vID, 0, vUV) )
			return false;
		if ( trigen.AddPoint( vUV[0], vUV[1], vID+nDestOffset ) != vID )		// assuming this in next loop
			lgBreakToDebugger();
	}

	// add boundary segments from current dest mesh
	size_t nDestLoop = m_vDestLoops[0].size();
	for ( unsigned int k = 0; k < nDestLoop; ++k )
		trigen.AddSegment(m_vDestLoops[0][k], m_vDestLoops[0][(k+1)%nDestLoop]);

	// add points and edges from source polygons
	int nSourceOffset = 2*m_pDestMesh->GetMaxVertexID();
	size_t nLoops = m_vInsertLoops.size();
	for ( unsigned int li = 0; li < nLoops; ++li ) {
		if ( m_nUseSingleInsertionLoop >= 0 && li != m_nUseSingleInsertionLoop )
			continue;
		int nLoop = (int)m_vInsertLoops[li].size();
		unsigned int nPrev = -1;  unsigned int nFirst = -1;
		for ( int k = 0; k < nLoop; ++k ) {
			IMesh::VertexID vID = m_vInsertLoops[li][k];
			Wml::Vector2f vUV;
			if (! m_pInsertMesh->GetUV(vID, 0, vUV) )
				return false;
			//vUV *= m_fInsertionScale;
			vUV = (vUV-m_vInsertionOrigin) * m_fInsertionScale + m_vInsertionOrigin;
			unsigned int nCur = trigen.AddPoint( vUV[0], vUV[1], vID+nSourceOffset );
			if ( nPrev != -1 )
				trigen.AddSegment( nPrev, nCur );
			nPrev = nCur;
			if ( nFirst == -1 )  
				nFirst = nCur;
		}
		trigen.AddSegment( nPrev, nFirst );

		// need to add hole somewhere inside this loop. Centroid of one of the triangles will do...
		if ( m_bUseHoleHack ) {
			trigen.AddHole( m_vHoleHack[0], m_vHoleHack[1] );

		} else {
			IMesh::EdgeID eID = m_pInsertMesh->FindEdge(m_vInsertLoops[li][0], m_vInsertLoops[li][1]);
			IMesh::VertexID edgeV[2];  IMesh::VertexID edgeT[2];
			m_pInsertMesh->GetEdge( eID, edgeV, edgeT);
			Wml::Vector2f vTriUV[3];
			if ( ! m_pInsertMesh->GetTriangleUV( (edgeT[0] == IMesh::InvalidID) ? edgeT[1] : edgeT[0], 0, vTriUV ) )
				return false;
			Wml::Vector2f vCentroid = (vTriUV[0] + vTriUV[1] + vTriUV[2]) / 3.0f;
			//vCentroid *= m_fInsertionScale;
			vCentroid = (vCentroid-m_vInsertionOrigin) * m_fInsertionScale + m_vInsertionOrigin;
			trigen.AddHole( vCentroid[0], vCentroid[1] );
		}
	}

	trigen.SetEnclosingSegmentsProvided(true);
	trigen.SetSubdivideAnySegments(false);
	bool bOK = trigen.Compute();
	if ( ! bOK )
		lgBreakToDebugger();

	// The triangulator will by default output a mesh which contains all the input vertices,
	// even those which were removed because they were inside a hole, etc.
	// Passing true here will compact mesh, but if some of the inserted boundary vertices
	// are exactly the same as detination vertices, then one of them will be left unused
	// by the triangulator. It is very bad if it is the inserted boundary vertex, because
	// then m_vInsertBdryToMergeMap won't have an entry for it. Even if we don't compact,
	// though, that vertex might not actually be connected to any triangles!
	// Need to find some way to deal with this case...
	trigen.MakeTriMesh(	m_MergedMesh, &Wml::Vector3f::UNIT_Y, 0, 2, false );

	//if (! m_MergedMesh.IsCompact())
	//	lgBreakToDebugger();

	// find maps between boundary vertices using markers, 
	// and project merged mesh vertices back to surface.
	m_vDestToMergeMap.Resize( m_pDestMesh->GetMaxVertexID(), m_MergedMesh.GetMaxVertexID() );
	m_vInsertBdryToMergeMap.Resize( m_pInsertMesh->GetMaxVertexID(), m_MergedMesh.GetMaxVertexID() );
	VFTriangleMesh::vertex_iterator curm(m_MergedMesh.BeginVertices()), endm(m_MergedMesh.EndVertices());
	while ( curm != endm ) {
		IMesh::VertexID vID = *curm++;
		int nMarker = trigen.GetOutputMarker_MeshVtx(vID);
		if ( nMarker >= nSourceOffset ) {
			IMesh::VertexID vSourceID = nMarker - nSourceOffset;
			m_vInsertBdryToMergeMap.SetMap(vSourceID, vID);
			Wml::Vector2f vUV = ToUV( m_MergedMesh.GetVertex(vID), 0, 2 );
			Wml::Vector3f vNormal;
			Wml::Vector3f vXYZ = m_pProjector->ProjectTo3D(vUV, &vNormal);
			m_MergedMesh.SetVertex(vID, vXYZ, &vNormal);

		} else if ( nMarker >= nDestOffset ) {
			IMesh::VertexID vDestID = nMarker - nDestOffset;
			m_vDestToMergeMap.SetMap(vDestID, vID);
			Wml::Vector3f vVtx, vNormal;
			m_pDestMesh->GetVertex(vDestID, vVtx, &vNormal);
			m_MergedMesh.SetVertex(vID, vVtx, &vNormal);

		} else {
			Wml::Vector2f vUV = ToUV( m_MergedMesh.GetVertex(vID), 0, 2 );
			Wml::Vector3f vNormal;
			Wml::Vector3f vXYZ = m_pProjector->ProjectTo3D(vUV, &vNormal);
			m_MergedMesh.SetVertex(vID, vXYZ, &vNormal);
			
		}
	}

	// append the inserted mesh, merging the boundary vertices
	unsigned int nCount = m_pInsertMesh->GetMaxVertexID();
	unsigned int nCurMergeTID = m_MergedMesh.GetMaxTriangleID();
	m_vInsertToMergeVMap.Resize( m_pInsertMesh->GetMaxVertexID(), m_MergedMesh.GetMaxVertexID() + m_pInsertMesh->GetMaxVertexID() + 1);
	m_vInsertToMergeTMap.Resize( m_pInsertMesh->GetMaxTriangleID(), m_MergedMesh.GetMaxTriangleID() + m_pInsertMesh->GetMaxTriangleID() + 1);
	m_MergedMesh.Append( *m_pInsertMesh, m_vInsertToMergeVMap, &m_vInsertToMergeTMap, &m_vInsertBdryToMergeMap );
//	m_MergedMesh.Append( *m_pInsertMesh, m_vInsertToMergeVMap, &m_vInsertToMergeTMap );

	unsigned int nTCount = m_MergedMesh.GetMaxTriangleID();
	m_vMergeInsertBitmap.resize(nTCount);
	for ( unsigned int k = nCurMergeTID; k < nTCount; ++k ) {
		if ( m_vInsertToMergeTMap.GetOld(k) != IMesh::InvalidID )
			m_vMergeInsertBitmap.set(k,true);
	}


	//if (! m_MergedMesh.IsCompact())
	//	lgBreakToDebugger();

	return true;
}


bool MeshInsertion::ProjectInsertInterior()
{
	// project interior of inserted mesh to 3D using UV coordinates
	VFTriangleMesh::vertex_iterator curs(m_pInsertMesh->BeginVertices()), ends(m_pInsertMesh->EndVertices());
	while ( curs != ends ) {
		IMesh::VertexID vInsertID = *curs++;
		if ( m_vInsertBdryToMergeMap.GetNew(vInsertID) != IMesh::InvalidID )
			continue;

		Wml::Vector2f vUV;
		if (! m_pInsertMesh->GetUV( vInsertID, 0, vUV ) ) {
			lgBreakToDebugger();
			return false;
		}
		//vUV *= m_fInsertionScale;
		vUV = (vUV-m_vInsertionOrigin) * m_fInsertionScale + m_vInsertionOrigin;
		Wml::Vector3f vNormal;
		Wml::Vector3f vXYZ = m_pProjector->ProjectTo3D(vUV, &vNormal);
		m_MergedMesh.SetVertex( m_vInsertToMergeVMap.GetNew(vInsertID), vXYZ, &vNormal);
	}

	return true;
}


void MeshInsertion::CopyInsertInterior(bool bIncludeBoundary)
{
	VFTriangleMesh::vertex_iterator curv(m_pInsertMesh->BeginVertices()), endv(m_pInsertMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vInsertID = *curv++;
		if ( !bIncludeBoundary && m_vInsertBdryToMergeMap.GetNew(vInsertID) != IMesh::InvalidID )
			continue;

		Wml::Vector3f vVtx, vNormal;
		m_pInsertMesh->GetVertex(vInsertID, vVtx, &vNormal);
		m_MergedMesh.SetVertex( m_vInsertToMergeVMap.GetNew(vInsertID), vVtx, &vNormal );
	}

}