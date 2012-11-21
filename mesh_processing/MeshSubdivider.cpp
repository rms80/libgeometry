// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "MeshSubdivider.h"

#include <limits>
#include <rmsdebug.h>

using namespace rms;


MeshSubdivider::MeshSubdivider(void)
{
	m_eIterState = NotIterating;

//	m_fIterEdgeLengthThresh = std::numeric_limits<float>::max();		// don't care about edge length
	m_fIterEdgeLengthThresh = 0.0025f;
	m_fIterEdgeAngleThresh = 1.0f - (float)cos(15.0f * Wml::Mathf::DEG_TO_RAD);
	m_bSkipBoundaryEdges = true;
	m_bComputeSubdividedUVs = false;
}

MeshSubdivider::~MeshSubdivider(void)
{
}

void MeshSubdivider::Subdivide( VFTriangleMesh & mesh )
{
	m_pMesh = &mesh;

	m_vEdges.resize( mesh.GetMaxVertexID() );

	unsigned int nCount = mesh.GetMaxTriangleID();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		IMesh::TriangleID tID = i;
		if ( ! mesh.IsTriangle( tID ) )
			continue;

		IMesh::VertexID vID[3];
		mesh.GetTriangle(tID, vID);

		IMesh::VertexID vEdge[3];
		for ( int j = 0; j < 3; ++j )
			vEdge[j] = GetEdgeVert( vID[j], vID[ (j+1)%3 ] ).vNew;

		mesh.RemoveTriangle( tID );

		mesh.AppendTriangle( vID[0], vEdge[0], vEdge[2] );
		mesh.AppendTriangle( vID[1], vEdge[1], vEdge[0] );
		mesh.AppendTriangle( vID[2], vEdge[2], vEdge[1] );
		mesh.AppendTriangle( vEdge[0], vEdge[1], vEdge[2] );
	}
}


MeshSubdivider::EdgeVert & MeshSubdivider::GetEdgeVert( IMesh::VertexID e1, IMesh::VertexID e2 )
{
	if ( e1 > e2 ) {
		IMesh::VertexID eTmp = e1; e1 = e2; e2 = eTmp;
	}
	EdgeVertList & vList = m_vEdges[e1];

	EdgeVert eSearch; eSearch.ve2 = e2;
	EdgeVertList::iterator found( vList.find(eSearch) );
	
	if ( found != vList.end() ) {
	  return const_cast<MeshSubdivider::EdgeVert&>(*found);
		//return *found;
	}

	Wml::Vector3f v1, v2, n1, n2;
	m_pMesh->GetVertex( e1, v1, &n1 );
	m_pMesh->GetVertex( e2, v2, &n2 );
	
	Wml::Vector3f vEdge( 0.5f * (v1 + v2) );
	Wml::Vector3f vNorm( 0.5f * (n1 + n2) );
	vNorm.Normalize();
	eSearch.vNew = m_pMesh->AppendVertex( vEdge, &vNorm );
	m_vNewVerts.push_back( eSearch.vNew );
	EdgeVertList::iterator eNew = vList.insert( eSearch ).first;
	
	 return const_cast<MeshSubdivider::EdgeVert&>(*eNew);
	//return *eNew;
}


void MeshSubdivider::InitializeIterativeSubdivide( VFTriangleMesh & mesh )
{
	m_eIterState = MeshSubdivider::Initialized;
	m_vNewVerts.resize(0);
}




bool MeshSubdivider::DoIterativeSubdivide( VFTriangleMesh & mesh, IterationCallback & callback )
{
	IterationCallback::State callbackState = callback.CheckIteration();

	if ( callbackState != IterationCallback::Continue )
		goto pauseorterminate;

	if ( m_eIterState == MeshSubdivider::Initialized ) {
		m_vCurEdges.clear();
		m_initCurEdge = mesh.BeginEdges();
		m_eIterState = MeshSubdivider::ComputingEdgeLengths;
	} 
	
	if ( m_eIterState == MeshSubdivider::ComputingEdgeLengths ) {

		Wml::Vector3f vVerts[2], vNorms[2];

    assert(1==0);
    // not yet ported to Linux-compiler
		//while (m_initCurEdge != mesh.EndEdges() ) 
		while(false)
		{
    
			// allow caller to halt iteration
			callbackState = callback.CheckIteration();
			if ( callbackState != IterationCallback::Continue )
				goto pauseorterminate;

			IMesh::EdgeID eID = *m_initCurEdge++;

			if ( m_bSkipBoundaryEdges && mesh.IsBoundaryEdge(eID) )
				continue;

			mesh.GetEdge(eID, vVerts, vNorms);

			Edge e;
			e.eID = eID;
			e.fLength = (vVerts[0] - vVerts[1]).SquaredLength();
			e.fCosAngle = 1.0f - (vNorms[0].Dot(vNorms[1]));
			m_vCurEdges.insert(e);			
		}

		// move on to subdivding stage
		m_eIterState = MeshSubdivider::Subdividing;

	} 
	
	if ( m_eIterState == MeshSubdivider::Subdividing ) {

		bool bFinished = false;
		while ( ! bFinished ) {
	
			// allow caller to halt iteration
			callbackState = callback.CheckIteration();
			if ( callbackState != IterationCallback::Continue )
				goto pauseorterminate;

			// get edge at top of queue
			Edge cure = * m_vCurEdges.begin();
//			if ( cure.fLength < m_fIterEdgeLengthThresh && cure.fCosAngle < m_fIterEdgeAngleThresh ) {
//			if ( cure.fLength < m_fIterEdgeLengthThresh ) {
			if ( cure.fCosAngle < m_fIterEdgeAngleThresh ) {
				bFinished = true;
				continue;
			}

			// create new vertex
			Wml::Vector3f vVerts[2], vNorms[2];
			mesh.GetEdge(cure.eID, vVerts, vNorms);
			Wml::Vector3f vEdge( 0.5f * (vVerts[0] + vVerts[1]) );
			Wml::Vector3f vNorm( 0.5f * (vNorms[0] + vNorms[1]) );
			vNorm.Normalize();
			IMesh::VertexID vNew = mesh.AppendVertex( vEdge, &vNorm );

			// look up edge verts and triangles
			IMesh::VertexID vEdgeV[2];  IMesh::TriangleID vEdgeT[2];
			mesh.GetEdge(cure.eID, vEdgeV, vEdgeT);
			IMesh::TriangleID nTri1[3], nTri2[3];
			mesh.GetTriangle( vEdgeT[0], nTri1 );		mesh.GetTriangle( vEdgeT[1], nTri2 );


			if ( m_bComputeSubdividedUVs ) {
				Wml::Vector2f vUVs[2];
				bool bOK1 = mesh.GetUV(vEdgeV[0], 0, vUVs[0]);
				bool bOK2 = mesh.GetUV(vEdgeV[1], 0, vUVs[1]);
				//if ( ! bOK1 || ! bOK2 )
				//	DebugBreak();
				Wml::Vector2f vUV( 0.5f * (vUVs[0] + vUVs[1]) );
				mesh.SetUV( vNew, 0, vUV );
			}


			// figure out triangle indices
			int iOther1, iOther2, ie1[2], ie2[2];
			for ( int j = 0; j < 3; ++j ) {
				if ( nTri1[j] == vEdgeV[0] )			ie1[0] = j;
				else if ( nTri1[j] == vEdgeV[1] )		ie1[1] = j;
				else									iOther1 = j;

				if ( nTri2[j] == vEdgeV[0] )			ie2[0] = j;
				else if ( nTri2[j] == vEdgeV[1] )		ie2[1] = j;
				else									iOther2 = j;
			}

			// change old triangles
			nTri1[ie1[0]] = vNew;
			nTri2[ie2[0]] = vNew;
			mesh.SetTriangle( vEdgeT[0], nTri1[0], nTri1[1], nTri1[2] );
			mesh.SetTriangle( vEdgeT[1], nTri2[0], nTri2[1], nTri2[2] );
			nTri1[ie1[0]] = vEdgeV[0];
			nTri2[ie2[0]] = vEdgeV[0];

			// add new triangles
			nTri1[ie1[1]] = vNew;
			nTri2[ie2[1]] = vNew;
			IMesh::TriangleID tNew1 = mesh.AppendTriangle( nTri1[0], nTri1[1], nTri1[2] );
			IMesh::TriangleID tNew2 = mesh.AppendTriangle( nTri2[0], nTri2[1], nTri2[2] );

			// remove subdivided edge from queue
			m_vCurEdges.erase( m_vCurEdges.begin() );

			// TODO: should we wait until we have done surface convergence to subdivide again??
			// insert new edges into queue
			//IMesh::VertexID vUpdate[4] = { vEdgeT[0], vEdgeT[1], vOther1, vOther2 };
			//for ( int j = 0; j < 4; ++j ) {
			//	IMesh::EdgeID eID = mesh.FindEdge( vNew, vUpdate[j] );
			//	if ( eID == IMesh::InvalidID )
			//		DebugBreak();
			//	Edge e;
			//	e.eID = eID;
			//	e.fLength = (vVerts[0] - vVerts[1]).SquaredLength();
			//	e.fCosAngle = 1.0f - (vNorms[0].Dot(vNorms[2]));
			//	m_vCurEdges.insert(e);
			//}

			m_vNewVerts.push_back( vNew);

		}

		// nothing left to subdivide
		m_eIterState = MeshSubdivider::NotIterating;
	}

	return true;


pauseorterminate:
	if ( callbackState == IterationCallback::Terminate )
		m_eIterState = MeshSubdivider::NotIterating;
	return false;
}

