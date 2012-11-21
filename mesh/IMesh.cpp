// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "IMesh.h"

#include <limits>

using namespace rms;

unsigned int IMesh::InvalidID = IMESH_INVALID_ID;

IMesh::IMesh()
{
}

IMesh::~IMesh()
{
	for ( unsigned int k = 0; k < m_vUVSets.size(); ++k )
		delete m_vUVSets[k];
	for ( unsigned int k = 0; k < m_vScalarSets.size(); ++k )
		delete m_vScalarSets[k];
}

void IMesh::FindNeighbours( TriangleID tID, TriangleID vNbrs[3] ) const
{
	// how can we implement this...?
	lgBreakToDebugger();
}


//! determine if a vertex is on the mesh boundary (if there is one)
bool IMesh::IsBoundaryVertex( VertexID vID ) const
{
	// RMS TODO: it should be possible to do this just using
	//  a 3-vertex buffer (ie determine direction by looking
	//  at last and second-last vertices...)
	//  (Maybe even 2-vertex buffer?)

	VtxNbrItr v(vID);
	BeginVtxTriangles(v);
	int nCount = 0;
	while ( GetNextVtxTriangle(v) != InvalidID )
		++nCount;
	if ( nCount == 0 )
		return true;

	std::vector< TriangleID > vTris;
	vTris.resize( nCount-1 );
	BeginVtxTriangles(v);
	TriangleID tID;
	unsigned int i = 0;
	while ( (tID = GetNextVtxTriangle(v)) != InvalidID )
		vTris[i++] = tID;

	// pick first edge
	VertexID vCurID = InvalidID;
	VertexID vStopID = InvalidID;
	VertexID vTri[3];
	GetTriangle( vTris[0], vTri );
	for ( int i = 0; i < 3; ++i ) {
		if ( vTri[i] == vID ) {
			vCurID = vTri[ (i+1) % 3 ];
			vStopID = vTri[ (i+2) % 3];
			break;
		} else if ( vTri[ (i+1) % 3 ] == vID ) {
			vCurID = vTri[i];
			vStopID = vTri[ (i+2) % 3 ];
			break;
		}
	}
	nCount--;   // used up first tri

	// loop until we hit vStopID
	while ( vCurID != vStopID ) {
		
		// find unused tri w/ [vID, vCurID, X]
		int nCurTri = InvalidID;
		for ( int i = 0; i < nCount; ++i ) {
			if ( vTris[i] == InvalidID )
				continue;
			GetTriangle( vTris[i], vTri );
			if ( vTri[0] == vCurID || vTri[1] == vCurID || vTri[2] == vCurID ) {
				nCurTri = i;
				break;
			}
		}
		if ( nCurTri == InvalidID )
			return true;			// 1-ring is not connected - must be invalid!

		// mark tri as done
		vTris[ nCurTri ] = InvalidID;

		// go to next tri in one-ring
		GetTriangle( vTris[nCurTri], vTri );
		if ( vTri[0] == vID ) 
			vCurID = ( vTri[1] == vCurID ) ? vTri[2] : vTri[1];
		else if ( vTri[1] == vID )
			vCurID = ( vTri[0] == vCurID ) ? vTri[2] : vTri[0];
		else
			vCurID = ( vTri[0] == vCurID ) ? vTri[1] : vTri[0];
	}

	return false;
}


void IMesh::NeighbourIteration( VertexID vID, IMesh::NeighborTriCallback * pCallback )
{
	IMesh::VtxNbrItr itr(vID);
	BeginVtxTriangles(itr);
	TriangleID tID = GetNextVtxTriangle(itr);

	pCallback->BeginTriangles();

	while ( tID != InvalidID ) {
		pCallback->NextTriangle(tID);
		tID = GetNextVtxTriangle(itr);
	}

	pCallback->EndTriangles();
}










// force instantiation (compiler check)
namespace rms
{
	template class VertexFunction<float,1>;
	template class VertexFunction<double,1>;
}
