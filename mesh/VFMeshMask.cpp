// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "VFMeshMask.h"

#include <VectorUtil.h>
#include <MeshUtils.h>
#include <WmlExtPlane3.h>
#include <Wm4ContPointInPolygon2.h>

#include "rmsdebug.h"

using namespace rms;

VFMeshMask::VFMeshMask( ) 
{
	m_eMaskMode = Intersection;
	m_pMesh = NULL;
	m_nVertexSize = 0;
	m_nTriSize = 0;
	UVSetID nSetID = AppendUVSet();
	lgASSERT(nSetID == 0 );

	m_bvTree.SetMesh(this);
}


VFMeshMask::VFMeshMask( IMesh * pMesh ) 
{ 
	m_eMaskMode = Intersection;
	SetMesh( pMesh );
	UVSetID nSetID = AppendUVSet();
	lgASSERT(nSetID == 0 );

	m_bvTree.SetMesh(this);
}


void VFMeshMask::InitializeUnion( IMesh * pMesh, VFMeshMask & mask1, VFMeshMask & mask2 )
{
	SetMesh(pMesh);
	if ( ! HasUVSet(0) ) {
		UVSetID nSetID = AppendUVSet();
		lgASSERT(nSetID == 0 );
	}
	InitializeUVSet(0);

	VFMeshMask::vertex_iterator 
		curv( mask1.begin_vertices() ), endv( mask1.end_vertices() );
	while ( curv != endv ) {
		SetCutVtx( *curv );  ++curv;
	}
	curv = mask2.begin_vertices(); endv = mask2.end_vertices();
	while ( curv != endv ) { 
		SetCutVtx( *curv );  ++curv;
	}

	VFMeshMask::triangle_iterator
		curt( mask1.begin_triangles() ), endt( mask1.end_triangles() );
	while ( curt != endt ) {
		SetCutTri( *curt ); ++curt;
	}
	curt = mask2.begin_triangles();  endt = mask2.end_triangles();
	while ( curt != endt ) {
		SetCutTri( *curt );  ++curt;
	}
}


void VFMeshMask::Initialize( const MeshSelection & triSelection )
{
	const std::set<IMesh::TriangleID> & vTris = triSelection.Triangles();
	std::set<IMesh::TriangleID>::const_iterator curt(vTris.begin()), endt(vTris.end());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		SetCutTri( tID );
		VertexID nTri[3];
		m_pMesh->GetTriangle(tID, nTri);
		for ( int j = 0; j < 3; ++j )
			SetCutVtx( nTri[j] );	
	}
}



void VFMeshMask::SetMesh( IMesh * pMesh )
{
	m_pMesh = pMesh;
	Clear(false);
	m_vCutVertices.resize( pMesh->GetMaxVertexID() );  m_nVertexSize = pMesh->GetMaxVertexID();
	m_vCutTriangles.resize( pMesh->GetMaxTriangleID() );  m_nTriSize = pMesh->GetMaxTriangleID();
}


void VFMeshMask::InitializeUVMask( IMesh * pMesh, IMesh::UVSetID nUVSetID )
{
	if ( nUVSetID != 0 ) 
		lgBreakToDebugger();

	SetMesh(pMesh);
	InitializeUVSet(nUVSetID);

	Wml::Vector2f vUV[3];
	VertexID nTri[3];
	IMesh::ITriIterator
		curt( pMesh->BeginITriangles() ), endt( pMesh->EndITriangles() );
	while ( curt != endt ) {
		TriangleID tID = *curt;  curt++;

		if ( pMesh->GetTriangleUV(tID, nUVSetID, vUV ) ) {
			SetCutTri(tID);
			pMesh->GetTriangle(tID, nTri);
			for ( int j = 0; j < 3; ++j ) {
				SetCutVtx( nTri[j] );

				// Each UV is set multiple times...is this worth it, or would
				// it be better to do a vertex-iterate afterwards??
				SetUV( nTri[j], nUVSetID, vUV[j] );
			}
		}
	}
}


void VFMeshMask::InitializeUVMask( IMesh * pMesh, rms::Polygon2f & vUVClipPolygon, IMesh::UVSetID nUVSetID, bool bFilterConnectedComponents )
{
	if ( nUVSetID != 0 ) 
		lgBreakToDebugger();

	SetMesh(pMesh);
	InitializeUVSet(nUVSetID);

	size_t nTris = pMesh->GetMaxTriangleID();
	std::vector<bool> vCutTris(nTris, false);

	Wml::Vector2f vUV[3];
	IMesh::ITriIterator
		curt( pMesh->BeginITriangles() ), endt( pMesh->EndITriangles() );
	while ( curt != endt ) {
		TriangleID tID = *curt;  curt++;

		if ( pMesh->GetTriangleUV(tID, nUVSetID, vUV ) ) {

			if ( ! vUVClipPolygon.IsInside(vUV[0]) && ! vUVClipPolygon.IsInside(vUV[1]) && ! vUVClipPolygon.IsInside(vUV[2]) ) {
				//continue;

				// [RMS] this works slightly better but still fails...
				bool bIntersect = false;
				for ( unsigned int k = 0; !bIntersect && k < vUVClipPolygon.VertexCount(); ++k ) {
					float fBary1,fBary2,fBary3;
					rms::BarycentricCoords( vUV[0], vUV[1], vUV[2], vUVClipPolygon[k], fBary1,fBary2,fBary3);
					if ( fBary1 >=0 && fBary2 >= 0 && fBary3 >= 0 )
						bIntersect = true;
				}
				if ( ! bIntersect )
					continue;

				// [RMS] need proper overlap testing (intersections? bvtree?)
			}

			vCutTris[tID] = true;
		}
	}

	if ( bFilterConnectedComponents ) {
		std::set<TriangleID> vTris;
		for ( unsigned int tID = 0; tID < nTris; ++tID ) {
			if ( vCutTris[tID] )
				vTris.insert(tID);
		}
		std::vector<int> vComponents;
		int nComponents, nLargest;
		MeshUtils::FindConnectedComponents(pMesh, vTris, vComponents, nComponents, nLargest);
		if ( nComponents < 1 || vComponents.size() != nTris)
			lgBreakToDebugger();

		vCutTris.resize(0);  vCutTris.resize(nTris,false);
		for ( unsigned int k = 0; k < nTris; ++k )
			if ( vComponents[k] == nLargest )
				vCutTris[k] = true;
	}


	VertexID nTri[3];
	for ( unsigned int tID = 0; tID < nTris; ++tID ) {
		if ( ! vCutTris[tID] )
			continue;
		SetCutTri(tID);
		pMesh->GetTriangle(tID, nTri);
		pMesh->GetTriangleUV(tID, nUVSetID, vUV);
		for ( int j = 0; j < 3; ++j ) {
			SetCutVtx( nTri[j] );		
			SetUV( nTri[j], nUVSetID, vUV[j] );
		}
	}

}



void VFMeshMask::ExpandToPolygons( const MeshPolygons & polygons )
{
	std::set<IMesh::TriangleID> vCurTris, vPolyTris;

	// find sets
	SparseArray<bool>::iterator curc(m_vCutTriangles.begin()), endc(m_vCutTriangles.end());
	while ( curc != endc ) {
		vCurTris.insert( curc.index() );
		++curc;
	}
	polygons.ToSets(vCurTris, vPolyTris);

	// Need to handle UV stuff...return bool if any polygon triangles were missing UVs
	//   (pass flag requiring this?)
	lgBreakToDebugger();

	VertexID nTri[3];
	std::set<IMesh::TriangleID>::iterator curt(vPolyTris.begin()), endt(vPolyTris.end());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		SetCutTri(tID);
		m_pMesh->GetTriangle(tID, nTri);
		//bool bTriHasUV = m_pMesh->GetTriangleUV(tID, nUVSetID, vUV);
		for ( int j = 0; j < 3; ++j ) {
			SetCutVtx( nTri[j] );		

			// HOW TO DO THIS??
			//SetUV( nTri[j], nUVSetID, vUV[j] );
		}
	}
	
}


Wml::Vector3f VFMeshMask::Project3D( const Wml::Vector2f & vUV, Wml::Vector3f * pNormal, bool * bStatus )
{
#if 1
	IMesh::TriangleID nTriID;
	bool bFound = m_bvTree.FindTriangle(vUV, nTriID);
	if ( bFound ) {
		if  ( nTriID == rms::IMesh::InvalidID )
			lgBreakToDebugger();
		Wml::Vector2f vTriUV[3];
		GetTriangleUV(nTriID, 0, vTriUV);
		float fBary[3];
		rms::BarycentricCoords( vTriUV[0], vTriUV[1], vTriUV[2], vUV, fBary[0], fBary[1], fBary[2] );

		Wml::Vector3f vTri[3];
		if ( pNormal ) {
			Wml::Vector3f vNorm[3];
			GetTriangle(nTriID, vTri, vNorm);
			*pNormal = fBary[0]*vNorm[0] + fBary[1]*vNorm[1] + fBary[2]*vNorm[2];
		} else
			GetTriangle(nTriID, vTri);
		if ( bStatus )
			*bStatus = true;
		return fBary[0]*vTri[0] + fBary[1]*vTri[1] + fBary[2]*vTri[2];
	} else {
		if ( bStatus )
			*bStatus = false;
		return Wml::Vector3f::ZERO;
	}

#else

	Wml::Vector2f vTriUV[3];

	triangle_iterator curt( begin_triangles() ), endt( end_triangles() );
	while ( curt != endt ) {
		TriangleID tID = *curt;  ++curt;

		if (! GetTriangleUV(tID, 0, vTriUV) )
			continue;

		if ( Wml::PointInConvexOrderN(3, vTriUV, vUV ) ) {

			float fBary[3];
			Wml::BarycentricCoords( vTriUV[0], vTriUV[1], vTriUV[2], vUV, fBary[0], fBary[1], fBary[2] );

			Wml::Vector3f vTri[3];
			if ( pNormal ) {
				Wml::Vector3f vNorm[3];
				GetTriangle(tID, vTri, vNorm);
				*pNormal = fBary[0]*vNorm[0] + fBary[1]*vNorm[1] + fBary[2]*vNorm[2];
			} else
				GetTriangle(tID, vTri);
			return fBary[0]*vTri[0] + fBary[1]*vTri[1] + fBary[2]*vTri[2];
		}
	}

	return Wml::Vector3f::ZERO;
#endif
}


Wml::Vector2f VFMeshMask::Project2D( const Wml::Vector3f & vPoint, bool * bStatus )
{
	// find 3 nearest vertices in 3D space
	int nNearest[3] = {-1,-1,-1};
	float fNearestSqr[3] = {std::numeric_limits<float>::max(), 
							std::numeric_limits<float>::max(), 
							std::numeric_limits<float>::max()};
	Wml::Vector3f vVtx;

	vertex_iterator curv( begin_vertices() ), endv( end_vertices() );
	while ( curv != endv ) {
		VertexID vID = *curv;  ++curv;
		GetVertex(vID, vVtx);
		float fDistSqr = (vVtx - vPoint).SquaredLength();
		if ( fDistSqr < fNearestSqr[0] ) {
			fNearestSqr[2] = fNearestSqr[1];
			nNearest[2] = nNearest[1];
			fNearestSqr[1] = fNearestSqr[0];
			nNearest[1] = nNearest[0];
			fNearestSqr[0] = fDistSqr;
			nNearest[0] = vID;
		} else if ( fDistSqr < fNearestSqr[1] ) {
			fNearestSqr[2] = fNearestSqr[1];
			nNearest[2] = nNearest[1];
			fNearestSqr[1] = fDistSqr;
			nNearest[1] = vID;
		} else if ( fDistSqr < fNearestSqr[2] ) {
			fNearestSqr[2] = fDistSqr;
			nNearest[2] = vID;
		}
	}

	if ( nNearest[0] == -1 || nNearest[1] == -1 || nNearest[2] == -1 ) {
		if ( bStatus )
			*bStatus = false;
		return Wml::Vector2f::ZERO;
	}

	Wml::Vector2f vTriUV[3];
	Wml::Vector3f vTriVtx[3];
	for ( int i = 0; i < 3; ++i ) {
		GetVertex( nNearest[i], vTriVtx[i] );
		GetUV( nNearest[i], 0, vTriUV[i] );
	}

	// project point to triangle plane
	ExtPlane3f vPlane( vTriVtx[0], vTriVtx[1], vTriVtx[2] );
	Wml::Vector3f vPlanePoint( vPlane.ProjectPointToPlane(vPoint) );

	float fBary[3];
	rms::BarycentricCoords( vTriVtx[0], vTriVtx[1], vTriVtx[2], vPlanePoint, fBary[0], fBary[1], fBary[2] );

	// ok, now compute point
	Wml::Vector2f vUV = fBary[0] * vTriUV[0] + fBary[1] * vTriUV[1] + fBary[2] * vTriUV[2];

	if ( bStatus )
		*bStatus = true;
	return vUV;
}




//! initialize vertex neighbour iteration
void VFMeshMask::BeginVtxTriangles( VtxNbrItr & v ) const
{
	m_pMesh->BeginVtxTriangles(v);
}

bool VFMeshMask::IsVisibleTri( IMesh::TriangleID tID )
{
	if ( m_eMaskMode == Intersection )
		return ( tID == IMesh::InvalidID && ! IsCutTri(tID) );
	else
		return ( tID == IMesh::InvalidID && IsCutTri(tID) );
}

//! (possibly) un-ordered iteration around one-ring of a vertex. Returns InvalidID when done
IMesh::TriangleID VFMeshMask::GetNextVtxTriangle( VtxNbrItr & v ) const
{
	TriangleID tID = m_pMesh->GetNextVtxTriangle(v);
	if ( m_eMaskMode == Intersection ) {
		while ( tID != IMesh::InvalidID && ! IsCutTri(tID) )
			tID = m_pMesh->GetNextVtxTriangle(v);
	} else {
		while ( tID != IMesh::InvalidID && IsCutTri(tID) )
			tID = m_pMesh->GetNextVtxTriangle(v);
	}
	return tID;
}


bool VFMeshMask::IsBoundaryVertex( VertexID vID ) const
{
	VtxNbrItr v(vID);
	m_pMesh->BeginVtxTriangles(v);
	TriangleID tID = m_pMesh->GetNextVtxTriangle(v);
	if ( m_eMaskMode == Intersection ) {
		while ( tID != IMesh::InvalidID && IsCutTri(tID) )
			tID = m_pMesh->GetNextVtxTriangle(v);
	} else {
		while ( tID != IMesh::InvalidID && ! IsCutTri(tID) )
			tID = m_pMesh->GetNextVtxTriangle(v);
	}
	if ( tID != IMesh::InvalidID )  // if stopped early, this is boundary tri
		return true;
	else
		return m_pMesh->IsBoundaryVertex(vID);	// could be boundary of base mesh
}


class MaskMeshSkipThruCallback : public IMesh::NeighborTriCallback
{
public:
	MaskMeshSkipThruCallback( IMesh::NeighborTriCallback * pPassThru, VFMeshMask * pMaskMesh )
		{ m_pPassThru = pPassThru; m_pMaskMesh = pMaskMesh; }

	virtual void NextTriangle( IMesh::TriangleID tID )
		{ if ( m_pMaskMesh->IsVisibleTri(tID) ) m_pPassThru->NextTriangle(tID); }

protected:
	IMesh::NeighborTriCallback * m_pPassThru;
	VFMeshMask * m_pMaskMesh;
};


//! iterate around neighbour triangles of vID, sending each to callback
void VFMeshMask::NeighbourIteration( VertexID vID, NeighborTriCallback * pCallback )
{
	pCallback->BeginTriangles();
	MaskMeshSkipThruCallback passThru(pCallback, this);
	m_pMesh->NeighbourIteration(vID, &passThru);
	pCallback->EndTriangles();
}

