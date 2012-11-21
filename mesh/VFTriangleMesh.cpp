// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "VFTriangleMesh.h"
#include "VectorUtil.h"
#include "MeshUtils.h"

using namespace rms;

#include "rmsdebug.h"

// this generates a log of all mesh actions
//#define PRINT_DEBUG_LOG

// comment this out to remove edge support (significantly speeds up mesh construction)
#define CREATE_EDGES



VFTriangleMesh::VFTriangleMesh(void)
{
}

VFTriangleMesh::~VFTriangleMesh(void)
{
}

VFTriangleMesh::VFTriangleMesh( const VFTriangleMesh & copy, VertexMap & vMap, TriangleMap * tMap, bool bCompact )
{
	Copy(copy, vMap, tMap, bCompact);
}
VFTriangleMesh::VFTriangleMesh( const VFTriangleMesh & copy, bool bCompact )
{
	Copy(copy, bCompact);
}


const VFTriangleMesh & VFTriangleMesh::operator=(const VFTriangleMesh & copy)
{
	Copy(copy,false);
	return *this;
}


void VFTriangleMesh::Copy( const VFTriangleMesh & mCopy, bool bCompact )
{
	VertexMap vMap(mCopy.GetMaxVertexID());
	this->Copy(mCopy,vMap,NULL,bCompact);
}

void VFTriangleMesh::Copy( const VFTriangleMesh & mCopy, VertexMap & vMap, TriangleMap * tMap, bool bCompact )
{
	VFTriangleMesh & mCopy_nonconst = const_cast<VFTriangleMesh &>(mCopy);

	unsigned int nMaxVID = mCopy_nonconst.GetMaxVertexID();
	vMap.Resize(nMaxVID, nMaxVID);

	unsigned int nMaxTID = mCopy.GetMaxTriangleID();
	if ( tMap )
		tMap->Resize( nMaxTID, nMaxTID );

	Clear(true);

	Wml::Vector3f vVertex, vNormal;
	vertex_iterator curv(mCopy_nonconst.BeginVertices()), endv(mCopy_nonconst.EndVertices());
	while ( curv != endv ) {
		VertexID vID = *curv;  ++curv;
		if ( bCompact && mCopy_nonconst.GetEdgeCount(vID) == 0 ) {
			vMap.SetMap(vID, IMesh::InvalidID );
			continue;
		}
		mCopy_nonconst.GetVertex(vID, vVertex, &vNormal);
		VertexID vNew = AppendVertex(vVertex, &vNormal);
		vMap.SetMap(vID, vNew);
	}

	VertexID nTri[3];
	triangle_iterator curt(mCopy_nonconst.BeginTriangles()), endt(mCopy_nonconst.EndTriangles());
	while ( curt != endt ) { 
		TriangleID tID = *curt; ++curt;
		mCopy_nonconst.GetTriangle(tID, nTri);
		TriangleID tNew = 
			AppendTriangle( vMap.GetNew(nTri[0]), vMap.GetNew(nTri[1]), vMap.GetNew(nTri[2]) );
		if ( tMap )
			tMap->SetMap( tID, tNew );
	}
}

void VFTriangleMesh::CopyVertInfo( VFTriangleMesh & mesh )
{
	if ( mesh.GetVertexCount() != GetVertexCount() )
		lgBreakToDebugger();
	Wml::Vector3f vVertex, vNormal;
	vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		VertexID vID = *curv;  ++curv;
		mesh.GetVertex(vID, vVertex, &vNormal);
		SetVertex(vID, vVertex, &vNormal);
	}
}


//! if vVertexMap is not large enough ( GetMaxVertexID() ), an internal vector is used, and false returns
//! pMergeVertices is a VertexID map from mAppend to *this. If a mAppend vertex is in this map, we use the *this version instead of adding it
bool VFTriangleMesh::Append( VFTriangleMesh & mAppend, std::vector<IMesh::VertexID> & vVertexMap, VertexMap * pMergeVertices )
{
	unsigned int nMaxID = mAppend.GetMaxVertexID();
	std::vector<IMesh::VertexID> vNewIDs;
	bool bUseInternal = (vVertexMap.size() < nMaxID);
	if ( bUseInternal )
		lgBreakToDebugger();
	std::vector<IMesh::VertexID> & vMapV = 
		bUseInternal ? vNewIDs : vVertexMap;
	vMapV.resize( nMaxID );

	Wml::Vector3f vVertex, vNormal;
	vertex_iterator curv( mAppend.BeginVertices() ), endv( mAppend.EndVertices() );
	while ( curv != endv ) {
		IMesh::VertexID vID =  *curv;  ++curv;
		
		if ( pMergeVertices && pMergeVertices->GetNew(vID) != IMesh::InvalidID ) {
			vMapV[vID] = pMergeVertices->GetNew(vID);
		} else {
			mAppend.GetVertex( vID, vVertex, &vNormal );
			IMesh::VertexID vNewID = AppendVertex( vVertex, &vNormal );
			vMapV[vID] = vNewID;
		}
	}

	IMesh::VertexID nTri[3];
	triangle_iterator curt( mAppend.BeginTriangles() ), endt( mAppend.EndTriangles() );
	while ( curt != endt ) {
		IMesh::TriangleID tID =  *curt;  ++curt;
		mAppend.GetTriangle( tID, nTri );
		AppendTriangle( 
			vMapV[ nTri[0] ], vMapV[ nTri[1] ], vMapV[ nTri[2] ] );
	}

	return ! bUseInternal;
}


bool VFTriangleMesh::Append( VFTriangleMesh & mAppend, VertexMap & vMap, TriangleMap * tMap, VertexMap * pMergeVertices )
{
	unsigned int nMaxVID = mAppend.GetMaxVertexID();
	vMap.Resize(nMaxVID, nMaxVID+GetMaxVertexID());

	unsigned int nMaxTID = mAppend.GetMaxTriangleID();
	if ( tMap )
		tMap->Resize( nMaxTID, nMaxTID+GetMaxTriangleID() );

	Wml::Vector3f vVertex, vNormal;
	vertex_iterator curv( mAppend.BeginVertices() ), endv( mAppend.EndVertices() );
	while ( curv != endv ) {
		IMesh::VertexID vID =  *curv;  ++curv;

		if ( pMergeVertices && pMergeVertices->GetNew(vID) != IMesh::InvalidID ) {
			vMap.SetMap( vID, pMergeVertices->GetNew(vID) );
		} else {
			mAppend.GetVertex( vID, vVertex, &vNormal );
			IMesh::VertexID vNewID = AppendVertex( vVertex, &vNormal );
			vMap.SetMap(vID, vNewID);
		} 
	}

	IMesh::VertexID nTri[3];
	triangle_iterator curt( mAppend.BeginTriangles() ), endt( mAppend.EndTriangles() );
	while ( curt != endt ) {
		IMesh::TriangleID tID =  *curt;  ++curt;
		mAppend.GetTriangle( tID, nTri );
		IMesh::TriangleID tNewID = 
			AppendTriangle( vMap.GetNew(nTri[0]), vMap.GetNew(nTri[1]), vMap.GetNew(nTri[2]) );
		if (tMap)
			tMap->SetMap(tID, tNewID);
	}	

	return true;
}









IMesh::VertexID VFTriangleMesh::AppendVertex( const Wml::Vector3f & vVertex, const Wml::Vector3f * pNormal )
{ 
	VertexID vNewID = 
		(VertexID)m_vVertices.insert( Vertex(vVertex, (pNormal) ? *pNormal : Wml::Vector3f::UNIT_Z ) );
#ifdef PRINT_DEBUG_LOG
	_RMSInfo("[VFMesh::AppendVertex      ] - adding vertex %6d\n", vNewID);
#endif

	if ( m_vVertices[vNewID].pData == NULL ) {
		m_vVertices[vNewID].pData = m_VertDataMemPool.Allocate();
		m_vVertices[vNewID].pData->vTriangles = m_VertListPool.GetList();
		m_vVertices[vNewID].pData->vEdges = m_VertListPool.GetList();
	}

	// clear lists...
	VertexData & v = * m_vVertices[vNewID].pData;
	m_VertListPool.Clear( v.vTriangles );
	m_VertListPool.Clear( v.vEdges );

	return vNewID;
}


int int_compare(const void * a, const void * b) 
{
	return *(const int *)a < *(const int *)b;
}

IMesh::TriangleID VFTriangleMesh::AppendTriangle( IMesh::VertexID v1, IMesh::VertexID v2, IMesh::VertexID v3 )
{ 
	lgASSERT( m_vVertices.isValid(v1) && m_vVertices.isValid(v2) && m_vVertices.isValid(v3) );

	// insert new triangle
	TriangleID tID = (TriangleID)m_vTriangles.insert( Triangle(v1,v2,v3) ); 
#ifdef PRINT_DEBUG_LOG
	_RMSInfo("[VFMesh::AppendTriangle    ] - adding triangle %6d  (%6d %6d %6d)\n", tID,  v1,v2,v3);
#endif

	// increment reference counts
	m_vVertices.increment( v1 );
	m_vVertices.increment( v2 );
	m_vVertices.increment( v3 );

	// add to triangle lists
	AddTriEntry( tID, v1 );
	AddTriEntry( tID, v2 );
	AddTriEntry( tID, v3 );

#ifdef CREATE_EDGES
	// add edges
	AddTriangleEdge(tID, v1, v2);
	AddTriangleEdge(tID, v2, v3);
	AddTriangleEdge(tID, v3, v1);
#endif

	return tID;
}



bool VFTriangleMesh::SetTriangle( IMesh::TriangleID tID, IMesh::VertexID v1, IMesh::VertexID v2, IMesh::VertexID v3 )
{
	if ( m_vTriangles.isValid(tID) ) {
		Triangle & t = m_vTriangles[tID];

		// this should never happen and indicates supreme broken-ness..
		if ( FindEdge(t.nVertices[0], t.nVertices[1]) == InvalidID ||
			 FindEdge(t.nVertices[1], t.nVertices[2]) == InvalidID ||
			 FindEdge(t.nVertices[2], t.nVertices[0]) == InvalidID )
			 return false;

		// remove existing edges
		RemoveTriangleEdge(tID, t.nVertices[0], t.nVertices[1]);
		RemoveTriangleEdge(tID, t.nVertices[1], t.nVertices[2]);
		RemoveTriangleEdge(tID, t.nVertices[2], t.nVertices[0]);

		// decrement existing reference counts
		lgASSERT( m_vVertices.isValid(t.nVertices[0]) && m_vVertices.isValid(t.nVertices[1]) && m_vVertices.isValid(t.nVertices[2]) );
		RemoveTriEntry( tID, t.nVertices[0] );
		RemoveTriEntry( tID, t.nVertices[1] );
		RemoveTriEntry( tID, t.nVertices[2] );
		m_vVertices.decrement(t.nVertices[0]);
		m_vVertices.decrement(t.nVertices[1]);
		m_vVertices.decrement(t.nVertices[2]);

		// set new IDs
		t.nVertices[0] = v1;
		t.nVertices[1] = v2;
		t.nVertices[2] = v3;

		// increment new reference counts
		lgASSERT( m_vVertices.isValid(v1) && m_vVertices.isValid(v2) && m_vVertices.isValid(v3) );
		m_vVertices.increment(v1);
		m_vVertices.increment(v2);
		m_vVertices.increment(v3);
		AddTriEntry( tID, v1 );
		AddTriEntry( tID, v2 );
		AddTriEntry( tID, v3 );

		// TODO: need to check that these are boundary edges (should check before we do the
		//   remove above !!!! )

		// add new edges
		AddTriangleEdge( tID, v1, v2 );
		AddTriangleEdge( tID, v2, v3 );
		AddTriangleEdge( tID, v3, v1 );

		return true;
	}
	return false;
}


void VFTriangleMesh::Clear( bool bFreeMem )
{
	IMesh::Clear(bFreeMem);
	m_vVertices.clear( bFreeMem );
	m_vEdges.clear( bFreeMem );
	m_vTriangles.clear( bFreeMem );
	m_VertDataMemPool.ClearAll();
	m_VertListPool.Clear(bFreeMem);
	m_vNonManifoldEdges.clear();
}


IMesh::EdgeID VFTriangleMesh::AddTriangleEdge( TriangleID tID, VertexID v1, VertexID v2 )
{
	if ( v1 == v2 )
		lgBreakToDebugger();
	if ( v1 > v2 ) {
		VertexID tmp = v1; v1 = v2; v2 = tmp;
	}
	EdgeID eID = FindEdge(v1, v2);
	if ( eID == InvalidID ) {
		EdgeID eID = (EdgeID)m_vEdges.insert( Edge(v1, v2, tID, InvalidID) );
		AddEdgeEntry( eID, v1 );
		AddEdgeEntry( eID, v2 );
	} else {
		lgASSERT( m_vEdges.isValid(eID) );
		Edge & e = m_vEdges[eID];
		lgASSERT(e.nVertices[0] == v1  && e.nVertices[1] == v2);
		if ( e.nTriangles[1] != InvalidID )
			m_vNonManifoldEdges.insert(eID);
		lgASSERT(e.nTriangles[1] == InvalidID );
		e.nTriangles[1] = tID;
	}
	return eID;
}




void VFTriangleMesh::HACK_ManuallyIncrementReferenceCount( VertexID vID )
{
	lgASSERT( m_vVertices.isValid(vID) );
	m_vVertices.increment(vID);
}
void VFTriangleMesh::HACK_ManuallyDecrementReferenceCount( VertexID vID )
{
	lgASSERT( m_vVertices.isValid(vID) );
	m_vVertices.decrement(vID);
	if ( m_vVertices.refCount( vID ) == 1 ) {
#ifdef PRINT_DEBUG_LOG
		_RMSInfo("[VFMesh::HACK_ManuallyDecrementReferenceCount] - removing vertex %6d\n", vID);
#endif
		m_vVertices.remove( vID );
	}
}

// RMS TODO OPTIMIZE add a version that takes Edge & e, to avoid multiple FindEdge....
bool VFTriangleMesh::RemoveTriangleEdge( TriangleID tID, VertexID v1, VertexID v2 )
{
	if ( v1 > v2 ) {
		VertexID tmp = v1; v1 = v2; v2 = tmp;
	}
	EdgeID eID = FindEdge(v1, v2);
	if ( ! IsEdge(eID) )
		return false;
	Edge & e = m_vEdges[eID];
	if ( e.nTriangles[1] == InvalidID ) {	// this edge will no longer be used
		lgASSERT(e.nTriangles[0] == tID);
		e.nTriangles[0] = InvalidID;		// clear for later (?)
		RemoveEdgeEntry(eID, v1);
		RemoveEdgeEntry(eID, v2);
		m_vEdges.remove(eID);
		m_vNonManifoldEdges.erase(eID);
	} else {
		e.nTriangles[0] = (e.nTriangles[0] == tID) ? e.nTriangles[1] : e.nTriangles[0];
		e.nTriangles[1] = InvalidID;
	}
	return true;
}



void VFTriangleMesh::RemoveVertex( VertexID vID )
{
	if ( ! m_vVertices.isValid( vID ) )
		return;
#ifdef PRINT_DEBUG_LOG
	_RMSInfo("[VFMesh::RemoveVertex      ] - removing vertex %6d\n", vID);
#endif

	VFTriangleMesh::Vertex & v = m_vVertices[vID];

	// [RMS] HACK! This case shouldn't happen, but it does in ::Weld() because
	//  SetTriangle() doesn't remove un-referenced vertices (which it really
	//   shouldn't, since we might be performing mesh surgery stuff....
	if ( v.pData->vTriangles.pFirst == NULL ) {
		if ( m_vVertices.refCount( vID ) == 1 )
			m_vVertices.remove( vID );
		else
			lgBreakToDebugger();
	} else { 
		// remove each attached face
		while ( m_vVertices.isValid( vID ) && v.pData->vTriangles.pFirst != NULL )
			RemoveTriangle(v.pData->vTriangles.pFirst->data );
	}

	// vertex should be gone now because no attached faces remain! 
	lgASSERT( ! m_vVertices.isValid( vID ) );
}


void VFTriangleMesh::RemoveTriangle( TriangleID tID )
{
	if ( ! m_vTriangles.isValid( tID ) )
		return;
	Triangle & t = m_vTriangles[tID];
#ifdef PRINT_DEBUG_LOG
	_RMSInfo("[VFMesh::RemoveTriangle    ] - removing triangle %6d  (%6d %6d %6d)\n", tID,  t.nVertices[0], t.nVertices[1], t.nVertices[2]);
#endif

	// decrement existing reference counts
	lgASSERT( m_vVertices.isValid(t.nVertices[0]) && m_vVertices.isValid(t.nVertices[1]) && m_vVertices.isValid(t.nVertices[2]) );
	for ( int i = 0; i < 3; ++i ) {
		RemoveTriEntry( tID, t.nVertices[i] );
		m_vVertices.decrement(t.nVertices[i]);
	}

	// remove edges
	RemoveTriangleEdge(tID, t.nVertices[0], t.nVertices[1]);
	RemoveTriangleEdge(tID, t.nVertices[1], t.nVertices[2]);
	RemoveTriangleEdge(tID, t.nVertices[2], t.nVertices[0]);

	// remove vertex if refcount == 1  (means that it is only referenced by self, so is safe to delete)
	for ( int i = 0; i < 3; ++i ) {
		if ( m_vVertices.refCount( t.nVertices[i] ) == 1 )
			m_vVertices.remove( t.nVertices[i]  );
	}

	// remove triangle
	m_vTriangles.remove( tID );
}

void VFTriangleMesh::RemoveUnreferencedGeometry()
{
	
}


//! initialize vertex neighbour iteration
void VFTriangleMesh::BeginVtxTriangles( VtxNbrItr & v ) const
{
	lgASSERT( m_vVertices.isValid(v.vID) );
	const VFTriangleMesh::Vertex & vtx = m_vVertices[v.vID];
	if ( vtx.pData == NULL || vtx.pData->vTriangles.pFirst == NULL )
		v.nData[0] = IMesh::InvalidID;
	else
		v.nData[0] = (unsigned long long)(vtx.pData->vTriangles.pFirst);
	v.nData[1] = 1234567890;
}

//! (possibly) un-ordered iteration around one-ring of a vertex. Returns InvalidID when done
IMesh::TriangleID VFTriangleMesh::GetNextVtxTriangle( VtxNbrItr & v ) const
{
	if ( v.nData[0] == IMesh::InvalidID )
		return IMesh::InvalidID;
	
	TriListEntry * pCur = (TriListEntry *)v.nData[0];
	IMesh::TriangleID tID = pCur->data;

	if ( pCur->pNext == NULL )
		v.nData[0] = IMesh::InvalidID;
	else
		v.nData[0] = (unsigned long long)(pCur->pNext);
	
	return tID;
}



//! initialize vertex neighbour iteration
void VFTriangleMesh::BeginVtxEdges( VtxNbrItr & v ) const
{
	lgASSERT( m_vVertices.isValid(v.vID) );
	const VFTriangleMesh::Vertex & vtx = m_vVertices[v.vID];
	if ( vtx.pData == NULL || vtx.pData->vEdges.pFirst == NULL  )
		v.nData[0] = IMesh::InvalidID;
	else
		v.nData[0] = (unsigned long long)(vtx.pData->vEdges.pFirst);
	v.nData[1] = 1234567890;
}

//! (possibly) un-ordered iteration around one-ring of a vertex. Returns InvalidID when done
IMesh::EdgeID VFTriangleMesh::GetNextVtxEdges( VtxNbrItr & v ) const
{
	if ( v.nData[0] == IMesh::InvalidID )
		return IMesh::InvalidID;
	
	const EdgeListEntry * pCur = (EdgeListEntry *)v.nData[0];
	IMesh::EdgeID eID = pCur->data;

	if ( pCur->pNext == NULL )
		v.nData[0] = IMesh::InvalidID;
	else
		v.nData[0] = (unsigned long long)(pCur->pNext);
	
	return eID;
}



void VFTriangleMesh::EdgeIteration( VertexID vID, VFTriangleMesh::NeighborEdgeCallback * pCallback )
{
	VtxNbrItr itr(vID);
	BeginVtxEdges(itr);
	EdgeID eID = GetNextVtxEdges(itr);

	pCallback->BeginEdges();

	while ( eID != InvalidID ) {
		pCallback->NextEdge(eID);
		eID = GetNextVtxEdges(itr);
	}

	pCallback->EndEdges();
}




//! bClosed flag is only set if bOrdered is true
bool VFTriangleMesh::VertexOneRing( VertexID vID, std::vector<VertexID> & vOneRing, 
									bool bOrdered, bool * bClosed )
{
	if ( ! bOrdered ) {
		vOneRing.resize(0);
		VtxNbrItr itr(vID);
		BeginVtxEdges(itr);
		EdgeID eID = GetNextVtxEdges(itr);
		while ( eID != InvalidID ) {
			VertexID nVerts[2], nTris[2];
			GetEdge(eID, nVerts, nTris);
			vOneRing.push_back( (nVerts[0] == vID) ? nVerts[1] : nVerts[0] );
			eID = GetNextVtxEdges(itr);
		}

	} else {

		// make edge set
		std::set<EdgeID> vEdges;
		std::set<EdgeID> vBoundaryEdges;
		VtxNbrItr edgeItr( vID );
		BeginVtxEdges(edgeItr);
		EdgeID eID = GetNextVtxEdges(edgeItr);
		while ( eID != InvalidID ) {
			vEdges.insert(eID);
			if ( IsBoundaryEdge(eID) )
				vBoundaryEdges.insert(eID);
			eID = GetNextVtxEdges(edgeItr);
		}

		// fail if we have non-manifold topology
		if ( vBoundaryEdges.size() != 0 && vBoundaryEdges.size() != 2 ) {
			_RMSInfo("Vertex %d has %d boundary edges - cannot find proper one-ring", vID, vBoundaryEdges.size());
			return false;
		}
		if ( bClosed )
			*bClosed = (vBoundaryEdges.size() == 0);

		// pick starting edge to be a boundary edge if we have any
		EdgeID eCurID = (vBoundaryEdges.size() > 0) ? *vBoundaryEdges.begin() : *vEdges.begin();
		VertexID vEdgeV[2], nTri[3];
		TriangleID vEdgeT[2];
		GetEdge(eCurID, vEdgeV, vEdgeT);

		vOneRing.resize(0);
		vOneRing.push_back( (vEdgeV[0] == vID) ? vEdgeV[1] : vEdgeV[0] );
		vEdges.erase( eCurID );

		while ( eCurID != IMesh::InvalidID ) {

			// find verts that are before and after current edge
			VertexID vOther[2] = { InvalidID, InvalidID };
			if ( vEdgeT[0] != InvalidID) {
				GetTriangle( vEdgeT[0], nTri );
				for ( int k = 0; k < 3; ++k )
					if ( nTri[k] != vEdgeV[0] && nTri[k] != vEdgeV[1] ) vOther[0] = nTri[k];
			}
			if ( vEdgeT[1] != InvalidID) {
				GetTriangle( vEdgeT[1], nTri );
				for ( int k = 0; k < 3; ++k )
					if ( nTri[k] != vEdgeV[0] && nTri[k] != vEdgeV[1] ) vOther[1] = nTri[k];
			}

			// figure out which is which
			VertexID vPrev = (vOneRing.size() == 1) ? vOneRing.back() : vOneRing[ vOneRing.size()-2 ];
			VertexID vNext = InvalidID;
			if ( vOther[0] == InvalidID && vOther[1] == InvalidID ) {
				break;	// something has gone horribly wrong
			} else if ( vOther[0] == InvalidID ) {
				vNext = (vOther[1] == vPrev) ? InvalidID : vOther[1];
			} else if ( vOther[1] == InvalidID ) {
				vNext = (vOther[0] == vPrev) ? InvalidID : vOther[0];
			} else {
				vNext = (vOther[0] == vPrev) ? vOther[1] : vOther[0];
			}

			if ( vNext == InvalidID || vNext == vOneRing.front() ) {
				eCurID = InvalidID;
				continue;
			}

			eCurID = FindEdge( vID, vNext );

			// ok sanity check - we should not have seen this edge yet...
			if ( eCurID == IMesh::InvalidID || vEdges.find(eCurID) == vEdges.end() )
				lgBreakToDebugger();

			vEdges.erase(eCurID);
			vOneRing.push_back( vNext );
			GetEdge( eCurID, vEdgeV, vEdgeT);
		}
	}

	return true;
}

//! bClosed flag is only set if bOrdered is true
bool VFTriangleMesh::TriangleOneRing( VertexID vID, std::vector<TriangleID> & vOneRing, 
									   bool bOrdered, bool * bClosed )
{
	if ( ! bOrdered ) {
		vOneRing.resize(0);
		VtxNbrItr itr(vID);
		BeginVtxTriangles(itr);
		TriangleID tID = GetNextVtxTriangle(itr);
		while ( tID != IMesh::InvalidID ) {
			vOneRing.push_back(tID);
			tID = GetNextVtxTriangle(itr);
		}
		return true;
	} else {
		lgBreakToDebugger();
		return false;
	}
}






bool VFTriangleMesh::IsBoundaryEdge( VertexID v1, VertexID v2 ) const
{
	return IsBoundaryEdge( FindEdge(v1,v2) );
}

bool VFTriangleMesh::IsBoundaryEdge( EdgeID eID ) const
{
	if ( eID == InvalidID || ! m_vEdges.isValid(eID) ) {
		lgASSERT( m_vEdges.isValid(eID) );
		return false;
	}
	const Edge & e = m_vEdges[eID];
	return ( e.nTriangles[0] == InvalidID || e.nTriangles[1] == InvalidID );
}


bool VFTriangleMesh::IsBoundaryTriangle( TriangleID tID ) const
{
	const Triangle & t = m_vTriangles[tID];
	EdgeID e1 = FindEdge(t.nVertices[0], t.nVertices[1]);
	if ( IsBoundaryEdge(e1) )
		return true;
	EdgeID e2 = FindEdge(t.nVertices[1], t.nVertices[2]);
	if ( IsBoundaryEdge(e2) )
		return true;
	EdgeID e3 = FindEdge(t.nVertices[2], t.nVertices[0]);
	if ( IsBoundaryEdge(e3) )
		return true;
	return false;
}



bool VFTriangleMesh::IsBoundaryVertex( VertexID vID ) const
{
	const Vertex & v = m_vVertices[vID];

#if 1
	EdgeListEntry * pCur = v.pData->vEdges.pFirst;
	while ( pCur != NULL ) {
		const Edge & e = m_vEdges[ pCur->data ];
		if ( e.nTriangles[0] == InvalidID || e.nTriangles[1] == InvalidID )
			return true;
		pCur = pCur->pNext;
	}
	return false;

#else
	// RMS TODO: it should be possible to do this just using
	//  a 3-vertex buffer (ie determine direction by looking
	//  at last and second-last vertices...)
	//  (Maybe even 2-vertex buffer?)

	// count triangles and make a list of them
	int nCount = 0;
	TriListEntry * pCur = v.pData->vTriangles.pFirst;
	while ( pCur != NULL ) {
		pCur = pCur->pNext;
		nCount++;
	}
	if ( nCount == 0 )
		return true;

	std::vector< TriangleID > vTris;
	vTris.resize( nCount-1 );
	pCur = v.pData->vTriangles.pFirst->pNext;
	int i = 0;
	while ( pCur != NULL ) {
		vTris[ i++ ] = pCur->data;
		pCur = pCur->pNext;
	}

	// pick first edge
	VertexID vCurID = InvalidID;
	VertexID vStopID = InvalidID;
	pCur = v.pData->vTriangles.pFirst;
	const VertexID * pTri = m_vTriangles[ pCur->data ].nVertices;
	for ( int i = 0; i < 3; ++i ) {
		if ( pTri[i] == vID ) {
			vCurID = pTri[ (i+1) % 3 ];
			vStopID = pTri[ (i+2) % 3];
			break;
		} else if ( pTri [ (i+1) % 3 ] == vID ) {
			vCurID = pTri[i];
			vStopID = pTri[ (i+2) % 3 ];
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
			const VertexID * pTri = m_vTriangles[ vTris[i] ].nVertices;
			if ( pTri[0] == vCurID || pTri[1] == vCurID || pTri[2] == vCurID ) {
				nCurTri = i;
				break;
			}
		}
		if ( nCurTri == InvalidID )
			return true;			// 1-ring is not connected - must be invalid!

		// mark tri as done
		const VertexID * pTri = m_vTriangles[ vTris[nCurTri] ].nVertices;
		vTris[ nCurTri ] = InvalidID;

		// go to next tri in one-ring
		if ( pTri[0] == vID ) 
			vCurID = ( pTri[1] == vCurID ) ? pTri[2] : pTri[1];
		else if ( pTri[1] == vID )
			vCurID = ( pTri[0] == vCurID ) ? pTri[2] : pTri[0];
		else
			vCurID = ( pTri[0] == vCurID ) ? pTri[1] : pTri[0];
	}

	return false;
#endif
}


void VFTriangleMesh::FindNeighbours( TriangleID tID, TriangleID vNbrs[3] ) const
{
	const Triangle & t = m_vTriangles[tID];
	for ( int i = 0; i < 3; ++i ) {

		VertexID v1 = t.nVertices[i];
		VertexID v2 = t.nVertices[ (i+1) % 3];

		// iterate over triangles of v1, looking for another tri with edge [v1,v2]
		const Vertex & v = m_vVertices[v1];
		TriListEntry * pCur = v.pData->vTriangles.pFirst;
		TriListEntry * pLast = NULL;
		bool bFound = false;
		while ( pCur != NULL && ! bFound ) {
			pLast = pCur;
			TriangleID tCurID = pCur->data; pCur = pCur->pNext;
			if ( tCurID == tID )
				continue;
			const VertexID * vTri2 = m_vTriangles[tCurID].nVertices;
			if ( vTri2[0] == v1 && ( vTri2[1] == v2 || vTri2[2] == v2 ) )
				bFound = true;
			else if ( vTri2[1] == v1 && ( vTri2[0] == v2 || vTri2[2] == v2 ) )
				bFound = true;
			else if ( vTri2[2] == v1 && ( vTri2[0] == v2 || vTri2[1] == v2 ) )
				bFound = true;
		}
		if ( bFound )
			vNbrs[i] = pLast->data;
		else
			vNbrs[i] = IMesh::InvalidID;
	}
}



IMesh::EdgeID VFTriangleMesh::FindEdge( VertexID v1, VertexID v2 ) const
{
	if ( v1 == v2 )
		lgBreakToDebugger();
	if ( v1 > v2 ) {
		VertexID tmp = v1; v1 = v2; v2 = tmp;
	}

	const Vertex & v = m_vVertices[v1];
	const EdgeListEntry * pCur = v.pData->vEdges.pFirst;
	while ( pCur != NULL ) {
		const Edge & e = m_vEdges[ pCur->data ];
		if ( (e.nVertices[0] == v1 && e.nVertices[1] == v2) ||
			 (e.nVertices[0] == v2 && e.nVertices[1] == v1 ) )
			 return pCur->data;
		pCur = pCur->pNext;
	}
	return IMesh::InvalidID;
}









void VFTriangleMesh::Weld( VertexID vKeep, VertexID vDiscard )
{
	lgASSERT( IsVertex(vKeep) && IsVertex(vDiscard) );
	
	// make list of tris to rewrite
	Vertex & v = m_vVertices[vDiscard];
	std::vector<IMesh::TriangleID> vTris;
	vTris.reserve(16);
	TriListEntry * pCur = v.pData->vTriangles.pFirst;
	while ( pCur != NULL ) {
		vTris.push_back( pCur->data );
		pCur = pCur->pNext;
	}

	// do rewriting
	size_t nTris = vTris.size();
	for ( unsigned int k = 0; k < nTris; ++k ) {
		VertexID nTri[3];
		GetTriangle(vTris[k], nTri);		
		bool bModified = false;
		for ( int j = 0; j < 3; ++j ) {
			if ( nTri[j] == vDiscard ) {
				nTri[j] = vKeep;
				bModified = true;
			}
		}
		if ( bModified ) 
			SetTriangle( vTris[k], nTri[0], nTri[1], nTri[2] );
	}

	// should be unreferenced now...
	if ( IsVertex(vDiscard) && GetTriangleCount(vDiscard) == 0 )
		RemoveVertex(vDiscard);
	else
		lgBreakToDebugger();
}


void VFTriangleMesh::SplitEdge( VertexID e1, VertexID e2 )
{
	TriangleID t[2] = { InvalidID, InvalidID };
	int ti = 0;

	Vertex * pV1 = &m_vVertices[e1];
	Vertex * pV2 = &m_vVertices[e2];

	// find two triangles with edge e1e2
	TriListEntry * pCur = pV1->pData->vTriangles.pFirst;
	while ( pCur != NULL ) {
		Triangle * pTri = &m_vTriangles[pCur->data];
		if ( (pTri->nVertices[0] == e1 || pTri->nVertices[1] == e1 || pTri->nVertices[2] == e1)
			&& (pTri->nVertices[0] == e2 || pTri->nVertices[1] == e2 || pTri->nVertices[2] == e2) )
			t[ti++] = pCur->data;
		pCur = pCur->pNext;
	}
	if ( ti != 2 )
		return;

	// append new vertex
	Wml::Vector3f vInterp = 0.5f * (pV1->vVertex + pV2->vVertex);
	Wml::Vector3f nInterp = 0.5f * (pV1->vNormal + pV2->vNormal);
	VertexID vNew = AppendVertex(vInterp, &nInterp);
	
	// update triangles
	for ( int j = 0; j < 2; ++j ) {

		// figure out which index is which
		VertexID nTri[3];
		GetTriangle(t[j],nTri);
		VertexID nE1, nE2, nOther;
		for ( int k = 0; k < 3; ++k ) {
			if ( nTri[k] == e1 ) nE1 = k;
			else if ( nTri[k] == e2 ) nE2 = k;
			else nOther = k;
		}
		
		// set triangles
		nTri[nE1] = vNew;
		AppendTriangle(nTri[0], nTri[1], nTri[2]);
		nTri[nE1] = e1;  nTri[nE2] = vNew;
		SetTriangle(t[j], nTri[0], nTri[1], nTri[2]);
	}
}


bool VFTriangleMesh::CollapseEdge( EdgeID eID )
{
	//if ( MeshUtils::CheckForFins(*this) )
	//	lgBreakToDebugger();

	if ( ! IsEdge(eID) )
		return false;
	if ( IsBoundaryEdge(eID) )
		return false;

	Edge & e = m_vEdges[eID];
	VertexID vKeep = e.nVertices[0];
	VertexID vErase = e.nVertices[1];
	bool bKeepIsBoundary = IsBoundaryVertex(vKeep);
	bool bEraseIsBoundary = IsBoundaryVertex(vErase);
	if ( bKeepIsBoundary && bEraseIsBoundary )			
		return false;		// don't collapse edges connecting fin to mesh
	if ( bEraseIsBoundary && !bKeepIsBoundary ) {
		VertexID vTmp = vKeep;  vKeep = vErase;  vErase = vTmp;
		bool bTmp = bKeepIsBoundary;  bKeepIsBoundary = bEraseIsBoundary;  bEraseIsBoundary = bTmp;
	}

	TriangleID tErase1 = e.nTriangles[0];
	TriangleID tErase2 = e.nTriangles[1];

	// find other verts of diamond
	VertexID vOther1, vOther2;
	Triangle & t1 = m_vTriangles[tErase1];
	Triangle & t2 = m_vTriangles[tErase2];
	for ( int j = 0; j < 3; ++j ) {
		if ( t1.nVertices[j] != vKeep && t1.nVertices[j] != vErase )
			vOther1 = t1.nVertices[j];
		if ( t2.nVertices[j] != vKeep && t2.nVertices[j] != vErase )
			vOther2 = t2.nVertices[j];
	}

	std::vector<IMesh::VertexID> vOneRingKeep, vOneRingErase;
	MeshUtils::VertexOneRing(*this, vKeep, vOneRingKeep);
	MeshUtils::VertexOneRing(*this, vErase, vOneRingErase);

	// if the one rings of the verts on the edge share any
	// other vertices, the result will be non-manifold. We
	// can't have that...
	// TODO: this could be more efficient if we just iterate over
	//  edges - no need to grab the one-ring vectors...
	bool bOneRingsShareVerts = false;
	for ( unsigned int i = 0; i < vOneRingKeep.size(); ++i ) {
		VertexID v1 = vOneRingKeep[i];
		if ( v1 == vKeep || v1 == vErase || v1 == vOther1 || v1 == vOther2 )
			continue;
		for ( unsigned int j = 0; j < vOneRingErase.size(); ++j )
			if ( v1 == vOneRingErase[j] )
				bOneRingsShareVerts = true;
	}
	if  ( bOneRingsShareVerts )
		return false;


	// algorithm currently can't handle 'tip' collapse...  (but this doesn't actually stop it!)
	if ( (vOneRingKeep.size() == 3 && !bKeepIsBoundary) ||
		 (vOneRingErase.size() == 3 && !bEraseIsBoundary) )
		 return false;
	//if ( GetTriangleCount(vErase) <= 3 || GetTriangleCount(vKeep) <= 3 || 
	//	 GetTriangleCount(vOther1) <= 3 || GetTriangleCount(vOther2) <= 3 )	
	//	return false;

	// get tris connected to vErase that need to be fixed
	std::vector<TriangleID> vUpdate;
	TriListEntry * pCur = m_vVertices[vErase].pData->vTriangles.pFirst;
	while ( pCur != NULL ) {
		if ( pCur->data != tErase1 && pCur->data != tErase2 )
			vUpdate.push_back(pCur->data);
		pCur = pCur->pNext;
	}

	// sanity check - if we do this collapse we will make a tri w/ two of the same vertex!
	TriangleID triVerts[3];
	for ( unsigned int k = 0; k < vUpdate.size(); ++k ) {
		GetTriangle(vUpdate[k], triVerts);
		if ( triVerts[0] == vKeep || triVerts[1] == vKeep || triVerts[2] == vKeep ) {
			return false;
		}
	}

	// find new position for vKeep
	if ( ! bKeepIsBoundary ) {
		Wml::Vector3f vNewPos = 0.5f * (m_vVertices[vKeep].vVertex + m_vVertices[vErase].vVertex);
		Wml::Vector3f vNewNorm = 0.5f * (m_vVertices[vKeep].vNormal + m_vVertices[vErase].vNormal);
		vNewNorm.Normalize();
		SetVertex(vKeep, vNewPos, &vNewNorm);
	}

	// erase edge faces
	HACK_ManuallyIncrementReferenceCount(vKeep);
	HACK_ManuallyIncrementReferenceCount(vOther1);
	HACK_ManuallyIncrementReferenceCount(vOther2);
	RemoveTriangle( tErase1 );
	RemoveTriangle( tErase2 );

	// connect vErase to vKeep
	for ( unsigned int k = 0; k < vUpdate.size(); ++k ) {
		GetTriangle(vUpdate[k], triVerts);

		for ( int j = 0; j < 3; ++j )
			if ( triVerts[j] == vErase )
				triVerts[j] = vKeep;

		// if this happens we're fucked...
		if ( triVerts[0] == triVerts[1] || triVerts[0] == triVerts[2] || triVerts[1] == triVerts[2] )
			lgBreakToDebugger();

		if ( (!IsVertex(triVerts[0])) || (!IsVertex(triVerts[1])) || (!IsVertex(triVerts[2])) )
			lgBreakToDebugger();

		bool bOK = SetTriangle(vUpdate[k], triVerts[0], triVerts[1], triVerts[2]);

		if ( (!IsVertex(triVerts[0])) || (!IsVertex(triVerts[1])) || (!IsVertex(triVerts[2])) )
			lgBreakToDebugger();

		if ( ! bOK )
			lgBreakToDebugger();
	}

	// have to manually remove vertices, because we don't want to automatically
	// discard them during mesh surgery... (TODO: rewrite algorithms so that would be ok...)
	RemoveVertex(vErase);
	if ( IsVertex(vErase) )
		lgBreakToDebugger();

	if ( (!IsVertex(vKeep)) || (!IsVertex(vOther1)) || (!IsVertex(vOther2)) )
		lgBreakToDebugger();

	HACK_ManuallyDecrementReferenceCount(vKeep);
	HACK_ManuallyDecrementReferenceCount(vOther1);
	HACK_ManuallyDecrementReferenceCount(vOther2);

	return true;
}



//! returns vertices connected to eID via winged-faces
void VFTriangleMesh::FindNeighboursEV( EdgeID eID, VertexID vNbrs[2] )
{
	Edge & e = m_vEdges[eID];
	VertexID vEdgeV[2] = {e.nVertices[0], e.nVertices[1]};

	VertexID vAll[6];  
	int a = 0;
	for ( int k = 0; k < 2; ++k ) {
		if ( e.nTriangles[k] != IMesh::InvalidID )
			for ( int j = 0; j < 3; ++j )
				vAll[a++] = m_vTriangles[e.nTriangles[k]].nVertices[j];
	}

	vNbrs[0] = vNbrs[1] = IMesh::InvalidID;

	int b = 0;
	for ( int k = 0; k < a; ++k ) {
		if ( vAll[k] != vEdgeV[0] && vAll[k] != vEdgeV[1] )
			vNbrs[b++] = vAll[k];
	}
}



static float GetOrientation( VFTriangleMesh & mesh, int v1, int v2, int v3 )
{
	Wml::Vector3f vVerts[3], vNorms[3];
	mesh.GetVertex(v1, vVerts[0], & vNorms[0] );
	mesh.GetVertex(v2, vVerts[1], & vNorms[1] );
	mesh.GetVertex(v3, vVerts[2], & vNorms[2] );
	vVerts[1] -= vVerts[0];   vVerts[1].Normalize();
	vVerts[2] -= vVerts[0];   vVerts[2].Normalize();
	Wml::Vector3f vCross(vVerts[1].Cross(vVerts[2]));
	//vCross.Normalize();
	return vCross.Dot(vNorms[0]); // > 0 ? 1 : -1;
}

bool VFTriangleMesh::FlipEdge( EdgeID eID )
{
	Edge & e = m_vEdges[eID];
	VertexID vEdgeV[2] = {e.nVertices[0], e.nVertices[1]};
	TriangleID vEdgeT[2] = {e.nTriangles[0], e.nTriangles[1]};
	Triangle & t1 = m_vTriangles[ vEdgeT[0] ];
	Triangle & t2 = m_vTriangles[ vEdgeT[1] ];

	// figure out triangle indices
	int iOther1, iOther2, ie1[2], ie2[2];
	for ( int j = 0; j < 3; ++j ) {
		if ( t1.nVertices[j] == vEdgeV[0] )			ie1[0] = j;
		else if ( t1.nVertices[j] == vEdgeV[1] )	ie1[1] = j;
		else										iOther1 = j;

		if ( t2.nVertices[j] == vEdgeV[0] )			ie2[0] = j;
		else if ( t2.nVertices[j] == vEdgeV[1] )	ie2[1] = j;
		else										iOther2 = j;
	}			

	int nOther[2] = { t1.nVertices[iOther1], t2.nVertices[iOther2] };
	EdgeID eFlipped = FindEdge( nOther[0], nOther[1] );

	if ( eFlipped != InvalidID )		// flipped edge already exists!
		return false;

	// avoid orientation flips...
	// [RMS TODO] this can be done using indices, which will be much cheaper...
	float fOrient = GetOrientation( *this, nOther[0], nOther[1], vEdgeV[0] );
	if ( fOrient < 0 ) {
		int nTmp = nOther[0];  nOther[0] = nOther[1];  nOther[1] = nTmp;
	}

	// remove edges
	RemoveTriangleEdge(vEdgeT[0], t1.nVertices[0], t1.nVertices[1]);
	RemoveTriangleEdge(vEdgeT[0], t1.nVertices[1], t1.nVertices[2]);
	RemoveTriangleEdge(vEdgeT[0], t1.nVertices[2], t1.nVertices[0]);

	RemoveTriangleEdge(vEdgeT[1], t2.nVertices[0], t2.nVertices[1]);
	RemoveTriangleEdge(vEdgeT[1], t2.nVertices[1], t2.nVertices[2]);
	RemoveTriangleEdge(vEdgeT[1], t2.nVertices[2], t2.nVertices[0]);

	// remove triangle references from vertices
	for ( int j = 0; j < 3; ++j ) {
		RemoveTriEntry( vEdgeT[0], t1.nVertices[j] );
		RemoveTriEntry( vEdgeT[1], t2.nVertices[j] );
	}

	// update triangles
	t1.nVertices[0] = nOther[0];  t1.nVertices[1] = nOther[1];  t1.nVertices[2] = vEdgeV[0];
	AddTriEntry( vEdgeT[0], nOther[0] );
	AddTriEntry( vEdgeT[0], nOther[1] );
	AddTriEntry( vEdgeT[0], vEdgeV[0] );
	
	t2.nVertices[0] = nOther[1];  t2.nVertices[1] = nOther[0];  t2.nVertices[2] = vEdgeV[1];
	AddTriEntry( vEdgeT[1], nOther[1] );
	AddTriEntry( vEdgeT[1], nOther[0] );
	AddTriEntry( vEdgeT[1], vEdgeV[1] );

	// add new edges
	AddTriangleEdge(vEdgeT[0], t1.nVertices[0], t1.nVertices[1]);
	AddTriangleEdge(vEdgeT[0], t1.nVertices[1], t1.nVertices[2]);
	AddTriangleEdge(vEdgeT[0], t1.nVertices[2], t1.nVertices[0]);

	AddTriangleEdge(vEdgeT[1], t2.nVertices[0], t2.nVertices[1]);
	AddTriangleEdge(vEdgeT[1], t2.nVertices[1], t2.nVertices[2]);
	AddTriangleEdge(vEdgeT[1], t2.nVertices[2], t2.nVertices[0]);

	return true;
}



void VFTriangleMesh::ReverseOrientation()
{
	triangle_iterator curt(BeginTriangles()), endt(EndTriangles());
	while ( curt != endt ) {
		TriangleID tID = *curt;  ++curt;
		Triangle & tri = m_vTriangles[tID];
		VertexID tmp = tri.nVertices[2];
		tri.nVertices[2] = tri.nVertices[1];
		tri.nVertices[1] = tmp;
	}
}


void VFTriangleMesh::Cleanup()
{
	// remove duplicate tris
	int nRemoved = 0;
	bool bDone = false;
	while ( ! bDone ) {
		bDone = true;

		std::vector<TriangleID> vTri1(3), vTri2(3);
		triangle_iterator curt(BeginTriangles()), endt(EndTriangles());
		while ( curt != endt && bDone ) {
			TriangleID tID1 = *curt; ++curt;
			GetTriangle(tID1, &vTri1[0]);
			std::sort(vTri1.begin(), vTri1.end());

			triangle_iterator curt2 = curt;  ++curt2;
			while ( curt2 != endt && bDone ) {
				TriangleID tID2 = *curt2; ++curt2;
				GetTriangle(tID2, &vTri2[0]);
				std::sort( vTri2.begin(), vTri2.end() );
				if ( vTri1[0]==vTri2[0] && vTri1[1]==vTri2[1] && vTri1[2]==vTri2[2] ) {
					RemoveTriangle(tID2);
					bDone = false;
					nRemoved++;
				}
			}

		}
	}

	_RMSInfo("Removed %d tris\n", nRemoved);
}


void VFTriangleMesh::ClipEarTriangles()
{
	bool bDone = false;
	while ( ! bDone ) {
		bDone = true;
		vertex_iterator curv(BeginVertices()), endv(EndVertices());
		while ( curv != endv && bDone) {
			VertexID vID = *curv; ++curv;
			if ( GetTriangleCount(vID) == 1 ) {
				TriangleID tID = m_vVertices[vID].pData->vTriangles.pFirst->data;
				RemoveTriangle(tID);
				bDone = false;
			}
		}
	}
}



void VFTriangleMesh::GetVertexFrame( VertexID vID, Wml::Vector3f & tan1, Wml::Vector3f & tan2, Wml::Vector3f & vNormal, VertexID vNbr )
{
	Vertex & v = m_vVertices[vID];
	EdgeListEntry * pFirstEdge = v.pData->vEdges.pFirst;
	EdgeID eFirstEdgeID = pFirstEdge->data;
	
	if ( vNbr == IMesh::InvalidID ) {
		vNbr = ( m_vEdges[eFirstEdgeID].nVertices[0] == vID ) ? 
			m_vEdges[eFirstEdgeID].nVertices[1] : m_vEdges[eFirstEdgeID].nVertices[0];
	}

	// compute frame
	Wml::Vector3f vNbrV = m_vVertices[vNbr].vVertex;
	vNormal = v.vNormal;
	tan2 = vNbrV - v.vVertex;
	tan1 = vNormal.Cross( tan2.Cross(vNormal) );
	tan1.Normalize();
	tan2 = vNormal.Cross( tan1 );
}



void VFTriangleMesh::GetBoundingBox( Wml::AxisAlignedBox3f & bounds ) const
{
	bounds.Min[0] = std::numeric_limits<float>::max();

	vertex_iterator curv( BeginVertices() ), endv( EndVertices() );
	while ( curv != endv ) {
		VertexID vID = *curv;  ++curv;
		Wml::Vector3f vVertex;
		GetVertex(vID, vVertex);
		if ( bounds.Min[0] == std::numeric_limits<float>::max() )
			bounds = Wml::AxisAlignedBox3f( vVertex.X(), vVertex.X(), vVertex.Y(), vVertex.Y(), vVertex.Z(), vVertex.Z() );
		else
			Union( bounds, vVertex);
	}
}


void VFTriangleMesh::GetEdgeLengthStats(float & fMin, float & fMax, float & fAverage) const
{
	fMax = 0.0f;
	fMin = std::numeric_limits<float>::max();
	double fAverageEdge = 0.0f;

	int nCount = 0;
	triangle_iterator curt( BeginTriangles()), endt(EndTriangles());
	while ( curt != endt ) {
		TriangleID nID = *curt;
		curt++;

		Wml::Vector3f vVertices[3];
		GetTriangle(nID, vVertices);

		for ( int i = 0; i < 3; ++i ) {
			float fLen = (vVertices[i] - vVertices[(i+1)%3]).Length();
			if ( fLen < fMin )
				fMin = fLen;
			if ( fLen > fMax )
				fMax = fLen;
			fAverageEdge += fLen;
			++nCount;
		}
	}

	fAverage = (float)(fAverageEdge / (double)nCount);	
}



bool VFTriangleMesh::IsIsolated( VertexID vID ) const
{
	const Vertex & v = m_vVertices[vID];
	const EdgeListEntry * pFirstEdge = v.pData->vEdges.pFirst;
	return ( pFirstEdge == NULL );
	//EdgeID eFirstEdgeID = pFirstEdge->data;
	//return ( eFirstEdgeID != IMesh::InvalidID );
}


bool VFTriangleMesh::IsCompact() const
{
	// [TODO] this would be more efficient if we used bitmasks...

	std::set<VertexID> vUsedVerts;
	triangle_iterator curt(BeginTriangles()), endt(EndTriangles());
	while ( curt != endt ) {
		VertexID nTri[3];
		GetTriangle( *curt++, nTri);
		for ( int j = 0; j < 3; ++j ) 
			vUsedVerts.insert( nTri[j] );
	}

	std::vector<VertexID> vIterVerts;
	vertex_iterator curv(BeginVertices()), endv(EndVertices());
	while ( curv != endv )
		vIterVerts.push_back( *curv++ );

	//_RMSInfo("VFTriangleMesh::IsCompact() - %d verts in iteration, %d used by faces\n", vIterVerts.size(), vUsedVerts.size() );

	return ( vIterVerts.size() == vUsedVerts.size() );
}




void VFTriangleMesh::ClearBit( unsigned int nBit )
{
	RefCountedVector<Vertex>::item_iterator 
		curv(m_vVertices.begin_items()), endv(m_vVertices.end_items());
	while ( curv != endv ) {
		(*curv).vecData.nBits &= ~(1<<nBit);
		++curv;
	}
}



struct OBJFace {
	int nVert[3];
	int nNormal[3];
	int nTex[3];
};


bool VFTriangleMesh::ReadOBJ( const char * pFilename, std::string & errString )
{
	Clear(false);

	ifstream in(pFilename);
	if(!in){
		errString = string("Cannot open file ") + string(pFilename);
		cerr << errString << endl;
		return false;
	}

	string command;
	char c1,c2;
	unsigned int tv1, tv2, tv3, tn1, tn2, tn3, tt1, tt2, tt3;
	char linebuf[1024];

	Wml::Vector3f fvec = Wml::Vector3f::ZERO;

	bool bHasTextures = false;
	bool bHasNormals = false;
	bool bInitializedUVSet = false;

	// need to save normals separately and then match to vertices (maya
	//  "optimizes" the mesh...argh!)
	std::vector<Wml::Vector3f> vNormals;
	std::vector<Wml::Vector2f> vUVs;

	unsigned int ivec[3];
	while(in){
		ostrstream s;
		in >> command;
		if(!in)
			continue;
		switch(command.c_str()[0]){

		case 'v':    
			in >> fvec[0] >> fvec[1];
			if(!in)
				continue;
			switch(command.c_str()[1]){
			case '\0':  // vertex
				in >> fvec[2];
				AppendVertex(fvec);
				break;
			case 'n': // vertex normal
				bHasNormals = true;
				in >> fvec[2];
				fvec.Normalize();
				vNormals.push_back( fvec );
				break;
			case 't':
				if ( ! bHasTextures ) 
					bHasTextures = true;
				if ( bInitializedUVSet )
					lgBreakToDebugger();			// need to rewrite so that we can save all uv's during reading, and set afterwards
											// because UV set size is set to VertexCount() during InitializeUVSet() and does not automatically expand!!
				vUVs.push_back( Wml::Vector2f(fvec) );
				break;
			default:
				string err("Got unknown OBJ command ");
				err += command;
				cerr << err << endl;
			}
		break;

		case 'f':
			if ( bHasTextures && bHasNormals ) {
				in >> tv1 >> c1 >> tt1 >> c2 >> tn1;
				in >> tv2 >> c1 >> tt2 >> c2 >> tn2;
				in >> tv3 >> c1 >> tt3 >> c2 >> tn3;
			} else if ( bHasTextures && ! bHasNormals ) {
				in >> tv1 >> c1 >> tt1;
				in >> tv2 >> c1 >> tt2;
				in >> tv3 >> c1 >> tt3;
			} else if ( bHasNormals ) {
				in >> tv1 >> c1 >> c2 >> tn1;
				in >> tv2 >> c1 >> c2 >> tn2;
				in >> tv3 >> c1 >> c2 >> tn3;
			} else {
				in >> tv1 >> tv2 >> tv3;
			}
			ivec[0] = tv1-1; ivec[1] = tv2-1; ivec[2] = tv3-1;
			AppendTriangle(ivec[0], ivec[1], ivec[2]);

			// set proper normals
			if ( bHasNormals ) {
				SetNormal( ivec[0], vNormals[ tn1-1 ] );
				SetNormal( ivec[1], vNormals[ tn2-1 ] );
				SetNormal( ivec[2], vNormals[ tn3-1 ] );
			}

			if ( bHasTextures ) {
				if ( ! bInitializedUVSet ) {
					AppendUVSet();
					InitializeUVSet(0);
					bInitializedUVSet = true;
				}
				SetUV( ivec[0], 0, vUVs[tt1-1] );
				SetUV( ivec[1], 0, vUVs[tt2-1] );
				SetUV( ivec[2], 0, vUVs[tt3-1] );
			}

			break;

		default:
			in.getline(linebuf, 1023, '\n');
		}
	}

	// if we have no triangles, assume 1-1 ordered matches
	unsigned int nVerts = GetVertexCount();
	if ( GetTriangleCount() == 0 ) {
		if ( vNormals.size() == nVerts ) {
			for ( unsigned int k = 0; k < nVerts; ++k )
				SetNormal( k, vNormals[k] );
		}
		if ( vUVs.size() == nVerts ) {
			if ( ! bInitializedUVSet ) {
				AppendUVSet();
				InitializeUVSet(0);
				bInitializedUVSet = true;
			}
			for ( unsigned int k = 0; k < nVerts; ++k )
				SetUV( k, 0, vUVs[k] );
		}

	}

	return true;
}


bool VFTriangleMesh::WriteOBJ( const char * pFilename, std::string & errString )
{
	std::ofstream out(pFilename);
	if (!out)
		return false;

	bool bHaveVertexTexCoords = HasUVSet(0);

	// force some UV-value to be set for each vertex
	if ( bHaveVertexTexCoords ) {
		vertex_iterator curv(BeginVertices()), endv(EndVertices());
		while ( curv != endv ) {
			VertexID vID = *curv;  ++curv;
			Wml::Vector2f vUV(10.0f, 10.0f);
			if ( ! GetUV(vID, 0, vUV) )
				SetUV(vID, 0, vUV);
		}
	}

	std::vector<VertexID> vertMap;
	vertMap.resize( GetMaxVertexID(), IMesh::InvalidID );
	unsigned int nCounter = 0;
	vertex_iterator curv(BeginVertices()), endv(EndVertices());
	while ( curv != endv ) {
		VertexID vID = *curv;  ++curv;
		vertMap[vID] = nCounter++;

		Wml::Vector3f vert, norm;

		GetVertex(vID, vert, &norm);
		out << "v " << vert.X() << " " << vert.Y() << " " << vert.Z() << std::endl;
		out << "vn " << norm.X() << " " << norm.Y() << " " << norm.Z() << std::endl; 
		if ( bHaveVertexTexCoords ) {
			Wml::Vector2f tex;
			if ( GetUV(vID, 0, tex) )
				out << "vt " << tex.X() << " " << tex.Y() << std::endl; 
			else
				out << "vt -5 -5" << std::endl;
		}
	}

	Wml::Vector2f vTriUV[3];

	triangle_iterator curt(BeginTriangles()), endt(EndTriangles());
	while ( curt != endt ) {
		TriangleID tID = *curt; ++curt;
		VertexID vTri[3];
		GetTriangle(tID, vTri);
		unsigned int tri[3];
		for ( int j = 0; j < 3; ++j )
			tri[j] = vertMap[vTri[j]];


		if ( bHaveVertexTexCoords ) {
			out << "f " << (tri[0]+1) << "/" << (tri[0]+1) << "/" << (tri[0]+1) 
				<< " " << (tri[1]+1) << "/" << (tri[1]+1) << "/" << (tri[1]+1)
				<< " " << (tri[2]+1) << "/" << (tri[2]+1) << "/" << (tri[2]+1) << std::endl;
		} else {
			out << "f " << (tri[0]+1) << "//" << (tri[0]+1) 
				<< " " << (tri[1]+1) << "//" << (tri[1]+1)
				<< " " << (tri[2]+1) << "//" << (tri[2]+1) << std::endl;
		}
	}

	out.close();

	errString = string("no error");
	return true;
}