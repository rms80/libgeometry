// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "MeshPolygons.h"

#include <limits>

using namespace rms;

unsigned int MeshPolygons::InvalidPolygonID = std::numeric_limits<unsigned int>::max();

const std::vector<IMesh::TriangleID> MeshPolygons::EMPTY_SET;
const std::vector<IMesh::VertexID> MeshPolygons::EMPTY_BOUNDARY;
const std::vector<unsigned int> MeshPolygons::EMPTY_BOUNDARY_UV;

MeshPolygons::MeshPolygons()
{
	m_nIDCounter = 0;
	m_nMaxPolygonSize = 0;
}


MeshPolygons::MeshPolygons(VFTriangleMesh & trimesh)
{
	m_nIDCounter = 0;
	m_nMaxPolygonSize = 0;

	std::vector<IMesh::VertexID> nTri(3);
	VFTriangleMesh::triangle_iterator curt(trimesh.BeginTriangles()), endt(trimesh.EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;

		trimesh.GetTriangle(tID, &nTri[0]);

		PolygonID sID = CreatePolygon(1);
		AppendTriangle(sID, tID);
		SetBoundary(sID, nTri);
	}
}



MeshPolygons::MeshPolygons(VFTriangleMesh & newMesh, const MeshPolygons & oldSets, const TriangleMap & TMap, const VertexMap * VMap)
{
	Initialize(newMesh, oldSets, TMap, VMap);
}



void MeshPolygons::Initialize(VFTriangleMesh & mesh, const MeshPolygons & oldSets, const TriangleMap & TMap, const VertexMap * VMap)
{
	Clear();

	std::set<Polygon>::const_iterator oldcur(oldSets.m_vPolygons.begin()), oldend(oldSets.m_vPolygons.end());
	while ( oldcur != oldend ) {
		const Polygon & oldSet = *oldcur++;

		const std::vector<IMesh::TriangleID> & vSet = oldSet.vTris; // oldSets.GetSet(sID);
		size_t nCount = vSet.size();

		// all triangles have to exist in new mesh for set to be maintained
		bool bSetExists = true;
		std::vector<IMesh::TriangleID> vNewSet(nCount);
		for ( unsigned int i = 0; bSetExists && i < nCount; ++i ) {
			vNewSet[i] = TMap.GetNew( vSet[i] );
			if ( vNewSet[i] == IMesh::InvalidID )
				bSetExists = false;
		}

		if ( bSetExists ) {
			// append new set
			PolygonID id = CreatePolygon((unsigned int)nCount);
			for ( unsigned int i = 0; i < nCount; ++i )
				AppendTriangle( id, vNewSet[i] );

			// maintain old boundary
			if ( VMap ) {
				std::vector<IMesh::VertexID> vBoundary( oldSet.vBoundary );
				size_t nBCount = vBoundary.size();
				for ( unsigned int k = 0; k < nBCount; ++k ) {
					vBoundary[k] = VMap->GetNew( vBoundary[k] );
				}
				SetBoundary( id, vBoundary, &oldSet.vBoundaryUV );
			}
		}
	}

	// ok now we have to add sets for all the other triangles
	std::vector<IMesh::VertexID> nTri(3);
	VFTriangleMesh::triangle_iterator curt(mesh.BeginTriangles()), endt(mesh.EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		if ( FindPolygon(tID) != InvalidPolygonID )
			continue;

		mesh.GetTriangle(tID, &nTri[0]);

		PolygonID sID = CreatePolygon(1);
		AppendTriangle(sID, tID);
		SetBoundary(sID, nTri);
	}
}



void MeshPolygons::Clear()
{
	m_PolygonMap.clear();
	m_vPolygons.clear();
	m_nIDCounter = 0;
	m_nMaxPolygonSize = 0;
}


MeshPolygons::PolygonID MeshPolygons::CreatePolygon(unsigned int nSizeHint)
{
	Polygon vTris;
	vTris.id = ++m_nIDCounter;
	//vTris.vTris.reserve(nSizeHint);		// [RMS TODO] is this copied during insert() ??
	m_vPolygons.insert(vTris);
	return vTris.id;
}

bool MeshPolygons::AppendTriangle( PolygonID sID, IMesh::TriangleID tID )
{
	// triangle can only exist in one set
	if ( m_PolygonMap.find(tID) != m_PolygonMap.end() )
		return false;

	// set has to exist
	Polygon tmp; tmp.id = sID;
	PolygonItr found = m_vPolygons.find(tmp);
	if (found == m_vPolygons.end() )
		return false;

	Polygon & tris = const_cast<Polygon&>(*found);
	tris.vTris.push_back(tID);
	m_PolygonMap.insert(std::pair<IMesh::TriangleID,PolygonID>(tID,sID));
	//m_PolygonMap[tID] = sID;

	if ( tris.vTris.size() > m_nMaxPolygonSize )
		m_nMaxPolygonSize = (unsigned int)tris.vTris.size();

	return true;
}

bool MeshPolygons::SetBoundary( PolygonID sID, const std::vector<IMesh::VertexID> & vBoundary, const std::vector<unsigned int> * pBoundaryUV )
{
	// set has to exist
	Polygon tmp; tmp.id = sID;
	std::set<Polygon>::iterator found = m_vPolygons.find(tmp);
	if (found == m_vPolygons.end() )
		return false;

	Polygon & poly = const_cast<Polygon&>(*found);
	poly.vBoundary = vBoundary;
	if ( pBoundaryUV && pBoundaryUV->size() == vBoundary.size() )
		poly.vBoundaryUV = *pBoundaryUV;

	return true;
}



void MeshPolygons::AppendTriangles( VFTriangleMesh & mesh, VFTriangleMesh & append, TriangleMap & TMap )
{
	std::vector<IMesh::VertexID> nTri(3);

	VFTriangleMesh::triangle_iterator curt(append.BeginTriangles()), endt(append.EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tOldID = *curt++;
		IMesh::TriangleID tID = TMap.GetNew(tOldID);

		mesh.GetTriangle(tID, &nTri[0]);

		PolygonID sID = CreatePolygon(1);
		AppendTriangle(sID, tID);
		SetBoundary(sID, nTri);
	}
}




// input is data from mesh.Append(append, TMap, VMap) - copy/rewrite faces from appendpolys
void MeshPolygons::AppendPolygons( VFTriangleMesh & mesh, 
								   VFTriangleMesh & append, MeshPolygons & appendpolys, 
								   VertexMap & VMap, TriangleMap & TMap, UVMap * pUVMap )
{
	std::vector<IMesh::VertexID> vBoundary;
	std::vector<unsigned int> vBoundaryUV;
	id_iterator curp(appendpolys.begin()), endp(appendpolys.end());
	while ( curp != endp ) {
		PolygonID sID = *curp++;

		const std::vector<IMesh::TriangleID> & vTris = appendpolys.GetTriangles(sID);
		size_t nCount = vTris.size();
		PolygonID sNewID = CreatePolygon((unsigned int)nCount);
		for ( unsigned int k = 0; k < nCount; ++k )
			AppendTriangle( sNewID, TMap.GetNew(vTris[k]) );

		const std::vector<IMesh::VertexID> & vOldBoundary = appendpolys.GetBoundary(sID);
		const std::vector<unsigned int> & vOldBoundaryUV = appendpolys.GetBoundaryUV(sID);
		nCount = vOldBoundary.size();
		vBoundary.resize(nCount);
		for ( unsigned int k = 0; k < nCount; ++k )
			vBoundary[k] = VMap.GetNew( vOldBoundary[k] );
		if ( pUVMap ) {
			nCount = vOldBoundaryUV.size();
			vBoundaryUV.resize(nCount);
			for ( unsigned int k = 0; k < nCount; ++k )
				vBoundaryUV[k] = pUVMap->GetNew( vOldBoundaryUV[k] );
		} else
			vBoundaryUV = vOldBoundaryUV;

		SetBoundary(sNewID, vBoundary, &vBoundaryUV);
	}
}



//! add triangular polys for any unused triangles
void MeshPolygons::TriangleFill(const VFTriangleMesh & mesh)
{
	std::vector<IMesh::VertexID> nTri(3);
	VFTriangleMesh::triangle_iterator curt(mesh.BeginTriangles()), endt(mesh.EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		if ( m_PolygonMap.find(tID) == m_PolygonMap.end() )  {
			mesh.GetTriangle(tID, &nTri[0]);
			PolygonID sID = CreatePolygon(1);
			AppendTriangle(sID, tID);
			SetBoundary(sID, nTri);
		}
	}
}



void MeshPolygons::Rewrite( const VertexMap & VMap, const TriangleMap & TMap )
{
	m_PolygonMap.clear();

	std::set<Polygon>::iterator curt(m_vPolygons.begin()), endt(m_vPolygons.end());
	while ( curt != endt ) {
		Polygon & t = const_cast<Polygon&>(*curt++);
		size_t nTris = t.vTris.size();
		for ( unsigned int k = 0 ; k < nTris; ++k ) {
			IMesh::TriangleID tNew = TMap.GetNew( t.vTris[k] );
			if ( tNew != IMesh::InvalidID )
				t.vTris[k] = tNew;
			m_PolygonMap.insert( std::pair<IMesh::TriangleID, PolygonID>( t.vTris[k], t.id ) );
		}
		size_t nVerts = t.vBoundary.size();
		for ( unsigned int k = 0; k < nVerts; ++k ) {
			IMesh::VertexID vNew = VMap.GetNew( t.vBoundary[k] );
			if ( vNew != IMesh::InvalidID )
				t.vBoundary[k] = vNew;
		}
	}
}

void MeshPolygons::RewriteBoundaries( const VertexMap & VMap )
{
	std::set<Polygon>::iterator curt(m_vPolygons.begin()), endt(m_vPolygons.end());
	while ( curt != endt ) {
		Polygon & t = const_cast<Polygon&>(*curt++);
		size_t nVerts = t.vBoundary.size();
		for ( unsigned int k = 0; k < nVerts; ++k ) {
			IMesh::VertexID vNew = VMap.GetNew(t.vBoundary[k]);
			if ( vNew != IMesh::InvalidID )
				t.vBoundary[k] = vNew;
		}
	}
}

void MeshPolygons::RewriteUVs( unsigned int nShift )
{
	std::set<Polygon>::iterator curt(m_vPolygons.begin()), endt(m_vPolygons.end());
	while ( curt != endt ) {
		Polygon & t = const_cast<Polygon&>(*curt++);
		size_t nVerts = t.vBoundaryUV.size();
		for ( unsigned int k = 0; k < nVerts; ++k ) 
			t.vBoundaryUV[k] += nShift;
	}
}


MeshPolygons::PolygonID MeshPolygons::FindPolygon( IMesh::TriangleID tID )
{
	std::map<IMesh::TriangleID, PolygonID>::iterator found = m_PolygonMap.find(tID);
	if ( found != m_PolygonMap.end() )
		return (*found).second;
	else
		return InvalidPolygonID;
}


const std::vector<IMesh::TriangleID> & MeshPolygons::GetTriangles( PolygonID sID ) const
{
	// set has to exist
	Polygon tmp; tmp.id = sID;
	std::set<Polygon>::const_iterator found = m_vPolygons.find(tmp);
	if (found == m_vPolygons.end() )
		return EMPTY_SET;

	return (*found).vTris;
}

const std::vector<IMesh::VertexID> & MeshPolygons::GetBoundary( PolygonID sID ) const
{
	// set has to exist
	Polygon tmp; tmp.id = sID;
	std::set<Polygon>::const_iterator found = m_vPolygons.find(tmp);
	if (found == m_vPolygons.end() )
		return EMPTY_BOUNDARY;

	return (*found).vBoundary;
}


const std::vector<unsigned int> & MeshPolygons::GetBoundaryUV( PolygonID sID ) const
{
	// set has to exist
	Polygon tmp; tmp.id = sID;
	std::set<Polygon>::const_iterator found = m_vPolygons.find(tmp);
	if (found == m_vPolygons.end() )
		return EMPTY_BOUNDARY;

	return (*found).vBoundaryUV;
}



void MeshPolygons::ClearUVs()
{

	std::set<Polygon>::iterator cur(m_vPolygons.begin()), end(m_vPolygons.end());
	while ( cur != end ) {
    Polygon & t = const_cast<Polygon&>(*cur++);	
		t.vBoundaryUV.clear();
	}
}


void MeshPolygons::ToSets( const std::set<IMesh::TriangleID> & vTris, std::set<IMesh::TriangleID> & vSetTris ) const
{
	std::set<IMesh::TriangleID>::const_iterator curt(vTris.begin()), endt(vTris.end());
	while ( curt != endt ) {
		IMesh::TriangleID tID = (*curt++);
		std::map<IMesh::TriangleID, PolygonID>::const_iterator found( m_PolygonMap.find( tID ) );
		if ( found == m_PolygonMap.end() )
			lgBreakToDebugger();

		PolygonID sID = (*found).second;
		Polygon tmp; tmp.id = sID;
		std::set<Polygon>::const_iterator foundset = m_vPolygons.find(tmp);
		if (foundset != m_vPolygons.end() ) {
			const Polygon & tris = *foundset;
			vSetTris.insert( tris.vTris.begin(), tris.vTris.end() );
		}
	}		
}



void MeshPolygons::SanityCheck(VFTriangleMesh & mesh)
{
	std::set<Polygon>::iterator curt(m_vPolygons.begin()), endt(m_vPolygons.end());
	while ( curt != endt ) {
		Polygon & t = const_cast<Polygon&>(*curt++);

		for ( unsigned int k = 0; k < t.vTris.size(); ++k )
			if ( ! mesh.IsTriangle( t.vTris[k] ) )
				lgBreakToDebugger();

		for ( unsigned int k = 0; k < t.vBoundary.size(); ++k )
			if ( ! mesh.IsVertex( t.vBoundary[k] ) )
				lgBreakToDebugger();
	}
}
