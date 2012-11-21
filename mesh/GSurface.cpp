// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "GSurface.h"
#include "MeshUtils.h"

using namespace rms;


UVList::UVList( unsigned int nSize )
{
	resize(nSize);
}

NormalList::NormalList( unsigned int nSize )
{
	resize(nSize);
}




GSurface::GSurface()
{
	m_pMesh = new VFTriangleMesh();
	m_pPolygons = new MeshPolygons();
	m_bvTree.SetMesh( m_pMesh );
	m_bOwnsData = true;
}
GSurface::GSurface(const GSurface & copy)
{
	m_pMesh = new VFTriangleMesh();
	m_pPolygons = new MeshPolygons();
	m_bvTree.SetMesh( m_pMesh );
	m_bOwnsData = true;

	*this = copy;
}
GSurface::GSurface(VFTriangleMesh * pMesh, MeshPolygons * pPolygons, bool bOwnsData)
{
	m_pMesh = pMesh;
	m_pPolygons = pPolygons;
	m_bvTree.SetMesh( m_pMesh );
	m_bOwnsData = bOwnsData;
}

GSurface::~GSurface()
{
	if ( m_bOwnsData ) {
		delete m_pMesh;  m_pMesh = NULL;
		delete m_pPolygons;  m_pPolygons = NULL;
	}
}

const GSurface & GSurface::operator=(const GSurface & copy)
{
	ResetToOwnedData();
	VertexMap VMap;  TriangleMap TMap;
	m_pMesh->Clear(true);
	m_pMesh->Copy(*copy.m_pMesh, VMap, &TMap );
	if ( copy.m_pMesh->HasUVSet(0) )
		MeshUtils::CopyUVs(*copy.m_pMesh, *m_pMesh, 0, 0, VMap);
	m_pPolygons->Initialize(*m_pMesh, *copy.m_pPolygons, TMap, &VMap); 
	m_bvTree.SetMesh( m_pMesh );

	m_vUVs = copy.m_vUVs;
	m_vNormals = copy.m_vNormals;

	return *this;
}

void GSurface::Copy( const VFTriangleMesh & mesh, const MeshPolygons & polygons)
{
	ResetToOwnedData();
	VertexMap VMap;  TriangleMap TMap;
	m_pMesh->Clear(true);
	m_pMesh->Copy(mesh, VMap, &TMap );
	if ( mesh.HasUVSet(0) )
		MeshUtils::CopyUVs(mesh, *m_pMesh, 0, 0, VMap);
	m_bvTree.SetMesh( m_pMesh );
	m_pPolygons->Initialize(*m_pMesh, polygons, TMap, &VMap); 
}


void GSurface::ResetToOwnedData()
{
	if ( ! m_bOwnsData ) {
		m_pMesh = new VFTriangleMesh();
		m_pPolygons = new MeshPolygons();
		m_bOwnsData = true;
		m_bvTree.SetMesh( m_pMesh );
	}
}




void GSurface::Compact()
{
	if ( m_pMesh->IsCompact() )
		return;

	VFTriangleMesh * pCurMesh = m_pMesh;
	MeshPolygons * pCurPolygons = m_pPolygons;
	if ( m_bOwnsData ) {
		m_pMesh = new VFTriangleMesh();
		m_pPolygons = new MeshPolygons();
		m_bvTree.SetMesh( m_pMesh );
	}

	rms::VertexMap vMap;		rms::TriangleMap tMap;
	m_pMesh->Copy(*pCurMesh, vMap, &tMap);
	*m_pPolygons = *pCurPolygons;
	m_pPolygons->Rewrite(vMap, tMap);

	// ok?

	if ( m_bOwnsData ) {
		delete pCurMesh;
		delete pCurPolygons;
	}
}