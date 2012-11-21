// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "MeshProjection.h"

#include <limits>
#include <rmsdebug.h>

#include <VectorUtil.h>


using namespace rms;


MeshProjection::MeshProjection(GSurface * pSurface)
{
	m_pSurface = pSurface;
}

MeshProjection::~MeshProjection(void)
{
}


bool MeshProjection::GetUVFromNearest( Wml::Vector3f vPoint, Wml::Vector2f & vUV )
{
	VFTriangleMesh & mesh = m_pSurface->Mesh();
	MeshPolygons & polygons = m_pSurface->Polygons();
	IMeshBVTree & bvTree = m_pSurface->BVTree();
	UVList & UVs = m_pSurface->UV();

	Wml::Vector3f vNearest;		IMesh::TriangleID tNearestID;
	if (! bvTree.FindNearest( vPoint, vNearest, tNearestID ) ) 
		return false;

	float fBary[3];
	Wml::Vector3f vTri[3];
	mesh.GetTriangle(tNearestID, vTri);
	BarycentricCoords( vTri[0], vTri[1], vTri[2], vNearest,
		fBary[0], fBary[1], fBary[2] );

	MeshPolygons::PolygonID polyID = polygons.FindPolygon(tNearestID);
	const std::vector<IMesh::VertexID> & vPolyLoop = polygons.GetBoundary(polyID);
	const std::vector<unsigned int> & vPolyUV = polygons.GetBoundaryUV(polyID);

	Wml::Vector2f vTriUV[3];		int nUVCount = 0;
	IMesh::VertexID nTri[3];
	mesh.GetTriangle(tNearestID, nTri);
	for ( unsigned int i = 0; i < vPolyLoop.size(); ++i ) {
		for ( unsigned int j = 0; j < 3; ++j ) {
			if ( vPolyLoop[i] == nTri[j] ) {
				vTriUV[j] = UVs[vPolyUV[i]];
				++nUVCount;
			}
		}
	}
	if ( nUVCount == 3 ) {
		vUV = fBary[0]*vTriUV[0] + fBary[1]*vTriUV[1] + fBary[2]*vTriUV[2];
		return true;
	} else
		return false;
}
