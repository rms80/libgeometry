// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include "config.h"
#include <VFTriangleMesh.h>
#include <MeshPolygons.h>
#include <IMeshBVTree.h>
#include <Frame.h>

namespace rms { 

/*
 * This class represents a set of selected faces on a triangle mesh (and contains operations on said set)
 */
class SurfaceAreaSelection
{
public:
	SurfaceAreaSelection(void);
	~SurfaceAreaSelection(void);

	//! must set at least pMesh and pBVTree. If polygons are set, selection will automatically expand to contain all tris inside polygons
	void SetSurface(rms::VFTriangleMesh * pMesh, rms::MeshPolygons * pPolygons, rms::IMeshBVTree * pBVTree);

	//! explicitly select a set of faces  (clears current selection)
	void Select(const std::set<IMesh::TriangleID> & vTriangles );

	//! clear selection
	void Reset();

	//! append a point to internal stroke
	void AppendToStroke( const rms::Frame3f & vFrame );
	//! done appending points, process stroke
	void AppendCurrentStroke( );
	void CancelStroke();

	//! check if nearest face to vFrame.Origin() is selected
	bool Contains( const rms::Frame3f & vFrame );

	bool Contains( IMesh::TriangleID tID ) const 
		{ return m_vInterior.find(tID) != m_vInterior.end(); }

	//! flip selected faces
	bool ReverseSelection( );

	//! expand/contract
	void Expand(unsigned int nIters = 1);
	void Contract();

	//! computes geodesic distances from boundary and expands such that dist < offset*avgedgelenth (or < offset if !bScaleByMesh)
	void SetGrowOffset(float fOffset, bool bScaleByMesh = true);
	float GrowOffset() const { return m_fGrowOffset; }

	//! select/de-selet face that contains vFrame.Origin()
	bool Paint( const rms::Frame3f & vFrame, bool bAdd );

	//! returns true if there are selected faces, or if at least one stroke has been added (ie selection in progress)
	bool IsInitialized() const;

	//! returns true if there are selected faces
	bool IsValid() const;

	//! determine if set is connected
	bool IsConnected(rms::VFTriangleMesh & mesh) const;

	void Render(const Wml::Vector3f & vColor, bool bWireframe);

	const std::set<IMesh::TriangleID> & Selected() const { return m_vInterior; }
	void GetTriangleSet( std::set<rms::IMesh::TriangleID> & vTriangles, bool bSelected );
	void GetTriangleSet( std::vector<rms::IMesh::TriangleID> & vTriangles, bool bSelected );

	void GetVertexSet( std::vector<IMesh::VertexID> & vVertices, bool bSelected );

	void GetSubMesh( rms::VFTriangleMesh & mesh, bool bSelected );

	void SaveSelectionMesh(const char * pFilename);

	// stores list of selected triangles
	void SaveSelection(const char * pFilename);
	void LoadSelection(const char * pFilename);

protected:
	rms::VFTriangleMesh * m_pMesh;
	std::vector<Wml::Vector3f> m_vFaceNormalCache;
	rms::MeshPolygons * m_pPolygons;
	rms::IMeshBVTree * m_pBVTree;

	int m_nGLDisplayList;
	bool m_bGLDisplayListValid;

	Wml::AxisAlignedBox3f m_bounds;
	float m_fAvgEdgeLength;
	float m_fMeshSizeFactor;

	std::vector< rms::Frame3f > m_vCurStroke;
	std::vector< rms::Frame3f > m_vOriginalPath;
	std::vector< rms::Frame3f > m_vPath;

	bool m_bClosed;
	void TryClose();

	float m_fGrowOffset;

	std::set< unsigned int > m_vBoundaryTris;
	std::set< unsigned int > m_vInterior;
	void SetInterior( const std::set<unsigned int> & vSet, bool bToPolygons = true );
	void FindSelectionPatch();
	bool ComputeBoundaryTrisFromPath();
	bool FindNbrTriPoint( Wml::Vector3f vStart, Wml::Vector3f vEnd, unsigned int tStartTID, unsigned int tPrevID, Wml::Vector3f & vFound );
	void ClipEarTriangles();
	void FillBoundaryTriangles();
	void UpdateBoundaryTriangles();

	std::vector<float> m_vGeoDistances;
	void ValidateGeoDistances();
	void InvalidateGeoDistances();

	bool ProjectToSurface( Wml::Vector3f & vPoint, rms::Frame3f & vFrame );

	//! clip ears and fill inverse-ears  (this could be improved...)
	void Optimize();
};

} // end namespace rms