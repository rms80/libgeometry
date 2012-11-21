// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include "config.h"
#include <VFTriangleMesh.h>
#include <BitSet.h>
#include <vector>
#include <set>

namespace rms {

class MeshSelection
{
public:
	MeshSelection(const VFTriangleMesh * pMesh = NULL);
	~MeshSelection(void);

	void ClearAll();

	const std::set<IMesh::TriangleID> & Triangles() const
		{ return m_vTriangles; }

	const std::set<IMesh::VertexID> & Vertices() const
		{ return m_vVertices; }

	const std::set<IMesh::TriangleID> & LastVertexDelta() const
		{ return m_vVertexDelta; }

	//! append vertex
	void SelectVertex( IMesh::VertexID vID );

	//! appends vertices of tID
	void SelectVertices( IMesh::TriangleID tID );

	//! append face
	void SelectFace( IMesh::TriangleID vID );

	//! appends faces of vID 
	void SelectFaces( IMesh::VertexID vID );

	void SelectFaces( BitSet & vFaces );

	//! appends all faces connected to vVerts to vSelection
	void SelectFaces( const std::vector<IMesh::VertexID> & vVerts );

	void SelectAllFaces();

	// TODO: this would be more efficient if we could find the boundary verts of a selection...
	//! appends all one-ring neighbours of current selection. Stores added verts, if requested
	void GrowFaces( bool bComputeVertDelta = false );

	//! appends all one-ring neighbours of current selection. Stores added verts, if requested
	void GrowVertices( bool bComputeVertDelta = false, int nIters = 1 );

	//! select vertices for faces
	void SelectFaceVertices();

	//! select one-ring of this vertex
	void SelectVertexFaces( IMesh::VertexID vID );

	//! select faces for vertices. If bFullOnly, restrict to faces where all vertices are selected
	void SelectVertexFaces(bool bFullOnly = false);


	//! flood-fill selection of all vertices connected to vID
	void FloodSelectVertices( IMesh::VertexID vID );


	//! currently not very efficient...
	void GetSelectionMesh( rms::VFTriangleMesh & mesh );

	//! convert to bitset
	void SelectedVertices( rms::BitSet & bits ) const;

	bool HasVertex(IMesh::VertexID vID) const;
	bool HasFace(IMesh::VertexID vID) const;


protected:
	std::set<IMesh::TriangleID> m_vTriangles;
	std::set<IMesh::VertexID> m_vVertices;

	std::set<IMesh::VertexID> m_vVertexDelta;

	const VFTriangleMesh * m_pMesh;
};



}  // end namespace rms
