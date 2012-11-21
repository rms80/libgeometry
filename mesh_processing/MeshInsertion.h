// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include <VFTriangleMesh.h>
#include <Triangulator2D.h>
#include <BitSet.h>
#include <ISurfaceProjector.h>



namespace rms {


/*
 * this class inserts one mesh into another, in UV-space, and then optionally supports projection back to a 3D surface
 */
class MeshInsertion
{
public:
	MeshInsertion();

	bool SetDestMesh( rms::VFTriangleMesh * pDestMesh );

	void SetInsertMesh( rms::VFTriangleMesh * pInsertMesh );
	void SetInsertionScale( float fScale, const Wml::Vector2f & vInsertionOrigin = Wml::Vector2f::ZERO );

	void Set3DProjector( ISurfaceProjector * pProjector );

	rms::VFTriangleMesh & MergeMesh() { return m_MergedMesh; }
	const VertexMap & InsertToMergeMap() const { return m_vInsertToMergeVMap; }
	const TriangleMap & InsertToMergeTMap() const { return m_vInsertToMergeTMap; }
	const VertexMap & InsertBdryToMergeMap() const { return m_vInsertBdryToMergeMap; }
	const VertexMap & DestToMergeMap() const { return m_vDestToMergeMap; }
	const BitSet & InsertToMergeTBits() const { return m_vMergeInsertBitmap; }

	void SetHoleHack( const Wml::Vector2f & vHole ) { m_vHoleHack = vHole; m_bUseHoleHack = true; }
	void SetUseInsertLoopHack( int nLoop ) { m_nUseSingleInsertionLoop = nLoop; }

	bool Compute();

	bool ProjectInsertInterior();

	void CopyInsertInterior(bool bIncludeBoundary = false);

protected:
	rms::VFTriangleMesh * m_pDestMesh;			// initial mesh, with UV coordinates for all vertices

	rms::VFTriangleMesh * m_pInsertMesh;		// inserted mesh, with UV coordinates for all vertices
	float m_fInsertionScale;					// scaling factor for insertion mesh
	Wml::Vector2f m_vInsertionOrigin;			// scaling origin for insertion mesh

	std::vector< std::vector<IMesh::VertexID> > m_vDestLoops;		// boundary loops of destination mesh
	std::vector<std::vector<IMesh::VertexID> > m_vInsertLoops;		// boundary loops of insertion mesh

	rms::VFTriangleMesh m_MergedMesh;		// merged result mesh

	VertexMap m_vInsertToMergeVMap;			// vertex map from InsertionMesh to MergedMesh
	TriangleMap m_vInsertToMergeTMap;		// triangle map from InsertionMesh to MergedMesh
	VertexMap m_vInsertBdryToMergeMap;		// map from InsertionMesh boundary IDs  to  MergedMesh
	VertexMap m_vDestToMergeMap;			// map from DestMesh to MergedMesh

	BitSet m_vMergeInsertBitmap;			// per-face bits, set to 1 if merge face is from insert mesh

	// projection to 3D mesh
	ISurfaceProjector * m_pProjector;

	bool m_bUseHoleHack;
	Wml::Vector2f m_vHoleHack;

	int m_nUseSingleInsertionLoop;
};



}   // end namespace rms