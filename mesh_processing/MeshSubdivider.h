// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_MESH_SUBDIVIDER_H__
#define __RMS_MESH_SUBDIVIDER_H__

#include <SparseArray.h>
#include <IMesh.h>
#include <VFTriangleMesh.h>
#include <IterativeAlgorithm.h>
#include "rmsprofile.h"
#include <set>

namespace rms {

class MeshSubdivider
{
public:
	MeshSubdivider(void);
	~MeshSubdivider(void);

	//! global subdivision 1->4 subdivision
	void Subdivide( VFTriangleMesh & mesh );


	void SetPreserveBoundaryEdges( bool bEnable ) { m_bSkipBoundaryEdges = bEnable; }
	void SetLengthThreshold( float fThresh ) { m_fIterEdgeLengthThresh = fThresh; }
	void SetAngleThreshold( float fThresh ) { m_fIterEdgeAngleThresh = fThresh; }
	void SetComputeSubdividedUVs( bool bEnable) { m_bComputeSubdividedUVs = bEnable; }
	
	//! initialize iterative (interruptible) 2->4 edge split subdivision
	void InitializeIterativeSubdivide( VFTriangleMesh & mesh );

	//! returns true when finished (false if interrupted before completion - but can be called again)
	bool DoIterativeSubdivide( VFTriangleMesh & mesh, IterationCallback & callback );

	const std::vector< IMesh::VertexID > & GetNewVerts() { return m_vNewVerts; }

protected:
	VFTriangleMesh * m_pMesh;

	struct EdgeVert {
		IMesh::VertexID ve2;
		IMesh::VertexID vNew;
		bool operator<( const EdgeVert & e2 ) const {
			return ve2 < e2.ve2;
		}
	};
	typedef std::set< EdgeVert > EdgeVertList;
	// RMS TODO: use a more efficient data structure for edge lists
	//  (pooled memory allocator?)

	std::vector< EdgeVertList > m_vEdges;
	EdgeVert & GetEdgeVert( IMesh::VertexID e1, IMesh::VertexID e2 );

	std::vector< IMesh::VertexID > m_vNewVerts;


	enum IterationState {
		NotIterating,
		Initialized,
		ComputingEdgeLengths,
		Subdividing
	};
	IterationState m_eIterState;



	struct Edge {
		IMesh::EdgeID eID;
		float fLength;
		float fCosAngle;

		inline float GetScore() const { 
			return fCosAngle;
		}

		// want reverse sort on edges!
		inline bool operator<( const Edge & e2 ) const {
			return GetScore() > e2.GetScore();
		}
	};
	typedef std::set<Edge> EdgeList;
	EdgeList m_vCurEdges;

	VFTriangleMesh::edge_iterator m_initCurEdge;

	float m_fIterEdgeAngleThresh;
	float m_fIterEdgeLengthThresh;
	bool m_bSkipBoundaryEdges;
	bool m_bComputeSubdividedUVs;

};


} // end namespace rmsmesh


#endif  // __RMS_MESH_SUBDIVIDER_H__
