// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include <VFTriangleMesh.h>


namespace rms {


/*
 * Compute vertex laplacians for multiple orders. Computation is done on demand.
 */
class MeshLaplacian
{
public:
	struct VertexLaplacian {
		int nOrder;
		float fWi;
		std::vector<IMesh::VertexID> vNbrs;
		std::vector<float> vWij;
		float fWii;
	};
	enum WeightType {
		UniformWeight,
		CotangentWeight
	};

	MeshLaplacian( VFTriangleMesh * pMesh, WeightType eWeightType );

	VertexLaplacian & Get(IMesh::VertexID vID, unsigned int nOrder = 1);
	const VertexLaplacian & Get(IMesh::VertexID vID, unsigned int nOrder = 1) const;


protected:
	VFTriangleMesh * m_pMesh;
	WeightType m_eWeightType;

	struct VertexSet {
		std::vector<VertexLaplacian> vOrders;
	};
	std::vector<VertexSet> m_vVertices;

	void ComputeLaplacian( IMesh::VertexID vID, unsigned int nOrder );
};




}   // end namespace rms