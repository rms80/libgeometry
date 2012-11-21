// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include <VFTriangleMesh.h>

namespace rms {

class VertexWeights
{
public:

	/*
	 * weights that work for any neighbourhood
	 */

	static void Uniform( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vNeighbourhood, 
						   std::vector<float> & vWeights, bool bNormalize = true );

	static void InverseDistance( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vNeighbourhood, 
								 std::vector<float> & vWeights, float fPow = 2.0f, float fEps = 0.0001f, bool bNormalize = true );


	/*
	 * weights that only work for a vertex one-ring
	 */

	//! assumes one-ring is ordered
	static void Cotangent( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vOneRing, 
						   std::vector<float> & vWeights, bool bNormalize = true );


};



}   // end namespace rms
