// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include <VFTriangleMesh.h>


// predecl to avoid include
namespace gsi {
	class SparseSymmetricMatrixSolver;
};


namespace rms {



class MeshVFunctionf
{
public:
	VFTriangleMesh * m_pMesh;
	IVertexFunctionf * V;

	MeshVFunctionf(VFTriangleMesh * pMesh, IVertexFunctionf * pFunction);

	void ClearConstraints();
	void ConstrainValue( IMesh::VertexID vID, float * pValue, unsigned int nValueSize = 1 );

	//! interpolates boundary values to interior. Can also constrain interior vertices.
	bool SolveDirichlet();

protected:

	struct Constraint {
		IMesh::VertexID vID;
		std::vector<float> vValue;
		bool operator<(const Constraint & c2 ) const 
			{ return vID < c2.vID; }
		Constraint() 
			{ vID = IMesh::InvalidID; }
		Constraint(IMesh::VertexID id) 
			{ vID = id; }
	};
	std::set<Constraint> m_vConstraints;

	gsi::SparseSymmetricMatrixSolver * m_pSolver;
	gsi::SparseSymmetricMatrixSolver * Solver();
};



}   // end namespace rms