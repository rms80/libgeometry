// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "MeshFunction.h"
#include "VectorUtil.h"
#include "MeshUtils.h"
#include "VertexWeights.h"
#include <SparseSymmetricMatrixSolver.h>

#include "rmsdebug.h"

using namespace rms;

MeshVFunctionf::MeshVFunctionf(VFTriangleMesh * pMesh, IVertexFunctionf * pFunction)
{ 
	m_pMesh = pMesh;
	V = pFunction; 

	m_pSolver = NULL;
}


gsi::SparseSymmetricMatrixSolver * MeshVFunctionf::Solver()
{
	if ( m_pSolver == NULL )
		m_pSolver = new gsi::SparseSymmetricMatrixSolver();
	return m_pSolver;
}


void MeshVFunctionf::ClearConstraints()
{
	m_vConstraints.clear();
}

void MeshVFunctionf::ConstrainValue( IMesh::VertexID vID, float * pValue, unsigned int nValueSize )
{
	Constraint c(vID);
	std::set<Constraint>::iterator found = m_vConstraints.find(c);
	if ( found == m_vConstraints.end() ) {
		for ( unsigned int k = 0; k < nValueSize; ++k )
			c.vValue.push_back( pValue[k] );
		m_vConstraints.insert(c);
	} else {
		Constraint & f = *found;
		for ( unsigned int k = 0; k < nValueSize; ++k )
			f.vValue[k] = pValue[k];	
	}
}



bool MeshVFunctionf::SolveDirichlet(  )
{
	if ( V == NULL )
		return false;

	gsi::SparseSymmetricMatrixSolver * pSolver = Solver();

	// make linear list of vertices (some could be missing)
	std::vector<IMesh::VertexID> vID;
	std::vector<unsigned int> reverseMap(m_pMesh->GetMaxVertexID());
	VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		vID.push_back( *curv ); 
		reverseMap[*curv] = (unsigned int)vID.size()-1;

		if ( m_pMesh->IsBoundaryVertex(*curv) && m_vConstraints.find(Constraint(*curv)) == m_vConstraints.end() )
			return false;	// all boundary verts need to be constrained

		++curv;
	}
	unsigned int nVerts = (unsigned int)vID.size();
	unsigned int nRHS = V->Components();

	// build sparse matrix
	pSolver->Resize(nVerts, nVerts);
	pSolver->ResizeRHS( nRHS );

	// set interior rows
	std::vector<IMesh::VertexID> vOneRing;
	std::vector<float> vWeights;
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		std::set<Constraint>::iterator found = m_vConstraints.find(Constraint(vID[i]));
		if ( found == m_vConstraints.end() ) {
			m_pMesh->VertexOneRing( vID[i], vOneRing, true );
			VertexWeights::Cotangent(*m_pMesh, vID[i], vOneRing, vWeights);
			size_t nOneRing = vOneRing.size();
			for ( unsigned int j = 0; j < nOneRing; ++j ) {
				pSolver->Set( i, reverseMap[vOneRing[j]], -vWeights[j] );
				pSolver->Set( i, i, 1.0f );		// weights sum to 1
			}
			for ( unsigned int k = 0; k < nRHS; ++k )
				pSolver->SetRHS(i, 0, k);
		} else {
			Constraint & c = *found;
			pSolver->Set(i, i, 1.0f);
			for ( unsigned int k = 0; k < nRHS; ++k )
				pSolver->SetRHS(i, c.vValue[k], k );
		}
	}

	if ( ! pSolver->Solve() )
		return false;

	for ( unsigned int i = 0; i < nVerts; ++i ) {
		for ( unsigned int k = 0; k < nRHS; ++k )
			(*V)(vID[i],k) = (float)pSolver->GetSolution(i, k);
	}

	return true;
}

