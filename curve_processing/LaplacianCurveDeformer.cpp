// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "opengl.h"
#include "LaplacianCurveDeformer.h"
#include "MeshUtils.h"
#include <Wm4LinearSystem.h>
#include <SparseLinearSystem.h>
#include <Solver_TAUCS.h>


using namespace rms;


LaplacianCurveDeformer::LaplacianCurveDeformer()
{
	m_pCurve = NULL;
	m_pSolver = NULL;
	m_pSystemM = NULL;
	m_pLs = new gsi::SparseMatrix();
	m_fLaplacianScale = 1.0f;
}


gsi::Solver_TAUCS * LaplacianCurveDeformer::GetSolver()
{
	if ( m_pSolver == NULL )
		m_pSolver = new gsi::Solver_TAUCS(GetSystem());
	return m_pSolver;
}


gsi::SparseLinearSystem * LaplacianCurveDeformer::GetSystem()
{
	if ( m_pSystemM == NULL )
		m_pSystemM = new gsi::SparseLinearSystem();
	return m_pSystemM;
}



void LaplacianCurveDeformer::SetCurve(rms::PolyLoop3f * pCurve)
{
	m_pCurve = pCurve;
	m_vConstraints.resize(0);
	ComputeWeights();
	m_bMatricesValid = false;
}


void LaplacianCurveDeformer::AddBoundaryConstraints(float fWeight)
{
	// polyloop has no boundaries!!
	lgBreakToDebugger();
}



void LaplacianCurveDeformer::UpdatePositionConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition, float fWeight )
{
	bool bFound = false;
	size_t nCount = m_vConstraints.size();
	for ( unsigned int k = 0; !bFound && k < nCount; ++k ) {
		if ( m_vConstraints[k].vID == vID ) {
			m_vConstraints[k].vPosition = vPosition;
			if ( m_vConstraints[k].fWeight != fWeight )
				m_bMatricesValid = false;
			m_vConstraints[k].fWeight = fWeight;
			bFound = true;
		}
	}
	if ( ! bFound ) {
		Constraint c;
		c.vID = vID;
		c.vPosition = vPosition;
		c.fWeight = fWeight;
		m_vConstraints.push_back(c);
		m_bMatricesValid = false;
	}
}




void LaplacianCurveDeformer::ComputeWeights()
{
	unsigned int nVerts = m_pCurve->VertexCount();
	m_vVertices.resize(0);
	m_vVertices.resize( nVerts );

	for ( unsigned int i = 0; i < nVerts; ++i ) {
		VtxInfo & vi = m_vVertices[i];
		
		IPolyCurve::VertexID nLeft, nRight;
		m_pCurve->GetNeighbours(i, nLeft, nRight);

		if ( nLeft != IPolyCurve::InvalidID )
			vi.vNbrs.push_back(nLeft);
		if ( nRight != IPolyCurve::InvalidID )
			vi.vNbrs.push_back(nRight);

		// just use uniform weights
		size_t nNbrs = vi.vNbrs.size();
		for ( unsigned int k = 0; k < nNbrs; ++k )
			vi.vNbrWeights.push_back(1.0f / (float)nNbrs);

		// compute laplacian vector
		vi.vLaplacian = Wml::Vector3f::ZERO;
		for ( unsigned int k = 0; k < nNbrs; ++k )
			vi.vLaplacian += vi.vNbrWeights[k] * ( m_pCurve->Vertex(vi.vNbrs[k]) - m_pCurve->Vertex(i) );
	}
}






void LaplacianCurveDeformer::UpdateMatrices()
{
	if ( m_bMatricesValid )
		return;

	unsigned int nVerts = (unsigned int)m_vVertices.size();

	GetSystem()->Resize(nVerts, nVerts);
	GetSystem()->ResizeRHS(3);

	gsi::SparseMatrix & Ls = (*m_pLs);
	Ls.Resize(nVerts, nVerts);
	
	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		size_t nNbrs = vi.vNbrs.size();

		double dSum = 0.0f;
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			Ls(ri, vi.vNbrs[k]) = vi.vNbrWeights[k];
			dSum += vi.vNbrWeights[k];
		}
		Ls(ri, ri) = -dSum;
	}

	// fold in area weights matrix M here

	// construct system
	gsi::SparseMatrix Msys( Ls * Ls );

	// add soft constraints
	unsigned int nCons = (unsigned int)m_vConstraints.size();
	for ( unsigned int ci = 0; ci < nCons; ++ci ) {
		Constraint & c = m_vConstraints[ci];
		int ri = c.vID;
		Msys(ri,ri) = Msys(ri,ri) + c.fWeight*c.fWeight;
	}

	GetSystem()->SetMatrix(Msys);

	if ( ! GetSystem()->Matrix().IsSymmetric() )
		lgBreakToDebugger();

	GetSolver()->OnMatrixChanged();
	GetSolver()->SetStoreFactorization(true);
	GetSolver()->SetSolverMode( gsi::Solver_TAUCS::TAUCS_LLT );
	GetSolver()->SetOrderingMode( gsi::Solver_TAUCS::TAUCS_METIS );

	m_bMatricesValid = true;
}




void LaplacianCurveDeformer::UpdateRHS()
{
	unsigned int nVerts = (unsigned int)m_vVertices.size();

	gsi::SparseLinearSystem * pSystem = GetSystem();

	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		Wml::Vector3f vLaplacian = vi.vLaplacian * m_fLaplacianScale;
		for ( int k = 0; k < 3; ++k )
			pSystem->SetRHS(ri, vLaplacian[k], k);
	}

	for ( int k = 0; k < 3; ++k ) {
		const gsi::Vector & vRHS = pSystem->GetRHS(k);
		gsi::Vector vLsRHS( (*m_pLs) * vRHS );
		pSystem->SetRHS( k, vLsRHS );
	}

	unsigned int nCons = (unsigned int)m_vConstraints.size();
	for ( unsigned int ci = 0; ci < nCons; ++ci ) {
		Constraint & c = m_vConstraints[ci];
		int ri = c.vID;
		Wml::Vector3f vConsVal = c.fWeight*c.fWeight*c.vPosition;
		for ( int k = 0; k < 3; ++k ) 
			pSystem->SetRHS( ri, pSystem->GetRHS(ri,k) + vConsVal[k], k );
	};
}




void LaplacianCurveDeformer::Solve()
{
	UpdateMatrices();
	UpdateRHS();

	bool bOK = GetSolver()->Solve();
	if ( ! bOK )
		lgBreakToDebugger();

	int nMatrixCols = (int)m_vVertices.size();
	for ( int i = 0; i < nMatrixCols; ++i ) {
		Wml::Vector3f v((float)GetSystem()->GetSolution(i,0), (float)GetSystem()->GetSolution(i,1), (float)GetSystem()->GetSolution(i,2) );
		m_pCurve->SetVertex(i, v);
	}
}








void LaplacianCurveDeformer::DebugRender()
{
	glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	
//	glDepthFunc(GL_LEQUAL);

	Wml::Vector3f v;


	// render constraint points
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	for ( unsigned int i = 0; i < m_vConstraints.size(); ++i ) {
		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3fv( m_pCurve->Vertex( m_vConstraints[i].vID ) );
		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3fv(m_vConstraints[i].vPosition);
	}
	glEnd();
	glBegin(GL_LINES);
	glColor3f(0.0f, 0.0f, 0.0f);
	for ( unsigned int i = 0; i < m_vConstraints.size(); ++i ) {
		glVertex3fv( m_pCurve->Vertex( m_vConstraints[i].vID ) );
		glVertex3fv(m_vConstraints[i].vPosition);
	}
	glEnd();
		

	// draw current laplacian vectors
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);
	for ( unsigned int i = 0; i < m_vVertices.size(); ++i ) {
		Wml::Vector3f v = m_pCurve->Vertex( i );
		glVertex3fv( v );
		glVertex3fv( v + m_vVertices[i].vLaplacian );
	}
	glEnd();

	glPopAttrib();

}