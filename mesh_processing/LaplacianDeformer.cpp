// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "LaplacianDeformer.h"
#include "MeshUtils.h"
#include <Wm4LinearSystem.h>
#include <SparseLinearSystem.h>
#include <Solver_TAUCS.h>


using namespace rms;


LaplacianDeformer::LaplacianDeformer()
{
	m_pMesh = NULL;
	m_pSolver = NULL;
	m_pSystemM = NULL;
	m_pLs = new gsi::SparseMatrix();
}


gsi::Solver_TAUCS * LaplacianDeformer::GetSolver()
{
	if ( m_pSolver == NULL )
		m_pSolver = new gsi::Solver_TAUCS(GetSystem());
	return m_pSolver;
}


gsi::SparseLinearSystem * LaplacianDeformer::GetSystem()
{
	if ( m_pSystemM == NULL )
		m_pSystemM = new gsi::SparseLinearSystem();
	return m_pSystemM;
}



void LaplacianDeformer::SetMesh(rms::VFTriangleMesh * pMesh)
{
	m_pMesh = pMesh;
	m_vConstraints.resize(0);
	ComputeWeights();
	m_bMatricesValid = false;
}


void LaplacianDeformer::AddBoundaryConstraints(float fWeight)
{
	VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		if ( m_pMesh->IsBoundaryVertex(vID) ) {
			Wml::Vector3f v;
			m_pMesh->GetVertex(vID, v);
			UpdateConstraint(vID, v, fWeight);
		}
	}
}



void LaplacianDeformer::UpdateConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition, float fWeight )
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



rms::Frame3f LaplacianDeformer::GetCurrentFrame( IMesh::VertexID vID )
{
	Wml::Vector3f vNormal(MeshUtils::EstimateNormal(*m_pMesh, vID));

	// compute frame
	const VtxInfo & vi = m_vVertices[vID];
	Wml::Vector3f vVtx, vNbr;
	m_pMesh->GetVertex(vID, vVtx);
	m_pMesh->GetVertex(vi.nTangentNbr, vNbr);
	Wml::Vector3f xik = vNbr - vVtx;
	Wml::Vector3f xikbar = vNormal.Cross( xik.Cross(vNormal) );
	xikbar.Normalize();
	xik = vNormal.Cross( xikbar );

	rms::Frame3f vFrame(vVtx);
	vFrame.SetFrame( xikbar, xik, vNormal );
	return vFrame;
}



void LaplacianDeformer::SetTargetFrames(VFTriangleMesh * pMesh)
{
	unsigned int nVerts = pMesh->GetMaxVertexID();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		pMesh->GetNormal(i, m_vVertices[i].vTargetNormal);
		
		Wml::Vector3f vVtx;
		pMesh->GetVertex(i, vVtx);
		pMesh->GetVertex(m_vVertices[i].nTangentNbr, m_vVertices[i].vTargetTangentDir);
		m_vVertices[i].vTargetTangentDir -= vVtx;
		m_vVertices[i].bHasTargetNormal = true;
		m_vVertices[i].fTargetScale = 1.0f;
	}
}

void LaplacianDeformer::ZeroLaplacianVectors(bool bInteriorOnly)
{
	size_t nVerts = m_vVertices.size();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		if ( bInteriorOnly && m_pMesh->IsBoundaryVertex(i) )
			continue;
		m_vVertices[i].vLaplacian = Wml::Vector3f::ZERO;
	}
}

void LaplacianDeformer::SetLaplacianVector( IMesh::VertexID vID, const Wml::Vector3f & vLaplacian )
{
	m_vVertices[vID].vLaplacian = vLaplacian;
	m_vVertices[vID].vSaveLaplacian = vLaplacian;
}


void LaplacianDeformer::PostProcess_SnapConstraints()
{
	size_t nVerts = m_vConstraints.size();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint & ci = m_vConstraints[i];
		m_pMesh->SetVertex( ci.vID, ci.vPosition );
	}
}



void LaplacianDeformer::ComputeWeights()
{
	unsigned int nVerts = m_pMesh->GetMaxVertexID();
	if ( nVerts != m_pMesh->GetVertexCount() )
		lgBreakToDebugger();		// can happen...

	m_vVertices.resize(0);
	m_vVertices.resize( nVerts );

	for ( unsigned int i = 0; i < nVerts; ++i ) {
		VtxInfo & vi = m_vVertices[i];
		
		MeshUtils::VertexOneRing(*m_pMesh, i, vi.vNbrs, false);
		size_t nNbrs = vi.vNbrs.size();

		MeshUtils::CotangentWeights(*m_pMesh, i, vi.vNbrs, vi.vNbrWeights);
//		MeshUtils::UniformWeights(*m_pMesh, i, vi.vNbrs, vi.vNbrWeights);

		Wml::Vector3f vVtx, vNormal, vNbr;
		m_pMesh->GetVertex(i, vVtx, &vNormal);

		// find most-orthogonal outgoing edge (for tangent-frame calculation)
		float fMinNbrDot = 1.0f;   unsigned int nBestNbr = -1;
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			m_pMesh->GetVertex( vi.vNbrs[k], vNbr );
			vNbr -= vVtx;   vNbr.Normalize();
			float fDot = (float)fabs(vNbr.Dot(vNormal));
			if ( fDot < fMinNbrDot ) {
				fMinNbrDot = fDot;
				nBestNbr = k;
			}
		}
		if ( nBestNbr == -1 )
			nBestNbr = 0;
		vi.nTangentNbr = vi.vNbrs[nBestNbr];

		// compute laplacian vector
		vi.vLaplacian = MeshUtils::MeshLaplacian(*m_pMesh, i, vi.vNbrs, vi.vNbrWeights);
		vi.vFrameLaplacian = ToNbrFrame( i, vi.vLaplacian );
	}
}






void LaplacianDeformer::UpdateMatrices()
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

	if ( ! GetSystem()->Matrix().IsSymmetric() ) {
		lgBreakToDebugger();
		return;
	}

	GetSolver()->OnMatrixChanged();
	GetSolver()->SetStoreFactorization(true);
	GetSolver()->SetSolverMode( gsi::Solver_TAUCS::TAUCS_LLT );
	GetSolver()->SetOrderingMode( gsi::Solver_TAUCS::TAUCS_METIS );

	m_bMatricesValid = true;




	//int nVerts = (int)m_vVertices.size();
	//int nCons = (int)m_vConstraints.size();
	//int nMatrixCols = nVerts;
	//int nMatrixRows = nVerts + nCons;


	//Wml::GMatrixd M;
	//M.SetSize(nMatrixRows, nMatrixCols);
	//m_RHS[0].SetSize(nMatrixRows);      m_RHS[1].SetSize(nMatrixRows);    m_RHS[2].SetSize(nMatrixRows);

	//for ( int ri = 0; ri < nVerts; ++ri ) {
	//	VtxInfo & vi = m_vVertices[ri];
	//	size_t nNbrs = vi.vNbrs.size();

	//	M(ri, ri) = 1.0f;
	//	for ( unsigned int k = 0; k < nNbrs; ++k )
	//		M(ri, vi.vNbrs[k]) = - vi.vNbrWeights[k];

	//	m_RHS[0][ri] = vi.vLaplacian.X();
	//	m_RHS[1][ri] = vi.vLaplacian.Y();
	//	m_RHS[2][ri] = vi.vLaplacian.Z();
	//}


	//for ( int ci = 0; ci < nCons; ++ci ) {
	//	Constraint & c = m_vConstraints[ci];
	//	int ri = nVerts + ci;
	//	M(ri,c.vID) = c.fWeight;
	//	m_RHS[0][ri] = c.fWeight * c.vPosition.X();
	//	m_RHS[1][ri] = c.fWeight * c.vPosition.Y();
	//	m_RHS[2][ri] = c.fWeight * c.vPosition.Z();
	//};

	//m_MT = M.Transpose();
	//m_MTM = m_MT * M;


	//if ( m_MTM.GetRows() != m_MTM.GetColumns() )
	//	lgBreakToDebugger();
	//unsigned int nSize = m_MTM.GetRows();
	//GetSystem()->Resize(nSize,nSize);

	//for ( unsigned int r = 0; r < nSize; ++r ) {
	//	for ( unsigned int c = 0; c < nSize; ++c )
	//		if ( m_MTM(r,c) != 0 )
	//			GetSystem()->Set(r, c, m_MTM(r,c));
	//}
	//GetSystem()->ResizeRHS(3);

	//if ( ! GetSystem()->Matrix().IsSymmetric() )
	//	lgBreakToDebugger();

	//GetSolver()->OnMatrixChanged();
	//GetSolver()->SetStoreFactorization(true);
	//GetSolver()->SetSolverMode( gsi::Solver_TAUCS::TAUCS_LLT );
	//GetSolver()->SetOrderingMode( gsi::Solver_TAUCS::TAUCS_METIS );

	//m_bMatricesValid = true;
}




void LaplacianDeformer::UpdateRHS(bool bUseTargetNormals, bool bUseTargetTangents, bool bEstimateNormals)
{
	unsigned int nVerts = (unsigned int)m_vVertices.size();

	gsi::SparseLinearSystem * pSystem = GetSystem();


	if ( bEstimateNormals )
		MeshUtils::EstimateNormals(*m_pMesh);

	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];

		if ( bUseTargetNormals ) {
			Wml::Vector3f vTransformed = FromNbrFrame(ri, vi.vFrameLaplacian, 
				&vi.vTargetNormal, 
				(bUseTargetTangents) ? &vi.vTargetTangentDir : NULL);
			vi.vLaplacian = vTransformed;
		} else if ( bEstimateNormals ) {
			Wml::Vector3f vTransformed = FromNbrFrame(ri, vi.vFrameLaplacian, NULL, NULL);
			vi.vLaplacian = vTransformed;
		}

		Wml::Vector3f vLaplacian = vi.vLaplacian;
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




	//int nVerts = (int)m_vVertices.size();
	//int nCons = (int)m_vConstraints.size();

	//if ( bUseTargetNormals ) {
	//	for ( int ri = 0; ri < nVerts; ++ri ) {
	//		VtxInfo & vi = m_vVertices[ri];
	//		Wml::Vector3f vTransformed = FromNbrFrame(ri, vi.vFrameLaplacian, 
	//			&vi.vTargetNormal, 
	//			(bUseTargetTangents) ? &vi.vTargetTangentDir : NULL);
	//		vi.vLaplacian = vTransformed;
	//		m_RHS[0][ri] = vTransformed.X();
	//		m_RHS[1][ri] = vTransformed.Y();
	//		m_RHS[2][ri] = vTransformed.Z();
	//	}
	//} else if ( bEstimateNormals ) {
	//	MeshUtils::EstimateNormals(*m_pMesh);
	//	for ( int ri = 0; ri < nVerts; ++ri ) {
	//		VtxInfo & vi = m_vVertices[ri];
	//		Wml::Vector3f vTransformed = FromNbrFrame(ri, vi.vFrameLaplacian, NULL, NULL);
	//		vi.vLaplacian = vTransformed;
	//		m_RHS[0][ri] = vTransformed.X();
	//		m_RHS[1][ri] = vTransformed.Y();
	//		m_RHS[2][ri] = vTransformed.Z();
	//	}
	//} else {
	//	for ( int ri = 0; ri < nVerts; ++ri ) {
	//		VtxInfo & vi = m_vVertices[ri];
	//		m_RHS[0][ri] = vi.vLaplacian.X();
	//		m_RHS[1][ri] = vi.vLaplacian.Y();
	//		m_RHS[2][ri] = vi.vLaplacian.Z();
	//	}
	//}

	//for ( int ci = 0; ci < nCons; ++ci ) {
	//	Constraint & c = m_vConstraints[ci];
	//	int ri = nVerts + ci;
	//	m_RHS[0][ri] = c.fWeight * c.vPosition.X();
	//	m_RHS[1][ri] = c.fWeight * c.vPosition.Y();
	//	m_RHS[2][ri] = c.fWeight * c.vPosition.Z();
	//};

	//for ( int k = 0; k < 3; ++k )
	//	m_MTRHS[k] = m_MT * m_RHS[k];

	//for ( int i = 0; i < nVerts; ++i ) {
	//	for ( unsigned int k = 0; k < 3; ++k )
	//		GetSystem()->SetRHS( i, m_MTRHS[k][i], k );
	//}
}



void LaplacianDeformer::Solve_Lipman04(int nIterations)
{
	lgBreakToDebugger();
	//UpdateMatrices();
	//UpdateRHS(false,false,false);

	//// store original laplacians
	//int nVerts = (int)m_vVertices.size();
	//for ( int ri = 0; ri < nVerts; ++ri ) {
	//	VtxInfo & vi = m_vVertices[ri];
	//	vi.vSaveLaplacian = vi.vLaplacian;
	//}

	//bool bOK = GetSolver()->Solve();
	//if ( ! bOK )
	//	lgBreakToDebugger();

	//for ( int k = 0; k < nIterations; ++k ) {
	//	int nMatrixCols = (int)m_vVertices.size();
	//	for ( int i = 0; i < nMatrixCols; ++i ) {
	//		Wml::Vector3f v((float)GetSystem()->GetSolution(i,0), (float)GetSystem()->GetSolution(i,1), (float)GetSystem()->GetSolution(i,2) );
	//		m_pMesh->SetVertex(i, v);
	//	}
	//	UpdateRHS(false, false, true);
	//	bOK = GetSolver()->Solve();
	//	if ( ! bOK )
	//		lgBreakToDebugger();
	//}

	//int nMatrixCols = (int)m_vVertices.size();
	//for ( int i = 0; i < nMatrixCols; ++i ) {
	//	Wml::Vector3f v((float)GetSystem()->GetSolution(i,0), (float)GetSystem()->GetSolution(i,1), (float)GetSystem()->GetSolution(i,2) );
	//	m_pMesh->SetVertex(i, v);
	//}

	//// restore original laplacians
	//for ( int ri = 0; ri < nVerts; ++ri ) {
	//	VtxInfo & vi = m_vVertices[ri];
	//	vi.vLaplacian = vi.vSaveLaplacian;
	//}

}

void LaplacianDeformer::Solve(bool bUseTargetNormals)
{
	UpdateMatrices();
	UpdateRHS(bUseTargetNormals, true, false);

	bool bOK = GetSolver()->Solve();
	if ( ! bOK )
		lgBreakToDebugger();

	int nMatrixCols = (int)m_vVertices.size();
	for ( int i = 0; i < nMatrixCols; ++i ) {
		Wml::Vector3f v((float)GetSystem()->GetSolution(i,0), (float)GetSystem()->GetSolution(i,1), (float)GetSystem()->GetSolution(i,2) );
		m_pMesh->SetVertex(i, v);
	}


	//UpdateMatrices();
	//UpdateRHS(bUseTargetNormals, true, false);
	////UpdateRHS(false, false, true);

	//bool bOK = GetSolver()->Solve();
	//if ( ! bOK )
	//	lgBreakToDebugger();

	//int nMatrixCols = (int)m_vVertices.size();
	//for ( int i = 0; i < nMatrixCols; ++i ) {
	//	Wml::Vector3f v((float)GetSystem()->GetSolution(i,0), (float)GetSystem()->GetSolution(i,1), (float)GetSystem()->GetSolution(i,2) );
	//	m_pMesh->SetVertex(i, v);
	//}

	//Wml::GVectord sol[3];
	//sol[0].SetSize( m_MTRHS[0].GetSize() );
	//sol[1].SetSize( m_MTRHS[0].GetSize() );
	//sol[2].SetSize( m_MTRHS[0].GetSize() );
	//for ( int k = 0; k < 3; ++k ) {
	//	Wml::LinearSystemd linsys;
	//	linsys.Solve( m_MTM, (const double *)m_MTRHS[k], (double *)sol[k] );
	//}
	//int nMatrixCols = (int)m_vVertices.size();
	//for ( int i = 0; i < nMatrixCols; ++i ) {
	//	Wml::Vector3f v((float)sol[0][i], (float)sol[1][i], (float)sol[2][i] );
	//	m_pMesh->SetVertex(i, v);
	//}
}




Wml::Vector3f LaplacianDeformer::ToNbrFrame( IMesh::VertexID vID, const Wml::Vector3f & v)
{
	Wml::Vector3f vNormal, vVtx, vNbr;
	m_pMesh->GetVertex(vID, vVtx, &vNormal);

	VtxInfo & vi = m_vVertices[vID];
	m_pMesh->GetVertex(vi.nTangentNbr, vNbr);

	Wml::Vector3f e1( vNbr - vVtx );
	e1 = e1 - (e1.Dot(vNormal))*vNormal;
	e1.Normalize();
	Wml::Vector3f e2 = e1.Cross(vNormal);  
	e2.Normalize();
	return Wml::Vector3f( vNormal.Dot(vi.vLaplacian),  e1.Dot(vi.vLaplacian),  e2.Dot(vi.vLaplacian) );
}



Wml::Vector3f LaplacianDeformer::FromNbrFrame( IMesh::VertexID vID, const Wml::Vector3f & v,  Wml::Vector3f * pNormalDir, Wml::Vector3f * pTangentDir )
{
	VtxInfo & vi = m_vVertices[vID];

	Wml::Vector3f vNormal, vVtx, e1, e2;
	m_pMesh->GetVertex(vID, vVtx, &vNormal);

	if ( pNormalDir != NULL )
		vNormal = *pNormalDir;

	if ( pTangentDir == NULL ) {
		Wml::Vector3f vNbr;
		m_pMesh->GetVertex(vi.nTangentNbr, vNbr);
		e1 = Wml::Vector3f( vNbr - vVtx );
	} else
		e1 = *pTangentDir;

	e1 = e1 - (e1.Dot(vNormal))*vNormal;
	e1.Normalize();
	e2 = e1.Cross(vNormal);  
	e2.Normalize();

	return v.X()*vNormal + v.Y()*e1 + v.Z()*e2;
}




void LaplacianDeformer::DebugRender()
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
		glVertex3fv( m_pMesh->GetVertex(m_vConstraints[i].vID) );
		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3fv(m_vConstraints[i].vPosition);
	}
	glEnd();
	glBegin(GL_LINES);
	glColor3f(0.0f, 0.0f, 0.0f);
	for ( unsigned int i = 0; i < m_vConstraints.size(); ++i ) {
		glVertex3fv( m_pMesh->GetVertex(m_vConstraints[i].vID) );
		glVertex3fv(m_vConstraints[i].vPosition);
	}
	glEnd();
		

	// draw current laplacian vectors
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);
	for ( unsigned int i = 0; i < m_vVertices.size(); ++i ) {
		m_pMesh->GetVertex( i, v );
		glVertex3fv( v );
		glVertex3fv( v + m_vVertices[i].vLaplacian );
	}
	glEnd();


	// draw target normals if we have them
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glColor3f(0.0f, 0.0f, 1.0f);
	for ( unsigned int i = 0; i < m_vVertices.size(); ++i ) {
		if ( ! m_vVertices[i].bHasTargetNormal )
			continue;
		m_pMesh->GetVertex( i, v );
		glVertex3fv( v );
		glVertex3fv( v + 0.05f*m_vVertices[i].vTargetNormal );
	}
	glEnd();

	glPopAttrib();

}