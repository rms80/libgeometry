// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "LaplacianSmoother.h"
#include <MeshUtils.h>
#include <Wm4LinearSystem.h>
#include <SparseLinearSystem.h>
#include <Solver_TAUCS.h>
#include <rmsdebug.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
typedef Eigen::SparseMatrix<double,Eigen::RowMajor> SparseMatrixType;

using namespace rms;


LaplacianSmoother::LaplacianSmoother()
{
	m_pMesh = NULL;
	m_pSolver = NULL;
	m_pSystemM = NULL;
	m_pLs = new gsi::SparseMatrix();
	m_pM = new gsi::SparseMatrix();
	m_pSystem = new gsi::SparseMatrix();
	m_pRHS = new gsi::Vector[3];

	m_fLaplacianVectorScale = 0.0f;
	m_fInteriorConstraintWeightScale = 1.0f;

	m_eWeightMode = Weights_Cotan;
	m_eVertexAreaMode = Area_Mixed;

	m_nROISize = 0;

	m_bWeightsValid = false;
	m_bMatricesValid = false;
	m_bSolutionValid = false;
}
LaplacianSmoother::~LaplacianSmoother()
{
	if ( m_pLs )
		delete m_pLs;
	if ( m_pM )
		delete m_pM;
	if ( m_pSystem )
		delete m_pSystem;
	if ( m_pRHS )
		delete [] m_pRHS;
}

gsi::Solver_TAUCS * LaplacianSmoother::GetSolver()
{
	if ( m_pSolver == NULL )
		m_pSolver = new gsi::Solver_TAUCS(GetSystem());
	return m_pSolver;
}


gsi::SparseLinearSystem * LaplacianSmoother::GetSystem()
{
	if ( m_pSystemM == NULL )
		m_pSystemM = new gsi::SparseLinearSystem();
	return m_pSystemM;
}



void LaplacianSmoother::SetMesh(rms::VFTriangleMesh * pMesh)
{
	m_pMesh = pMesh;
	m_vConstraints.resize(0);

	m_vROI.clear();
	m_vROIBoundary.clear();
	m_nROISize = 0;

	if ( m_pMesh == NULL )
		return;

	m_vMap.Resize(m_pMesh->GetMaxVertexID(), m_pMesh->GetMaxVertexID());

	VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	unsigned int nIndex = 0;
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;

		if ( m_pMesh->IsIsolated(vID) )
			continue;

		m_vROI.insert(vID);
		m_vMap.SetMap( vID, nIndex );
		if ( m_pMesh->IsBoundaryVertex(vID) )
			m_vROIBoundary.insert(nIndex);
		nIndex++;
	}
	m_nROISize = nIndex;

	m_vVertices.resize(0);
	m_bWeightsValid = false;
	m_bMatricesValid = false;
}
void LaplacianSmoother::SetROI( const std::vector<IMesh::VertexID> & vROI )
{
	m_vConstraints.resize(0);

	m_nROISize = vROI.size();
	m_vROI.clear();
	m_vMap.Resize(m_pMesh->GetMaxVertexID(), (unsigned int)m_nROISize);
	m_vROIBoundary.clear();
	std::vector<IMesh::VertexID> vUseROI;

	unsigned int nIndex = 0;
	for ( unsigned int k = 0; k < m_nROISize; ++k ) {
		IMesh::VertexID vID = vROI[k];
		if ( m_pMesh->IsIsolated(vID) )
			continue;
		m_vROI.insert(vID);
		m_vMap.SetMap( vID, nIndex );
		vUseROI.push_back(vID);
		nIndex++;
	}
	m_nROISize = nIndex;

	std::vector<IMesh::VertexID> vOneRing;
	for ( unsigned int k = 0; k < m_nROISize; ++k ) {
		IMesh::VertexID vID = vUseROI[k];

		bool bBoundary = m_pMesh->IsBoundaryVertex(vID);
		if ( ! bBoundary ) {
			vOneRing.resize(0);
			MeshUtils::VertexOneRing(*m_pMesh, vID, vOneRing, false);
			size_t nCount = vOneRing.size();
			for ( unsigned int j = 0; !bBoundary && j < nCount; ++j ) {
				if ( m_vROI.find(vOneRing[j]) ==  m_vROI.end() )
					bBoundary = true;
			}
		}
		if ( bBoundary )
			m_vROIBoundary.insert(k);
	}

	m_vVertices.resize(0);
	m_bWeightsValid = false;
	m_bMatricesValid = false;
}



void LaplacianSmoother::AddBoundaryConstraints(float fWeight)
{
	std::set<unsigned int>::iterator curv(m_vROIBoundary.begin()), endv(m_vROIBoundary.end());
	while ( curv != endv ) {
		IMesh::VertexID vID = m_vMap.GetOld( *curv++ );
		Wml::Vector3f v;
		m_pMesh->GetVertex(vID, v);
		UpdateConstraint(vID, v, fWeight, CType_SoftBoundary);
	}
}
void LaplacianSmoother::AddSoftBoundaryConstraints(float fWeight, int nRings, bool bBlendWeight)
{
	ValidateWeights();		// have to do this because we use m_vVertices.vNbrs below...

	std::set<unsigned int> vDone;
	std::set<unsigned int> vCur(m_vROIBoundary);
	std::vector<unsigned int> vNext;
	std::vector<IMesh::VertexID> vOneRing;
	for ( int k = 0; k < nRings; ++k ) {

		vNext.resize(0);
		std::set<unsigned int>::iterator curv(vCur.begin()), endv(vCur.end());
		while ( curv != endv ) {
			unsigned int nIndex = *curv++;
			IMesh::VertexID vID = m_vMap.GetOld( nIndex );
			Wml::Vector3f v;
			m_pMesh->GetVertex(vID, v);
			UpdateConstraint(vID, v, fWeight, 
				(k == 0) ? CType_SoftBoundary_Ring0 : CType_SoftBoundary );
			vNext.push_back( nIndex );
			vDone.insert(nIndex);
		}

		if ( bBlendWeight )
			fWeight = fWeight/2;

		vCur.clear();
		size_t nNext = vNext.size();
		for ( unsigned int k = 0; k < nNext; ++k ) {
			std::vector<unsigned int> & vNbrs = m_vVertices[vNext[k]].vNbrs;
			size_t nCount = vNbrs.size();
			for ( unsigned int k = 0; k < nCount; ++k ) {
				if ( vDone.find(vNbrs[k]) == vDone.end() )
					vCur.insert(vNbrs[k]);
			}
		}
	}

}

void LaplacianSmoother::AddAllInteriorConstraints(float fWeight)
{
	for ( unsigned int k = 0; k < m_nROISize; ++k ) {
		if ( m_vROIBoundary.find(k) != m_vROIBoundary.end() )
			continue;
		IMesh::VertexID vID = m_vMap.GetOld( k );
		Wml::Vector3f v;
		m_pMesh->GetVertex(vID, v);
		UpdateConstraint(vID, v, fWeight, CType_SoftInterior);
	}
}



void LaplacianSmoother::UpdateConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition, float fWeight, ConstraintType eType )
{
	unsigned int nIndex = m_vMap.GetNew(vID);
	if ( nIndex == IMesh::InvalidID )
		return;

	bool bFound = false;
	size_t nCount = m_vConstraints.size();
	for ( unsigned int k = 0; !bFound && k < nCount; ++k ) {
		if ( m_vConstraints[k].vID == vID ) {
			m_vConstraints[k].vPosition = vPosition;
			if ( m_vConstraints[k].eType != eType && eType != CType_Unspecified ) {
				m_bMatricesValid = false;
				m_vConstraints[k].eType = eType;
			}
			if ( m_vConstraints[k].fWeight != fWeight ) {
				m_bMatricesValid = false;
				m_vConstraints[k].fWeight = fWeight;
			}
			bFound = true;
		}
	}
	if ( ! bFound ) {
		Constraint c;
		c.eType = eType;
		c.vID = vID;
		c.nIndex = nIndex;
		c.vPosition = vPosition;
		c.fWeight = fWeight;
		m_vConstraints.push_back(c);
		m_bMatricesValid = false;
	} else
		m_bSolutionValid = false;
}


void LaplacianSmoother::UpdateConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition )
{
	unsigned int nIndex = m_vMap.GetNew(vID);
	if ( nIndex == IMesh::InvalidID )
		return;

	size_t nCount = m_vConstraints.size();
	for ( unsigned int k = 0; k < nCount; ++k ) {
		if ( m_vConstraints[k].vID == vID ) {
			m_vConstraints[k].vPosition = vPosition;
			m_bSolutionValid = false;
			return;
		}
	}
}


LaplacianSmoother::ConstraintType LaplacianSmoother::GetConstraint( IMesh::VertexID vID )
{
	size_t nCount = m_vConstraints.size();
	for ( unsigned int k = 0; k < nCount; ++k ) {
		if ( m_vConstraints[k].vID == vID )
			return m_vConstraints[k].eType;
	}
	return CType_Unconstrained;
}


void LaplacianSmoother::UpdateConstraintsFromMesh()
{
	size_t nVerts = m_vConstraints.size();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint & ci = m_vConstraints[i];
		ci.vPosition = m_pMesh->GetVertex( ci.vID );
	}		
	m_bSolutionValid = false;
}

void LaplacianSmoother::SnapRing0BoundaryConstraints()
{
	size_t nVerts = m_vConstraints.size();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint & ci = m_vConstraints[i];
		if ( ci.eType == CType_SoftBoundary_Ring0 )
			m_pMesh->SetVertex( ci.vID, ci.vPosition );
	}	
}



void LaplacianSmoother::ValidateWeights()
{
	if ( m_bWeightsValid )
		return;

	// restore vertex positions (?? only need to do this after SetWeightMode/SetVertexArea!! should set flag or something)
	if (! m_vVertices.empty() && m_vVertices.size() == m_nROISize ) {
		for ( unsigned int i = 0; i < m_nROISize; ++i ) {
			VtxInfo & vi = m_vVertices[i];
			IMesh::VertexID vID = m_vMap.GetOld(i);
			m_pMesh->SetVertex( vID, vi.vOrigPosition );
		}
	}
	m_vVertices.resize(0);

	m_vVertices.resize( m_nROISize );
	m_nEdges = 0;

	m_fAvgVtxArea = 0.0f;
	std::vector<IMesh::VertexID> vOneRing;
	for ( unsigned int i = 0; i < m_nROISize; ++i ) {
		VtxInfo & vi = m_vVertices[i];
		IMesh::VertexID vID = m_vMap.GetOld(i);

		vi.vOrigPosition = m_pMesh->GetVertex(vID);
		
		vOneRing.resize(0);
		MeshUtils::VertexOneRing(*m_pMesh, vID, vOneRing, false);

		// don't need to do this if we do not have ROI...
		size_t nNbrs = vOneRing.size();
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			if ( m_vROI.find(vOneRing[k]) != m_vROI.end() )
				vi.vNbrs.push_back(vOneRing[k]);
		}
		if ( vi.vNbrs.size() == 0 ) {
			bool bIsolated = m_pMesh->IsIsolated(vID);
			lgBreakToDebugger();
		}
		m_nEdges += (unsigned int)vi.vNbrs.size();

		// compute laplacian vector
		switch ( m_eWeightMode ) {
			case Weights_Uniform:
				MeshUtils::UniformWeights(*m_pMesh, vID, vi.vNbrs, vi.vNbrWeights);
				break;
			case Weights_Cotan:
			default:
				MeshUtils::CotangentWeights(*m_pMesh, vID, vi.vNbrs, vi.vNbrWeights);
				break;
		}
		vi.vMeshLaplacian = MeshUtils::MeshLaplacian(*m_pMesh, vID, vi.vNbrs, vi.vNbrWeights);
		vi.vCurLaplacian = vi.vMeshLaplacian;

		// compute vertex area
		switch ( m_eVertexAreaMode ) {
			case Area_One:
				vi.vVtxArea = 1.0f;
				break;
			case Area_Mixed:
			default:
				vi.vVtxArea = MeshUtils::VertexArea_Mixed(*m_pMesh, vID);
		}
		m_fAvgVtxArea += (vi.vVtxArea / (float)m_nROISize);

		// convert neighbour VertexIDs to indices
		for ( unsigned int k = 0; k < vi.vNbrs.size(); ++k )
			vi.vNbrs[k] = m_vMap.GetNew(vi.vNbrs[k]);
	}

	// rescale vertex areas, so that constraint weight scales are not affected
	for ( unsigned int i = 0; i < m_nROISize; ++i ) {
		VtxInfo & vi = m_vVertices[i];
		vi.vVtxArea /= m_fAvgVtxArea;
	}

	m_bWeightsValid = true;
	m_bMatricesValid = false;
	m_bSolutionValid = false;
}



void LaplacianSmoother::UpdateSytemMatrix_ThinPlate()
{
	ValidateWeights();

	if ( m_bMatricesValid )
		return;

	unsigned int nVerts = (unsigned int)m_vVertices.size();

	GetSystem()->Resize(nVerts, nVerts);
	GetSystem()->ResizeRHS(3);
	for ( int k = 0; k < 3; ++k )
		m_pRHS[k].Resize( nVerts );

	gsi::SparseMatrix & Msys = (*m_pSystem);
	Msys.Clear();
	Msys.Resize(nVerts,nVerts);
	
#if 0
	gsi::SparseMatrix & Minv = (*m_pM);
	Minv.Resize(nVerts, nVerts);
	gsi::SparseMatrix & Ls = (*m_pLs);
	Ls.Resize(nVerts, nVerts);

	gsi::SparseMatrix & Minv = (*m_pM);
	Minv.Resize(nVerts, nVerts);

	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		size_t nNbrs = vi.vNbrs.size();

		double dSum = 0.0f;
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			Ls(ri, vi.vNbrs[k]) = vi.vNbrWeights[k];
			dSum += vi.vNbrWeights[k];
		}
		Ls(ri, ri) = -dSum;
		Minv(ri,ri) = 1.0f / vi.vVtxArea;
	}

	// construct system
	Msys = Ls * Minv * Ls;
#endif


#if 1
	SparseMatrixType Ls(nVerts,nVerts);
	Ls.reserve(m_nEdges+nVerts);
	SparseMatrixType Minv(nVerts, nVerts);
	Minv.reserve(nVerts);
	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		size_t nNbrs = vi.vNbrs.size();

		double dSum = 0.0f;
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			Ls.insert(ri, vi.vNbrs[k]) = vi.vNbrWeights[k];
			dSum += vi.vNbrWeights[k];
		}
		Ls.insert(ri, ri) = -dSum;

		Minv.insert(ri,ri) = 1.0f / vi.vVtxArea;
	}
	Ls.finalize();
	Minv.finalize();
	SparseMatrixType LsMinvLs = Ls * Minv * Ls;

	for (unsigned int k=0; k < nVerts; ++k) {
		for ( SparseMatrixType::InnerIterator it(LsMinvLs,k); it; ++it)
			Msys.Set( it.row(), it.col(), it.value() );
	}
#endif



	m_bMatricesValid = true;
	m_bSolverValid = false;
	m_bSolutionValid = false;
}

void LaplacianSmoother::UpdateSolver_ThinPlate()
{
	if ( m_bSolverValid )
		return;

	GetSystem()->SetMatrix(*m_pSystem);

	// add soft constraints
	unsigned int nCons = (unsigned int)m_vConstraints.size();
	for ( unsigned int ci = 0; ci < nCons; ++ci ) {
		Constraint & c = m_vConstraints[ci];
		int ri = c.nIndex;
		float fConsWeight = c.fWeight;
		if ( c.eType == CType_SoftInterior )
			fConsWeight *= m_fInteriorConstraintWeightScale;
		GetSystem()->Set(  ri,ri,   GetSystem()->Get(ri,ri) + (fConsWeight*fConsWeight)  );
	}

	if ( ! GetSystem()->Matrix().IsSymmetric() )
		lgBreakToDebugger();

	GetSolver()->OnMatrixChanged();
	GetSolver()->SetStoreFactorization(true);
	GetSolver()->SetSolverMode( gsi::Solver_TAUCS::TAUCS_LLT );
	GetSolver()->SetOrderingMode( gsi::Solver_TAUCS::TAUCS_METIS );
	
	m_bSolverValid = true;
	m_bSolutionValid = false;
}




void LaplacianSmoother::UpdateRHS_ThinPlate()
{
	unsigned int nVerts = (unsigned int)m_vVertices.size();

	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		double x[3] = {0,0,0};
		float fWeightSum = 0.0f;
		size_t nNbrs = vi.vNbrs.size();
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			unsigned int nNbrIndex = vi.vNbrs[k];
			float fWeight = vi.vNbrWeights[k];
			fWeightSum += fWeight;
			const Wml::Vector3f & vNbrLaplacian = m_vVertices[nNbrIndex].vCurLaplacian;
			for ( int i = 0; i < 3; ++i ) {
				x[i] += fWeight*vNbrLaplacian[i] * m_fLaplacianVectorScale;
			}
		}
		const Wml::Vector3f & vLaplacian = vi.vCurLaplacian;
		for ( int i = 0; i < 3; ++i )
			m_pRHS[i][ri] = x[i] - fWeightSum*vLaplacian[i] * m_fLaplacianVectorScale;
	}

	unsigned int nCons = (unsigned int)m_vConstraints.size();
	for ( unsigned int ci = 0; ci < nCons; ++ci ) {
		Constraint & c = m_vConstraints[ci];
		int ri = c.nIndex;
		float fConsWeight = c.fWeight;
		if ( c.eType == CType_SoftInterior )
			fConsWeight *= m_fInteriorConstraintWeightScale;
		for ( int k = 0; k < 3; ++k ) 
			m_pRHS[k][ri] += c.vPosition[k]*fConsWeight*fConsWeight;
	};


	gsi::SparseLinearSystem * pSystem = GetSystem();
	pSystem->SetRHS(0, m_pRHS[0]);
	pSystem->SetRHS(1, m_pRHS[1]);
	pSystem->SetRHS(2, m_pRHS[2]);


	m_bSolutionValid = false;
}





void LaplacianSmoother::UpdateMatrices_Shell()
{
	ValidateWeights();

	if ( m_bMatricesValid )
		return;

	unsigned int nVerts = (unsigned int)m_vVertices.size();

	GetSystem()->Resize(nVerts, nVerts);
	GetSystem()->ResizeRHS(3);

	gsi::SparseMatrix & Ls = (*m_pLs);
	Ls.Resize(nVerts, nVerts);
	gsi::SparseMatrix & Minv = (*m_pM);
	Minv.Resize(nVerts, nVerts);
	
	float ks = 1;		// weight on membrane term
	float kb = 0;		// weight on thin-plate term

	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		size_t nNbrs = vi.vNbrs.size();

		double dSum = 0.0f;
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			Ls(ri, vi.vNbrs[k]) = vi.vNbrWeights[k];
			dSum += vi.vNbrWeights[k];
		}
		Ls(ri, ri) = -dSum;
		Minv(ri,ri) = 1.0f / vi.vVtxArea;
	}

	// fold in area weights matrix M here

	// construct system
	gsi::SparseMatrix Msys( Ls*(-ks)   +   Ls * Minv * Ls * kb);

	// add soft constraints
	unsigned int nCons = (unsigned int)m_vConstraints.size();
	for ( unsigned int ci = 0; ci < nCons; ++ci ) {
		Constraint & c = m_vConstraints[ci];
		int ri = c.nIndex;
		float fConsWeight = c.fWeight;
		if ( c.eType == CType_SoftInterior )
			fConsWeight *= m_fInteriorConstraintWeightScale;
		Msys(ri,ri) = Msys(ri,ri) + (fConsWeight*fConsWeight);
	}

	GetSystem()->SetMatrix(Msys);

	if ( ! GetSystem()->Matrix().IsSymmetric() )
		lgBreakToDebugger();

	GetSolver()->OnMatrixChanged();
	GetSolver()->SetStoreFactorization(true);
	GetSolver()->SetSolverMode( gsi::Solver_TAUCS::TAUCS_LLT );
	GetSolver()->SetOrderingMode( gsi::Solver_TAUCS::TAUCS_METIS );

	m_bMatricesValid = true;
	m_bSolverValid = true;
	m_bSolutionValid = false;
}



void LaplacianSmoother::UpdateRHS_Shell()
{
	unsigned int nVerts = (unsigned int)m_vVertices.size();

	gsi::SparseLinearSystem * pSystem = GetSystem();

	// RHS is 0 in shell energy
	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		Wml::Vector3f vLaplacian = Wml::Vector3f::ZERO;
		for ( int k = 0; k < 3; ++k )
			pSystem->SetRHS(ri, vLaplacian[k], k);
	}

	unsigned int nCons = (unsigned int)m_vConstraints.size();
	for ( unsigned int ci = 0; ci < nCons; ++ci ) {
		Constraint & c = m_vConstraints[ci];
		int ri = c.nIndex;
		float fConsWeight = c.fWeight;
		if ( c.eType == CType_SoftInterior )
			fConsWeight *= m_fInteriorConstraintWeightScale;
		Wml::Vector3f vConsVal = fConsWeight*fConsWeight*c.vPosition;
		for ( int k = 0; k < 3; ++k ) 
			pSystem->SetRHS( ri, pSystem->GetRHS(ri,k) + vConsVal[k], k );
	};

	m_bSolutionValid = false;
}





bool LaplacianSmoother::Solve()
{
	if ( ! m_bSolutionValid ) {

		UpdateSytemMatrix_ThinPlate();
		UpdateSolver_ThinPlate();
		UpdateRHS_ThinPlate();

		//UpdateMatrices_Shell();
		//UpdateRHS_Shell();

		bool bOK = GetSolver()->Solve();
		if ( ! bOK )
			return false;

		m_bSolutionValid = true;
	}

	int nMatrixCols = (int)m_vVertices.size();
	for ( int i = 0; i < nMatrixCols; ++i ) {
		Wml::Vector3f v((float)GetSystem()->GetSolution(i,0), (float)GetSystem()->GetSolution(i,1), (float)GetSystem()->GetSolution(i,2) );
		IMesh::VertexID vID = m_vMap.GetOld(i);
		m_pMesh->SetVertex(vID, v);
	}
	return true;
}




void LaplacianSmoother::Render()
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
		glVertex3fv( v + m_vVertices[i].vCurLaplacian );
	}
	glEnd();


	glPopAttrib();

}