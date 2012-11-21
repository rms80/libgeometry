// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "RotInvCoordDeformer.h"
#include "MeshUtils.h"
#include <Wm4LinearSystem.h>
#include <SparseLinearSystem.h>
#include <Solver_TAUCS.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
typedef Eigen::SparseMatrix<double,Eigen::RowMajor> SparseMatrixType;


#include <rmsdebug.h>
#include <rmsprofile.h>


using namespace rms;


RotInvCoordDeformer::RotInvCoordDeformer()
{
	m_pMesh = NULL;
	m_pSolverPos = NULL;
	m_pSystemPos = NULL;
	m_pSolverRot = NULL;
	m_pSystemRot = NULL;
	m_pLs = new gsi::SparseMatrix();
	m_pM = new gsi::SparseMatrix();
	m_pRHSPos = new gsi::Vector[3];
}

RotInvCoordDeformer::~RotInvCoordDeformer()
{
	if ( m_pSolverPos ) 
		delete m_pSolverPos;
	if ( m_pSystemPos )
		delete m_pSystemPos;
	if ( m_pSolverRot )
		delete m_pSolverRot;
	if ( m_pSystemRot )
		delete m_pSystemRot;
	if ( m_pLs )
		delete m_pLs;
	if ( m_pM )
		delete m_pM;
	if ( m_pRHSPos )
		delete [] m_pRHSPos;
}


gsi::Solver_TAUCS * RotInvCoordDeformer::GetSolverPos()
{
	if ( m_pSolverPos == NULL )
		m_pSolverPos = new gsi::Solver_TAUCS(GetSystemPos());
	return m_pSolverPos;
}
gsi::SparseLinearSystem * RotInvCoordDeformer::GetSystemPos()
{
	if ( m_pSystemPos == NULL )
		m_pSystemPos = new gsi::SparseLinearSystem();
	return m_pSystemPos;
}


gsi::Solver_TAUCS * RotInvCoordDeformer::GetSolverRot()
{
	if ( m_pSolverRot == NULL )
		m_pSolverRot = new gsi::Solver_TAUCS(GetSystemRot());
	return m_pSolverRot;
}
gsi::SparseLinearSystem * RotInvCoordDeformer::GetSystemRot()
{
	if ( m_pSystemRot == NULL )
		m_pSystemRot = new gsi::SparseLinearSystem();
	return m_pSystemRot;
}


void RotInvCoordDeformer::SetMesh(rms::VFTriangleMesh * pMesh)
{
	m_pMesh = pMesh;
	m_vPosConstraints.resize(0);
	m_vRotConstraints.resize(0);
	ComputeWeights();
	m_bMatricesValid = false;
}


void RotInvCoordDeformer::AddBoundaryConstraints(float fWeight)
{
	VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		if ( m_pMesh->IsBoundaryVertex(vID) ) {
			Wml::Vector3f v;
			m_pMesh->GetVertex(vID, v);
			UpdatePositionConstraint(vID, v, fWeight);
			UpdateOrientationConstraint(vID, m_vVertices[vID].vFrame, fWeight);
		}
	}
}


void RotInvCoordDeformer::UpdatePositionConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition, float fWeight )
{
	bool bFound = false;
	size_t nCount = m_vPosConstraints.size();
	for ( unsigned int k = 0; !bFound && k < nCount; ++k ) {
		if ( m_vPosConstraints[k].vID == vID ) {
			m_vPosConstraints[k].vPosition = vPosition;
			if ( m_vPosConstraints[k].fWeight != fWeight )
				m_bMatricesValid = false;
			m_vPosConstraints[k].fWeight = fWeight;
			bFound = true;
		}
	}
	if ( ! bFound ) {
		PosConstraint c;
		c.vID = vID;
		c.vPosition = vPosition;
		c.fWeight = fWeight;
		m_vPosConstraints.push_back(c);
		m_bMatricesValid = false;
	}
}

void RotInvCoordDeformer::UpdateOrientationConstraint( IMesh::VertexID vID, const rms::Frame3f & vFrame, float fWeight )
{
	bool bFound = false;
	size_t nCount = m_vRotConstraints.size();
	for ( unsigned int k = 0; !bFound && k < nCount; ++k ) {
		if ( m_vRotConstraints[k].vID == vID ) {
			m_vRotConstraints[k].vFrame = vFrame;
			if ( m_vRotConstraints[k].fWeight != fWeight )
				m_bMatricesValid = false;
			m_vRotConstraints[k].fWeight = fWeight;
			bFound = true;
		}
	}
	if ( ! bFound ) {
		RotConstraint c;
		c.vID = vID;
		c.vFrame = vFrame;
		c.fWeight = fWeight;
		m_vRotConstraints.push_back(c);
		m_bMatricesValid = false;
	}
}



rms::Frame3f RotInvCoordDeformer::GetCurrentFrame( IMesh::VertexID vID )
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
rms::Frame3f RotInvCoordDeformer::GetSolvedFrame( IMesh::VertexID vID )
{
	return m_vVertices[vID].vTransFrame;
}

void RotInvCoordDeformer::SetVertexWeight( IMesh::VertexID vID, float fWeight )
{
	VtxInfo & vi = m_vVertices[vID];
	vi.fVertexWeight = fWeight;
	m_bMatricesValid = false;
}
void RotInvCoordDeformer::SetVertexScale( IMesh::VertexID vID, float fScale )
{
	VtxInfo & vi = m_vVertices[vID];
	vi.fVertexScale = fScale;
}



void RotInvCoordDeformer::ComputeWeights()
{
	unsigned int nVerts = m_pMesh->GetMaxVertexID();
	if ( nVerts != m_pMesh->GetVertexCount() )
		lgBreakToDebugger();		// can happen...

	m_vVertices.resize(0);
	m_vVertices.resize( nVerts );
	m_nEdges = 0;

	m_fAvgVtxArea = 0.0f;
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		VtxInfo & vi = m_vVertices[i];
		vi.vID = i;
		
		MeshUtils::VertexOneRing(*m_pMesh, vi.vID, vi.vNbrs, false);
		size_t nNbrs = vi.vNbrs.size();
		m_nEdges += (unsigned int)nNbrs;

		MeshUtils::CotangentWeights(*m_pMesh, vi.vID, vi.vNbrs, vi.vNbrWeights);
//		MeshUtils::UniformWeights(*m_pMesh, vi.vID, vi.vNbrs, vi.vNbrWeights);

		Wml::Vector3f vVtx, vNormal, vNbr;
		m_pMesh->GetVertex(vi.vID, vVtx);

		vNormal = MeshUtils::EstimateNormal(*m_pMesh, vi.vID);

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

		// compute frame
		m_pMesh->GetVertex(vi.nTangentNbr, vNbr);
		Wml::Vector3f xik = vNbr - vVtx;
		Wml::Vector3f xikbar = vNormal.Cross( xik.Cross(vNormal) );
		xikbar.Normalize();
		xik = vNormal.Cross( xikbar );
		vi.vFrame.SetFrame( xikbar, xik, vNormal);

		// compute laplacian vector
		vi.vLaplacian = MeshUtils::MeshLaplacian(*m_pMesh, vi.vID, vi.vNbrs, vi.vNbrWeights);
		vi.vFrameLaplacian = vi.vLaplacian;
		vi.vFrame.ToFrameLocal( vi.vFrameLaplacian );

		// compute vertex area
		vi.fVertexArea = 1.0f;
		//vi.fVertexArea = MeshUtils::VertexArea_Mixed(*m_pMesh, vi.vID);
		m_fAvgVtxArea += (vi.fVertexArea / (float)nVerts);

		// additional weighting term
		vi.fVertexWeight = 1.0f;

		// per-vertex scaling factor
		vi.fVertexScale = 1.0f;
	}

	// rescale vertex areas, so that constraint weight scales are not affected
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		VtxInfo & vi = m_vVertices[i];
		vi.fVertexArea /= m_fAvgVtxArea;
	}
}







void RotInvCoordDeformer::UpdateMatrices()
{
	if ( m_bMatricesValid )
		return;

	unsigned int nVerts = (unsigned int)m_vVertices.size();


	/*
	 * build system matrix for orientations
	 */
#if 0
	gsi::SparseMatrix MRot;
	MRot.Resize(3*m_nEdges, 3*nVerts);
	gsi::SparseMatrix RotMInv;
	RotMInv.Resize( 3*m_nEdges, 3*m_nEdges );

	// [TODO] this loop uses vertex ID indexing...need to rewrite w/ vertex<->row map
	unsigned int rij = 0;
	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		IMesh::VertexID i = vi.vID;
		unsigned int ci = 3*i;
		float fVtxWeight = 1.0f / ( vi.fVertexArea ) / vi.fVertexWeight;

		size_t nNbrs = vi.vNbrs.size();
		for ( unsigned int ni = 0; ni < nNbrs; ++ni ) {
			IMesh::VertexID j = vi.vNbrs[ni];
			unsigned int cj = 3*j;
			float fWeight = vi.vNbrWeights[ni];

			Wml::Matrix3f Fi = m_vVertices[i].vFrame.FrameMatrix();
			Wml::Matrix3f Fj = m_vVertices[j].vFrame.FrameMatrix();
			Wml::Matrix3f Rij = Fj.TransposeTimes(Fi);

			for ( int k = 0; k < 3; ++k ) {
				// Mrot(rij+k,ci:ci+2) = Rij(k,:);
				// Mrot(rij+k,cj+k) = -1;
				MRot.Set( rij+k, ci+0, Rij(k, 0) * fWeight );
				MRot.Set( rij+k, ci+1, Rij(k, 1) * fWeight );
				MRot.Set( rij+k, ci+2, Rij(k, 2) * fWeight );
				MRot.Set( rij+k, cj+k, -1 * fWeight);

				RotMInv.Set( rij+k, rij+k, fVtxWeight );
			}
			rij += 3;
		}
	}

	// construct weighted normal equations	
	gsi::SparseMatrix MRotTrans;
	MRot.Transpose(MRotTrans);

	//gsi::SparseMatrix MTransTimesMInv;
	//MRotTrans.Multiply(RotMInv, MTransTimesMInv);
	//gsi::SparseMatrix MSysRot( MTransTimesMInv * MRot );
	gsi::SparseMatrix MSysRot;
	MRotTrans.Multiply( MRot, MSysRot );
#endif


#if 1
	SparseMatrixType Rs(3*m_nEdges, 3*nVerts);
	Rs.reserve( (3*m_nEdges)*4 );

	unsigned int rij = 0;
	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		IMesh::VertexID i = vi.vID;
		unsigned int ci = 3*i;
		float fVtxWeight = 1.0f / ( vi.fVertexArea ) / vi.fVertexWeight;

		size_t nNbrs = vi.vNbrs.size();
		for ( unsigned int ni = 0; ni < nNbrs; ++ni ) {
			IMesh::VertexID j = vi.vNbrs[ni];
			unsigned int cj = 3*j;
			float fWeight = vi.vNbrWeights[ni];

			Wml::Matrix3f Fi = m_vVertices[i].vFrame.FrameMatrix();
			Wml::Matrix3f Fj = m_vVertices[j].vFrame.FrameMatrix();
			Wml::Matrix3f Rij = Fj.TransposeTimes(Fi);

			for ( int k = 0; k < 3; ++k ) {
				// Mrot(rij+k,ci:ci+2) = Rij(k,:);
				// Mrot(rij+k,cj+k) = -1;
				Rs.insert( rij+k, ci+0 ) = fWeight * Rij(k, 0);
				Rs.insert( rij+k, ci+1 ) = fWeight * Rij(k, 1);
				Rs.insert( rij+k, ci+2 ) = fWeight * Rij(k, 2);
				
				Rs.insert( rij+k, cj+k ) = fWeight * -1.0f;
			}
			rij += 3;
		}
	}
	Rs.finalize();
	SparseMatrixType RsT_Rs = SparseMatrixType(Rs.transpose()) * Rs;


	gsi::SparseMatrix MSysRot;
	MSysRot.Resize(3*nVerts,3*nVerts);
	for (unsigned int k=0; k < 3*nVerts; ++k) {
		for ( SparseMatrixType::InnerIterator it(RsT_Rs,k); it; ++it)
			MSysRot.Set( it.row(), it.col(), it.value() );
	}
#endif

	
	// add soft constraints
	// [TODO] this loop uses vertex ID indexing...need to rewrite w/ vertex<->row map
	size_t nRotCons = m_vRotConstraints.size();
	for ( unsigned int ci = 0; ci < nRotCons; ++ci ) {
		RotConstraint & c = m_vRotConstraints[ci];
		int ri = c.vID * 3;
		for ( int k = 0; k < 3; ++k )
			MSysRot(ri+k,ri+k) = MSysRot(ri+k,ri+k) + c.fWeight*c.fWeight;
	}	

	GetSystemRot()->SetMatrix( MSysRot );
	GetSystemRot()->ResizeRHS(3);

	if ( ! GetSystemRot()->Matrix().IsSymmetric() )
		lgBreakToDebugger();

	GetSolverRot()->OnMatrixChanged();
	GetSolverRot()->SetStoreFactorization(true);
	GetSolverRot()->SetSolverMode( gsi::Solver_TAUCS::TAUCS_LLT );
	GetSolverRot()->SetOrderingMode( gsi::Solver_TAUCS::TAUCS_METIS );




	/*
	 * build Laplacian system to find positions
	 */
	GetSystemPos()->Resize(nVerts, nVerts);
	GetSystemPos()->ResizeRHS(3);
	for ( int k = 0; k < 3; ++k )
		m_pRHSPos[k].Resize( nVerts );

#if 0
	_RMSTUNE_start(2);

	gsi::SparseMatrix & Ls = (*m_pLs);
	Ls.Resize(nVerts, nVerts);
	//gsi::SparseMatrix & Minv = (*m_pM);
	//Minv.Resize(nVerts, nVerts);
	
	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		size_t nNbrs = vi.vNbrs.size();

		double dSum = 0.0f;
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			Ls(ri, vi.vNbrs[k]) = vi.vNbrWeights[k];
			dSum += vi.vNbrWeights[k];
		}
		Ls(ri, ri) = -dSum;
		//Minv(ri,ri) = ( 1.0f / ( vi.fVertexArea ) ) / vi.fVertexWeight;
	}

	// construct system
	//gsi::SparseMatrix Msys( Ls * Minv * Ls );
	gsi::SparseMatrix Msys( Ls * Ls );

	_RMSTUNE_end(2);
	_RMSInfo("gsi Ls*Ls time was %f\n", _RMSTUNE_time(2));
#endif


#if 1
	_RMSTUNE_start(2);


	SparseMatrixType Ls(nVerts,nVerts);
	Ls.reserve(m_nEdges+nVerts);
	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		size_t nNbrs = vi.vNbrs.size();

		double dSum = 0.0f;
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			Ls.insert(ri, vi.vNbrs[k]) = vi.vNbrWeights[k];
			dSum += vi.vNbrWeights[k];
		}
		Ls.insert(ri, ri) = -dSum;
		//Minv(ri,ri) = ( 1.0f / ( vi.fVertexArea ) ) / vi.fVertexWeight;
	}
	Ls.finalize();
	SparseMatrixType LsLs = Ls * Ls;

	gsi::SparseMatrix Msys;
	Msys.Resize(nVerts,nVerts);
	for (unsigned int k=0; k < nVerts; ++k) {
		for ( SparseMatrixType::InnerIterator it(LsLs,k); it; ++it)
			Msys.Set( it.row(), it.col(), it.value() );
	}

	_RMSTUNE_end(2);
	_RMSInfo("Eigen Ls*Ls time was %f\n", _RMSTUNE_time(2));
#endif

	// add soft constraints
	unsigned int nCons = (unsigned int)m_vPosConstraints.size();
	for ( unsigned int ci = 0; ci < nCons; ++ci ) {
		PosConstraint & c = m_vPosConstraints[ci];
		int ri = c.vID;
		Msys(ri,ri) = Msys(ri,ri) + c.fWeight*c.fWeight;
	}

	GetSystemPos()->SetMatrix(Msys);

	if ( ! GetSystemPos()->Matrix().IsSymmetric() )
		lgBreakToDebugger();

	GetSolverPos()->OnMatrixChanged();
	GetSolverPos()->SetStoreFactorization(true);
	GetSolverPos()->SetSolverMode( gsi::Solver_TAUCS::TAUCS_LLT );
	GetSolverPos()->SetOrderingMode( gsi::Solver_TAUCS::TAUCS_METIS );

	m_bMatricesValid = true;
}



void RotInvCoordDeformer::UpdateRHSRot()
{
	gsi::SparseLinearSystem * pSystem = GetSystemRot();

	// update soft constraints
	// [TODO] this loop uses vertex ID indexing...need to rewrite w/ vertex<->row map
	size_t nRotCons = m_vRotConstraints.size();
	for ( unsigned int ci = 0; ci < nRotCons; ++ci ) {
		RotConstraint & c = m_vRotConstraints[ci];
		int ri = c.vID * 3;

		Wml::Vector3f vConsValA = c.fWeight*c.fWeight*c.vFrame.X();
		for ( int k = 0; k < 3; ++k ) 
			pSystem->SetRHS( ri, vConsValA[k], k );

		Wml::Vector3f vConsValB = c.fWeight*c.fWeight*c.vFrame.Y();
		for ( int k = 0; k < 3; ++k ) 
			pSystem->SetRHS( ri+1, vConsValB[k], k );

		Wml::Vector3f vConsValN = c.fWeight*c.fWeight*c.vFrame.Z();
		for ( int k = 0; k < 3; ++k ) 
			pSystem->SetRHS( ri+2, vConsValN[k], k );
	}	
}



void RotInvCoordDeformer::UpdateRHSPos()
{
	unsigned int nVerts = (unsigned int)m_vVertices.size();

	// transform laplacians to new frames
	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];

		// transform frame-encoded laplacian vector into new 3D frame
		vi.vLaplacian = vi.vFrameLaplacian;
		vi.vTransFrame.ToWorld( vi.vLaplacian );
	}

	float fGlobalScale = GlobalScale();
	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];
		double x[3] = {0,0,0};
		float fWeightSum = 0.0f;
		size_t nNbrs = vi.vNbrs.size();
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			VtxInfo & ni = m_vVertices[ vi.vNbrs[k] ];
			float fWeight = vi.vNbrWeights[k];
			fWeightSum += fWeight;
			const Wml::Vector3f & vNbrLaplacian = ni.vLaplacian;
			for ( int i = 0; i < 3; ++i ) {
				x[i] += fWeight*vNbrLaplacian[i]*fGlobalScale*ni.fVertexScale;
			}
		}
		const Wml::Vector3f & vLaplacian = vi.vLaplacian;
		for ( int i = 0; i < 3; ++i )
			m_pRHSPos[i][ri] = x[i] - fWeightSum*vLaplacian[i]*fGlobalScale*vi.fVertexScale ;
	}

	// add soft constraints
	unsigned int nCons = (unsigned int)m_vPosConstraints.size();
	for ( unsigned int ci = 0; ci < nCons; ++ci ) {
		PosConstraint & c = m_vPosConstraints[ci];
		int ri = c.vID;
		float fConsWeight = c.fWeight;
		for ( int k = 0; k < 3; ++k ) 
			m_pRHSPos[k][ri] += c.vPosition[k]*fConsWeight*fConsWeight;
	};


	gsi::SparseLinearSystem * pSystem = GetSystemPos();
	pSystem->SetRHS(0, m_pRHSPos[0]);
	pSystem->SetRHS(1, m_pRHSPos[1]);
	pSystem->SetRHS(2, m_pRHSPos[2]);


#if 0
	gsi::SparseLinearSystem * pSystem = GetSystemPos();

	for ( unsigned int ri = 0; ri < nVerts; ++ri ) {
		VtxInfo & vi = m_vVertices[ri];

		// transform frame-encoded laplacian vector into new 3D frame
		vi.vLaplacian = vi.vFrameLaplacian;
		vi.vTransFrame.ToWorld( vi.vLaplacian );
		Wml::Vector3f vLaplacian = vi.vLaplacian;
		vLaplacian *= GlobalScale() * vi.fVertexScale;
		for ( int k = 0; k < 3; ++k )
			pSystem->SetRHS(ri, vLaplacian[k], k);
	}

	for ( int k = 0; k < 3; ++k ) {
		const gsi::Vector & vRHS = pSystem->GetRHS(k);
		gsi::Vector vLsRHS( (*m_pLs) * vRHS );
		pSystem->SetRHS( k, vLsRHS );
	}

	unsigned int nCons = (unsigned int)m_vPosConstraints.size();
	for ( unsigned int ci = 0; ci < nCons; ++ci ) {
		PosConstraint & c = m_vPosConstraints[ci];
		int ri = c.vID;
		Wml::Vector3f vConsVal = c.fWeight*c.fWeight*c.vPosition;
		for ( int k = 0; k < 3; ++k ) 
			pSystem->SetRHS( ri, pSystem->GetRHS(ri,k) + vConsVal[k], k );
	};
#endif


}




void RotInvCoordDeformer::Solve()
{
	UpdateMatrices();

	UpdateRHSRot();
	bool bOKRot = GetSolverRot()->Solve();
	if ( ! bOKRot )
		lgBreakToDebugger();

	// extract solved frames 
	gsi::SparseLinearSystem * pSystemRot = GetSystemRot();
	int nSize = (int)m_vVertices.size();
	for ( int i = 0; i < nSize; ++i ) {
		VtxInfo & vi = m_vVertices[i];
		int ri = 3*i;
		Wml::Vector3f vFrameV[3];
		for ( int k = 0; k < 3; ++k ) {
			vFrameV[k] = Wml::Vector3f((float)pSystemRot->GetSolution(ri+k,0), (float)pSystemRot->GetSolution(ri+k,1), (float)pSystemRot->GetSolution(ri+k,2) );
			vFrameV[k].Normalize();
		}
		Wml::Vector3f vA = vFrameV[1].Cross(vFrameV[2]);
		vA.Normalize();
		Wml::Vector3f vB = vFrameV[2].Cross( vA );
		vB.Normalize();
		vi.vTransFrame.SetFrame( vA, vB, vFrameV[2] );
	}


	UpdateRHSPos();
	bool bOK = GetSolverPos()->Solve();
	if ( ! bOK )
		lgBreakToDebugger();

	gsi::SparseLinearSystem * pSystem = GetSystemPos();
	int nMatrixCols = (int)m_vVertices.size();
	for ( int i = 0; i < nMatrixCols; ++i ) {
		Wml::Vector3f v((float)pSystem->GetSolution(i,0), (float)pSystem->GetSolution(i,1), (float)pSystem->GetSolution(i,2) );
		m_pMesh->SetVertex(i, v);
	}

}



float RotInvCoordDeformer::GetLaplacianError()
{
	float fErr = 0.0f;
	unsigned int nVerts = m_pMesh->GetMaxVertexID();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		VtxInfo & vi = m_vVertices[i];
		Wml::Vector3f vCurLaplacian = MeshUtils::MeshLaplacian(*m_pMesh, vi.vID, vi.vNbrs, vi.vNbrWeights);
		Wml::Vector3f vWantLaplacian = vi.vLaplacian * GlobalScale() * vi.fVertexScale;
		//fErr += (vCurLaplacian-vWantLaplacian).SquaredLength();
		vCurLaplacian.Normalize();
		vWantLaplacian.Normalize();
		float fDot = vCurLaplacian.Dot(vWantLaplacian);
		fErr += (1.0f - fDot);
	}
	return fErr;
}


void RotInvCoordDeformer::DebugRender()
{
	glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	
//	glDepthFunc(GL_LEQUAL);

	Wml::Vector3f v;

	// render constraint points
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	for ( unsigned int i = 0; i < m_vPosConstraints.size(); ++i ) {
		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3fv( m_pMesh->GetVertex(m_vPosConstraints[i].vID) );
		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3fv(m_vPosConstraints[i].vPosition);
	}
	glEnd();
	glBegin(GL_LINES);
	glColor3f(0.0f, 0.0f, 0.0f);
	for ( unsigned int i = 0; i < m_vPosConstraints.size(); ++i ) {
		glVertex3fv( m_pMesh->GetVertex(m_vPosConstraints[i].vID) );
		glVertex3fv(m_vPosConstraints[i].vPosition);
	}
	glEnd();
		

	// draw current laplacian vectors
	//glLineWidth(2.0f);
	//glBegin(GL_LINES);
	//glColor3f(1.0f, 0.0f, 0.0f);
	//for ( unsigned int i = 0; i < m_vVertices.size(); ++i ) {
	//	m_pMesh->GetVertex( i, v );
	//	glVertex3fv( v );
	//	glVertex3fv( v + m_vVertices[i].vLaplacian );
	//}
	//glEnd();

	float fLengthScale = 0.005f;
	float fLengthXY = 3 * fLengthScale;
	float fLengthZ = 5 * fLengthScale;

	//// draw initial frames
	//glBegin(GL_LINES);
	//for ( unsigned int i = 0; i < m_vVertices.size(); i += 5 ) {
	//	m_pMesh->GetVertex( i, v );
	//	glColor3f(1,0,0);
	//	glVertex3fv( v );
	//	glVertex3fv( v + fLengthXY*m_vVertices[i].vFrame.X() );
	//	glColor3f(0,1,0);
	//	glVertex3fv( v );
	//	glVertex3fv( v + fLengthXY*m_vVertices[i].vFrame.Y() );
	//	glColor3f(0,0,1);
	//	glVertex3fv( v );
	//	glVertex3fv( v + fLengthZ*m_vVertices[i].vFrame.Z() );
	//}
	//glEnd();


	// draw solved frames
	glLineWidth(4.0f);
	glBegin(GL_LINES);
	for ( unsigned int i = 0; i < m_vVertices.size(); i += 1 ) {
		m_pMesh->GetVertex( i, v );
		glColor3f(1,0,0);
		glVertex3fv( v );
		glVertex3fv( v + fLengthXY*m_vVertices[i].vTransFrame.X() );
		glColor3f(0,1,0);
		glVertex3fv( v );
		glVertex3fv( v + fLengthXY*m_vVertices[i].vTransFrame.Y() );
		glColor3f(0,0,1);
		glVertex3fv( v );
		glVertex3fv( v + fLengthZ*m_vVertices[i].vTransFrame.Z() );
	}
	glEnd();

	glPopAttrib();

}