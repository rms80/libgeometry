// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "PlanarParameterization.h"

// [RMS] was this removed from Wml? Or renamed?
//#include "Wm4IntrLinComp2LinComp2.h"

#include <MeshUtils.h>
#include <SparseLinearSystem.h>
#include <Solver_TAUCS.h>
#include <Solver_UMFPACK.h>
#include <Wm4LinearSystem.h>

#include "rmsdebug.h"


using namespace rms;

PlanarParameterization::PlanarParameterization(void)
{
	m_bComputeNeighbourExpMaps = true;

	m_nGeoNbrhoodSize = 8;
	m_nCurMaxGeoNbrs = 30;
	m_bUseFixedGeoNbrhoodSize = true;
	InvalidateGeodesicNbrhoods();

	m_eBoundaryMap = PlanarParameterization::SquareBoundary;
	m_eEmbedType = PlanarParameterization::Optimal2DExpMap;

	m_fMixedDCDCConformalWeight = 0.5f;

	m_bScaleUVs = true;
	m_fUVScaleFactor = 1.9f;
}

PlanarParameterization::~PlanarParameterization(void)
{
}

void PlanarParameterization::SetMesh( rms::VFTriangleMesh * pMesh, rms::ExpMapGenerator * pExpMap )
{
	m_pMesh = pMesh;
	m_pExpMap = pExpMap;
	InvalidateGeodesicNbrhoods();

	float fMin, fMax, fAvg;
	m_pMesh->GetEdgeLengthStats(fMin, fMax, fAvg);
	m_fGeoNbrDistance = fMax * 2.0f;
	m_fCurMinMaxGeoNbrDistance = 0.0f;

	PrecomputeMeshData();
}
void PlanarParameterization::Invalidate()
{
	SetMesh(m_pMesh, m_pExpMap);
}


void PlanarParameterization::SetEmbeddingType( EmbeddingType eType )
{
	m_eEmbedType = eType;
}

void PlanarParameterization::SetMixedDCDAConformalWeight(float fVal) 
{ 
	m_fMixedDCDCConformalWeight = rms::Clamp(fVal, 0.0f, 1.0f); 
}


bool PlanarParameterization::Compute()
{
	bool bResult = false;
	switch ( m_eEmbedType ) {
		case UniformWeights:
		case InverseDistance:
		case FloaterShapePreserving:
		case GeodesicOneRing:
		case Optimal3DOneRing:
		case Optimal2DOneRing:
			bResult = Parameterize_OneRing();
			break;

		case DiscreteConformal:
		case DiscreteAuthalic:
		case MixedConformalAuthalic:
		case DiscreteNaturalConformal:
			bResult = Parameterize_OneRing_Intrinsic();
			break;

		case Optimal2DExpMap:
		case Optimal3DExpMap:
			bResult = Parameterize_ExpMap();
			break;

		case ExpMapLLE:
		case StandardLLE:
			bResult = Parameterize_LLE();
			break;
	}


	// do analysis
	if ( bResult ) {
		ComputeMeshOneRingStretch();
	}

	return bResult;
}


bool PlanarParameterization::Parameterize_OneRing()
{
	size_t nCount = m_vVertInfo.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		switch ( m_eEmbedType ) {
			case UniformWeights:
				ComputeWeights_Uniform( m_vVertInfo[i].GetNeighbourSet(OneRing) );
				break;
			case InverseDistance:
				ComputeWeights_InvDist( m_vVertInfo[i].GetNeighbourSet(OneRing) );
				break;
			case FloaterShapePreserving:
				ComputeWeights_ShapePreserving( m_vVertInfo[i].GetNeighbourSet(OneRing) );
				break;
			case GeodesicOneRing:
				ComputeWeights_Geodesic( m_vVertInfo[i].GetNeighbourSet(OneRing) );
				break;
			case Optimal3DOneRing:
				ComputeWeights_Optimal3D( m_vVertInfo[i].GetNeighbourSet(OneRing) );
				break;
			case Optimal2DOneRing:
				ComputeWeights_Optimal2D( m_vVertInfo[i].GetNeighbourSet(OneRing) );
				break;
		}
	}

	if ( ! EmbedBoundary() )
		return false;

	std::vector<double> vU, vV;
	bool bResult = Solve_FixBoundary(vU, vV, OneRing);
	if ( ! bResult ) {
		_RMSInfo("Solve_FixBoundary failed in PlanarParameterization::Parameterize_OneRing() !\n");
		return false;
	}
	SetMeshUVs(&vU[0], &vV[0]);

	return true;
}




void PlanarParameterization::MakeBoundaryConstraints( std::vector<Constraint> & vConstraints )
{
	BoundaryLoop & loop = m_boundaryInfo.vBoundaryLoops[0];

#if 1
	// now pin two boundary verts that are furthest apart
	int vPin1 = loop.vVerts[0], vPin2 = -1;
	float fPin2 = 0.0f;
	Wml::Vector3f v1, v2; 
	m_pMesh->GetVertex( loop.vVerts[0], v1 );
	size_t nLoopCount = loop.vVerts.size();
	for ( unsigned int j = 1; j < nLoopCount; ++j ) {
		m_pMesh->GetVertex( loop.vVerts[j], v2 );
		float fDistSqr = (v1 - v2).SquaredLength();
		if ( fDistSqr > fPin2 ) {
			vPin2 = loop.vVerts[j];
			fPin2 = fDistSqr;
		}
	}
	fPin2 = (float)sqrt(fPin2);

	//vConstraints.push_back( Constraint(vPin1, Wml::Vector2f(-fPin2/2,0)) );
	//vConstraints.push_back( Constraint(vPin2, Wml::Vector2f( fPin2/2,0)) );

	vConstraints.push_back( Constraint(vPin1, Wml::Vector2f(0,0)) );
	vConstraints.push_back( Constraint(vPin2, Wml::Vector2f(1,0)) );

#else
	int vPin1 = loop.vVerts[0];
	int vPin2 = loop.vVerts[1];

	Wml::Vector3f v1, v2;
	m_pMesh->GetVertex(m_vVertInfo[ vPin1 ].vID, v1);
	m_pMesh->GetVertex(m_vVertInfo[ vPin2 ].vID, v2);
	float fPin2 = (v1-v2).Length();

	vConstraints.push_back( Constraint(vPin1, Wml::Vector2f(-fPin2, 0)) );
	vConstraints.push_back( Constraint(vPin2, Wml::Vector2f(fPin2, 0) ) );
#endif

	_RMSInfo("pinning %d(%d) and %d(%d) (dist is %f)\n", vPin1, m_vVertInfo[ vPin1 ].vID, vPin2, m_vVertInfo[ vPin2 ].vID, fPin2);
	if ( ! m_pMesh->IsBoundaryVertex(m_vVertInfo[ vPin1 ].vID) ) 
		lgBreakToDebugger();
	if ( ! m_pMesh->IsBoundaryVertex(m_vVertInfo[ vPin2 ].vID) ) 
		lgBreakToDebugger();
}



bool PlanarParameterization::Parameterize_OneRing_Intrinsic()
{
	double dConformalWeight = 1.0f;
	switch ( m_eEmbedType ) {
		default:
		case DiscreteConformal:
		case DiscreteNaturalConformal:
			dConformalWeight = 1.0f;
			break;
		case DiscreteAuthalic:
			dConformalWeight = 0.0f;
			break;
		case MixedConformalAuthalic:
			dConformalWeight = m_fMixedDCDCConformalWeight;
			break;
	}
	double dAuthalicWeight = 1.0f - dConformalWeight;

	// generate constraints if we are using DNCP
	bool bUseNaturalBoundary = (m_eEmbedType == DiscreteNaturalConformal);
	std::vector<Constraint> vConstraints;
	if ( bUseNaturalBoundary )
		MakeBoundaryConstraints(vConstraints);
	size_t nConstraints = vConstraints.size();
	std::set<IMesh::VertexID> vConstrained;
	for ( unsigned int k = 0; k < nConstraints; ++k )
		vConstrained.insert( vConstraints[k].nVertex );

	// [RMS] warning: I may have broken some stuff here w/ explicit constraints...
	gsi::SparseLinearSystem p;
	size_t nCount = m_vVertInfo.size();
	p.Resize(2*(unsigned int)nCount, 2*(unsigned int)nCount);
	p.ResizeRHS(1);
	int N = (int)nCount;

	// fill matrix
	Wml::Vector3f vi, vj, vo;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		NeighbourSet & nbrs = m_vVertInfo[i].GetNeighbourSet(OneRing);
		m_pMesh->GetVertex(m_vVertInfo[i].vID, vi);

		double dRowSum = 0;
		size_t nNbrs = nbrs.vNbrs.size();
		for ( unsigned int j = 0; j < nNbrs; ++j ) {
			m_pMesh->GetVertex(nbrs.vNbrs[j], vj);

			IMesh::EdgeID eID = m_pMesh->FindEdge(m_vVertInfo[i].vID, nbrs.vNbrs[j]);
			IMesh::VertexID edgeV[2];
			m_pMesh->FindNeighboursEV(eID, edgeV);

			double dCotSum = 0;
			for ( unsigned int k = 0; k < 2; ++k ) {
				if ( edgeV[k] == IMesh::InvalidID )
					continue;
				m_pMesh->GetVertex(edgeV[k], vo);
				Wml::Vector3f v1 = vi-vo;
				Wml::Vector3f v2 = vj-vo;
				dCotSum += rms::VectorCot(v1, v2);
			}
			dRowSum += dCotSum;

			int nNbrJ = m_vVertMap[nbrs.vNbrs[j]];

			p.Set(i,   nNbrJ,   -dCotSum);
			p.Set(i+N, nNbrJ+N, -dCotSum);

		}

		p.Set(i,   i,   dRowSum);
		p.Set(i+N, i+N, dRowSum);

		p.SetRHS(i,   0.0);
		p.SetRHS(i+N, 0.0);
	}


	if ( ! bUseNaturalBoundary ) {

		if ( ! EmbedBoundary() )
			return false;

		// now set boundary values
		size_t nBdry = m_boundaryInfo.vBoundaryLoops.size();
		for ( unsigned int i = 0; i < nBdry; ++i ) {
			BoundaryLoop & loop = m_boundaryInfo.vBoundaryLoops[i];
			size_t nLoopCount = loop.vVerts.size();
			for ( unsigned int j = 0; j < nLoopCount; ++j ) {
				int r = loop.vVerts[j];
				
				// clear rows
				p.Matrix().ClearRow(r);
				p.Matrix().ClearRow(r+N);
				
				p.Set(r, r, 1.0);
				p.SetRHS(r, loop.vUVs[j].X() );

				p.Set(r+N, r+N, 1.0);
				p.SetRHS(r+N, loop.vUVs[j].Y() );
			}
		}
	
	} else { 

		//// initialize boundary node rows in matrix (except for Rot90 term)
		//size_t nBdry = m_boundaryInfo.vBoundaryLoops.size();
		//for ( unsigned int i = 0; i < nBdry; ++i ) {
		//	BoundaryLoop & loop = m_boundaryInfo.vBoundaryLoops[i];
		//	size_t nLoopCount = loop.vVerts.size();
		//	for ( unsigned int j = 0; j < nLoopCount; ++j ) {
		//		int r = loop.vVerts[j];

		//		// clear row
		//		for ( unsigned int k = 0; k < nCount; ++k ) {
		//			p.SetMatrixLAPACK(2*r, 2*k, 0.0);
		//			p.SetMatrixLAPACK(2*r+1, 2*k+1, 0.0);
		//		}

		//		// set neighbour elements
		//		NeighbourSet & nbrs = m_vVertInfo[r].GetNeighbourSet(OneRing);
		//		size_t nTris = nbrs.vTriAngles.size();		if ( nTris == 0 )	lgBreakToDebugger();
		//		for ( unsigned int k = 0; k < nTris; ++k ) {
		//			TriangleAngles & tri = nbrs.vTriAngles[k];

		//			// in desbrun's EG02 paper, it would seem that these terms should have
		//			// the opposite signs. However, he says they are  "the same as the
		//			// conformal weights", which means these signs are correct
		//			p.AddMatrixLAPACK( 2*r, 2*r,	 -(tri.fAlpha + tri.fBeta) );
		//			p.AddMatrixLAPACK( 2*r+1, 2*r+1, -(tri.fAlpha + tri.fBeta) );
		//			int c1 = m_vVertMap[tri.nJ];
		//			p.AddMatrixLAPACK( 2*r, 2*c1,	  tri.fBeta );
		//			p.AddMatrixLAPACK( 2*r+1, 2*c1+1, tri.fBeta );
		//			int c2 = m_vVertMap[tri.nK];
		//			p.AddMatrixLAPACK( 2*r, 2*c2,	  tri.fAlpha );
		//			p.AddMatrixLAPACK( 2*r+1, 2*c2+1, tri.fAlpha );
		//		}
		//	}
		//}

		// add terms from A matrix
		size_t nBdry = m_boundaryInfo.vBoundaryLoops.size();
		for ( unsigned int i = 0; i < nBdry; ++i ) {
			BoundaryLoop & loop = m_boundaryInfo.vBoundaryLoops[i];
			size_t nLoopCount = loop.vVerts.size();
			for ( unsigned int j = 0; j < nLoopCount; ++j ) {
				int vID = loop.vVerts[j];

				IMesh::VtxNbrItr itr(vID);
				m_pMesh->BeginVtxTriangles(itr);
				IMesh::TriangleID tID = m_pMesh->GetNextVtxTriangle(itr);
				while ( tID != IMesh::InvalidID ) {
					IMesh::VertexID vTri[3];
					m_pMesh->GetTriangle(tID, vTri);

					IMesh::VertexID j,k;
					if      ( vTri[0] == vID ) { j = vTri[1];  k = vTri[2]; }
					else if ( vTri[1] == vID ) { j = vTri[2];  k = vTri[0]; }
					else                       { j = vTri[0];  k = vTri[1]; }
					unsigned int jx = m_vVertMap[j];   unsigned int jy = jx+N;
					unsigned int kx = m_vVertMap[k];   unsigned int ky = kx+N;
					unsigned int ix = m_vVertMap[vID]; unsigned int iy = ix+N;

					p.Set( ix,ky, p.Get(ix,ky) + 1 );
					p.Set( ix,jy, p.Get(ix,jy) - 1 );
					p.Set( iy,jx, p.Get(iy,jx) + 1 );
					p.Set( iy,kx, p.Get(iy,kx) - 1 );

					tID = m_pMesh->GetNextVtxTriangle(itr);
				}
			}
		}

		// add constraints
		for ( unsigned int i = 0; i < nConstraints; ++i ) {
			unsigned int r = vConstraints[i].nVertex;

			// clear row
			p.Matrix().ClearRow(r);
			p.Matrix().ClearRow(r+N);

			p.Set( r,   r,   1.0 );
			p.Set( r+N, r+N, 1.0 );

			p.SetRHS(r,   vConstraints[i].vConstraint.X() );
			p.SetRHS(r+N, vConstraints[i].vConstraint.Y() );
		}

	}

	bool bResult = false;
	if ( ! p.Matrix().IsSymmetric() ) {
		gsi::Solver_UMFPACK solver(&p);
		bResult = solver.Solve();
	} else {
		gsi::Solver_TAUCS solver(&p);
		solver.SetSolverMode(gsi::Solver_TAUCS::TAUCS_LLT);
		bResult = solver.Solve();
	}
	if ( ! bResult ) {
		_RMSInfo("Solve() failed in PlanarParameterization::Compute() !\n");
		return false;
	}

	SetMeshUVs(p.GetSolution(0).GetValues());


	return true;
}



bool PlanarParameterization::Parameterize_ExpMap()
{
	ValidateGeodesicNbrhoods();

	size_t nCount = m_vVertInfo.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		switch ( m_eEmbedType ) {
			case Optimal2DExpMap:
				ComputeWeights_Optimal2D( m_vVertInfo[i].GetNeighbourSet(ExpMap) );
				break;

			case Optimal3DExpMap:
				ComputeWeights_Optimal3D( m_vVertInfo[i].GetNeighbourSet(ExpMap) );
				break;
		}
	}

	if ( ! EmbedBoundary() )
		return false;

	std::vector<double> vU, vV;
	bool bResult = Solve_FixBoundary(vU, vV, ExpMap);
	if ( ! bResult ) {
		_RMSInfo("Solve_FixBoundary failed in PlanarParameterization::Parameterize_ExpMap() !\n");
		return false;
	}
	SetMeshUVs(&vU[0], &vV[0]);



	return true;
}


bool PlanarParameterization::Parameterize_LLE()
{
	lgBreakToDebugger();
#if 0
	ValidateGeodesicNbrhoods();

	int nAvgNumNbrs = 0;

	size_t nCount = m_vVertInfo.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		switch ( m_eEmbedType ) {
			case ExpMapLLE:
				ComputeWeights_Optimal2D( m_vVertInfo[i].GetNeighbourSet(ExpMap) );
				break;

			case StandardLLE:
				ComputeWeights_Optimal3D( m_vVertInfo[i].GetNeighbourSet(ExpMap) );
				break;
		}
		nAvgNumNbrs += m_vVertInfo[i].GetNeighbourSet(ExpMap).nUseNbrs;
	}
	_RMSInfo("Average neighbourhood size: %f\n", (double)nAvgNumNbrs / (double)nCount);

	size_t nVerts = m_vVertInfo.size();
	rmssolver::SparseSymmetricEigenSolver solver((unsigned int)nVerts);
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		NeighbourSet & n = m_vVertInfo[i].GetNeighbourSet(ExpMap);
		size_t nNbrs = n.nUseNbrs;
		for ( unsigned int ni = 0; ni < nNbrs; ++ni ) {
			unsigned int j = m_vVertMap[ n.vNbrs[ni] ];
			float fWeight = n.fWeights[ni];
			solver.Set(i, j, fWeight);
		}
	}

	// set solver flags
	solver.SetRightMultiplyMode( rmssolver::SparseSymmetricEigenSolver::LLE );
	solver.SetSolverMode( rmssolver::SparseSymmetricEigenSolver::PRIMME_JDQMR );

	// solve eigensystem
	int nNumEigens = 4;
	solver.SetNumEigens(nNumEigens);
	bool bResult = solver.Solve();
	if ( ! bResult )
		lgBreakToDebugger();

	// get solver statistics
	double fTolerance; int nIterations; int nRestarts;
	int nMatVecMuls; int nPreconditions; int nRecommender;
	solver.GetStats( fTolerance, nIterations, nRestarts, nMatVecMuls, nPreconditions, nRecommender );
	_RMSInfo("LLE Solve - %d vertices: \n", nVerts );
	_RMSInfo("Tolerance: %-22.15E\n", fTolerance);
	_RMSInfo("Iterations: %d  Restarts: %d  MatVecs: %d  Preconditions: %d  Recommend: %d\n",
		nIterations, nRestarts, nMatVecMuls, nPreconditions, nRecommender );

	SetMeshUVs(&solver);
#endif
	return true;
}




bool PlanarParameterization::EmbedBoundary()
{
	if ( m_boundaryInfo.IsCached(m_eBoundaryMap) ) {
		m_boundaryInfo.UseCache(m_eBoundaryMap);
		return true;
	}

	switch ( m_eBoundaryMap ) {
		case ProjectBoundary:
			if ( m_boundaryInfo.vBoundaryLoops.size() != 1 )
				return false;
			ProjectBoundaryLoop( m_boundaryInfo.vBoundaryLoops[0] );
			break;

		case CircleBoundary:
			if ( m_boundaryInfo.vBoundaryLoops.size() != 1 )
				return false;
			MapBoundaryLoopToCircle( m_boundaryInfo.vBoundaryLoops[0] );
			break;

		case SquareBoundary:
			if ( m_boundaryInfo.vBoundaryLoops.size() != 1 )
				return false;
			MapBoundaryLoopToUnitSquare( m_boundaryInfo.vBoundaryLoops[0] );
			break;

		case NaturalConformalBoundary:
			InitializeBoundaries_Conformal(m_boundaryInfo.vBoundaryLoops);
			break;

		case LLEBoundary:
			InitializeBoundaries_LLE(m_boundaryInfo.vBoundaryLoops);
			break;

		case BoundaryLLEBoundary:
			InitializeBoundary_BoundaryLLE(m_boundaryInfo.vBoundaryLoops[0]);
			break;

		default:
			return false;
	}

	m_boundaryInfo.Cache(m_eBoundaryMap);
	return true;
}




void PlanarParameterization::SetMeshUVs(const double * pValuesU, const double * pValuesV)
{
	if (! m_pMesh->HasUVSet(0) ) {
		m_pMesh->AppendUVSet();
		m_pMesh->InitializeUVSet(0);
	}
	m_pMesh->ClearUVSet(0);

	Wml::Vector2f vUV( (float)pValuesU[0], (float)pValuesV[0] );
	Wml::AxisAlignedBox2f bounds(vUV.X(), vUV.X(), vUV.Y(), vUV.Y());
	for (unsigned int i = 0; i < m_vVertInfo.size(); ++i ) {
		Wml::Vector2f vUV( (float)pValuesU[i], (float)pValuesV[i] );
		rms::Union(bounds, vUV);
	}
	float fMaxDim = std::max( rms::GetDimension(bounds,0), rms::GetDimension(bounds,1) );
	Wml::Vector2f vCenter( rms::Center(bounds) );
	float fScale = (m_bScaleUVs) ? m_fUVScaleFactor / fMaxDim : 1.0f;

	for (unsigned int i = 0; i < m_vVertInfo.size(); ++i ) {
		rms::IMesh::VertexID vID = m_vVertInfo[i].vID;

		Wml::Vector2f vUV( (float)pValuesU[i], (float)pValuesV[i] );
		vUV -= vCenter;
		vUV *= fScale;

		m_pMesh->SetUV( vID, 0, vUV );
	}
}



void PlanarParameterization::SetMeshUVs(const double * pValues)
{
	if (! m_pMesh->HasUVSet(0) ) {
		m_pMesh->AppendUVSet();
		m_pMesh->InitializeUVSet(0);
	}
	m_pMesh->ClearUVSet(0);

	unsigned int N = (unsigned int)m_vVertInfo.size();
	Wml::Vector2f vUV( (float)pValues[0], (float)pValues[N] );
	Wml::AxisAlignedBox2f bounds(vUV.X(), vUV.X(), vUV.Y(), vUV.Y());
	for (unsigned int i = 0; i < N; ++i ) {
		Wml::Vector2f vUV;
		vUV = Wml::Vector2f( (float)pValues[i], (float)pValues[i+N] );
		rms::Union(bounds, vUV);
	}
	float fMaxDim = std::max( rms::GetDimension(bounds,0), rms::GetDimension(bounds,1) );
	Wml::Vector2f vCenter( rms::Center(bounds) );
	float fScale = (m_bScaleUVs) ? m_fUVScaleFactor / fMaxDim : 1.0f;

	for (unsigned int i = 0; i < m_vVertInfo.size(); ++i ) {
		rms::IMesh::VertexID vID = m_vVertInfo[i].vID;

		Wml::Vector2f vUV;
		vUV = Wml::Vector2f( (float)pValues[i], (float)pValues[i+N] );

		vUV -= vCenter;
		vUV *= fScale;

		m_pMesh->SetUV( vID, 0, vUV );
	}
}


#if 0
void PlanarParameterization::SetMeshUVs(rmssolver::SparseSymmetricEigenSolver * pEigenSolver)
{
	lgBreakToDebugger();

	size_t nVerts = m_vVertInfo.size();
	int nNumEigens = 3;
	std::vector< double > eigenVals;
	std::vector< std::vector<double> > eigenVecs;
	eigenVecs.resize(nNumEigens);
	for ( int i = 0; i < nNumEigens; ++i )
		eigenVecs[i].resize(nVerts);
	for ( unsigned int i = 0; i < (unsigned int)nNumEigens; ++i ) {
		eigenVals.push_back( pEigenSolver->GetEigenValue(i) );
		pEigenSolver->GetEigenVector(i, &eigenVecs[i][0] );
	}

	_RMSInfo("Top eigenvalues are %.12f %.12f %.12f %.12f\n", 
		eigenVals[0], eigenVals[1], eigenVals[2], eigenVals[3] );
	//double k = 100000.0;
	//for ( int j = 1; j <= 3; ++j )
	//	eigenVals[j] *= k;

	if (! m_pMesh->HasUVSet(0) ) {
		m_pMesh->AppendUVSet();
		m_pMesh->InitializeUVSet(0);
	}
	m_pMesh->ClearUVSet(0);

	// [RMS: this is supposed to fix the orientation of the LLE parameterization,
	//   but it doesn't actually work... ]]

	// compute orientation matrix (align first triangle edge w/ x axis)
	Wml::Matrix2f mat = Wml::Matrix2f(1,0,0,1);
	rms::IMesh::TriangleID nFirstTri = 0;		rms::IMesh::VertexID nTri[3];
	m_pMesh->GetTriangle( nFirstTri, nTri );
	Wml::Vector2f vA( (float)eigenVecs[1][nTri[0]], (float)eigenVecs[2][nTri[0]] );
	Wml::Vector2f vB( (float)eigenVecs[1][nTri[1]], (float)eigenVecs[2][nTri[1]] );
	Wml::Vector2f vC( (float)eigenVecs[2][nTri[1]], (float)eigenVecs[2][nTri[2]] );
	//float fAreaSum = 0.5f * ( (vA.X()*vB.Y() - vB.X()*vA.Y()) + (vB.X()*vC.Y() - vC.X()*vB.Y()) + (vC.X()*vA.Y() - vA.X()*vC.Y()) );
	//_RMSInfo("Area sum: %f\n", fAreaSum);
	//if ( fAreaSum > 0 ) {
	//	mat = mat * Wml::Matrix2f( -1, 0, 0, 1 );	// flip
	//	vA = mat * vA;
	//	vB = mat * vB;
	//	vC = mat * vC;
	//}
	Wml::Vector2f vLine(vB - vA);
	vLine.Normalize();
	float fCosTheta = vLine.Dot(Wml::Vector2f::UNIT_X);
	float fTmp = 1 - fCosTheta*fCosTheta;
	if ( fTmp < 0 ) fTmp = 0;		// need to clamp so that sqrt works...
	float fSinTheta = (float)sqrt(fTmp);
	float fDotPerp = vLine.DotPerp(Wml::Vector2f::UNIT_X);
	if ( fDotPerp > 0 )	// get the right sign...
		fSinTheta = -fSinTheta;
	Wml::Matrix2f matRotate2( fCosTheta, fSinTheta, -fSinTheta, fCosTheta );
	mat = matRotate2 * mat;

	float fDotX = ( mat * vLine ).Dot( Wml::Vector2f::UNIT_X );
	if ( fDotX < 0 ) {
		mat = Wml::Matrix2f(-1,0,0,1) * mat;
	}

	Wml::Vector2f vPerpY = (mat * vLine.Perp());
	float fDotY = vPerpY.Dot( Wml::Vector2f::UNIT_Y );
	if ( fDotY < 0 ) {
		mat = Wml::Matrix2f(1,0,0,-1) * mat;
	}

	//float fDotX2 = ( mat * vLine ).Dot( Wml::Vector2f::UNIT_X );
	//float fDotY2 = ( (mat * vLine.Perp()) ).Dot( Wml::Vector2f::UNIT_Y );
	//float fDotY3 = ( (mat * vLine).Perp() ).Dot( Wml::Vector2f::UNIT_Y );
	//_RMSInfo("DotX : %f %f\n", fDotX, fDotX2);
	//_RMSInfo("DotY : %f %f %f\n", fDotY, fDotY2, fDotY3);


	// find bounds for param
	Wml::AxisAlignedBox2f bounds(9999999.0f,-9999999.0f,9999999.0f,-9999999.0f);
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Wml::Vector2f vUV( (float)eigenVecs[1][i], (float)eigenVecs[2][i] );
		vUV = mat * vUV;
		rms::Union(bounds,vUV);
	}
	float fMaxDim = std::max( rms::GetDimension(bounds,0), rms::GetDimension(bounds,1) );
	Wml::Vector2f vCenter( rms::Center(bounds) );
	float fScale = (m_bScaleUVs) ? m_fUVScaleFactor / fMaxDim : 1.0f;

	for ( unsigned int i = 0; i < nVerts; ++i ) {
		rms::IMesh::VertexID vID = m_vVertInfo[i].vID;

		Wml::Vector2f vUV( (float)eigenVecs[1][i], (float)eigenVecs[2][i] );
		vUV = mat * vUV;
		vUV -= vCenter;
		vUV *= fScale;

		m_pMesh->SetUV( vID, 0, vUV );
	}

}
#endif





bool PlanarParameterization::Compute_LLE_Boundary()
{
	lgBreakToDebugger();
#if 0
	std::vector<bool> bBoundary(m_pMesh->GetMaxVertexID(), false);
	std::vector<unsigned int> vRewrite( m_pMesh->GetMaxVertexID() );
	std::vector<unsigned int> vBoundary;
	int nRewriteCounter = 0;
	rms::VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		rms::IMesh::VertexID vID = *curv;  ++curv;
		bBoundary[vID] = m_pMesh->IsBoundaryVertex(vID);
		if ( bBoundary[vID] ) {
			vRewrite[vID] = nRewriteCounter++;
			vBoundary.push_back(vID);
		} else
			vRewrite[vID] = rms::IMesh::InvalidID;
	}
	size_t nBoundaryCount = vBoundary.size();


	rmssolver::SparseSymmetricEigenSolver solver((unsigned int)nBoundaryCount);


	std::vector<rms::IMesh::VertexID> vBdryNbrs;
	std::vector<float> vWeights;
	std::vector<Wml::Vector2f> vNbrUVs;
	for ( unsigned int i = 0; i < nBoundaryCount; ++i ) {
		NeighbourSet & n = m_vVertInfo[ vBoundary[i] ].GetNeighbourSet(ExpMap);
		
		// find nbrs that are boundaries
		vBdryNbrs.resize(0);
		vNbrUVs.resize(0);
		size_t nNbrs = n.vNbrs.size();
		for ( unsigned int j = 0; j < nNbrs; ++j ) {
			rms::IMesh::VertexID vNbrID = n.vNbrs[j];
			if ( bBoundary[vNbrID] ) {
				vBdryNbrs.push_back(vNbrID);
				vNbrUVs.push_back( n.vLocalUVs[j] );
			}
		}
		size_t nBdryNbrCount = vBdryNbrs.size();

		ParamSystem sys;
		sys.Resize( (unsigned int)nBdryNbrCount );

#if 0
		Wml::Vector2f vC, vJ, vK;
		vC = Wml::Vector2f::ZERO;
		for ( unsigned int k = 0; k < nBdryNbrCount; ++k ) {
			for ( unsigned int j = k; j < nBdryNbrCount; ++j ) {
				vJ = vNbrUVs[j];
				vK = vNbrUVs[k];
				float cjk = ( vC - vJ ).Dot( vC - vK );
				sys.SetMatrixLAPACK(j,k, cjk);
				sys.SetMatrixLAPACK(k,j, cjk);
			}
		}
#else
		Wml::Vector3f vC, vJ, vK;
		m_pMesh->GetVertex( vBoundary[i], vC );
		for ( unsigned int k = 0; k < nBdryNbrCount; ++k ) {
			for ( unsigned int j = k; j < nBdryNbrCount; ++j ) {
				m_pMesh->GetVertex( vBdryNbrs[j], vJ );
				m_pMesh->GetVertex( vBdryNbrs[k], vK );
				float cjk = ( vC - vJ ).Dot( vC - vK );
				sys.SetMatrixLAPACK(j,k, cjk);
				sys.SetMatrixLAPACK(k,j, cjk);
			}
		}

#endif

		// add fraction of identity matrix to improve system
		double fTrace = 0.0f;
		for ( unsigned int k = 0; k < sys.nDim; ++k )
			fTrace += sys.GetMatrixLAPACK(k,k);
		double fDelta = fTrace / 100;
		fDelta /= (double)sys.nDim;
		for ( unsigned int k = 0; k < sys.nDim; ++k )
			sys.SetMatrixLAPACK(k, k,  sys.GetMatrixLAPACK(k,k) + fDelta);

		for ( unsigned int k = 0; k < sys.nDim; ++k )
			sys.SetRHS(k, 1);

		bool bResult = SolveDirect(sys);
		if (! bResult )
			_RMSInfo("SolveDirect failed in PlanarParameterization::Compute_LLE_Boundary on vertex %d\n", i );

		// re-scale weights so that they sum to one
		vWeights.resize(nBdryNbrCount);
		float fWeightSum = 0.0f;
		for ( unsigned int j = 0; j < sys.nDim; ++j )
			fWeightSum += (float)sys.vSolution[j];
		for ( unsigned int j = 0; j < nBdryNbrCount; ++j )
			vWeights[j] = (float)sys.vSolution[j] / fWeightSum;

		_RMSInfo("Nbrs %d: ", n.vID);
		for ( unsigned int k = 0; k < nBdryNbrCount; ++k )
			_RMSInfo("%d/%f ", vBdryNbrs[k], vWeights[k]);
		_RMSInfo("\n");

		for ( unsigned int j = 0; j < nBdryNbrCount; ++j ) {
			rms::IMesh::VertexID vNbrID = vBdryNbrs[j];
			solver.Set(i, vRewrite[ vNbrID ], vWeights[j]);
		}
	}

	// set solver flags
	solver.SetRightMultiplyMode( rmssolver::SparseSymmetricEigenSolver::LLE );
	solver.SetSolverMode( rmssolver::SparseSymmetricEigenSolver::PRIMME_JDQMR );

	// solve eigensystem
	int nNumEigens = 4;
	solver.SetNumEigens(nNumEigens);
	bool bResult = solver.Solve();
	if ( ! bResult )
		lgBreakToDebugger();

	// get solver statistics
	double fTolerance; int nIterations; int nRestarts;
	int nMatVecMuls; int nPreconditions; int nRecommender;
	solver.GetStats( fTolerance, nIterations, nRestarts, nMatVecMuls, nPreconditions, nRecommender );
	_RMSInfo("LLE Solve - %d vertices: \n", nBoundaryCount);
	_RMSInfo("Tolerance: %-22.15E\n", fTolerance);
	_RMSInfo("Iterations: %d  Restarts: %d  MatVecs: %d  Preconditions: %d  Recommend: %d\n",
		nIterations, nRestarts, nMatVecMuls, nPreconditions, nRecommender );

	std::vector< double > eigenVals;
	std::vector< std::vector<double> > eigenVecs;
	eigenVecs.resize(nNumEigens);
	for ( int i = 0; i < nNumEigens; ++i )
		eigenVecs[i].resize(nBoundaryCount);
	for ( unsigned int i = 0; i < (unsigned int)nNumEigens; ++i ) {
		eigenVals.push_back( solver.GetEigenValue(i) );
		solver.GetEigenVector(i, &eigenVecs[i][0] );
	}

	_RMSInfo("Top eigenvalues are %.12f %.12f %.12f %.12f\n", 
		eigenVals[0], eigenVals[1], eigenVals[2], eigenVals[3] );
	//double k = 100000.0;
	//for ( int j = 1; j <= 3; ++j )
	//	eigenVals[j] *= k;

	if (! m_pMesh->HasUVSet(0) ) {
		m_pMesh->AppendUVSet();
		m_pMesh->InitializeUVSet(0);
	}
	m_pMesh->ClearUVSet(0);

	// find bounds for param
	Wml::AxisAlignedBox2f bounds(9999999.0f,-9999999.0f,9999999.0f,-9999999.0f);
	for ( unsigned int i = 0; i < nBoundaryCount; ++i ) {
		double d1 = eigenVecs[1][i];
		double d2 = eigenVecs[2][i];
		Wml::Vector2f vUV( (float)d1, (float)d2 );
		rms::Union(bounds,vUV);
	}
	float fMaxDim = std::max( bounds.Max[0]-bounds.Min[0], bounds.Max[1]-bounds.Min[1] );
	float fScale = 2.0f / fMaxDim;
	Wml::Vector2f vCenter( 0.5f * (bounds.Max[0]+bounds.Min[0]), 0.5f * (bounds.Max[1]+bounds.Min[1]) );

	curv = m_pMesh->BeginVertices();
	while ( curv != endv ) {
		rms::IMesh::VertexID vID = *curv;  ++curv;
		if ( ! bBoundary[vID] )
			m_pMesh->SetUV(vID, 0, Wml::Vector2f::ZERO );
		else {
			int i = vRewrite[vID];
			double d1 = eigenVecs[1][i];
			double d2 = eigenVecs[2][i];
			double d3 = eigenVecs[3][i];
			Wml::Vector2f vUV( (float)d1, (float)d2 );
			vUV -= vCenter;
			vUV *= fScale;
			m_pMesh->SetUV( vID, 0, vUV );
		}
	}

#endif

	return true;
}







void PlanarParameterization::FindReconstructionError(NeighbourhoodType eNbrType)
{
	float fErr3D[3] = {999999.0f, 0.0f, 0.0f};
	float fErr3DNoBdry[3] = {999999.0f, 0.0f, 0.0f};
	float fErr2D[3] = {999999.0f, 0.0f, 0.0f};
	float fErr2DNoBdry[3] = {999999.0f, 0.0f, 0.0f};

	size_t nCount = m_vVertInfo.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {

		float fReconsErr3D = GetReconstructionError_3D( m_vVertInfo[i].GetNeighbourSet(eNbrType) );
		fErr3D[0] = std::min(fErr3D[0], fReconsErr3D);
		fErr3D[1] = std::max(fErr3D[1], fReconsErr3D);
		if ( _finite(fReconsErr3D) )
			fErr3D[2] += fReconsErr3D;
		if ( ! m_pMesh->IsBoundaryVertex( m_vVertInfo[i].vID ) ) {
			fErr3DNoBdry[0] = std::min(fErr3DNoBdry[0], fReconsErr3D);
			fErr3DNoBdry[1] = std::max(fErr3DNoBdry[1], fReconsErr3D);
			fErr3DNoBdry[2] += fReconsErr3D;
		}
		float fReconsErr2D = GetReconstructionError_2D( m_vVertInfo[i].GetNeighbourSet(eNbrType) );
		fErr2D[0] = std::min(fErr2D[0], fReconsErr2D);
		fErr2D[1] = std::max(fErr2D[1], fReconsErr2D);
		fErr2D[2] += fReconsErr2D;
		if ( ! m_pMesh->IsBoundaryVertex( m_vVertInfo[i].vID ) ) {
			fErr2DNoBdry[0] = std::min(fErr2DNoBdry[0], fReconsErr2D);
			fErr2DNoBdry[1] = std::max(fErr2DNoBdry[1], fReconsErr2D);
			fErr2DNoBdry[2] += fReconsErr2D;
		}
	}
	_RMSInfo("Local Reconstruction 3D:\n");
	_RMSInfo("    Min: %f  Max: %f  Avg: %f\n", fErr3D[0], fErr3D[1], fErr3D[2] / (float)nCount);
	_RMSInfo("  Interior Only:\n");
	_RMSInfo("    Min: %f  Max: %f  Avg: %f\n", fErr3DNoBdry[0], fErr3DNoBdry[1], fErr3DNoBdry[2] / (float)nCount);
	_RMSInfo("Local Reconstruction 2D:\n");
	_RMSInfo("    Min: %f  Max: %f  Avg: %f\n", fErr2D[0], fErr2D[1], fErr2D[2] / (float)nCount);
	_RMSInfo("  Interior Only:\n");
	_RMSInfo("    Min: %f  Max: %f  Avg: %f\n", fErr2DNoBdry[0], fErr2DNoBdry[1], fErr2DNoBdry[2] / (float)nCount);
}



struct TriStretchInfo
{
	float fLinf;
	float fL2;
	float fArea3;
	float fArea2;
};

void PlanarParameterization::ComputeMeshOneRingStretch()
{
	Wml::Vector3f vVerts[3];
	Wml::Vector2f vTexCoords[3];
	float fMin, fMax;		// not sure what this is...

	// pre-compute stretch metric for each triangle
	std::vector<TriStretchInfo> vStretch;
	unsigned int nMax = m_pMesh->GetMaxTriangleID();
	vStretch.resize( nMax );
	rms::VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
	while ( curt != endt ) {
		rms::IMesh::TriangleID tID = *curt;  ++curt;
		m_pMesh->GetTriangle(tID, vVerts);
		m_pMesh->GetTriangleUV(tID, 0, vTexCoords);

		TriStretchInfo t;

		rms::StretchMetric1( vVerts[0], vVerts[1], vVerts[2], 
			vTexCoords[0], vTexCoords[1], vTexCoords[2],
			fMin, fMax, t.fL2, t.fLinf );
		if ( t.fL2 < 0 )
			t.fL2 = std::numeric_limits<float>::max();
		t.fArea3 = rms::Area( vVerts[0], vVerts[1], vVerts[2] );
		t.fArea2 = rms::Area( vTexCoords[0], vTexCoords[1], vTexCoords[2] );

		//if ( fabs(t.fArea2) < 0.0000001f )
		//	lgBreakToDebugger();

		vStretch[tID] = t;
//		_RMSInfo("%f %f %f %f\n", t.fArea2, t.fArea3, t.fL2, t.fLinf);
	}


	// ok now find global stretch
	double dArea3Sum = 0.0, dArea2Sum = 0.0, dL2Sum = 0.0, dLinfMax = 0.0;
	curt = m_pMesh->BeginTriangles();
	while ( curt != endt ) {
		rms::IMesh::TriangleID tID = *curt;  ++curt;
		dArea3Sum += vStretch[tID].fArea3;
		dArea2Sum += vStretch[tID].fArea2;
		dL2Sum += vStretch[tID].fL2 * vStretch[tID].fL2 * vStretch[tID].fArea3;
		if ( vStretch[tID].fLinf > dLinfMax )
			dLinfMax = vStretch[tID].fLinf;
	}
	double dScaleFactor = sqrt(dArea2Sum / dArea3Sum);
	double dL2 = sqrt( dL2Sum / dArea3Sum );
	double dLinf = dLinfMax;

	_RMSInfo("L2: %f  LInf: %f\n", dL2, dLinf);
/*
	size_t nCount = m_vVertInfo.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::IMesh::VertexID vID = m_vVertInfo[i].vID;
		NeighbourSet & nbrs = m_vVertInfo[i].GetNeighbourSet(OneRight);

		float fL2Accum = 0, fLInfAccum = 0;
		rms::IMesh::VtxNbrItr itr(vID);
		m_pMesh->BeginVtxTriangles(v);
		rms::IMesh::TriangleID tID = m_pMesh->GetNextVtxTriangle(

		for ( unsigned int j = 0; j < nbrs.nUseNbrs; ++j ) {

		info.pMesh->GetTriangle(vTris[i], vVerts);
		info.pMesh->GetTextureCoords(vTris[i], vTexCoords);

		// compute stretch metric
		float fMin, fMax, fL2, fLinf;
		Wml::StretchMetric1( vVerts[0], vVerts[1], vVerts[2], 
			vTexCoords[0], vTexCoords[1], vTexCoords[2],
			fMin, fMax, fL2, fLinf );
		if ( fL2 < 0 )
			fL2 = 1000000.0f;
		fL2Accum += (float)fabs(1.0 - fL2);
//		_RMSInfo("%f ", fL2);
		fLInfAccum += (float)fabs(1.0 - fLinf);
	}
//	_RMSInfo("\n");
*/
}






void PlanarParameterization::PrecomputeMeshData()
{
	m_vVertInfo.resize(0);
	m_vVertMap.clear();

	unsigned int nCount = 0;
	rms::VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		rms::IMesh::VertexID vID = *curv; ++curv;

		MeshVertex v;
		v.vID = vID;
		m_vVertInfo.push_back(v);
		MeshVertex & newv = m_vVertInfo.back();

		// pre-compute one-rings...no real reason to do this...
		MakeOneRingNeighbourSet(vID, newv.m_OneRingNbrs );

		m_vVertMap[vID] = (unsigned int)(m_vVertInfo.size()-1);
	}

	ComputeBoundaryInfo(m_boundaryInfo);
}



void PlanarParameterization::ComputeBoundaryInfo( BoundaryInfo & info )
{
	info.Clear();

	// find boundary edges
	std::vector<Edge> boundaryEdges;
	VFTriangleMesh::edge_iterator
		cure( m_pMesh->BeginEdges()), ende(m_pMesh->EndEdges());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if ( m_pMesh->IsBoundaryEdge(eID) ) {
			IMesh::VertexID nVerts[2], nTris[2];
			m_pMesh->GetEdge(eID, nVerts, nTris);
			Edge e;  e.v1 = nVerts[0];  e.v2 = nVerts[1];
			if (! m_pMesh->IsBoundaryVertex(e.v1) || ! m_pMesh->IsBoundaryVertex(e.v2) )
				lgBreakToDebugger();

			if ( m_pMesh->GetEdgeCount(e.v1) < 3 || m_pMesh->GetEdgeCount(e.v2) < 3 ) {
				_RMSInfo("Mesh has ear triangles...\n");
				//lgBreakToDebugger();
			}

			boundaryEdges.push_back(e);
		}
	}
	//rms::VFTriangleMesh::triangle_iterator 
	//	curt( m_pMesh->BeginTriangles() ), endt( m_pMesh->EndTriangles() );
	//while ( curt != endt ) {
	//	rms::IMesh::TriangleID tID = *curt;  ++curt;
	//	rms::IMesh::VertexID vTri[3];
	//	m_pMesh->GetTriangle(tID, vTri);
	//	rms::IMesh::TriangleID vNbrs[3];
	//	m_pMesh->FindNeighbours( tID, vNbrs );
	//	for ( int j = 0; j < 3; ++j ) {
	//		if ( vNbrs[j] == rms::IMesh::InvalidID ) {
	//			Edge e;  e.v1 = vTri[j];  e.v2 = vTri[(j+1)%3];
	//			boundaryEdges.push_back(e);
	//		}
	//	}
	//}	

	

	// [RMS] O(N^2)-ish version. Could be improved by adding multiple verts 
	//  in the inner loop....
	std::set<Edge> edges(boundaryEdges.begin(), boundaryEdges.end());
	std::vector<rms::IMesh::VertexID> boundaryLoop;
	while (! edges.empty() ) {

		boundaryLoop.resize(0);

		std::set<Edge>::iterator cure(edges.begin()), ende(edges.end());
		rms::IMesh::VertexID vCur = (*cure).v1;
		while ( vCur != rms::IMesh::InvalidID ) {
			rms::IMesh::VertexID vNext = rms::IMesh::InvalidID;
			cure = edges.begin();
			while ( cure != ende && vNext == rms::IMesh::InvalidID ) {
				Edge & edge = const_cast<Edge&>(*cure);
				if ( edge.v1 == vCur )
					vNext = edge.v2;
				else if ( edge.v2 == vCur )
					vNext = edge.v1;

				// found next edge, add to loop and remove from edge set
				if ( vNext != rms::IMesh::InvalidID ) {
					if ( ! m_pMesh->IsBoundaryVertex(vNext) )
						lgBreakToDebugger();
					boundaryLoop.push_back(vNext);
					edges.erase(cure);
					cure = ende;
				} else
					cure++;
			}
			vCur = vNext;
		}

		// ok, finished a loop. Add it to the boundary loop list
		if ( boundaryLoop.size() == 0 )
			lgBreakToDebugger();
		BoundaryLoop newLoop;
		info.vBoundaryLoops.push_back(newLoop);
		BoundaryLoop & curLoop = info.vBoundaryLoops.back();
		size_t nCount = boundaryLoop.size();
		curLoop.vVerts.resize(nCount);
		for ( unsigned int i = 0; i < nCount; ++i )
			curLoop.vVerts[i] = m_vVertMap[boundaryLoop[i]];
	}


}



class NeighbourSetBuilder : public rms::IMesh::NeighborTriCallback {
public:
	NeighbourSetBuilder() {}
	rms::VFTriangleMesh * pMesh;
	std::set<rms::IMesh::VertexID> vNbrs;
	std::set<rms::IMesh::TriangleID> vTris;
	virtual void NextTriangle( rms::IMesh::TriangleID tID ) { 
		vTris.insert(tID);
		rms::IMesh::VertexID vTri[3];
		pMesh->GetTriangle(tID, vTri);
		for ( int j = 0; j < 3; ++j )
			vNbrs.insert(vTri[j]);
	}
};

void PlanarParameterization::MakeOneRingNeighbourSet( rms::IMesh::VertexID vID, NeighbourSet & nbrs )
{
	nbrs.Clear();
	nbrs.vID = vID;

	// find un-ordered neighbours
	//NeighbourSetBuilder n;
	//n.pMesh = m_pMesh;
	//m_pMesh->NeighbourIteration(vID, &n);

	// this makes an un-ordered one ring...
	//std::set<rms::IMesh::VertexID>::iterator cur(n.vNbrs.begin()), end(n.vNbrs.end());
	//while (cur != end) {
	//	rms::IMesh::VertexID vNbrID = *cur++;
	//	if ( vNbrID != vID )
	//		nbrs.vNbrs.push_back(vNbrID);
	//}
	//size_t nCount = nbrs.vNbrs.size();


	// make edge set
	unsigned int nBoundaryEdges = 0;
	//std::set<IMesh::EdgeID> vEdges;
	//std::set<IMesh::EdgeID> vBoundaryEdges;
	VFTriangleMesh::VtxNbrItr edgeItr( vID );
	m_pMesh->BeginVtxEdges(edgeItr);
	IMesh::EdgeID eID = m_pMesh->GetNextVtxEdges(edgeItr);
	while ( eID != IMesh::InvalidID ) {
		//vEdges.insert(eID);
		if ( m_pMesh->IsBoundaryEdge(eID) )
			nBoundaryEdges++;
			//vBoundaryEdges.insert(eID);
		eID = m_pMesh->GetNextVtxEdges(edgeItr);
	}

	// if we have more than 2 boundary edges, we're going to fail
	//if ( vBoundaryEdges.size() > 2 )
	//	_RMSInfo("Vertex %d has %d boundary edges - cannot find proper one-ring", vID, vBoundaryEdges.size());
	if ( nBoundaryEdges > 2 ) {
		_RMSInfo("Vertex %d has %d boundary edges - cannot find proper one-ring", vID, nBoundaryEdges);
		lgBreakToDebugger();
	}


	// pick starting edge to be a boundary edge if we have any
	//IMesh::EdgeID eCurID = (vBoundaryEdges.size() > 0) ? *vBoundaryEdges.begin() : *vEdges.begin();
	//IMesh::VertexID vEdgeV[2], nTri[3];
	//IMesh::TriangleID vEdgeT[2];
	//m_pMesh->GetEdge(eCurID, vEdgeV, vEdgeT);

	std::vector< rms::IMesh::VertexID > vOneRing;
	rms::MeshUtils::VertexOneRing(*m_pMesh, vID, vOneRing, true);	

#if 0
	vOneRing.push_back( (vEdgeV[0] == vID) ? vEdgeV[1] : vEdgeV[0] );
	vEdges.erase( eCurID );

	while ( eCurID != IMesh::InvalidID ) {

		// find verts that are before and after current edge
		IMesh::VertexID vOther[2] = { IMesh::InvalidID, IMesh::InvalidID };
		if ( vEdgeT[0] != IMesh::InvalidID) {
			m_pMesh->GetTriangle( vEdgeT[0], nTri );
			for ( int k = 0; k < 3; ++k )
				if ( nTri[k] != vEdgeV[0] && nTri[k] != vEdgeV[1] ) vOther[0] = nTri[k];
		}
		if ( vEdgeT[1] != IMesh::InvalidID) {
			m_pMesh->GetTriangle( vEdgeT[1], nTri );
			for ( int k = 0; k < 3; ++k )
				if ( nTri[k] != vEdgeV[0] && nTri[k] != vEdgeV[1] ) vOther[1] = nTri[k];
		}

		// figure out which is which
		IMesh::VertexID vPrev = vOneRing[ vOneRing.size() - 2];
		IMesh::VertexID vNext = IMesh::InvalidID;
		if ( vOther[0] == IMesh::InvalidID && vOther[1] == IMesh::InvalidID ) {
			break;	// something has gone horribly wrong
		} else if ( vOther[0] == IMesh::InvalidID ) {
			vNext = (vOther[1] == vPrev) ? IMesh::InvalidID : vOther[1];
		} else if ( vOther[1] == IMesh::InvalidID ) {
			vNext = (vOther[0] == vPrev) ? IMesh::InvalidID : vOther[0];
		} else {
			vNext = (vOther[0] == vPrev) ? vOther[1] : vOther[0];
		}

		if ( vNext == IMesh::InvalidID || vNext == vOneRing.front() ) {
			eCurID = IMesh::InvalidID;
			continue;
		}

		eCurID = m_pMesh->FindEdge( vID, vNext );

		// ok sanity check - we should not have seen this edge yet...
		if ( eCurID == IMesh::InvalidID || vEdges.find(eCurID) == vEdges.end() )
			lgBreakToDebugger();

		vEdges.erase(eCurID);
		vOneRing.push_back( vNext );
		m_pMesh->GetEdge( eCurID, vEdgeV, vEdgeT);
	}
#endif
		

#if 0
	// do ordered walk around neighbour triangles. abort if there are gaps
	std::vector< rms::IMesh::VertexID > vOneRing;
	rms::IMesh::TriangleID tCur = *n.vTris.begin();
	rms::IMesh::TriangleID tStart = tCur;
	n.vTris.erase(tCur);
	rms::IMesh::VertexID vTri[3];
	m_pMesh->GetTriangle(tCur, vTri);
	for ( int j = 0; j < 3; ++j ) {
		if ( vTri[j] != vID )
			vOneRing.push_back(vTri[j]);
	}

	bool bAbort = false;
	bool bReversed = false;
	while ( ! n.vTris.empty() && ! bAbort ) {
		rms::IMesh::TriangleID tNbrs[3];
		m_pMesh->FindNeighbours(tCur, tNbrs);

		// find next triangle
		rms::IMesh::TriangleID tNext = rms::IMesh::InvalidID;
		for ( int ni = 0; ni < 3 && tNext == rms::IMesh::InvalidID; ++ni ) {
			m_pMesh->GetTriangle( tNbrs[ni], vTri );
			for ( int j = 0; j < 3; ++j ) {
				if ( vTri[j] == vID && (vTri[(j+1)%3] == vOneRing.back() || vTri[(j+2)%3] == vOneRing.back()) )
					tNext = tNbrs[ni];
			}
		}

		if ( tNext == rms::IMesh::InvalidID ) {
			if ( ! bReversed ) {		// if not found, try reversing ring
				std::reverse( vOneRing.begin(), vOneRing.end() );
				tCur = tStart;
				bReversed =  true;
				continue;
			} else {
				bAbort = true;		// otherwise we are done...
			}
		} else {
			// remove triangle
			n.vTris.erase(tNext);
			tCur = tNext;

			// append vert to one ring (unless it is the first vertex, in which case we are done...)
			m_pMesh->GetTriangle( tCur, vTri );
			rms::IMesh::VertexID vNext = rms::IMesh::InvalidID;
			for ( int j = 0; j < 3 && vNext == rms::IMesh::InvalidID; ++j ) {
				if ( vTri[j] == vID ) 
					vNext = (vTri[(j+1)%3] == vOneRing.back()) ? vTri[(j+2)%3] : vTri[(j+1)%3];
				if ( vNext != rms::IMesh::InvalidID && vNext != vOneRing.front() )
					vOneRing.push_back(vNext);
			}
		}
	}

	if ( vOneRing.size() != n.vNbrs.size() - 1 )
		_RMSInfo("One ring did not use all nbrs for vert %d\n", vID  );
	for ( unsigned int i = 0; i < vOneRing.size(); ++i  ) {
		for ( unsigned int j = 0; j < vOneRing.size(); ++j  )
			if ( i != j && vOneRing[i] == vOneRing[j] )
				lgBreakToDebugger();
	}

#endif

	// sanity checks
	//if ( ! n.vTris.empty() )
	//	_RMSInfo("Did not use all tris for one-ring of vert %d\n", vID  );

	// reverse one ring if necessary, so it is CCW
	Wml::Vector3f vMiddle, vNormal, vA, vB;
	m_pMesh->GetVertex( vID, vMiddle, &vNormal );
	m_pMesh->GetVertex( vOneRing[0], vA );
	m_pMesh->GetVertex( vOneRing[1], vB );
	Wml::Vector3f vEdge1( vA - vMiddle );
	Wml::Vector3f vEdge2( vB - vMiddle );
	Wml::Vector3f vCross( vEdge1.Cross(vEdge2) );
	if ( vCross.Dot(vNormal) < 0.0f )
		std::reverse( vOneRing.begin(), vOneRing.end() );

	nbrs.vNbrs = vOneRing;
	size_t nCount = nbrs.vNbrs.size();

	// set 3D distances
	nbrs.fDistances3D.resize(nCount);
	Wml::Vector3f vVtx;
	m_pMesh->GetVertex( nbrs.vID, vVtx );
	for ( unsigned int i = 0; i < nCount; ++i ) {
		Wml::Vector3f vNbrVtx;
		m_pMesh->GetVertex( nbrs.vNbrs[i], vNbrVtx );
		nbrs.fDistances3D[i] = (vVtx - vNbrVtx).Length();
	}

	// generate flattened one-ring
	std::vector<float> vAngle, vRadius;

	Wml::Vector3f vFirst, vFirstEdge;
	m_pMesh->GetVertex( vOneRing[0], vFirst );
	vFirstEdge = (vFirst - vMiddle);
	vRadius.push_back( vFirstEdge.Normalize() );
	vAngle.push_back(0.0);

	// compute angle and distance for each triangle
	float fAngleSum = 0.0f;
	Wml::Vector3f vLast(vFirst), vLastEdge(vFirstEdge), vCur;
	// Wml::Vector3f ;
	for ( unsigned int i = 1; i < nCount+1; ++i ) {
		m_pMesh->GetVertex( vOneRing[i%nCount], vCur );

		// compute angle between vectors
		Wml::Vector3f vCurEdge( vCur - vMiddle );
		vRadius.push_back( vCurEdge.Normalize() );
		float fDot = vCurEdge.Dot( vLastEdge );
		float fAngle = acos( rms::Clamp(fDot, -1.0f, 1.0f) );
		vAngle.push_back( fAngle );
		fAngleSum += fAngle;

		vLast = vCur;		vLastEdge = vCurEdge;
	}

	// add final angle to anglesum
	//float fDot = vLastEdge.Dot(vFirstEdge);
	//float fAngle = acos( rms::Clamp(fDot, -1.0f, 1.0f) );
	//fAngleSum += fAngle;

	// ok, now scale angles and generate 2D vertices
	nbrs.vFlattened.resize( nCount );
	nbrs.vLocalUVs.resize(nCount);
	float fAngleScale = (2.0f * Wml::Mathf::PI) / fAngleSum;
	float fTotalAngle = 0.0f;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		fTotalAngle += (vAngle[i] * fAngleScale);
		Wml::Vector2f vUV( vRadius[i] * cos( fTotalAngle ), vRadius[i] * sin( fTotalAngle ) );
		nbrs.vFlattened[i] = vUV;
		nbrs.vLocalUVs[i] = vUV;
	}

	// ok now find edge intersections
	nbrs.vIntersectEdge.resize(nCount, std::numeric_limits<unsigned int>::max() );
	nbrs.vIntersect2D.resize(nCount);
	nbrs.vIntersect3D.resize(nCount);
	for ( unsigned int i = 0; i < nCount; ++i ) {

		// [RMS] disabled because Wml::IntrLinComp2LinComp2d seems to be missing...
		//Wml::Vector2d Origin(nbrs.vFlattened[i].X(), nbrs.vFlattened[i].Y());
		//Wml::Vector2d Direction(Wml::Vector2d::ZERO - Origin);
		//Direction.Normalize();
		//Wml::LinComp2d ray(Origin, Direction, 0.0f, std::numeric_limits<double>::max() );

		// now try to intersect with other edges...
		bool bFoundHit = false;
		for ( unsigned int j = 0; j < nCount; ++j ) {
			if ( j == i || (j+1)%nCount == i )
				continue;

			//Origin = Wml::Vector2d(nbrs.vFlattened[j].X(), nbrs.vFlattened[j].Y());
			//Direction = Wml::Vector2d( nbrs.vFlattened[(j+1)%nCount].X(), nbrs.vFlattened[(j+1)%nCount].Y() );
			//Direction -= Origin;
			//double dLength = Direction.Normalize();
			//Wml::LinComp2d seg(Origin, Direction, 0.0f, dLength);

			nbrs.vIntersectEdge[i] = -1;
			nbrs.vIntersect2D[i] = Wml::Vector2f::ZERO;
			nbrs.vIntersect3D[i] = Wml::Vector3f::ZERO;

			//Wml::IntrLinComp2LinComp2d isect(ray, seg);
			//bool bIsect = isect.Find();
			//if ( bIsect && isect.GetIntersectionType() == Wml::LinCompd::CT_POINT ) {
			//	bFoundHit = true;
			//	nbrs.vIntersectEdge[i] = j;
			//	Wml::Vector2d vHit( ray.Origin + isect.GetParameter0() * ray.Direction );

			//	Wml::Vector2d vHitVec( vHit - Wml::Vector2d::ZERO );
			//	vHitVec.Normalize();
			//	double fDot = vHitVec.Dot( ray.Direction );
			//	if ( fDot < 0.99 )
			//		_RMSInfo("Weird! dot is %f\n", fDot);

			//	nbrs.vIntersect2D[i] = Wml::Vector2f( (float)vHit.X(), (float)vHit.Y() );

			//	float fEdgeLength = (float)dLength;
			//	float fPointLength = (float)( vHit - Origin ).Length();
			//	float fT = fPointLength / fEdgeLength;

			//	Wml::Vector3f vA, vB;
			//	m_pMesh->GetVertex( vOneRing[j], vA );
			//	m_pMesh->GetVertex( vOneRing[(j+1)%nCount], vB );
			//	nbrs.vIntersect3D[i] = vA  + ((vB - vA) * fT);

			//	//NEED TO FIND ZERO POINTS HERE (in 3D triangle!)
			//	//HINT: it is on the 3D edge from 3D vertex i to ring.vIntersectionPoint3D
			// //    **OR** maybe we don't need this. Do they use the real v_i in the paper?
			//}

		}
		//if (!bFoundHit && ! m_pMesh->IsBoundaryVertex(vID))
		//	_RMSInfo("Could not find ring edge intersection for neighbour %d\n", nbrs.vNbrs[i]);
	}

	// now find interior angles
	nbrs.vAngles.resize(nCount);
	for ( unsigned int i = 0; i < nCount; ++i ) {

		unsigned int nCur = i;
		unsigned int nNext = (i+1) % nCount;
		unsigned int nPrev = (i == 0) ? (int)nCount-1 : (i-1) % (int)nCount;

		// grab vertices from mesh and pre-compute normalized vectors between them
		Wml::Vector3f vCur, vPrev, vNext;
		m_pMesh->GetVertex( nbrs.vNbrs[nCur], vCur );
		m_pMesh->GetVertex( nbrs.vNbrs[nNext], vNext );
		m_pMesh->GetVertex( nbrs.vNbrs[nPrev], vPrev );

		Wml::Vector3f vCurMid( vMiddle - vCur );		vCurMid.Normalize();

		Wml::Vector3f vNextMid( vMiddle - vNext );		vNextMid.Normalize();
		Wml::Vector3f vPrevMid( vMiddle - vPrev );		vPrevMid.Normalize();
		Wml::Vector3f vCurNext( vNext - vCur );			vCurNext.Normalize();
		Wml::Vector3f vCurPrev( vPrev - vCur );			vCurPrev.Normalize();
		
		// now find angles
		VertexAngles & angles = nbrs.vAngles[i];

		angles.fDistSquared = (vMiddle - vCur).SquaredLength();

		angles.fAlpha = rms::VectorCot( vNextMid, -vCurNext );
		angles.fBeta = rms::VectorCot( vPrevMid, -vCurPrev );
		angles.fDelta = rms::VectorCot( vCurMid, vCurNext );
		angles.fGamma = rms::VectorCot( vCurMid, vCurPrev );

		if (!_finite(angles.fAlpha) || !_finite(angles.fBeta) || !_finite(angles.fDelta) || !_finite(angles.fGamma))
			_RMSInfo("Bad Angles!\n");

	}


	// find boundary angles (only used for boundary verts in desbrun discrete conformal param)
	nbrs.vTriAngles.resize( 0 );
	rms::VFTriangleMesh::VtxNbrItr itr(vID);
	m_pMesh->BeginVtxTriangles(itr);
	rms::IMesh::TriangleID tID = m_pMesh->GetNextVtxTriangle(itr);
	while ( tID != rms::IMesh::InvalidID ) {
		
		rms::IMesh::VertexID nTriangle[3];
		m_pMesh->GetTriangle( tID, nTriangle );
		Wml::Vector3f vTriVerts[3];
		m_pMesh->GetTriangle( tID, vTriVerts );

		// find 'other' vertices (not nVertex)
		rms::IMesh::VertexID vJ, vK;
		for ( int j = 0; j < 3; ++j ) {
			if ( nTriangle[j] == vID ) {
				vJ = nTriangle[(j+1)%3];
				vK = nTriangle[(j+2)%3];
				break;
			}
		}

		// figure out if we are CCW or not
		Wml::Vector3f vMiddle, vNormal, vA, vB;
		m_pMesh->GetVertex( vID, vMiddle, &vNormal );
		m_pMesh->GetVertex( vJ, vA );			m_pMesh->GetVertex( vK, vB );
		Wml::Vector3f vEdge1( vA - vMiddle );	Wml::Vector3f vEdge2( vB - vMiddle );
		Wml::Vector3f vCross( vEdge1.Cross(vEdge2) );
		bool bReverse = vCross.Dot(vNormal) < 0.0f;

		vEdge1.Normalize();		vEdge2.Normalize();
		float fDot = vEdge1.Dot(vEdge2);
		double fMidAngle = acos( rms::Clamp((double)fDot,-1.0,1.0) );
		if ( ! _finite(fMidAngle) )
			lgBreakToDebugger();

		// if not, swap v1/v2, vA/vB, and vEdge1/vEdge2
		if ( bReverse ) {
			rms::IMesh::VertexID nTmp = vJ; vJ = vK; vK = nTmp;
			Wml::Vector3f vTmp = vA; vA = vB; vB = vTmp;
			vTmp = vEdge1; vEdge1 = vEdge2; vEdge2 = vTmp;
		}

		// ok now compute angles
		Wml::Vector3f vAB( vB - vA );		vAB.Normalize();

		float fDotAlpha = vAB.Dot( -vEdge1 );
		double fAlpha = acos( rms::Clamp((double)fDotAlpha, -1.0, 1.0) );
		float fDotBeta = (-vAB).Dot( -vEdge2);
		double fBeta = acos( rms::Clamp((double)fDotBeta, -1.0, 1.0) );
		if ( ! _finite(fAlpha) || ! _finite(fBeta) )
			lgBreakToDebugger();

		double fAngleSum = fAlpha + fBeta + fMidAngle;
		if ( fabs( fAngleSum - Wml::Mathf::PI ) > 0.001f )
			_RMSInfo("Bad angles for boundary vertex %d - sum is %f\n", vID, fAngleSum);

		// store TriAngle
		TriangleAngles tri;
		tri.nJ = vJ;
		tri.nK = vK;
		tri.fAlpha = (float)(1.0 / tan(fAlpha));		// store cot's (only need them)
		tri.fBeta = (float)(1.0 / tan(fBeta));
		nbrs.vTriAngles.push_back( tri );

		tID = m_pMesh->GetNextVtxTriangle(itr);
	}



	// always use all one-ring nbrs
	nbrs.nUseNbrs = (int)nbrs.vNbrs.size();
}


struct ExpMapNbr {
	rms::IMesh::VertexID vID;
	Wml::Vector2f vUV;
	float fDist;
	bool operator<( const ExpMapNbr & n2 ) const {
		return fDist < n2.fDist;
	}
};

void PlanarParameterization::MakeGeoDistNeighbourSet( rms::IMesh::VertexID vID, NeighbourSet & nbrs )
{
	nbrs.Clear();
	nbrs.vID = vID;

	// compute expmap
	Wml::Vector3f vVtx, vNormal;
	m_pMesh->GetVertex( vID, vVtx, &vNormal );
	rms::Frame3f vFrame(vVtx);
	vFrame.AlignZAxis(vNormal);
	if ( m_bUseFixedGeoNbrhoodSize ) {
		m_pExpMap->SetSurfaceDistances( vFrame, 0.0f, m_nCurMaxGeoNbrs + 1 );
	} else {
		m_pExpMap->SetSurfaceDistances( vFrame.Origin(), 0.0f, m_fGeoNbrDistance, &vFrame );
	}


	// copy neighbour set from expmap
	if ( ! m_pMesh->HasUVSet(0) ) {
		rms::IMesh::UVSetID nSetID = m_pMesh->AppendUVSet();
		if ( nSetID != 0)  
			lgBreakToDebugger();
		m_pMesh->InitializeUVSet(0);
	}
	m_pExpMap->CopyVertexUVs(m_pMesh, 0);
	rms::IMesh::UVSet & uvs = m_pMesh->GetUVSet(0);
	rms::VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	std::vector<ExpMapNbr> vNbrList;
	while ( curv != endv ) {
		rms::IMesh::VertexID vNbrID = *curv; ++curv;
		if ( vNbrID != vID && uvs.HasUV(vNbrID) ) {
			ExpMapNbr nbr;
			nbr.vID = vNbrID;
			uvs.GetUV(vNbrID, nbr.vUV );
			nbr.fDist = nbr.vUV.Length();
			if ( nbr.fDist > 0 )
				vNbrList.push_back(nbr);
		}
	}
	std::sort(vNbrList.begin(), vNbrList.end());

	if ( m_bUseFixedGeoNbrhoodSize ) {
		for ( unsigned int i = 0; i < m_nCurMaxGeoNbrs; ++i ) {
			nbrs.vNbrs.push_back( vNbrList[i].vID );
		}
	} else {
		size_t nCount = vNbrList.size();
		for ( unsigned int i = 0; i < nCount; ++i )
			nbrs.vNbrs.push_back( vNbrList[i].vID );
	}
	size_t nCount = nbrs.vNbrs.size();

	// set expmap UV info
	nbrs.vLocalUVs.resize(nCount);
	for ( unsigned int i = 0; i < nCount; ++i ) {
		Wml::Vector2f vUV;
		if ( ! m_pMesh->GetUV( nbrs.vNbrs[i], 0, vUV ) )
			lgBreakToDebugger();
		nbrs.vLocalUVs[i] = vUV;
	}
		
	// set 3D distances
	nbrs.fDistances3D.resize(nCount);
	m_pMesh->GetVertex( nbrs.vID, vVtx );
	for ( unsigned int i = 0; i < nCount; ++i ) {
		Wml::Vector3f vNbrVtx;
		m_pMesh->GetVertex( nbrs.vNbrs[i], vNbrVtx );
		nbrs.fDistances3D[i] = (vVtx - vNbrVtx).Length();
	}

	// update current min distance
	float fMaxDistance = nbrs.vLocalUVs.back().Length();
	if ( m_fCurMinMaxGeoNbrDistance == 0 || fMaxDistance < m_fCurMinMaxGeoNbrDistance )
		m_fCurMinMaxGeoNbrDistance = fMaxDistance;

	// use all nbrs by default...
	nbrs.nUseNbrs = (int)nbrs.vNbrs.size();
}


void PlanarParameterization::SetUseFixedGeodesicNbrhood(bool bUseFixed)
{
	m_bUseFixedGeoNbrhoodSize = bUseFixed;
	if ( bUseFixed )
		SetGeodesicNbrhoodSize( m_nGeoNbrhoodSize );
	else
		SetGeodesicNbrhoodRadiusSize( m_fGeoNbrDistance );
}


void PlanarParameterization::SetGeodesicNbrhoodSize( int nNbrs )
{
	if ( nNbrs > 3 ) {
		m_nGeoNbrhoodSize = nNbrs;
		if ( m_nGeoNbrhoodSize >= m_nCurMaxGeoNbrs )
			InvalidateGeodesicNbrhoods();
	}
}

float PlanarParameterization::SuggestGeodesicNbrhoodRadiusSize()
{
	float fMin, fMax, fAvg;
	m_pMesh->GetEdgeLengthStats(fMin, fMax, fAvg);
	return fMax * 2.0f;
}

void PlanarParameterization::SetGeodesicNbrhoodRadiusSize(float fRadius)
{
	if ( fRadius > 0 ) {
		m_fGeoNbrDistance = fRadius;
		if ( m_fGeoNbrDistance > m_fCurMinMaxGeoNbrDistance )
			InvalidateGeodesicNbrhoods();
	}
}

void PlanarParameterization::ValidateGeodesicNbrhoods()
{
	if ( m_bGeodesicNbrhoodsValid == false ) {

		m_nCurMaxGeoNbrs = std::max( (int)30, (int)((m_nGeoNbrhoodSize * 3) / 2) );
		m_fCurMinMaxGeoNbrDistance = 0.0f;

		rms::VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
		int nAvgCount = 0;
		while ( curv != endv ) {
			rms::IMesh::VertexID vID = *curv; ++curv;

			MeshVertex & v = m_vVertInfo[ m_vVertMap[vID] ];
			MakeGeoDistNeighbourSet(vID, v.m_ExpMapNbrs);
			nAvgCount += v.m_ExpMapNbrs.nUseNbrs;
		}

		_RMSInfo("Average neighbourhood size: %d\n", nAvgCount / m_vVertInfo.size() );

		m_bGeodesicNbrhoodsValid = true;
	}

	// update nUseNbrs values
	if ( m_bUseFixedGeoNbrhoodSize ) {
		rms::VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
		while ( curv != endv ) {
			rms::IMesh::VertexID vID = *curv; ++curv;
			NeighbourSet & nNbrs = m_vVertInfo[ m_vVertMap[vID] ].m_ExpMapNbrs;
			nNbrs.nUseNbrs = m_nGeoNbrhoodSize;
		}
	} else {
		rms::VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
		while ( curv != endv ) {
			rms::IMesh::VertexID vID = *curv; ++curv;
			NeighbourSet & nNbrs = m_vVertInfo[ m_vVertMap[vID] ].m_ExpMapNbrs;
			size_t nMax = nNbrs.vNbrs.size();
			unsigned int nUse = 0;
			float fStop = m_fGeoNbrDistance*m_fGeoNbrDistance;
			while ( nUse < nMax ) {
				if ( nNbrs.vLocalUVs[nUse].SquaredLength() > fStop && nUse > 0)
					break;
				nUse++;
			}
			nNbrs.nUseNbrs = nUse;
		}
	}

}





/*
 * Weight computation
 */




void PlanarParameterization::ComputeWeights_Uniform( NeighbourSet & nbrs )
{
	size_t nCount = nbrs.nUseNbrs;
	nbrs.fWeights.resize(nCount);
	for ( unsigned int i = 0; i < nCount; ++i )
		nbrs.fWeights[i] = 1.0f / (float)nCount;
}

void PlanarParameterization::ComputeWeights_InvDist( NeighbourSet & nbrs )
{
	Wml::Vector3f vCenter, vNbr;
	m_pMesh->GetVertex(nbrs.vID, vCenter);
	size_t nCount = nbrs.nUseNbrs;
	nbrs.fWeights.resize(nCount);
	float fWeightSum = 0.0f;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		m_pMesh->GetVertex( nbrs.vNbrs[i], vNbr );
		float fDist = (vNbr-vCenter).Length();
		if ( fabsf(fDist) < 0.0001f || ! _finite(fDist) )
			lgBreakToDebugger();
		nbrs.fWeights[i] = 1.0f / fDist;
		fWeightSum += nbrs.fWeights[i];
	}

	// normalize
	for ( unsigned int i = 0; i < nCount; ++i )
		nbrs.fWeights[i] /= fWeightSum;
}

void PlanarParameterization::ComputeWeights_ShapePreserving( NeighbourSet & nbrs )
{
	size_t nCount = nbrs.nUseNbrs;
	nbrs.fWeights.resize(0,0); nbrs.fWeights.resize(nCount,0);

	float fWeightSum = 0.0f;
	for  ( unsigned int j = 0; j < nCount; ++j ) {
		unsigned int vJi = j;
		unsigned int vKi = nbrs.vIntersectEdge[j];
		if ( vKi == std::numeric_limits<unsigned int>::max() )
			continue;
		unsigned int vLi = (vKi + 1) % nCount;

		// find barycentric coords
		float fBary[3];
		rms::BarycentricCoords( 
			nbrs.vFlattened[vJi], nbrs.vFlattened[vKi], nbrs.vFlattened[vLi],
			Wml::Vector2f::ZERO, fBary[0], fBary[1], fBary[2] );

		nbrs.fWeights[vJi] += fBary[0];
		nbrs.fWeights[vKi] += fBary[1];
		nbrs.fWeights[vLi] += fBary[2];
		fWeightSum += fBary[0]+fBary[1]+fBary[2];
	}

	// scale weights so that they sum to one
	float fScale = 1.0f / fWeightSum;
	for ( unsigned int i = 0; i < nCount; ++i )
		nbrs.fWeights[i] *= fScale;
}


void PlanarParameterization::ComputeWeights_Geodesic( NeighbourSet & nbrs )
{
	size_t nCount = nbrs.nUseNbrs;
	nbrs.fWeights.resize(0,0); nbrs.fWeights.resize(nCount,0);

	float fWeightSum = 0.0f;
	for  ( unsigned int j = 0; j < nCount; ++j ) {
		unsigned int vJi = j;
		unsigned int vKi = nbrs.vIntersectEdge[j];
		if ( vKi == std::numeric_limits<unsigned int>::max() )
			continue;
		unsigned int vLi = (vKi + 1) % nCount;

		Wml::Vector3f vI, vJ, vK, vL, vJPrime;
		m_pMesh->GetVertex( nbrs.vID, vI );
		m_pMesh->GetVertex( nbrs.vNbrs[ vJi ], vJ );
		m_pMesh->GetVertex( nbrs.vNbrs[ vKi ], vK );
		m_pMesh->GetVertex( nbrs.vNbrs[ vLi ], vL );
		vJPrime = nbrs.vIntersect3D[ j ];

		// compute necessary distances
		float fIJPrime = (vI - vJPrime).Length();
		float fIJ = (vI - vJ).Length();
		float fLJPrime = (vL - vJPrime).Length();
		float fKJPrime = (vK - vJPrime).Length();
		float fKL = (vK - vL).Length();

		// now find coords
		float fWJ = fIJPrime / ( fIJ + fIJPrime );
		float fWK = (1 - fWJ) * ( fLJPrime / fKL );
		float fWL = (1 - fWJ) * ( fKJPrime / fKL );

		nbrs.fWeights[ vJi ] += fWJ;
		nbrs.fWeights[ vKi ] += fWK;
		nbrs.fWeights[ vLi ] += fWL;
		fWeightSum += fWJ + fWK + fWL;
	}

	// scale weights so that they sum to one
	float fScale = 1.0f / fWeightSum;
	for ( unsigned int i = 0; i < nCount; ++i )
		nbrs.fWeights[i] *= fScale;
}

void PlanarParameterization::ComputeWeights_Optimal3D( NeighbourSet & nbrs )
{
	size_t nCount = nbrs.nUseNbrs;
	nbrs.fWeights.resize(nCount,0);

	Wml::GMatrixd sys( (int)nCount, (int)nCount );

	Wml::Vector3f vC, vJ, vK;
	m_pMesh->GetVertex( nbrs.vID, vC );
	for ( unsigned int k = 0; k < nCount; ++k ) {
		for ( unsigned int j = k; j < nCount; ++j ) {
			m_pMesh->GetVertex( nbrs.vNbrs[j], vJ );
			m_pMesh->GetVertex( nbrs.vNbrs[k], vK );
			float cjk = ( vC - vJ ).Dot( vC - vK );
			sys(j,k) = cjk;
			sys(k,j) = cjk;
		}
	}

	// add fraction of identity matrix to improve system
	double fTrace = 0.0f;
	for ( unsigned int k = 0; k < nCount; ++k )
		fTrace += sys(k,k);
	double fDelta = fTrace / 100;
	fDelta /= (double)nCount;
	for ( unsigned int k = 0; k < nCount; ++k )
		sys(k,k) += fDelta;

	std::vector<double> X(nCount,1);
	std::vector<double> RHS(nCount,0);
	Wml::LinearSystemd linsys;
	if ( ! linsys.Solve(sys, &X[0], &RHS[0]) )
		_RMSInfo("Solve failed in PlanarParameterization::ComputeWeights_Optimal2D on vertex %d\n", nbrs.vID );

	// re-scale weights so that they sum to one
	float fWeightSum = 0.0f;
	for ( unsigned int i = 0; i < nCount; ++i )
		fWeightSum += (float)RHS[i];
	for ( unsigned int i = 0; i < nCount; ++i )
		nbrs.fWeights[i] = (float)RHS[i] / fWeightSum;
}


void PlanarParameterization::ComputeWeights_Optimal2D( NeighbourSet & nbrs )
{
	size_t nCount = nbrs.nUseNbrs;

	nbrs.fWeights.resize(nCount,0);

	Wml::GMatrixd sys( (int)nCount, (int)nCount );

	Wml::Vector2f vC, vJ, vK;
	Wml::Vector3f vCN, vJN, vKN;
	vC = Wml::Vector2f::ZERO;
	//m_pMesh->GetNormal( nbrs.vID, vCN );
	for ( unsigned int k = 0; k < nCount; ++k ) {
		for ( unsigned int j = k; j < nCount; ++j ) {
			vJ = nbrs.vLocalUVs[j];
			vK = nbrs.vLocalUVs[k];

			float cjk = ( vC - vJ ).Dot( vC - vK );

			sys(j,k) = cjk;
			sys(k,j) = cjk;
		}
	}

	// add fraction of identity matrix to improve system
	double fTrace = 0.0f;
	for ( unsigned int k = 0; k < nCount; ++k )
		fTrace += sys(k,k);
	double fDelta = fTrace / 100;
	fDelta /= (double)nCount;
	for ( unsigned int k = 0; k < nCount; ++k )
		sys(k, k) += fDelta;

	std::vector<double> X(nCount,1);
	std::vector<double> RHS(nCount,0);
	Wml::LinearSystemd linsys;
	if ( ! linsys.Solve(sys, &X[0], &RHS[0]) )
		_RMSInfo("Solve failed in PlanarParameterization::ComputeWeights_Optimal2D on vertex %d\n", nbrs.vID );

	// re-scale weights so that they sum to one
	float fWeightSum = 0.0f;
	for ( unsigned int i = 0; i < nCount; ++i )
		fWeightSum += (float)RHS[i];
	for ( unsigned int i = 0; i < nCount; ++i )
		nbrs.fWeights[i] = (float)RHS[i] / fWeightSum;
}









/*
 * Boundary mapping
 */

// RMS TODO: change this to arbitrary projection plane...
void PlanarParameterization::ProjectBoundaryLoop( BoundaryLoop & loop )
{
	int nCoords[2] = {0,2};
	Wml::Vector3f v;

	// project verts
	size_t nCount = loop.vVerts.size();
	std::vector<Wml::Vector2f> & vUVs = loop.vUVs;
	vUVs.resize(nCount); 
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::IMesh::VertexID vID = loop.vVerts[i];
		m_pMesh->GetVertex(vID, v);
		vUVs[i] = Wml::Vector2f( v[nCoords[0]], v[nCoords[1]] );
	}
}



void PlanarParameterization::MapBoundaryLoopToCircle( BoundaryLoop & loop )
{
	size_t nCount = loop.vVerts.size();

	// find perimeter length
	float fPerimeter = 0.0f;
	Wml::Vector3f v1,v2;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::IMesh::VertexID vID1 = loop.vVerts[i];
		rms::IMesh::VertexID vID2 = loop.vVerts[(i+1)%nCount];
		m_pMesh->GetVertex(vID1, v1);
		m_pMesh->GetVertex(vID2, v2);
		fPerimeter += (v1-v2).Length();
	}
	float fScale = 1.0f / fPerimeter;
	float fRadius = fPerimeter / Wml::Mathf::TWO_PI;

	// map verts to perimeter
	std::vector<Wml::Vector2f> & vUVs = loop.vUVs;
	vUVs.resize(nCount); 
	float fAccumPerimeter = 0.0f;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::IMesh::VertexID vID1 = loop.vVerts[i];
		rms::IMesh::VertexID vID2 = loop.vVerts[(i+1)%nCount];
		m_pMesh->GetVertex(vID1, v1);
		m_pMesh->GetVertex(vID2, v2);
		float fAngle = fAccumPerimeter * (float)fScale * Wml::Mathf::TWO_PI;
		fAccumPerimeter += (v1-v2).Length();

		//float fOffset = 1.0f + 0.1f * (float)sin(8.0f * fAngle);
		//float fOffset = 1.0f;
		float fOffset = fRadius;

		vUVs[i] = Wml::Vector2f( fOffset*(float)cos(fAngle), fOffset*(float)sin(fAngle) );
	}

	//float fUVPerimeter = 0.0f;
	//Wml::Vector2f uv1, uv2;
	//for ( unsigned int i = 0; i < nCount; ++i ) {
	//	fUVPerimeter += (vUVs[(i+1)%nCount]-vUVs[i]).Length();
	//}
	//_RMSInfo("Original perimeter: %f  UV perimeter: %f\n", fPerimeter, fUVPerimeter );
	

}



void PlanarParameterization::MapBoundaryLoopToUnitSquare( BoundaryLoop & loop )
{
	size_t nCount = loop.vVerts.size();

	// find perimeter length
	float fPerimeter = 0.0f;
	Wml::Vector3f v1,v2;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::IMesh::VertexID vID1 = loop.vVerts[i];
		rms::IMesh::VertexID vID2 = loop.vVerts[(i+1)%nCount];
		m_pMesh->GetVertex(vID1, v1);
		m_pMesh->GetVertex(vID2, v2);
		fPerimeter += (v1-v2).Length();
	}

	// split verts into 4 bins
	float fBinSplit = fPerimeter / 4.0f;
	float fAccum = 0.0f;
	std::vector< std::vector< int > > vBins;
	vBins.resize(4);
	int nBin = 0;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::IMesh::VertexID vID1 = loop.vVerts[i];
		rms::IMesh::VertexID vID2 = loop.vVerts[(i+1)%nCount];
		m_pMesh->GetVertex(vID1, v1);
		m_pMesh->GetVertex(vID2, v2);
		fAccum += (v1-v2).Length();
		vBins[nBin].push_back(i);
		if ( fAccum > fBinSplit ) {
			++nBin;
			if ( nBin >= 4 ) lgBreakToDebugger();
			vBins[nBin].push_back( vBins[nBin-1].back() );
			fAccum = 0;
		}
	}
	vBins[3].push_back( vBins[0].front() );

	// map bins to edges
	const Wml::Vector2f vEdges[4][2] = { 
		{Wml::Vector2f(-1,-1), Wml::Vector2f(2,0)},		{Wml::Vector2f(1,-1), Wml::Vector2f(0,2)},
		{Wml::Vector2f(1,1), Wml::Vector2f(-2,0)},		{Wml::Vector2f(-1,1), Wml::Vector2f(0,-2)} };

	std::vector<Wml::Vector2f> & vUVs = loop.vUVs;
	vUVs.resize(nCount);
	for ( unsigned int bi = 0; bi < 4; ++bi ) {
		float fEdgeLen = 0.0f;
		size_t nBinCount = vBins[bi].size();
		for ( unsigned int i = 0; i < nBinCount-1; ++i ) {
			rms::IMesh::VertexID vID1 = loop.vVerts[ vBins[bi][i] ];
			rms::IMesh::VertexID vID2 = loop.vVerts[ vBins[bi][i+1] ];
			m_pMesh->GetVertex(vID1, v1);
			m_pMesh->GetVertex(vID2, v2);
			fEdgeLen += (v1-v2).Length();
		}
		float fScale = 1.0f / fEdgeLen;
		fEdgeLen = 0;
		for ( unsigned int i = 0; i < nBinCount-1; ++i ) {
			rms::IMesh::VertexID vID1 = loop.vVerts[ vBins[bi][i] ];
			rms::IMesh::VertexID vID2 = loop.vVerts[ vBins[bi][i+1] ];
			m_pMesh->GetVertex(vID1, v1);
			m_pMesh->GetVertex(vID2, v2);
			Wml::Vector2f vUV = vEdges[bi][0] + fEdgeLen * fScale * vEdges[bi][1];
			fEdgeLen += (v1-v2).Length();
			vUVs[ vBins[bi][i] ] = vUV;
		}
	}
}


void PlanarParameterization::InitializeBoundaries_Conformal( std::vector<BoundaryLoop> & vBoundaryLoops )
{
	EmbeddingType eCurType = m_eEmbedType;
	m_eEmbedType = DiscreteNaturalConformal;

	bool bOK = Parameterize_OneRing_Intrinsic();

	size_t nLoops = vBoundaryLoops.size();
	for ( unsigned int li = 0; li < nLoops; ++li ) {
		BoundaryLoop & b = vBoundaryLoops[li];
		size_t nCount = b.vVerts.size();
		b.vUVs.resize(nCount);
		for ( unsigned int i = 0; i < nCount; ++i ) {
			Wml::Vector2f vUV;
			if ( ! m_pMesh->GetUV( m_vVertInfo[b.vVerts[i]].vID, 0, vUV ) )
				lgBreakToDebugger();
			b.vUVs[i] = vUV;
		}
	}

	m_eEmbedType = eCurType;
}


void PlanarParameterization::InitializeBoundaries_LLE( std::vector<BoundaryLoop> & vBoundaryLoops )
{
	EmbeddingType eCurType = m_eEmbedType;
	m_eEmbedType = ExpMapLLE;

	bool bOK = Parameterize_LLE();

	size_t nLoops = vBoundaryLoops.size();
	for ( unsigned int li = 0; li < nLoops; ++li ) {
		BoundaryLoop & b = vBoundaryLoops[li];
		size_t nCount = b.vVerts.size();
		b.vUVs.resize(nCount);
		for ( unsigned int i = 0; i < nCount; ++i ) {
			Wml::Vector2f vUV;
			if ( ! m_pMesh->GetUV( m_vVertInfo[b.vVerts[i]].vID, 0, vUV ) )
				lgBreakToDebugger();
			b.vUVs[i] = vUV;
		}
	}

	m_eEmbedType = eCurType;
}






struct NbrInfo {
	rms::IMesh::VertexID vID;
	float fDist;
	NbrInfo() { fDist = 0; }
	NbrInfo(rms::IMesh::VertexID id, float dist) { vID = id; fDist = dist; }
	bool operator<( const NbrInfo & n2 ) { return fDist < n2.fDist; }
};

void PlanarParameterization::InitializeBoundary_BoundaryLLE( BoundaryLoop & loop )
{
	lgBreakToDebugger();
#if 0
	std::vector<bool> bBoundary(m_pMesh->GetMaxVertexID(), false);
	std::vector<unsigned int> vRewrite( m_pMesh->GetMaxVertexID(), rms::IMesh::InvalidID );
	std::vector<unsigned int> vBoundary;
	size_t nLoopCount = loop.vVerts.size();
	for ( unsigned int i = 0; i < nLoopCount; ++i ) {
		vBoundary.push_back( loop.vVerts[i] );
		vRewrite[ loop.vVerts[i] ] = i;
	}
	size_t nBoundaryCount = vBoundary.size();

	rmssolver::SparseSymmetricEigenSolver solver((unsigned int)nBoundaryCount);

	// find neighbours
	unsigned int K2 = 2;
	std::vector<float> vWeights;
	for ( unsigned int i = 0; i < nBoundaryCount; ++i ) {

		// pick out 2*K nearest nbrs and keep track of boundary distance
		std::vector<NbrInfo> vNbrs;
		float fDistFW = 0.0f;
		float fDistBW = 0.0f;
		Wml::Vector3f vFW[2], vBW[2];
		m_pMesh->GetVertex( vBoundary[i], vFW[0] );
		m_pMesh->GetVertex( vBoundary[i], vBW[0] );
		for ( unsigned int k = 1; k <= K2; ++k ) {
			int nFW = (i+k) % nBoundaryCount;
			m_pMesh->GetVertex( vBoundary[ nFW ], vFW[1] );
			fDistFW += (vFW[0]-vFW[1]).Length();
			vNbrs.push_back( NbrInfo(vBoundary[ nFW ], fDistFW) );
			vFW[0] = vFW[1];

			int nBW = ((int)i-(int)k);  if (nBW < 0) nBW += (int)nBoundaryCount;
			m_pMesh->GetVertex( vBoundary[ nBW ], vBW[1] );
			fDistBW += (vBW[0]-vBW[1]).Length();
			vNbrs.push_back( NbrInfo(vBoundary[ nBW ], fDistBW) );
			vBW[0] = vBW[1];
		}

		size_t nNbrCount = vNbrs.size();
		ParamSystem sys;
		sys.Resize( (unsigned int)nNbrCount );


		// sort row and pick out K nearest
		Wml::Vector3f vC, vJ, vK;
		m_pMesh->GetVertex( vBoundary[i], vC );
		for ( unsigned int k = 0; k < nNbrCount; ++k ) {
			m_pMesh->GetVertex( vNbrs[k].vID, vK );
			for ( unsigned int j = k; j < nNbrCount; ++j ) {
				m_pMesh->GetVertex( vNbrs[j].vID, vJ );
				float cjk = ( vC - vJ ).Dot( vC - vK );
				sys.SetMatrixLAPACK(j, k, cjk);
				sys.SetMatrixLAPACK(k, j, cjk);
			}
		}

		// add fraction of identity matrix to improve system
		double fTrace = 0.0f;
		for ( unsigned int k = 0; k < sys.nDim; ++k )
			fTrace += sys.GetMatrixLAPACK(k,k);
		double fDelta = fTrace / 100;
		fDelta /= (double)sys.nDim;
		for ( unsigned int k = 0; k < sys.nDim; ++k )
			sys.SetMatrixLAPACK(k, k,  sys.GetMatrixLAPACK(k,k) + fDelta);

		for ( unsigned int k = 0; k < sys.nDim; ++k )
			sys.SetRHS(k, 1);

		bool bResult = SolveDirect(sys);
		if (! bResult )
			_RMSInfo("SolveDirect failed in PlanarParameterization::Compute_LLE_Boundary on vertex %d\n", i );

		// re-scale weights so that they sum to one
		vWeights.resize(nNbrCount);
		float fWeightSum = 0.0f;
		for ( unsigned int j = 0; j < sys.nDim; ++j )
			fWeightSum += (float)sys.vSolution[j];
		for ( unsigned int j = 0; j < sys.nDim; ++j )
			vWeights[j] = (float)sys.vSolution[j] / fWeightSum;

		_RMSInfo("Nbrs: ");
		for ( unsigned int j = 0; j < nNbrCount; ++j ) 
			_RMSInfo("%d/%f/%f ", vNbrs[j].vID, vNbrs[j].fDist, vWeights[j]);
		_RMSInfo("\n");


		for ( unsigned int j = 0; j < nNbrCount; ++j ) {
			rms::IMesh::VertexID vNbrID = vNbrs[j].vID;
			solver.Set(i, vRewrite[ vNbrID ], vWeights[j]);
		}
	}

	// set solver flags
	solver.SetRightMultiplyMode( rmssolver::SparseSymmetricEigenSolver::LLE );
	solver.SetSolverMode( rmssolver::SparseSymmetricEigenSolver::PRIMME_JDQMR );

	// solve eigensystem
	int nNumEigens = 4;
	solver.SetNumEigens(nNumEigens);
	bool bResult = solver.Solve();
	if ( ! bResult )
		lgBreakToDebugger();

	// get solver statistics
	double fTolerance; int nIterations; int nRestarts;
	int nMatVecMuls; int nPreconditions; int nRecommender;
	solver.GetStats( fTolerance, nIterations, nRestarts, nMatVecMuls, nPreconditions, nRecommender );
	_RMSInfo("LLE Solve - %d vertices: \n", nBoundaryCount);
	_RMSInfo("Tolerance: %-22.15E\n", fTolerance);
	_RMSInfo("Iterations: %d  Restarts: %d  MatVecs: %d  Preconditions: %d  Recommend: %d\n",
		nIterations, nRestarts, nMatVecMuls, nPreconditions, nRecommender );

	std::vector< double > eigenVals;
	std::vector< std::vector<double> > eigenVecs;
	eigenVecs.resize(nNumEigens);
	for ( int i = 0; i < nNumEigens; ++i )
		eigenVecs[i].resize(nBoundaryCount);
	for ( unsigned int i = 0; i < (unsigned int)nNumEigens; ++i ) {
		eigenVals.push_back( solver.GetEigenValue(i) );
		solver.GetEigenVector(i, &eigenVecs[i][0] );
	}

	_RMSInfo("Top eigenvalues are %.12f %.12f %.12f %.12f\n", 
		eigenVals[0], eigenVals[1], eigenVals[2], eigenVals[3] );
	//double k = 100000.0;
	//for ( int j = 1; j <= 3; ++j )
	//	eigenVals[j] *= k;

	if (! m_pMesh->HasUVSet(0) ) {
		m_pMesh->AppendUVSet();
		m_pMesh->InitializeUVSet(0);
	}
	m_pMesh->ClearUVSet(0);

	// find bounds for param
	Wml::AxisAlignedBox2f bounds(9999999.0f,-9999999.0f,9999999.0f,-9999999.0f);
	for ( unsigned int i = 0; i < nBoundaryCount; ++i ) {
		double d1 = eigenVecs[1][i];
		double d2 = eigenVecs[2][i];
		Wml::Vector2f vUV( (float)d1, (float)d2 );
		rms::Union(bounds,vUV);
	}
	float fMaxDim = std::max( bounds.Max[0]-bounds.Min[0], bounds.Max[1]-bounds.Min[1] );
	float fScale = 2.0f / fMaxDim;
	Wml::Vector2f vCenter( 0.5f * (bounds.Max[0]+bounds.Min[0]), 0.5f * (bounds.Max[1]+bounds.Min[1]) );

	
	loop.vUVs.resize( nLoopCount );
	for ( unsigned int i = 0; i < nLoopCount; ++i ) {
		double d1 = eigenVecs[1][i];
		double d2 = eigenVecs[2][i];
		double d3 = eigenVecs[3][i];
		Wml::Vector2f vUV( (float)d1, (float)d2 );
		vUV -= vCenter;
		vUV *= fScale;
		loop.vUVs[i] = vUV;
	}
#endif
}









float PlanarParameterization::GetReconstructionError_3D( NeighbourSet & nbrs )
{
	Wml::Vector3f vSum( Wml::Vector3f::ZERO );
	size_t nCount = nbrs.nUseNbrs;
	Wml::Vector3f vV;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		m_pMesh->GetVertex( nbrs.vNbrs[i], vV );
		vSum += (vV * nbrs.fWeights[i]);
	}
	m_pMesh->GetVertex( nbrs.vID, vV );
	return (vSum - vV).Length();
}


float PlanarParameterization::GetReconstructionError_2D( NeighbourSet & nbrs )
{
	Wml::Vector2f vSum( Wml::Vector2f::ZERO );
	size_t nCount = nbrs.vLocalUVs.size();
	Wml::Vector3f vV;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		vSum += (nbrs.vLocalUVs[i] * nbrs.fWeights[i]);
	}
	return vSum.Length();
}





bool PlanarParameterization::Solve_FixBoundary( std::vector<double> & vU, std::vector<double> & vV, NeighbourhoodType eNbrType)
{
	gsi::SparseLinearSystem p;
	size_t nCount = m_vVertInfo.size();
	p.Resize((unsigned int)nCount, (unsigned int)nCount);
	//p.ResizeRHS(2);
	p.ResizeRHS(1);

	gsi::SparseLinearSystem p2;
	p2.Resize((unsigned int)nCount, (unsigned int)nCount);
	//p.ResizeRHS(2);
	p2.ResizeRHS(1);

	for ( unsigned int i = 0; i < nCount; ++i ) {
	
		p.Set(i,i, 1.0);
		p2.Set(i,i, 1.0);

		NeighbourSet & nbrs = m_vVertInfo[i].GetNeighbourSet(eNbrType);
		size_t nNbrs = nbrs.nUseNbrs;
		for ( unsigned int j = 0; j < nNbrs; ++j ) {
			unsigned int nNbrJ = m_vVertMap[nbrs.vNbrs[j]];
			p.Set(i, nNbrJ, -nbrs.fWeights[j]);
			p2.Set(i, nNbrJ, -nbrs.fWeights[j]);
		}

		p.SetRHS(i, 0, 0);
		//p.SetRHS(i, 0, 1);
		p2.SetRHS(i, 0, 0);
	}

	size_t nBdry = m_boundaryInfo.vBoundaryLoops.size();
	for ( unsigned int i = 0; i < nBdry; ++i ) {
		BoundaryLoop & loop = m_boundaryInfo.vBoundaryLoops[i];
		size_t nLoopCount = loop.vVerts.size();
		for ( unsigned int j = 0; j < nLoopCount; ++j ) {
			int r = loop.vVerts[j];
			
			p.Matrix().ClearRow(r);
			p.Set(r,r, 1.0);
			p.SetRHS(r, loop.vUVs[j].X(), 0);
			//p.SetRHS(r, loop.vUVs[j].Y(), 1);

			p2.Matrix().ClearRow(r);
			p2.Set(r,r, 1.0);
			p2.SetRHS(r, loop.vUVs[j].Y(), 0);
		}
	}

	gsi::Solver_UMFPACK solver(&p);
	bool bOK = solver.Solve();
	if (! bOK )
		return false;
	gsi::Solver_UMFPACK solver2(&p2);
	bOK = solver2.Solve();
	if (! bOK )
		return false;

	vU.resize(nCount);  vV.resize(nCount);
	for ( unsigned int k = 0; k < nCount; ++k ) {
		vU[k] = p.GetSolution(0)[k];
		//vV[k] = p.GetSolution(1)[k];
		vV[k] = p2.GetSolution(0)[k];
	}
	return true;
}





