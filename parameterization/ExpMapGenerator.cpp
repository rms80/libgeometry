// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "ExpMapGenerator.h"
#include "rmsprofile.h"
#include "rmsdebug.h"
#include <limits>
#include <list>
#include <set>
#include <algorithm>

#include "VectorUtil.h"
#include "Triangulator2D.h"
#include "MeshUtils.h"

using namespace rms;


Wml::Vector2f ExpMapParticle::INVALID_PARAM = Wml::Vector2f( std::numeric_limits<float>::max(), std::numeric_limits<float>::max() );

ExpMapParticle::ExpMapParticle(const Wml::Vector3f & vPosition,
							   const Wml::Vector3f & vNormal)
							   : m_vPosition(vPosition), m_vNormal(vNormal)
{
	m_fSurfaceDistance = 0.0f;
	m_vSurfaceVector = Wml::Vector2f::ZERO;
	ClearNeighbourList();
	m_nVertexID = std::numeric_limits<unsigned int>::max();

	m_eState = ExpMapParticle::Inactive;
	m_pNearest = NULL;
	m_bNbrFlag = false;
	m_bNormalSmoothed = false;
}

ExpMapParticle::~ExpMapParticle()
{
	ClearNeighbourList();
}

void ExpMapParticle::AddNeighbour( ListEntry * pNeighbour )
{
	if ( m_pNeighbourList == NULL ) {
		m_pNeighbourList = pNeighbour;
		pNeighbour->pNext = NULL;
	} else {
		pNeighbour->pNext = m_pNeighbourList;
		m_pNeighbourList = pNeighbour;
	}
}

void ExpMapParticle::ClearNeighbourList()
{
	// cleared using memory pool in particle system!
	m_pNeighbourList = NULL;
}




/*
 * ExpMapGenerator
 */


ExpMapGenerator::ExpMapGenerator()
{
	m_bNeighbourListsValid = false;
	m_bParticleGridValid = false;
	m_fMaxNbrDist = 0.0f;
	m_pSeedParticle = NULL;

	m_pVFMesh = NULL;
	m_pIMesh = NULL;
	m_pMeshBVTree = NULL;

	m_bPrecomputeNeighbours = true;
	m_bUseMeshNeighbours = true;
	m_bUseUpwindAveraging = false;
	m_bUseNeighbourNormalSmoothing = false;

	m_bEnableSquareCulling = false;
	m_bUseClipPoly = false;
	m_fLastMaxRadius = 0.0f;
}

ExpMapGenerator::~ExpMapGenerator()
{
}




void ExpMapGenerator::EstimateEdgeLength(VFTriangleMesh * pMesh, float & fMin, float & fMax, float & fAverage)
{
	fMin = std::numeric_limits<float>::max();
	fMax = std::numeric_limits<float>::min();

	// randomly sample K-nearest distances for D points
	unsigned int D = (unsigned int)(log((float)pMesh->GetMaxVertexID()) * 10.0f);
	int K = 8;
	std::set<IMesh::VertexID> vIDs;
	while ( vIDs.size() < D )
		vIDs.insert( rand() % pMesh->GetMaxVertexID() );

	// list of distances
	std::vector<float> vDistances(pMesh->GetMaxVertexID(), std::numeric_limits<float>::max());

	int nEdges = 0;
	double fEdgeLengths = 0;

	std::set<IMesh::VertexID>::iterator cursv(vIDs.begin()), endsv(vIDs.end());
	while ( cursv != endsv ) {
		IMesh::VertexID vID = *cursv++;
		Wml::Vector3f vVtx;
		pMesh->GetVertex(vID, vVtx);

		// compute distance to all points
		VFTriangleMesh::vertex_iterator curv(pMesh->BeginVertices()), endv(pMesh->EndVertices());
		while ( curv != endv ) {
			IMesh::VertexID vNbr = *curv; ++curv;
			if ( vNbr == vID ) 
				vDistances[vNbr] = std::numeric_limits<float>::max();
			 else 
				vDistances[vNbr] = ( vVtx - Wml::Vector3f(pMesh->GetVertex(vNbr)) ).SquaredLength();
		}

		std::sort(vDistances.begin(), vDistances.end());
		for ( int k = 0; k < K; ++k ) {
			if ( vDistances[k] != std::numeric_limits<float>::max() ) {
				nEdges++;
				double fDist = sqrt(vDistances[k]);
				fEdgeLengths += fDist;
				if ( fDist > fMax )
					fMax = (float)fDist;
				if ( fDist < fMin )
					fMin = (float)fDist;
			}
		}
	}

	fAverage = (float)(fEdgeLengths / (double)nEdges);
	


}


void ExpMapGenerator::SetSurface( IMesh * pMesh, IMeshBVTree * pMeshBVTree )
{
	m_pVFMesh = NULL;
	m_pIMesh = pMesh;
	m_pMeshBVTree = pMeshBVTree;

	float fMin = 0, fAverage = 0;
	MeshUtils::GetEdgeLengthStats(pMesh, fMin, m_fMaxEdgeLength, m_fAvgEdgeLength);
	if ( m_fMaxEdgeLength == 0 )
		lgBreakToDebugger();

	Reset();

	unsigned int nMaxVert = pMesh->GetMaxVertexID();
	m_vVtxToParticleMap.resize(0);
	m_vVtxToParticleMap.resize( nMaxVert, IMesh::InvalidID );

	IMesh::IVtxIterator curv(pMesh->BeginIVertices()), endv(pMesh->EndIVertices());
	while ( curv != endv ) {
		IMesh::VertexID nID = *curv;	curv++;

		// create particle
		ExpMapParticle * p = AllocateParticle();
		pMesh->GetVertex( nID, p->Position(), & p->Normal() );

		// associate w/ vertex
		p->VertexID() = nID;
		m_vVtxToParticleMap[nID] = AddParticle(p);

		// set arbitrary surface frame
		p->WorldFrame() = Frame3f( p->Position(), p->Normal() );
	}

	if ( m_bUseNeighbourNormalSmoothing )
		SetUseNeighbourNormalSmoothing(true);
}




void ExpMapGenerator::SetSurface( VFTriangleMesh * pMesh, IMeshBVTree * pMeshBVTree )
{
	m_pVFMesh = pMesh;
	m_pIMesh = NULL;
	m_pMeshBVTree = pMeshBVTree;

	float fMin = 0, fMax = 0, fAverage = 0;
	if ( pMesh->GetTriangleCount() == 0 )		// happens if we are using mesh as point set
		//EstimateEdgeLength(pMesh, fMin, m_fMaxMeshEdgeLength, fAverage);
		EstimateEdgeLength(pMesh, fMin, m_fMaxEdgeLength, m_fAvgEdgeLength);
	else {
		pMesh->GetEdgeLengthStats(fMin, m_fMaxEdgeLength, m_fAvgEdgeLength);
		//m_fMaxMeshEdgeLength = 0.0676263f;
	}

	Reset();

	unsigned int nMaxVert = pMesh->GetMaxVertexID();
	m_vVtxToParticleMap.resize(0);
	m_vVtxToParticleMap.resize( nMaxVert, IMesh::InvalidID );

	IMesh::IVtxIterator curv(pMesh->BeginIVertices()), endv(pMesh->EndIVertices());
	while ( curv != endv ) {
		IMesh::VertexID nID = *curv;	curv++;

		// create particle
		ExpMapParticle * p = AllocateParticle();
		pMesh->GetVertex( nID, p->Position(), & p->Normal() );

		// associate w/ vertex
		p->VertexID() = nID;
		m_vVtxToParticleMap[nID] = AddParticle(p);

		// set arbitrary surface frame
		p->WorldFrame() = Frame3f( p->Position(), p->Normal() );
	}

	if ( m_bUseNeighbourNormalSmoothing )
		SetUseNeighbourNormalSmoothing(true);
}


IMesh * ExpMapGenerator::GetMesh()
{
	if ( m_pIMesh )
		return m_pIMesh;
	else
		return m_pVFMesh;
}



void ExpMapGenerator::Reset()
{
	m_bParticleGridValid = false;
	ClearNeighbourLists();
	ClearParticles();
	m_vLastParticles.resize(0);

	m_uvMesh.Clear(false);
	m_3dMesh.Clear(false);
}




void ExpMapGenerator::SetUseNeighbourNormalSmoothing( bool bEnable )
{
	m_bUseNeighbourNormalSmoothing = bEnable;
}

void ExpMapGenerator::SetSmoothNormal( ExpMapParticle * pParticle, ExpMapParticle::ListEntry * pNbrs, bool bEnable )
{
	if ( bEnable && pParticle->m_bNormalSmoothed )
		return;
	else if ( !bEnable && !pParticle->m_bNormalSmoothed)
		return;

	Wml::Vector3f vPos, vNorm, vNbrPos, vNbrNorm;
	if ( bEnable ) {
		float fWeightSum = 0.0f;
		Wml::Vector3f vAverage = Wml::Vector3f::ZERO;
		GetMesh()->GetVertex( pParticle->VertexID(), vPos, &vNorm);
		ExpMapParticle::ListEntry * pCur = pNbrs;
		while ( pCur != NULL ) {
			IMesh::VertexID nID = pCur->pParticle->VertexID();
			Wml::Vector3f vNbrPos, vNbrNorm;
			GetMesh()->GetVertex( pCur->pParticle->VertexID(), vNbrPos, &vNbrNorm);
			float fWeight = 1.0f / ( (vPos - vNbrPos).Length()  + (0.0001f*m_fMaxEdgeLength) );
			vAverage += fWeight * vNbrNorm;
			fWeightSum += fWeight;
			pCur = pCur->pNext;
		}
		vAverage /= fWeightSum;
		vAverage.Normalize();
		pParticle->Normal() = vAverage;
	} else {
		GetMesh()->GetNormal( pParticle->VertexID(), pParticle->Normal() );
	}

	pParticle->WorldFrame() = Frame3f( pParticle->Position(), pParticle->Normal() );
	pParticle->m_bNormalSmoothed = bEnable;
}



void ExpMapGenerator::CopyVertexUVs( IMesh * pMesh, IMesh::UVSetID nSetID )
{
	pMesh->ClearUVSet( nSetID );

	Wml::Vector2f vTexCoord;
	size_t nLastCount = m_vLastParticles.size();
	for ( unsigned int i = 0; i < nLastCount; ++i ) {
		ExpMapParticle * pParticle = m_vLastParticles[i];
		if ( pParticle->VertexID() != IMesh::InvalidID &&
			pParticle->SurfaceDistance() != std::numeric_limits<float>::max() )
			pMesh->SetUV( pParticle->VertexID(), nSetID, pParticle->SurfaceVector() );
	}
}


void ExpMapGenerator::GetVertexUVs(std::vector<unsigned int> & vID, std::vector<float> & vU, std::vector<float> & vV)
{
	size_t nLastCount = m_vLastParticles.size();
	for ( unsigned int i = 0; i < nLastCount; ++i ) {
		ExpMapParticle * pParticle = m_vLastParticles[i];
		if ( pParticle->VertexID() != IMesh::InvalidID &&
			pParticle->SurfaceDistance() != std::numeric_limits<float>::max() ) {
				vID.push_back( pParticle->VertexID() );
				vU.push_back( pParticle->SurfaceVector().X() );
				vV.push_back( pParticle->SurfaceVector().Y() );
		}
	}
}


void ExpMapGenerator::GetVertexFaceUVs( std::vector<unsigned int> & vIDs, std::vector<float> & vU, std::vector<float> & vV, std::vector<unsigned int> & vFaces, rms::VFTriangleMesh * pMesh )
{
	SparseArray<IMesh::TriangleID> vTris;
	vTris.resize(pMesh->GetMaxTriangleID());

	// mark triangles that have at least one vertex UV set
	size_t nLastCount = m_vLastParticles.size();
	for ( unsigned int k = 0; k < nLastCount; ++k ) {
		ExpMapParticle * pParticle = m_vLastParticles[k];
		if ( pParticle->VertexID() == IMesh::InvalidID || pParticle->SurfaceDistance() == std::numeric_limits<float>::max() )
			continue;
		IMesh::VtxNbrItr itr(pParticle->VertexID());
		pMesh->BeginVtxTriangles(itr);
		IMesh::TriangleID tID = pMesh->GetNextVtxTriangle(itr);
		while ( tID != IMesh::InvalidID ) {
			vTris.set(tID, tID);
			tID = pMesh->GetNextVtxTriangle(itr);
		}
	}

	// generate list of triangles with all vertex UVs set, and mark vertices
	SparseArray<IMesh::VertexID> vVerts;
	vVerts.resize(pMesh->GetMaxVertexID());
	SparseArray<IMesh::TriangleID>::iterator curf(vTris.begin()), endf(vTris.end());
	while ( curf != endf ) {
		IMesh::TriangleID tID = curf.index(); curf++;

		IMesh::VertexID nTri[3];
		pMesh->GetTriangle(tID, nTri);
		bool b1 = m_vParticles[ m_vVtxToParticleMap[nTri[0]] ]->SurfaceDistance() != std::numeric_limits<float>::max();
		bool b2 = m_vParticles[ m_vVtxToParticleMap[nTri[1]] ]->SurfaceDistance() != std::numeric_limits<float>::max();
		bool b3 = m_vParticles[ m_vVtxToParticleMap[nTri[2]] ]->SurfaceDistance() != std::numeric_limits<float>::max();
		if ( b1 && b2 && b3 ) {
			vVerts.set(nTri[0], nTri[0]);
			vVerts.set(nTri[2], nTri[1]);
			vVerts.set(nTri[1], nTri[2]);
			vFaces.push_back(tID);
		} 
	}

	// generate list of vertices and UVs who are contained in at least one triangle
	SparseArray<IMesh::VertexID>::iterator curv(vVerts.begin()), endv(vVerts.end());
	while ( curv != endv ) {
		IMesh::VertexID vID = curv.index();  ++curv;
		int nIndex = m_vVtxToParticleMap[vID];
		vIDs.push_back(vID);
		vU.push_back( m_vParticles[nIndex]->SurfaceVector().X() );
		vV.push_back( m_vParticles[nIndex]->SurfaceVector().Y() );
	}
}



bool ExpMapGenerator::MeshCurrentUVs()
{
	if ( m_vLastParticles.empty() )
		return false;

	m_uvMesh.Clear(false);
	m_3dMesh.Clear(false);

	Triangulator2D dt;

	size_t nLastCount = m_vLastParticles.size();
	for ( unsigned int i = 0; i < nLastCount; ++i ) {
		ExpMapParticle * pParticle = m_vLastParticles[i];
		if ( pParticle->VertexID() != IMesh::InvalidID ) {
			unsigned int nPointID = dt.AddPoint( 
				pParticle->SurfaceVector().X(), pParticle->SurfaceVector().Y(), pParticle->VertexID() );
		}
	}

	dt.Compute();
	dt.SetEnclosingSegmentsProvided(false);
	dt.SetSubdivideOuterSegments(false);
	dt.SetSubdivideAnySegments(false);
	dt.MakeTriMesh(m_uvMesh);

	const std::vector<int> & pointIDs = dt.GetOutputMarkers_MeshVtx();
	size_t nPoints = pointIDs.size();
	if ( m_uvMesh.GetMaxVertexID() != nPoints ) {
		lgBreakToDebugger();
		return false;
	}

	// make 3D version
	m_3dMesh.Copy(m_uvMesh);
	for ( unsigned int i = 0; i < nPoints; ++i ) {
		int nPointID = pointIDs[i];
		IMesh::VertexID vID = nPointID;
		ExpMapParticle * pParticle = m_vParticles[ m_vVtxToParticleMap[vID] ];
		m_3dMesh.SetVertex( i, pParticle->Position(), &pParticle->Normal() );
	}
		
	m_uvBVTree.SetMesh(&m_uvMesh);
	m_3dBVTree.SetMesh(&m_3dMesh);

	return true;
}


Wml::Vector2f ExpMapGenerator::FindUV( const Wml::Vector3f & vPoint, bool * bStatus )
{
	if (bStatus)
		*bStatus = true;

	Wml::Vector3f vNearest;
	IMesh::TriangleID tID;
	if ( ! m_3dBVTree.FindNearest( vPoint, vNearest, tID ) ) {
		if ( bStatus ) {
			*bStatus = false;
			return Wml::Vector2f::ZERO;
		}
		//lgBreakToDebugger();
	}
	Wml::Vector3f vTri[3];
	m_3dMesh.GetTriangle( tID, vTri );

	float fDist = (vPoint - vNearest).Length();
	if ( fDist > 0.1f ) {
		if ( bStatus )
			*bStatus = false;
		//lgBreakToDebugger();
	}

	float fBary[3];
	rms::BarycentricCoords(
		vTri[0], vTri[1], vTri[2], vPoint, fBary[0], fBary[1], fBary[2] );

	m_uvMesh.GetTriangle( tID, vTri );
	return
		fBary[0]*Wml::Vector2f(vTri[0].X(), vTri[0].Y()) + 
		fBary[1]*Wml::Vector2f(vTri[1].X(), vTri[1].Y()) + 
		fBary[2]*Wml::Vector2f(vTri[2].X(), vTri[2].Y());
}


Wml::Vector3f ExpMapGenerator::Find3D( const Wml::Vector2f & vUV, Wml::Vector3f * pNormal, bool * bStatus )
{
	if (bStatus)
		*bStatus = true;

	Wml::Vector3f vPoint(vUV.X(),vUV.Y(), 0.0f);
	Wml::Vector3f vNearest;
	IMesh::TriangleID tID;
	if ( ! m_uvBVTree.FindNearest( vPoint, vNearest, tID ) ) {
		if ( bStatus ) {
			*bStatus = false;
			return Wml::Vector3f::ZERO;
		}
		//lgBreakToDebugger();
	}
	Wml::Vector3f vTri[3], vNorm[3];
	m_uvMesh.GetTriangle( tID, vTri );

	float fDist = (vPoint - vNearest).Length();
	if ( fDist > 0.1f ) {
		if ( bStatus )
			*bStatus = false;
		//lgBreakToDebugger();
	}

	float fBary[3];
	rms::BarycentricCoords(
		Wml::Vector2f(vTri[0].X(), vTri[0].Y()),
		Wml::Vector2f(vTri[1].X(), vTri[1].Y()),
		Wml::Vector2f(vTri[2].X(), vTri[2].Y()),
		vUV, fBary[0], fBary[1], fBary[2] );

	//if ( fBary[0] < 0.0f || fBary[0] > 1.0f ||
	//	 fBary[1] < 0.0f || fBary[1] > 1.0f ||
	//	 fBary[2] < 0.0f || fBary[2] > 1.0f )
	//	 lgBreakToDebugger();

	m_3dMesh.GetTriangle( tID, vTri, vNorm );
	if ( pNormal ) {
		*pNormal = fBary[0]*vNorm[0] + fBary[1]*vNorm[1] + fBary[2]*vNorm[2];
		pNormal->Normalize();
	}

	return fBary[0]*vTri[0] + fBary[1]*vTri[1] + fBary[2]*vTri[2];
}




// neighbour list setup code


ExpMapParticle::ListEntry * ExpMapGenerator::GetNeighbourList( ExpMapParticle * pParticle )
{
	if ( pParticle->GetNeighbourList() == NULL ) {
		m_vNeighbourBuf.resize(0);
		if ( m_bUseMeshNeighbours ) 
			FindNeighbours(pParticle, m_vNeighbourBuf);
		else
			FindNeighbours( pParticle->Position(), pParticle->Normal(), m_vNeighbourBuf, m_fMaxNbrDist, pParticle );	

		size_t nCount = m_vNeighbourBuf.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			ExpMapParticle::ListEntry * pEntry = m_NeighbourMemoryPool.Allocate();
			pEntry->pParticle = m_vNeighbourBuf[i];
			pParticle->AddNeighbour(pEntry);
		}
	}

	if ( pParticle != m_pSeedParticle )
		SetSmoothNormal(pParticle, pParticle->GetNeighbourList(), m_bUseNeighbourNormalSmoothing);		
	
	return pParticle->GetNeighbourList();
}


class  NeighborTriBuffer : public IMesh::NeighborTriCallback
{
public:
	NeighborTriBuffer() {}

	const std::vector<IMesh::TriangleID> & Triangles() { return m_vTriangles; }

	virtual void BeginTriangles() 
		{ m_vTriangles.resize(0); }
	virtual void NextTriangle( IMesh::TriangleID tID )
		{ m_vTriangles.push_back(tID); }

protected:
	std::vector<IMesh::TriangleID> m_vTriangles;
};


void ExpMapGenerator::FindNeighbours( ExpMapParticle * pParticle,
									  std::vector<ExpMapParticle *> & vParticles)
{
	vParticles.resize(0);

	IMesh::VertexID vID = pParticle->VertexID();

	NeighborTriBuffer vBuffer;
	GetMesh()->NeighbourIteration(vID, &vBuffer);
	const std::vector<IMesh::TriangleID> & vTris = vBuffer.Triangles();
	size_t nCount = vTris.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		IMesh::VertexID nTri[3];
		GetMesh()->GetTriangle( vTris[i], nTri);

		// check each vertex
		for ( int j = 0; j < 3; ++j ) {
			if ( nTri[j] == vID )
				continue;
			ExpMapParticle * pParticle = m_vParticles[ m_vVtxToParticleMap[nTri[j]] ];
			bool bFound = false;
			for ( unsigned int k = 0; k < vParticles.size() && ! bFound; ++k ) {
				if ( vParticles[k] == pParticle )
					bFound = true;
			}
			if ( ! bFound )
				vParticles.push_back( pParticle );
		}
	}
}




#define USE_KNN

bool ParticleDistanceSort( ExpMapParticle * p1, ExpMapParticle * p2 )
{
	return p1->SurfaceDistance() < p2->SurfaceDistance();
}


void ExpMapGenerator::FindNeighbours( const Wml::Vector3f & vPoint, const Wml::Vector3f & vNormal,
									  std::vector<ExpMapParticle *> & vParticles, float fRadiusThreshold,
									  ExpMapParticle * pSkip )
{
	float fDistSqrThreshold = fRadiusThreshold*fRadiusThreshold;

	std::vector<ExpMapParticle *> vNeighbours;
	const unsigned int sMaxNbrs = 15;
//	const unsigned int sMaxNbrs = 8;

rinse_and_repeat:

	vNeighbours.resize(0);
	ParticleGrid<ExpMapParticle *>::BoxIterator itr( &m_particleGrid, vPoint, fRadiusThreshold );
	if ( itr.Done() ) {
		fRadiusThreshold *= 1.5f;
		goto rinse_and_repeat;
	}
	while ( ! itr.Done() ) {
		ExpMapParticle * pTmp = *itr;
		++itr;

		if ( pTmp == pSkip )
			continue;

		float fDistSqr = ( pTmp->Position() - vPoint ).SquaredLength();
		//float fNormalDir = pTmp->Normal().Dot( vNormal );
		//if ( fDistSqr < fDistSqrThreshold && fNormalDir > -0.1f) {
		//if ( fDistSqr < fDistSqrThreshold ) {
		if ( true ) {
			pTmp->SurfaceDistance() = fDistSqr;
			vNeighbours.push_back( pTmp );
		} 
	}

	size_t nFoundNbrs = vNeighbours.size();
	if ( nFoundNbrs < sMaxNbrs/2 ) {
		for ( unsigned int k = 0; k < nFoundNbrs; ++k )
			vNeighbours[k]->SurfaceDistance() = std::numeric_limits<float>::max();
		fRadiusThreshold *= 1.5f;
		goto rinse_and_repeat;
	}

	std::sort( vNeighbours.begin(), vNeighbours.end(), ParticleDistanceSort );
	unsigned int nMaxNbrs;

	if (sMaxNbrs < nFoundNbrs)
	  nMaxNbrs = (unsigned int)sMaxNbrs;
	else
	  nMaxNbrs = (unsigned int)nFoundNbrs;
	// = (unsigned int)std::min(sMaxNbrs, nFoundNbrs );
	for ( unsigned int k = 0; k < nFoundNbrs; ++k )
		vNeighbours[k]->SurfaceDistance() = std::numeric_limits<float>::max();

	vParticles.resize(nMaxNbrs);
	for ( unsigned int i = 0; i < nMaxNbrs; ++i ) {
		vParticles[i] = vNeighbours[i];
	}
}


void ExpMapGenerator::InitializeNeighbourLists( )
{
	if (! m_bUseMeshNeighbours ) {
		// find max nbr dist
		m_fMaxNbrDist = m_fAvgEdgeLength;
		InitializeParticleGrid(m_fMaxNbrDist);
	}

	if ( m_bPrecomputeNeighbours && ! m_bNeighbourListsValid ) {
		// force nbr generation for all particles
		size_t nParticleCount = m_vParticles.size();	
		for ( unsigned int i = 0; i < nParticleCount; ++i ) {
			ExpMapParticle::ListEntry * pCur = GetNeighbourList(m_vParticles[i]);
		}

		m_bNeighbourListsValid = true;
	}
}



void ExpMapGenerator::ClearNeighbourLists()
{
	size_t nParticleCount = m_vParticles.size();
	for ( unsigned int i = 0; i < nParticleCount; ++i ) {
		m_vParticles[i]->ClearNeighbourList();
	}
	m_NeighbourMemoryPool.ClearAll();

	m_bNeighbourListsValid = false;
}









// particle grid setup code

void ExpMapGenerator::InitializeParticleGrid(float fCellSize)
{
	if (m_bParticleGridValid == true )
		return;

	Wml::AxisAlignedBox3f partBounds;
	size_t nParticleCount = m_vParticles.size();	
	for ( unsigned int i = 0; i < nParticleCount; ++i ) {
		ExpMapParticle * pCur = m_vParticles[i];
		if ( i == 0 )
			partBounds = Wml::AxisAlignedBox3f( 
			pCur->Position().X(), pCur->Position().X(),
			pCur->Position().Y(), pCur->Position().Y(),
			pCur->Position().Z(), pCur->Position().Z() );
		else
			rms::Union( partBounds, pCur->Position() );
	}
	// dilate by one cell
	for ( int k = 0; k < 3; ++k ) {
		partBounds.Min[k] -= fCellSize;
		partBounds.Max[k] += fCellSize;
	}

	m_particleGrid.Initialize( rms::Center(partBounds), fCellSize );

	// add each particle
	for ( unsigned int i = 0; i < nParticleCount; ++i ) {
		ExpMapParticle * pCur = m_vParticles[i];

		m_particleGrid.AddParticle( pCur, pCur->Position() );
	}

	m_bParticleGridValid = true;
}	


void ExpMapGenerator::InitializeParticles( ExpMapParticle * pSkip )
{
	if ( m_vLastParticles.empty() ) {
		size_t nParticleCount = m_vParticles.size();
		for ( unsigned int i = 0; i < nParticleCount; ++i ) {
			if ( m_vParticles[i] == pSkip )
				continue;
			m_vParticles[i]->Clear();
		}
	} else {
		size_t nCount = m_vLastParticles.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			if ( m_vLastParticles[i] == pSkip )
				continue;
			m_vLastParticles[i]->Clear();
		}
		m_vLastParticles.resize(0);
	}
}









void ExpMapGenerator::SetSurfaceDistances( const Wml::Vector3f & vSeedPoint, float fNeighbourThreshold,
													   float fStopDistance,
													   const Frame3f * pSeedFrame)
{
	_RMSTUNE_start(4);

	// create seed particle
	InitializeNeighbourLists();
	InitializeSeedParticle( vSeedPoint, & pSeedFrame->Z(), pSeedFrame );

	// compute approximate geodesic distances to seed particle
	m_fLastMaxRadius = fStopDistance;
	ComputeExpMap( fStopDistance, 3 );

	_RMSTUNE_end(4);
	//_RMSInfo("Total time was %f\n", _RMSTUNE_time(4) );
}

void ExpMapGenerator::SetSurfaceDistances( const Frame3f & vSeedFrame, float fNeighbourThreshold, unsigned int nMaxCount )
{
	_RMSTUNE_start(4);

	// create seed particle
	InitializeNeighbourLists();
	InitializeSeedParticle( vSeedFrame.Origin(), & vSeedFrame.Z(), &vSeedFrame );

	// compute approximate geodesic distances to seed particle
	ComputeExpMap( std::numeric_limits<float>::max(), nMaxCount );

	_RMSTUNE_end(4);
//	_RMSInfo("Total time was %f\n", _RMSTUNE_time(4) );
}


void ExpMapGenerator::SetSurfaceDistances( const Frame3f & vSeedFrame, float fNeighbourThreshold, float fStopDistance, unsigned int nMaxCount )
{
	_RMSTUNE_start(4);

	// create seed particle
	InitializeNeighbourLists();
	InitializeSeedParticle( vSeedFrame.Origin(), & vSeedFrame.Z(), &vSeedFrame );

	// compute approximate geodesic distances to seed particle
	ComputeExpMap( fStopDistance, nMaxCount );

	_RMSTUNE_end(4);
	//_RMSInfo("Total time was %f\n", _RMSTUNE_time(4) );
}



// count has priority (ie will go over stopdistance to ensure count is met)
void ExpMapGenerator::ComputeExpMap( float fStopDistance, unsigned int nMaxCount )
{
	// set all particle distances to max and state to inactive
	size_t nParticleCount = m_vParticles.size();
	InitializeParticles( m_pSeedParticle );

	// now initialize pq
	std::multiset< ParticleQueueWrapper > pq;
	UpdateNeighbours( m_pSeedParticle, pq );

	// run until pq is empty
	float fStopDistSquare = fStopDistance / (float)sqrt(2.0f);
	// float fCurMaxGeoDist = 0;
	unsigned int nTouched = 0;
	while ( ! pq.empty() ) {

		if ( fStopDistance == std::numeric_limits<float>::max() && nTouched >= nMaxCount )
			break;

		// pop front	
		// ExpMapParticle * pFront = pq.begin()->Particle();

		ExpMapParticle * pFront = pq.begin()->Particle();
		pq.erase( pq.begin() );

		// freeze particle
		pFront->State() = ExpMapParticle::Frozen;
		m_vLastParticles.push_back( pFront );

		// set frame for pFront
		if ( m_bUseUpwindAveraging ) 
			PropagateFrameFromNearest_Average( pFront );
		else
			PropagateFrameFromNearest( pFront );

		if ( pFront->SurfaceDistance() > fStopDistance && nTouched >= nMaxCount )
			continue;

		// Square-culling. This gives a significant speed-up...
		if ( m_bEnableSquareCulling ) {
			if ( (float)abs(pFront->SurfaceVector().X()) > fStopDistSquare || 
				(float)abs(pFront->SurfaceVector().Y()) > fStopDistSquare )
				continue;
		} 

		if ( m_bUseClipPoly ) {
			bool bIsInside = m_ClipPoly.IsInside( pFront->SurfaceVector() );
			if ( ! bIsInside ) {
				ExpMapParticle::ListEntry * pCur = GetNeighbourList( pFront );
				while (! bIsInside &&  pCur != NULL ) {
					ExpMapParticle * pCurParticle = pCur->pParticle;
					pCur = pCur->pNext;
					if ( pCurParticle->State() == ExpMapParticle::Frozen &&
						 m_ClipPoly.IsInside( pCurParticle->SurfaceVector() ) )
							bIsInside = true;
				}
			}
			if ( ! bIsInside )
				continue;
		}

		// update neighbours
		RemoveNeighbours( pFront, pq );
		UpdateNeighbours( pFront, pq );
		++nTouched;
	}
//	_RMSInfo("Touched %d particles while updating\n", nTouched);

	// mark any left-over particles for cleanup
	std::multiset<ParticleQueueWrapper>::iterator cur(pq.begin()), end(pq.end());
	while ( cur != end ) {
		(*cur).Particle()->SurfaceDistance() = std::numeric_limits<float>::max();
		m_vLastParticles.push_back( (*cur).Particle() );
		++cur;
	}
}




void ExpMapGenerator::RemoveNeighbours( ExpMapParticle * pParticle, std::multiset< ParticleQueueWrapper > & pq )
{
	ExpMapParticle::ListEntry * pCur = GetNeighbourList( pParticle );
	if ( pCur == NULL ) lgBreakToDebugger();
	while ( pCur != NULL ) {

		ExpMapParticle * pCurParticle = pCur->pParticle;
		pCur = pCur->pNext;

		if ( pCurParticle->State() != ExpMapParticle::Active )
			continue;

		// find entry in pq
#if 1
		std::multiset<ParticleQueueWrapper>::iterator found( 
			pq.find( ParticleQueueWrapper( pCurParticle->SurfaceDistance() ) ) );
		if ( found != pq.end() ) {

			while ( (*found).Particle() != pCurParticle &&
						(*found).QueueValue() == pCurParticle->SurfaceDistance() )
				++found;

			// [RMS: this should always happen...]
			lgASSERT( (*found).Particle() == pCurParticle );
			if ( (*found).Particle() == pCurParticle ) {
				pq.erase( found );
			}
		} else {
			lgASSERT( found != pq.end() );
		}

#else 
		std::multiset<ParticleQueueWrapper>::iterator cur( pq.begin() );
		bool bFound = false;
		while ( !bFound && cur != pq.end() ) {
			if ( (*cur).Particle() == pCurParticle ) {
				pq.erase( cur );
				bFound = true;
			} else
				++cur;
		}
		lgASSERT( bFound );
#endif

	}
}


void ExpMapGenerator::UpdateNeighbours( ExpMapParticle * pParticle, std::multiset< ParticleQueueWrapper > & pq )
{
	// iterate through neighbours, updating particle distances and pushing onto pq
	ExpMapParticle::ListEntry * pCur = GetNeighbourList( pParticle );
	if ( pCur == NULL ) lgBreakToDebugger();
	while ( pCur != NULL ) {

		ExpMapParticle * pCurParticle = pCur->pParticle;
		pCur = pCur->pNext;

		// skip inactive particles
		if ( pCurParticle->State() == ExpMapParticle::Frozen )
			continue;

		// set active state
		pCurParticle->State() = ExpMapParticle::Active;

		// compute new distance
		float fDistToPoint = (pParticle->Position() - pCurParticle->Position()).Length();
		float fSurfDist = fDistToPoint + pParticle->SurfaceDistance();

		// update particle distance and/or nearest particle
		bool bUpdated = false;
		if ( fSurfDist < pCurParticle->SurfaceDistance() ) {
			pCurParticle->SetNearestParticle( pParticle );
			pCurParticle->SurfaceDistance() = fSurfDist;
			bUpdated = true;
		}
		if ( pCurParticle->SurfaceDistance() < std::numeric_limits<float>::max() && pCurParticle->NearestParticle() == NULL )
			lgBreakToDebugger();

		// re-insert particle into priority queue
		pq.insert( ParticleQueueWrapper(pCurParticle) );
	}	

}





ExpMapParticle * ExpMapGenerator::InitializeSeedParticle( const Wml::Vector3f & vPosition,
														  const Wml::Vector3f * pSeedNormal,
														  const Frame3f * pLastSeedPointFrame )
{
	if ( m_pSeedParticle == NULL )
		m_pSeedParticle = new ExpMapParticle();

	ExpMapParticle * pSeedParticle = m_pSeedParticle;
	pSeedParticle->Reset();
	pSeedParticle->Position() = vPosition;
	pSeedParticle->Normal() = *pSeedNormal;
	pSeedParticle->State() = ExpMapParticle::Frozen;


	if ( m_bUseMeshNeighbours ) {
		// just use 3 nearest mesh neighbours...

		Wml::Vector3f vNearest;
		IMesh::TriangleID tID;
		if ( ! m_pMeshBVTree->FindNearest( vPosition, vNearest, tID ) )
			lgBreakToDebugger();
		IMesh::VertexID nTri[3], nNbrTri[3];
		GetMesh()->GetTriangle( tID, nTri );

		// add direct nbrs
		m_vNeighbourBuf.resize(0);
		for ( int j = 0; j < 3; ++j ) {
			m_vNeighbourBuf.push_back( m_vParticles[ m_vVtxToParticleMap[nTri[j]] ] );
			m_vNeighbourBuf.back()->m_bNbrFlag = true;
		}

		// add each of their one-rings
		NeighborTriBuffer vBuffer;
		for ( int j = 0; j < 3; ++j ) {
			GetMesh()->NeighbourIteration( nTri[j], &vBuffer );
			const std::vector<IMesh::TriangleID> & vTriangles = vBuffer.Triangles();
			size_t nCount = vTriangles.size();
			for ( unsigned int i = 0; i < nCount; ++i ) {
				GetMesh()->GetTriangle( vTriangles[i], nNbrTri );
				for ( int k = 0; k < 3; ++k ) {
					ExpMapParticle * pParticle = m_vParticles[ m_vVtxToParticleMap[nNbrTri[j]] ];
					if ( pParticle->m_bNbrFlag == false ) {
						m_vNeighbourBuf.push_back(pParticle);
						m_vNeighbourBuf.back()->m_bNbrFlag = true;
					}
				}
			}
		}

	} else {
		// multiplying distance by 2 here is a hack, to try and fix
		// some problems where we don't get enough neighbours around the seed point 
		// if it is in the middle of a triangle or something like that...
		m_vNeighbourBuf.resize(0);
		FindNeighbours( pSeedParticle->Position(), pSeedParticle->Normal(), m_vNeighbourBuf, m_fMaxNbrDist*2.0f );	
	}

	// ok now add them all as neighbours, and clear neighbour flags
	size_t nCount = m_vNeighbourBuf.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		ExpMapParticle::ListEntry * pEntry = m_NeighbourMemoryPool.Allocate();
		pEntry->pParticle = m_vNeighbourBuf[i];
		pSeedParticle->AddNeighbour( pEntry );
		m_vNeighbourBuf[i]->m_bNbrFlag = false;
	}

	// estimate smooth normal
	// [TODO] make this work w/ m_pIMesh
	if ( m_pVFMesh && m_bUseNeighbourNormalSmoothing ) {
		// get average normal...
		std::vector<IMesh::VertexID> vNbrs;
		for ( unsigned int k = 0; k < m_vNeighbourBuf.size(); ++k )
			vNbrs.push_back(m_vNeighbourBuf[k]->VertexID());

		Wml::Vector3f vNewNormal(0,0,0);
		float fWeightSum = 0.0f;
		for ( unsigned int i = 0; i < m_vNeighbourBuf.size(); ++i) {
			ExpMapParticle * pTriParticle = m_vNeighbourBuf[i];
			float fWeight = 1.0f / ((pTriParticle->Position() - vPosition).Length() + (0.0001f*m_fMaxEdgeLength) );
			fWeightSum += fWeight;
			vNewNormal += fWeight * MeshUtils::GetAverageNormal( *m_pVFMesh, pTriParticle->VertexID() );
		}
		vNewNormal /= fWeightSum;
		vNewNormal.Normalize();
		pSeedParticle->Normal() = vNewNormal;
	}


	// compute 3D frame at seed point
	Frame3f startWorldFrame( pSeedParticle->Position(), pSeedParticle->Normal() );

	// if this argument was passed non-null, try to minimize the tangent-frame rotation of the
	// current seed point wrt the last one
	if ( pLastSeedPointFrame != NULL ) {
		Frame3f vLastFrame( *pLastSeedPointFrame );
		vLastFrame.AlignZAxis( startWorldFrame );
		vLastFrame.Origin() = startWorldFrame.Origin();
		startWorldFrame = vLastFrame;
	}

	pSeedParticle->WorldFrame() = startWorldFrame;
	//pSeedParticle->WorldFrame() = rms::Frame3f();

	return pSeedParticle;
}





void ExpMapGenerator::PropagateFrameFromNearest( ExpMapParticle * pParticle )
{
	ExpMapParticle * pCenterParticle = pParticle->NearestParticle();

	if ( pParticle->SurfaceDistance() == 0.0f ) {
		// pathological case where seed point == input point (or other duplicate points)
		pParticle->SurfaceVector() = pCenterParticle->SurfaceVector();
		return;
	} else { 
		ExtPlane3f vTangentPlane;
		Frame3f vCenterWorldFrame;
		Wml::Matrix2f matFrameRotate;
		PrecomputePropagationData( pCenterParticle, vTangentPlane, vCenterWorldFrame, matFrameRotate );

		pParticle->SurfaceVector() = ComputeSurfaceVector( pCenterParticle, pParticle, 
			vTangentPlane, vCenterWorldFrame, matFrameRotate );
	}

	// check error and re-set distance if it is too high
#if 1
	float fVecLengthSqr = pParticle->SurfaceVector().SquaredLength();
	float fError = fabs(
		fVecLengthSqr / (pParticle->SurfaceDistance()*pParticle->SurfaceDistance()) - 1.0f );
	static const float s_fMaxErr = 2*0.5f - 0.5f*0.5f;		// 0.5f == 50% - arbitrary threshold...
	if ( fError > s_fMaxErr ) {
		if ( true ) {
			pParticle->SurfaceVector().Normalize();
			pParticle->SurfaceVector() *= pParticle->SurfaceDistance();
			//_RMSInfo("Fix!\n");
		}
	}
#endif
}


struct NbrInfo
{
	float fNbrWeight;
	Wml::Vector2f vNbrUV;
};

void ExpMapGenerator::PropagateFrameFromNearest_Average( ExpMapParticle * pParticle )
{
	ExpMapParticle * pNearest = pParticle->NearestParticle();

	if ( pParticle->SurfaceDistance() == 0.0f )  {
		// pathological case where seed point == input point (or other duplicate points)
		pParticle->SurfaceVector() = pNearest->SurfaceVector();
		return;
	}

	static std::vector<NbrInfo> vNbrs;
	ExtPlane3f vTangentPlane;
	Frame3f vCenterWorldFrame;
	Wml::Matrix2f matFrameRotate;

	float fWeightSum = 0.0f;
	vNbrs.resize(0);

	//! need to look for nearest particle, because if it is seed
	//! point, it's not in nbr lists...
	if ( pNearest->State() != ExpMapParticle::Frozen )
		lgBreakToDebugger();

	bool bSawNearest = false;
	ExpMapParticle::ListEntry * pCurNbr = GetNeighbourList(pParticle);
	while ( pCurNbr != NULL ) {

		ExpMapParticle * pCenterParticle = pCurNbr->pParticle;
		if ( pCenterParticle == pNearest )
			bSawNearest = true;

		if ( pCenterParticle->State() == ExpMapParticle::Frozen ) {
			PrecomputePropagationData( pCenterParticle, vTangentPlane, vCenterWorldFrame, matFrameRotate );

			NbrInfo info;
			info.vNbrUV = ComputeSurfaceVector( pCenterParticle, pParticle, 
				vTangentPlane, vCenterWorldFrame, matFrameRotate );

			if ( ! _finite(info.vNbrUV.Length()) )
				lgBreakToDebugger();

			info.fNbrWeight = 1.0f / ( ( pCenterParticle->Position() - pParticle->Position() ).Length() + (0.00001f*m_fMaxEdgeLength) );
			if ( ! _finite(info.fNbrWeight) )
				lgBreakToDebugger();

			// weight by geodesic delta - tries to avoid bias from neigbours w/ same "time-of-arrival" (ie at same front-timestep)
			// [RMS: not sure if this has a significant effect - note that need to enable below as well....
			// [RMS] also this is broken for particles at the same 3D position, which seems to happen w/ surface tree...
			//float fGeoDelta = fabs(pCenterParticle->SurfaceVector().Length() - info.vNbrUV.Length());
			//info.fNbrWeight *= fGeoDelta;

			fWeightSum += info.fNbrWeight;

			vNbrs.push_back(info);
		}

		pCurNbr = pCurNbr->pNext;
	}

	if ( ! _finite(fWeightSum) )
		lgBreakToDebugger();

	// add un-seen "nearest" particle
	if ( ! bSawNearest ) {
		PrecomputePropagationData( pNearest, vTangentPlane, vCenterWorldFrame, matFrameRotate );
		NbrInfo info;
		info.vNbrUV =  ComputeSurfaceVector( pNearest, pParticle, 
				vTangentPlane, vCenterWorldFrame, matFrameRotate );

		info.fNbrWeight = 1.0f / ( ( pNearest->Position() - pParticle->Position() ).Length() + (0.00001f*m_fMaxEdgeLength) );

		// weight by geo delta
		//float fGeoDelta = fabs(pNearest->SurfaceVector().Length() - info.vNbrUV.Length());
		//info.fNbrWeight *= fGeoDelta;

		fWeightSum += info.fNbrWeight;
		vNbrs.push_back(info);
	}

	if ( ! _finite(fWeightSum) )
		lgBreakToDebugger();

	// take weighted average to get real UV
	Wml::Vector2f vUV = Wml::Vector2f::ZERO;
	size_t nCount = vNbrs.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		float fWeight = vNbrs[i].fNbrWeight / fWeightSum;
		vUV += fWeight * vNbrs[i].vNbrUV;
	}

	if ( ! _finite(vUV.Length()) )
		lgBreakToDebugger();

	pParticle->SurfaceVector() = vUV;


	// check error and re-set distance if it is too high
#if 1
	float fVecLengthSqr = pParticle->SurfaceVector().SquaredLength();
	float fError = fabs(
		fVecLengthSqr / (pParticle->SurfaceDistance()*pParticle->SurfaceDistance()) - 1.0f );
	static const float s_fMaxErr = 2*0.5f - 0.5f*0.5f;		// 0.5f == 50% - arbitrary threshold...
	if ( fError > s_fMaxErr ) {
		if ( true ) {
			pParticle->SurfaceVector().Normalize();
			pParticle->SurfaceVector() *= pParticle->SurfaceDistance();
//			_RMSInfo("Fix!\n");
		}
	}
#endif

}






void ExpMapGenerator::PrecomputePropagationData( ExpMapParticle * pCenterParticle,
															 ExtPlane3f & vTangentPlane, 
															 Frame3f & vCenterWorldFrame,
															 Wml::Matrix2f & matFrameRotate )
{
	// compute surface-tangent plane at particle
	Wml::Vector3f vNormal( pCenterParticle->Normal() );

	// compute plane at this point
	vTangentPlane = ExtPlane3f( vNormal, pCenterParticle->Position() );

	// compute 3D frame at center point
	//vCenterWorldFrame = Frame3f( pCenterParticle->Position(), vNormal );
	vCenterWorldFrame = pCenterParticle->WorldFrame();

	// rotate seed world frame Z into this particle's Z
	Frame3f seedWorldFrame( m_pSeedParticle->WorldFrame() );
	seedWorldFrame.AlignZAxis( vCenterWorldFrame );

	// compute cos(angle) between axis
	Wml::Vector3f vCenterAxisX( vCenterWorldFrame.Axis( Frame3f::AxisX ) );
	Wml::Vector3f vSeedAxisX( seedWorldFrame.Axis( Frame3f::AxisX ) );
	float fCosTheta = vCenterAxisX.Dot(vSeedAxisX);

	// compute rotated min-dist vector for this particle
	float fTmp = 1 - fCosTheta*fCosTheta;
	if ( fTmp < 0 ) fTmp = 0;		// need to clamp so that sqrt works...
	float fSinTheta = (float)sqrt(fTmp);
	Wml::Vector3f vCross = vCenterAxisX.Cross(vSeedAxisX);
	if ( vCross.Dot( vNormal ) < 0 )	// get the right sign...
		fSinTheta = -fSinTheta;

	// create transposed 2D rotation matrix
	matFrameRotate = Wml::Matrix2f( fCosTheta, fSinTheta, -fSinTheta, fCosTheta );
}

Wml::Vector2f ExpMapGenerator::ComputeSurfaceVector( ExpMapParticle * pCenterParticle, 
																 ExpMapParticle * pNbrParticle,
																 ExtPlane3f & vTangentPlane, 
																 Frame3f & vCenterWorldFrame, 
																 Wml::Matrix2f & matFrameRotate )
{
	// special case...
	if ( (pNbrParticle->Position() - pCenterParticle->Position()).Length() < Wml::Mathf::EPSILON )
		return pCenterParticle->SurfaceVector();

	// project point into plane
	Wml::Vector3f vPlanePoint = vTangentPlane.RotatePointIntoPlane( pNbrParticle->Position() );

	// project point into coord system of frame
	vPlanePoint -= pCenterParticle->Position();
	vCenterWorldFrame.ToFrameLocal(vPlanePoint);

	// now we can project into surface frame simply by dropping z (which should be 0 anyway,
	//   since the vector lies in the plane!)
	Wml::Vector2f vSurfaceFrame( vPlanePoint.X(), vPlanePoint.Y() );

	// reverse vector so it points back to current particle
	vSurfaceFrame *= -1.0f;

	// transform local vector into coord system of initial surface reference frame
	//  and add accumulated surface vector
	return pCenterParticle->SurfaceVector() + (matFrameRotate * vSurfaceFrame);
}


rms::Frame3f ExpMapGenerator::GetSeedFrame()
{
	if ( m_pSeedParticle )
		return m_pSeedParticle->WorldFrame();
	else
		return rms::Frame3f();
}
