// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "SurfaceAreaSelection.h"
#include "MeshSourceUtil.h"

#include <limits>
#include <Wm4DistVector3Segment3.h>
#include <DijkstraFrontProp.h>
#include <MeshUtils.h>

#include <rmsdebug.h>
#include <lgcolors.h>

using namespace rms;

SurfaceAreaSelection::SurfaceAreaSelection(void)
{
	m_pMesh = NULL;
	m_pPolygons = NULL;
	m_pBVTree = NULL;

	m_bGLDisplayListValid = false;
	m_nGLDisplayList = -1;

	Reset();
}

SurfaceAreaSelection::~SurfaceAreaSelection(void)
{
	if ( m_nGLDisplayList != -1 )
		glDeleteLists(m_nGLDisplayList,1);
}

void SurfaceAreaSelection::SetSurface(rms::VFTriangleMesh * pMesh, rms::MeshPolygons * pPolygons, rms::IMeshBVTree * pBVTree)
{
	m_pMesh = pMesh;
	m_pPolygons = pPolygons;
	m_pBVTree = pBVTree;

	m_pMesh->GetBoundingBox(m_bounds);
	float fMin = 0, fMax = 0;
	m_pMesh->GetEdgeLengthStats(fMin, fMax, m_fAvgEdgeLength);
	m_fMeshSizeFactor = MaxDimension(m_bounds) * 0.05f;

	IMesh::TriangleID tID = pMesh->GetMaxTriangleID();
	m_vFaceNormalCache.resize( tID );
	Wml::Vector3f vTri[3];
	for ( unsigned int k = 0; k < tID; ++k ) {
		pMesh->GetTriangle( k, vTri );
		m_vFaceNormalCache[k] = rms::Normal(vTri[0], vTri[1], vTri[2]);
	}
	
	Reset();
}


void SurfaceAreaSelection::Reset()
{
	m_vInterior.clear();
	m_vBoundaryTris.clear();
	m_bClosed = false;
	m_fGrowOffset = 0;
	m_vCurStroke.clear();
	m_vPath.clear();
	m_vOriginalPath.clear();
	InvalidateGeoDistances();

	m_bGLDisplayListValid = false;
}




void SurfaceAreaSelection::SetInterior( const std::set<unsigned int> & vSet, bool bToPolygons)
{
	if ( bToPolygons && m_pPolygons ) {
		m_vInterior.clear();
		m_pPolygons->ToSets( vSet, m_vInterior );
	} else {
		m_vInterior = vSet;
	}

	m_vBoundaryTris.clear();
	m_bGLDisplayListValid = false;
}



void SurfaceAreaSelection::Select(const std::set<IMesh::TriangleID> & vTriangles )
{
	Reset();

	std::set<IMesh::TriangleID> vSelected;
	std::set<IMesh::TriangleID>::const_iterator curt(vTriangles.begin()), endt(vTriangles.end());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		if ( m_pMesh->IsTriangle(tID) )
			vSelected.insert(tID);
	}
	SetInterior(vSelected);
}




void SurfaceAreaSelection::AppendToStroke( const rms::Frame3f & vFrame )
{
	if (! m_bClosed )
		m_vCurStroke.push_back(vFrame);
}
void SurfaceAreaSelection::CancelStroke()
{
	m_vCurStroke.resize(0);
}

void SurfaceAreaSelection::AppendCurrentStroke( )
{
	std::vector< rms::Frame3f > vStroke = m_vCurStroke;
	m_vCurStroke.resize(0);

	if ( m_bClosed || vStroke.empty() )
		return;
	if ( m_vPath.empty() ) {
		m_vPath = vStroke;
		TryClose();
		return;
	}

	// find connection start
	Wml::Vector3f vStart = vStroke[0].Origin();
	float fNearest = std::numeric_limits<float>::max();
	int iNearest = -1;
	for ( int i = (int)m_vPath.size()-1; i > 0; --i ) {
		float fDistSqr = (m_vPath[i].Origin() - vStart).SquaredLength();
		if ( fDistSqr < fNearest ) {
			fNearest = fDistSqr;
			iNearest = i;
		}
	}

	if ( fNearest > m_fMeshSizeFactor*m_fMeshSizeFactor  )
		return;

	// snap together
	Wml::Vector3f vSnap;
	IMesh::TriangleID nTriID;
	if ( ! m_pBVTree->FindNearest( 0.5f * (vStart + m_vPath[iNearest].Origin()), vSnap, nTriID ) )
		return;

	m_vPath.resize(iNearest+1);
	m_vPath[iNearest] = vSnap;

	for ( unsigned int i = 0; i < vStroke.size(); ++i )
		m_vPath.push_back( vStroke[i] );

	// check if we should close
	TryClose();
}


void SurfaceAreaSelection::TryClose()
{
	if ( m_bClosed )
		return;

	float fDistSqr = (m_vPath.front().Origin() - m_vPath.back().Origin()).SquaredLength();
	if ( fDistSqr > m_fMeshSizeFactor*0.1f )
		return;

  Wm4::Vector3f foo(0.5f * (m_vPath.front().Origin() + m_vPath.back().Origin()));
	if ( ProjectToSurface( foo , m_vPath.back() ) ) {
		m_bClosed = true;
		m_vOriginalPath = m_vPath;

		FindSelectionPatch();
	}
}


bool SurfaceAreaSelection::ReverseSelection(  )
{
	if ( ! IsValid() )
		return false;

	InvalidateGeoDistances();

	// add faces next to boundary triangles to stack
	IMesh::TriangleID tNbrs[3];
	std::set<IMesh::TriangleID> vStack;
	std::set<IMesh::TriangleID>::iterator curi(m_vInterior.begin()), endi(m_vInterior.end());
	while ( curi != endi ) {
		IMesh::TriangleID tID = *curi++;
		m_pMesh->FindNeighbours(tID, tNbrs);
		for ( int k = 0; k < 3; ++k ) 
			if ( tNbrs[k] != IMesh::InvalidID && m_vInterior.find(tNbrs[k]) == m_vInterior.end() )
				vStack.insert( tNbrs[k] );
	}

	// do a flood fill of connected triangles
	std::set<IMesh::TriangleID> vNewInterior;
	while (! vStack.empty() ) {
		IMesh::TriangleID tID = *vStack.begin();
		vStack.erase(*vStack.begin());
		vNewInterior.insert( tID );
		m_pMesh->FindNeighbours( tID, tNbrs );
		for ( int k = 0; k < 3; ++k ) {
			if ( tNbrs[k] == IMesh::InvalidID )
				continue;
			if ( m_vInterior.find(tNbrs[k]) != m_vInterior.end() )
				continue;
			if ( vNewInterior.find(tNbrs[k]) != vNewInterior.end() )
				continue;
			vStack.insert( tNbrs[k] );
		}
	}

	// find connected components in the flood fill and select the largest one. 
	// [RMS] I'm not entirely sure why this is necessary. In some cases, the entire
	//   mesh only has one connected component, but if I make a selection and invert,
	//   then I get multiple components. Maybe it is because of bad topology? 
	//   Anyway this is a reasonable kludge...
	int nComponents, nLargest;
	std::vector<int> vComponents;
	MeshUtils::FindConnectedComponents(m_pMesh, vNewInterior, vComponents, nComponents, nLargest);
	std::set<IMesh::TriangleID> vLargestInterior;
	for ( unsigned int k = 0; k < vComponents.size(); ++k ) {
		if ( vComponents[k] == nLargest )
			vLargestInterior.insert( k );
	}

	SetInterior(vLargestInterior);
	return true;
}


void SurfaceAreaSelection::Expand(unsigned int nIters)
{
	if ( ! IsValid() )
		return;

	InvalidateGeoDistances();

	std::set<IMesh::TriangleID> vCurInterior( m_vInterior );

	for ( unsigned int n = 0; n < nIters; ++n ) {
		std::set<IMesh::TriangleID> vNewInterior = vCurInterior;
		std::set<IMesh::TriangleID>::iterator curt(vCurInterior.begin()), endt(vCurInterior.end());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt++;
			IMesh::TriangleID vNbrs[3];
			m_pMesh->FindNeighbours(tID, vNbrs);
			for ( int k = 0; k < 3; ++k )
				vNewInterior.insert( vNbrs[k] );
		}

		vCurInterior = vNewInterior;
	}

	SetInterior(vCurInterior);
}

void SurfaceAreaSelection::Contract()
{
	if ( ! IsValid() )
		return;

	InvalidateGeoDistances();

	std::set<IMesh::TriangleID> vNewInterior( m_vInterior );
	std::set<IMesh::TriangleID>::iterator curt(m_vInterior.begin()), endt(m_vInterior.end());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		IMesh::TriangleID vNbrs[3];
		m_pMesh->FindNeighbours(tID, vNbrs);
		bool bBoundary = false;
		for ( int k = 0; k < 3; ++k )
			bBoundary = bBoundary || ( m_vInterior.find(vNbrs[k]) == m_vInterior.end() );
		if ( bBoundary )
			vNewInterior.erase(tID);
	}

	SetInterior(vNewInterior);
}



void SurfaceAreaSelection::SetGrowOffset(float fOffset, bool bScaleByMesh)
{
	// [TODO] would like to be able to make selection stick to boundary here...
	//    basically need to incrementally grow/shrink current selection until
	//    all current vertices <= fUseOffset, instead of doing entire triangles...

	ValidateGeoDistances();

	m_fGrowOffset = fOffset;
	float fUseOffset = (bScaleByMesh) ? (fOffset*m_fAvgEdgeLength) : fOffset;

	std::set<IMesh::TriangleID> vNewInterior;
	VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		if ( m_vGeoDistances[tID] <= fUseOffset )
			vNewInterior.insert(tID);
	}
	
	SetInterior(vNewInterior);
	Optimize();
}



void SurfaceAreaSelection::Optimize()
{
	std::set<IMesh::TriangleID> vSave(m_vInterior);

	FillBoundaryTriangles();
	ClipEarTriangles();

	// _NOT_ efficient...
	std::set<IMesh::TriangleID> vCopy( m_vInterior );
	SetInterior( vCopy );
	if ( m_vInterior.empty() )
		SetInterior(vSave);
}


bool SurfaceAreaSelection::Paint( const rms::Frame3f & vFrame, bool bAdd )
{
	InvalidateGeoDistances();

	Wml::Vector3f vNearest;
	IMesh::TriangleID nTriID;
	if ( ! m_pBVTree->FindNearest( vFrame.Origin(), vNearest, nTriID ) )
		return false;
	if ( bAdd && m_vInterior.find(nTriID) == m_vInterior.end() ) {

		std::set<IMesh::TriangleID> vTris, vPolyTris; 
		vTris.insert(nTriID);
		m_pPolygons->ToSets(vTris, vPolyTris);

		m_vInterior.insert(vPolyTris.begin(), vPolyTris.end());

		m_bGLDisplayListValid = false;
		return true;

	} else if ( !bAdd && m_vInterior.find(nTriID) != m_vInterior.end() ) {
		
		std::set<IMesh::TriangleID> vTris, vPolyTris; 
		vTris.insert(nTriID);
		m_pPolygons->ToSets(vTris, vPolyTris);

		std::set<IMesh::TriangleID>::iterator curt(vPolyTris.begin()), endt(vPolyTris.end());
		while ( curt != endt )
			m_vInterior.erase(*curt++);
	
		m_bGLDisplayListValid = false;
		return true;
	}

	return false;
}


bool SurfaceAreaSelection::IsInitialized() const
{
	if ( ! m_vInterior.empty() )
		return true;
	return ! m_vPath.empty();
}


bool SurfaceAreaSelection::IsValid() const
{
	return ! m_vInterior.empty();
}




bool SurfaceAreaSelection::IsConnected(rms::VFTriangleMesh & mesh) const
{
	std::set<IMesh::TriangleID> vLeft = m_vInterior;
	if ( vLeft.empty() )
		return false;

	std::set<IMesh::TriangleID> vStack;
	IMesh::TriangleID tID = *vLeft.begin();
	vStack.insert( tID );
	vLeft.erase( tID );
	while ( ! vStack.empty() && ! vLeft.empty() ) {
		tID = *vStack.begin();
		vStack.erase(tID);
		
		IMesh::TriangleID tNbrs[3];
		mesh.FindNeighbours(tID, tNbrs);
		for ( unsigned int j = 0; j < 3; ++j ) {
			if ( tNbrs[j] == IMesh::InvalidID )
				continue;
			std::set<IMesh::TriangleID>::iterator found(vLeft.find(tNbrs[j]));
			if ( found != vLeft.end() ) {
				vStack.insert(*found);
				vLeft.erase(found);
			}
		}
	}

	return vLeft.empty();
}




bool SurfaceAreaSelection::Contains( const rms::Frame3f & vFrame )
{
	Wml::Vector3f vNearest;
	IMesh::TriangleID nTriID;
	if ( ! m_pBVTree->FindNearest( vFrame.Origin(), vNearest, nTriID ) )
		return false;
	if ( m_vInterior.find(nTriID) == m_vInterior.end() )
		return false;
	return true;
}





void SurfaceAreaSelection::FindSelectionPatch()
{
	bool bOK = ComputeBoundaryTrisFromPath();
	if ( ! bOK )
		return;

	IMesh::TriangleID tNbrs[3];

	// find un-used triangle
	IMesh::TriangleID tFound = IMesh::InvalidID;
	std::set<IMesh::TriangleID>::iterator tcur(m_vBoundaryTris.begin()), tend(m_vBoundaryTris.end());
	while ( tcur != tend && tFound == IMesh::InvalidID) {
		IMesh::TriangleID tID = *tcur++;
		m_pMesh->FindNeighbours(tID, tNbrs);
		for ( int i = 0; i < 3; ++i ) {
			std::set<IMesh::TriangleID>::iterator found(m_vBoundaryTris.find(tNbrs[i]));
			if ( found == m_vBoundaryTris.end() )
				tFound = tNbrs[i];
		}
	}
	if ( tFound == IMesh::InvalidID )
		return;

	std::set<IMesh::TriangleID> vSet1(m_vBoundaryTris);
	std::set<IMesh::TriangleID> vTodo;
	vTodo.insert(tFound);

	while (! vTodo.empty() ) {
		IMesh::TriangleID tID = *(vTodo.begin());
		vTodo.erase(tID);
		vSet1.insert(tID);

		m_pMesh->FindNeighbours(tID, tNbrs);
		for ( int j = 0; j < 3; ++j ) {
			std::set<IMesh::TriangleID>::iterator found(vSet1.find(tNbrs[j]));
			if ( tNbrs[j] != IMesh::InvalidID && found == vSet1.end() )
				vTodo.insert(tNbrs[j]);
		}
	}


	// find triangle that is not in set1
	tFound = IMesh::InvalidID;
	tcur = m_vBoundaryTris.begin();
	while ( tcur != tend && tFound == IMesh::InvalidID) {
		IMesh::TriangleID tID = *tcur++;
		m_pMesh->FindNeighbours(tID, tNbrs);
		for ( int i = 0; i < 3; ++i ) {
			std::set<IMesh::TriangleID>::iterator found(vSet1.find(tNbrs[i]));
			if ( found == vSet1.end() )
				tFound = tNbrs[i];
		}
	}
	if ( tFound == IMesh::InvalidID )
		return;	


	std::set<IMesh::TriangleID> vSet2(m_vBoundaryTris);
	vTodo.clear();
	vTodo.insert(tFound);

	while (! vTodo.empty() ) {
		IMesh::TriangleID tID = *(vTodo.begin());
		vTodo.erase(tID);
		vSet2.insert(tID);

		m_pMesh->FindNeighbours(tID, tNbrs);
		for ( int j = 0; j < 3; ++j ) {
			std::set<IMesh::TriangleID>::iterator found(vSet2.find(tNbrs[j]));
			if ( tNbrs[j] != IMesh::InvalidID && found == vSet2.end() )
				vTodo.insert(tNbrs[j]);
		}
	}


	m_vInterior = ( vSet1.size() < vSet2.size() ) ? vSet1 : vSet2;
	//tcur = m_vBoundaryTris.begin();
	//while ( tcur != tend )
	//	m_vInterior.erase( *tcur++ );

	Optimize();
}



void SurfaceAreaSelection::ClipEarTriangles()
{
	bool bClipped = true;
	while ( bClipped ) {
		bClipped = false;

		std::vector<IMesh::TriangleID> vErase;
		std::set<IMesh::TriangleID>::iterator curt(m_vInterior.begin()), endt(m_vInterior.end());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt++;
		
			IMesh::TriangleID tNbrs[3];
			m_pMesh->FindNeighbours(tID, tNbrs);
			int nNbrs = 0;
			for ( int j = 0; j < 3; ++j )
				if ( m_vInterior.find(tNbrs[j]) != m_vInterior.end())
					++nNbrs;
			if ( nNbrs < 2 )
				vErase.push_back(tID);
		}
		for ( unsigned int i = 0; i < vErase.size(); ++i ) {
			m_vInterior.erase( vErase[i] );
			m_bGLDisplayListValid = false;
			m_vBoundaryTris.erase( vErase[i] );
		}
		bClipped = ! vErase.empty();
		//_RMSInfo("Clip Pass - clipped %d!\n", vErase.size());
	}
	//_RMSInfo("Done clip\n");
}


void SurfaceAreaSelection::FillBoundaryTriangles()
{
	bool bFilled = true;
	while ( bFilled ) {
		bFilled  = false;

		std::vector<IMesh::TriangleID> vFill;
		std::set<IMesh::TriangleID>::iterator curt(m_vInterior.begin()), endt(m_vInterior.end());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt++;

			IMesh::TriangleID tNbrs[3], tNbrs2[3];
			m_pMesh->FindNeighbours(tID, tNbrs);
			int nNbrs = 0;
			for ( int j = 0; j < 3; ++j ) {
				IMesh::TriangleID tNbrID = tNbrs[j];
				if ( tNbrID == IMesh::InvalidID || m_vInterior.find(tNbrID) != m_vInterior.end() )
					continue;
				m_pMesh->FindNeighbours(tNbrID, tNbrs2);

				int nIntNbrs = 0;
				for ( unsigned int k = 0; k < 3; ++k ) {
					if ( m_vInterior.find(tNbrs2[k]) != m_vInterior.end() )
						++nIntNbrs;
				}
				if ( nIntNbrs == 2 )
					vFill.push_back(tNbrID);
			}
		}
		size_t nFill = vFill.size();
		for ( unsigned int i = 0; i < nFill; ++i ) {
			m_vInterior.insert( vFill[i] );	
			m_bGLDisplayListValid = false;
			//m_vBoundaryTris.insert( vFill[i] );
		}
		bFilled = ! vFill.empty();
	}
}


bool SurfaceAreaSelection::ComputeBoundaryTrisFromPath()
{
	m_vBoundaryTris.clear();

	Wml::Vector3f vSnap, vLastPoint;
	IMesh::TriangleID tID, tLastNbrs[3], tLastID = IMesh::InvalidID;

	std::vector<Wml::Vector3f> vNewPath;
		
	bool bFailed = false;
	size_t nCount = m_vPath.size();
	for ( unsigned int j = 0; j < nCount + 1; ++j ) {
		unsigned int i = j % nCount;

		if ( i < nCount )
			vNewPath.push_back( m_vPath[i].Origin() );

		if ( ! m_pBVTree->FindNearest( m_vPath[i].Origin(), vSnap, tID ) )
			continue;
		if ( tID == tLastID ) {
			vLastPoint = m_vPath[i].Origin();
			continue;
		}
		
		// check if we have a gap - if so, fix
		IMesh::TriangleID tCurID = tID;
		IMesh::TriangleID tTempLastID = tLastID;
		while ( tLastID != IMesh::InvalidID && tLastNbrs[0] != tID && tLastNbrs[1] != tID && tLastNbrs[2] != tID && ! bFailed ) {
			//if ( tCurID == tID ) 
			//	_RMSInfo("New ");
			//_RMSInfo("Gap - cur %d  last %d,  nbrs %d %d %d\n", tCurID, tTempLastID, tLastNbrs[0], tLastNbrs[1], tLastNbrs[2]);

			Wml::Vector3f vNext;
			if ( ! FindNbrTriPoint( vLastPoint, m_vPath[i].Origin(), tTempLastID, 0, vNext ) ) {
				bFailed = true;
				continue;
			}

			IMesh::TriangleID tHitID;
			m_pBVTree->FindNearest( vNext, vSnap, tHitID );
			m_vBoundaryTris.insert(tHitID);
			m_pMesh->FindNeighbours(tHitID, tLastNbrs);
			tTempLastID = tHitID;
			tCurID = tHitID;
			vLastPoint = vNext;
			vNewPath.push_back( vNext );
			//_RMSInfo("  Mid: tri %d,  nbrs %d %d %d\n", tHitID, tLastNbrs[0], tLastNbrs[1], tLastNbrs[2]);
		}

		m_vBoundaryTris.insert(tID);
		m_pMesh->FindNeighbours(tID, tLastNbrs);
		vLastPoint = m_vPath[i].Origin();
		tLastID = tID;
	}


	// update path
	m_vPath.resize(0);
	rms::Frame3f vLastFrame;
	for ( unsigned int i = 0; i < vNewPath.size(); ++i ) {
		if ( ProjectToSurface( vNewPath[i], vLastFrame ) )
			m_vPath.push_back( vLastFrame );
	}

	return ! bFailed;
}


bool SurfaceAreaSelection::FindNbrTriPoint( Wml::Vector3f vStart, Wml::Vector3f vEnd, IMesh::TriangleID tStartTID, unsigned int tPrevID, Wml::Vector3f & vFound )
{
	IMesh::TriangleID tNbrs[3];
	m_pMesh->FindNeighbours(tStartTID, tNbrs);

	Wml::Vector3f vHit, vLastIn;
	IMesh::TriangleID tID;

	int nRepeat = 10;
	for ( int ri = 0; ri < nRepeat; ++ri ) {

		// walk forward from tStart until we find a new tri
		float fStart = -1, fEnd;
		int nSteps = 25;
		float fStep = 1.0f / (float)nSteps;
		for ( int i = 1; i < nSteps; ++i ) {
			float fT = (float)i * fStep;
			Wml::Vector3f vNew(  (1-fT) * vStart + (fT) * vEnd );
			if ( m_pBVTree->FindNearest( vNew, vHit, tID ) ) {
				//_RMSInfo("     Step %f - Hit %d  (start %d)\n", fT, tID, tStartTID);
				if ( tID != tStartTID ) {
					if ( tNbrs[0] == tID || tNbrs[1] == tID || tNbrs[2] == tID ) {
						vFound = vNew;
						return true;
					}
					fStart = (i-1) * fStep;
					fEnd = i * fStep;
					break;
				}
			}
		}
		if ( fStart == -1 ) {		// must be in last segment...
			//_RMSInfo("      In Last\n");
			fStart = 1-fStep;
			fEnd = 1;
		}

		// reset start and end
		Wml::Vector3f vOldStart = vStart;  Wml::Vector3f vOldEnd = vEnd;   
		vStart = (1-fStart)*vOldStart + (fStart)*vOldEnd;
		vEnd = (1-fEnd)*vOldStart + (fEnd)*vOldEnd;
		if (! m_pBVTree->FindNearest( vStart, vHit, tStartTID ) )
			return false;	// catastrophe!
		m_pBVTree->FindNearest( vEnd, vHit, tID );
		//_RMSInfo("   Subdivide! %d %d\n", tStartTID, tID);
	}
	
	return false;
}




bool SurfaceAreaSelection::ProjectToSurface( Wml::Vector3f & vPoint, rms::Frame3f & vFrame )
{
	Wml::Vector3f vSnap;
	IMesh::TriangleID tID;
	if (! m_pBVTree->FindNearest( vPoint, vSnap, tID ) )
		return false;	

	Wml::Vector3f vVerts[3], vNorms[3];
	m_pMesh->GetTriangle(tID, vVerts, vNorms);

	float fBary[3];
	rms::BarycentricCoords( vVerts[0], vVerts[1], vVerts[2], vSnap, fBary[0], fBary[1], fBary[2] );
	Wml::Vector3f vHitNormal = 
		fBary[0]*vNorms[0] + fBary[1]*vNorms[1] + fBary[2]*vNorms[2];
	vHitNormal.Normalize();

	vFrame.Origin() = vSnap;
	vFrame.AlignZAxis(vHitNormal);	

	return true;
}




void SurfaceAreaSelection::UpdateBoundaryTriangles()
{
	std::set<IMesh::TriangleID>::iterator curt(m_vInterior.begin()), endt(m_vInterior.end());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		IMesh::TriangleID vNbrs[3];
		m_pMesh->FindNeighbours(tID, vNbrs);
		bool bNbrsIn = true;
		for ( int j = 0; j < 3; ++j )
			bNbrsIn = bNbrsIn && (m_vInterior.find(vNbrs[j]) != m_vInterior.end());
		if ( ! bNbrsIn )
			m_vBoundaryTris.insert(tID);
	}
}



class SurfAreaSelection_FaceVtxSource : public IPositionSource<IMesh::TriangleID>
{
public:
	std::vector<Wml::Vector3f> * pCenters;
	SurfAreaSelection_FaceVtxSource(std::vector<Wml::Vector3f> * centers)
		{ pCenters = centers; }

	virtual void GetPosition( IMesh::TriangleID nID, Wml::Vector3f & vPosition, Wml::Vector3f * pNormal ) {
		vPosition = (*pCenters)[nID];
	}
	virtual IMesh::TriangleID MaxID() {
		return (IMesh::TriangleID)pCenters->size();
	}
};



void SurfaceAreaSelection::ValidateGeoDistances()
{
	if ( m_vGeoDistances.size() == m_pMesh->GetMaxTriangleID() )
		return;

	UpdateBoundaryTriangles();

	m_vGeoDistances.resize(0);
	m_vGeoDistances.resize( m_pMesh->GetMaxTriangleID(), -std::numeric_limits<float>::max() );

	// construct list of face vertices and exterior faces
	std::vector<Wml::Vector3f> vFaceCenters(m_pMesh->GetMaxTriangleID());
	std::set<IMesh::TriangleID> vExterior;
	VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		Wml::Vector3f vTri[3];
		m_pMesh->GetTriangle(tID, vTri);
		Wml::Vector3f vCentroid = ( vTri[0] + vTri[1] + vTri[2] ) / 3.0f;
		vFaceCenters[tID] = vCentroid;
		if ( m_vInterior.find(tID) == m_vInterior.end() )
			vExterior.insert(tID);
	}
	vExterior.insert( m_vBoundaryTris.begin(), m_vBoundaryTris.end() );

	SurfAreaSelection_FaceVtxSource pointSource(&vFaceCenters);
	TriangleNeighbourSource nbrSource(m_pMesh);
	DijkstraFrontProp<IMesh::TriangleID> prop;
	prop.SetSource(&pointSource, &nbrSource);

	prop.AppendFilterVerts(m_vInterior);
	std::set<IMesh::TriangleID>::iterator curb(m_vBoundaryTris.begin()), endb(m_vBoundaryTris.end());
	while ( curb != endb ) 
		prop.AppendStartValue( *curb++, 0 );
	prop.FindAllDistances();

	const std::vector<DijkstraFrontProp<IMesh::TriangleID>::VertInfo> & distances = prop.GetResults();
	size_t nCount = distances.size();
	int nMin = -1;
	for ( unsigned int k = 0; k < nCount; ++k ) {
		if ( distances[k].bFrozen ) {
			m_vGeoDistances[k] = -distances[k].fMinDist;
			if ( nMin < 0 || m_vGeoDistances[k] < m_vGeoDistances[nMin] )
				nMin = k;
		}
	}
	m_vGeoDistances[nMin] = -std::numeric_limits<float>::max();

	prop.Reset();
	prop.AppendFilterVerts(vExterior);
	std::set<IMesh::TriangleID>::iterator curb2(m_vBoundaryTris.begin()), endb2(m_vBoundaryTris.end());
	while ( curb2 != endb2 ) 
		prop.AppendStartValue( *curb2++, 0 );
	prop.FindAllDistances();

	const std::vector<DijkstraFrontProp<IMesh::TriangleID>::VertInfo> & distances2 = prop.GetResults();
	size_t nCount2 = distances2.size();
	for ( unsigned int k = 0; k < nCount2; ++k ) {
		if ( distances2[k].bFrozen )
			m_vGeoDistances[k] = distances2[k].fMinDist;
	}
}
void SurfaceAreaSelection::InvalidateGeoDistances()
{
	m_vGeoDistances.resize(0);
	m_fGrowOffset = 0;
}


void SurfaceAreaSelection::Render(const Wml::Vector3f & vColor, bool bWireframe)
{
	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_POINT_BIT );
	glDisable(GL_LIGHTING);
	glDepthFunc(GL_LEQUAL);
	
	static const float fNormOff = 0.001f;

	// render selection loop
	if ( ! m_vPath.empty() ) {
		glLineWidth(5.0f);
		if (m_bClosed) 
			glColor3fv(lgBLACK);
		else
			glColor3fv(lgRED);
		glBegin(GL_LINE_STRIP);
		size_t nCount = m_vPath.size();
		for ( unsigned int i = 0; i < ((m_bClosed) ? nCount+1 : nCount); ++i ) {
			rms::Frame3f & frame = m_vPath[i%nCount];
			Wml::Vector3f vNormOff = frame.Z() * fNormOff * 1.0f;
			glVertex3fv( frame.Origin() + vNormOff );
		}
		glEnd();

		// render selection verts
		//glPointSize(5.0f);
		//glBegin(GL_POINTS);
		//glColor3fv(lgBLUE);
		//nCount = m_vPath.size();
		//for ( unsigned int i = 0; i < nCount; ++i ) {
		//	rms::Frame3f & frame = m_vPath[i];
		//	Wml::Vector3f vNormOff = frame.Z() * fNormOff * 1.0f;
		//	glVertex3fv( frame.Origin() + vNormOff );
		//}
		//glEnd();
	}
	if ( ! m_vCurStroke.empty() ) {
		// render current stroke
		glLineWidth(3.0f);
		glColor3fv(lgGREEN);
		glBegin(GL_LINE_STRIP);
		size_t nCount = m_vCurStroke.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			rms::Frame3f & frame = m_vCurStroke[i];
			Wml::Vector3f vNormOff = frame.Z() * fNormOff * 1.0f;
			glVertex3fv( frame.Origin() + vNormOff );
		}
		glEnd();

	}


	glEnable(GL_LIGHTING);

	// render selection tris
//	glColor3f(0.43f, 1.0f, 0.6f);		// cool green
//	glColor3f(1.0f, 0.0f, 0.0f);		// red
	//glColor3f(0.95f, 0.45f, 0.0f);		// orange
	if ( bWireframe ) {
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f, 1.0f);
	}

	if ( ! m_bGLDisplayListValid ) {
	
		if ( m_nGLDisplayList == -1 )
			m_nGLDisplayList = glGenLists(1);
		glNewList(m_nGLDisplayList, GL_COMPILE_AND_EXECUTE);

		glColor3fv(vColor);
		Wml::Vector3f vTri[3], vNorm[3];
		std::set<unsigned int>::iterator curt(m_vInterior.begin()), endt(m_vInterior.end());
		glBegin(GL_TRIANGLES);
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt++;
			IMesh::VertexID nTri[3];
			m_pMesh->GetTriangle(tID, nTri);
			glNormal3fv( m_vFaceNormalCache[tID] );
			glVertex3fv( m_pMesh->GetVertex(nTri[0]) );
			glVertex3fv( m_pMesh->GetVertex(nTri[1]) );
			glVertex3fv( m_pMesh->GetVertex(nTri[2]) );
		}
		glEnd();


		// render boundary tris
		glColor3fv(lgGREEN);
		curt = m_vBoundaryTris.begin();   endt = m_vBoundaryTris.end();
		glBegin(GL_TRIANGLES);
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt++;
			IMesh::VertexID nTri[3];
			m_pMesh->GetTriangle(tID, nTri);
			glNormal3fv( m_vFaceNormalCache[tID] );
			glVertex3fv( m_pMesh->GetVertex(nTri[0]) );
			glVertex3fv( m_pMesh->GetVertex(nTri[1]) );
			glVertex3fv( m_pMesh->GetVertex(nTri[2]) );
		}
		glEnd();

		glEndList();
		m_bGLDisplayListValid = true;

		//glCallList( m_nGLDisplayList );

	} else {
		glCallList( m_nGLDisplayList );
	}


	glPopAttrib();	
}



void SurfaceAreaSelection::GetTriangleSet( std::set<IMesh::TriangleID> & vTriangles, bool bSelected )
{
	vTriangles.clear();
	if ( bSelected ) {
		vTriangles = m_vInterior;
	} else {
		VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt++;
			if ( m_vInterior.find(tID) == m_vInterior.end() )
				vTriangles.insert( tID );
		}
	}
}

void SurfaceAreaSelection::GetTriangleSet( std::vector<IMesh::TriangleID> & vTriangles, bool bSelected )
{
	vTriangles.clear();
	if ( bSelected ) {
		vTriangles.insert( vTriangles.begin(), m_vInterior.begin(), m_vInterior.end() );
	} else {
		VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt++;
			if ( m_vInterior.find(tID) == m_vInterior.end() )
				vTriangles.push_back( tID );
		}
	}
}


void SurfaceAreaSelection::GetVertexSet( std::vector<IMesh::VertexID> & vVertices, bool bSelected )
{
	vVertices.resize(0);
	std::vector<bool> vSelected(m_pMesh->GetMaxVertexID(), false);
	IMesh::VertexID vTri[3];

	if ( bSelected ) {
		std::set<IMesh::TriangleID>::iterator vcur(m_vInterior.begin()), vend(m_vInterior.end());
		while ( vcur != vend ) {
			IMesh::TriangleID tID = (*vcur++);
			m_pMesh->GetTriangle(tID, vTri);
			for ( int k = 0; k < 3; ++k ) {
				if ( ! vSelected[vTri[k]] ) {
					vVertices.push_back(vTri[k]);
					vSelected[vTri[k]] = true;
				}
			}
		} 
	} else {
		VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt++;
			if ( m_vInterior.find(tID) == m_vInterior.end() ) {
				m_pMesh->GetTriangle(tID, vTri);
				for ( int k = 0; k < 3; ++k ) {
					if ( ! vSelected[vTri[k]] ) {
						vVertices.push_back(vTri[k]);
						vSelected[vTri[k]] = true;
					}
				}
			}
		}
	}
}



void SurfaceAreaSelection::GetSubMesh( rms::VFTriangleMesh & mesh, bool bSelected )
{
	mesh.Clear(false);

	std::vector<IMesh::VertexID> vSaveTris;
	GetTriangleSet( vSaveTris, bSelected );

	MeshUtils::GetSubMesh( *m_pMesh, mesh, vSaveTris );
}


void SurfaceAreaSelection::SaveSelectionMesh(const char * pFilename)
{
	VFTriangleMesh save;
	GetSubMesh(save, true);
	
	std::string errString;
	save.WriteOBJ(pFilename, errString);
}


void SurfaceAreaSelection::SaveSelection(const char * pFilename)
{
	std::ofstream out(pFilename);
	if ( ! out )
		return;
	//size_t nCount = m_vOriginalPath.size();
	//for ( unsigned int i = 0; i < nCount; ++i ) 
	//	out << m_vOriginalPath[i].Origin() << std::endl;
	std::set<IMesh::TriangleID>::iterator vcur(m_vInterior.begin()), vend(m_vInterior.end());
	while ( vcur != vend )
		out << *vcur++ << std::endl;
	out.close();
}

void SurfaceAreaSelection::LoadSelection(const char * pFilename)
{
	Reset();

	std::ifstream in(pFilename);
	if ( ! in )
		return;

	//static const float fInvalid = std::numeric_limits<float>::max();
	//std::vector<rms::Frame3f> vPath;
	//while ( in ) {
	//	float fVals[3] = {fInvalid, fInvalid, fInvalid};
	//	in >> fVals[0] >> fVals[1] >> fVals[2];
	//	if ( fVals[0] != fInvalid && fVals[1] != fInvalid && fVals[2] != fInvalid ) {
	//		Wml::Vector3f vVert( fVals[0], fVals[1], fVals[2] );
	//		rms::Frame3f vFrame(vVert);
	//		vPath.push_back( vFrame );
	//	}
	//}
	//
	//m_vPath = vPath;
	//TryClose();

	while ( in ) {
		IMesh::TriangleID vID = IMesh::InvalidID;
		in >> vID;
		if ( vID != IMesh::InvalidID ) {
			if ( m_pMesh && m_pMesh->IsTriangle(vID) )
				m_vInterior.insert(vID);
		}
	}
	m_bClosed = true;

	m_vCurStroke.clear();
	m_vPath.clear();
}




