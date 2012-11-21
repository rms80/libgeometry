// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "IMeshUVBVTree.h"

#include <limits>
#include <VectorUtil.h>
#include <Wm4DistVector3Triangle3.h>
#include <Wm4ContPointInPolygon2.h>

//#include <WmlIntrLin3Tri3.h>
//#include <WmlDistVec3Tri3.h>
//#include <WmlContPointInPolygon2.h>

#include <rmsdebug.h>

using namespace rms;

IMeshUVBVTree::IMeshUVBVTree( )
{
	m_pMesh = NULL;
	m_pRoot = NULL;
}

IMeshUVBVTree::IMeshUVBVTree( IMesh * pMesh )
{
	m_pMesh = pMesh;
	m_pRoot = NULL;
}

void IMeshUVBVTree::SetMesh( IMesh * pMesh )
{
	Clear();
	m_pMesh = pMesh;
}


bool IMeshUVBVTree::FindTriangle( const Wml::Vector2f & vUV, IMesh::TriangleID & nTri )
{
	if ( m_pRoot == NULL )
		Initialize();
	if ( ! m_pRoot )
		return false;

	nTri = IMesh::InvalidID;
	bool bFound = FindTriangle( m_pRoot, vUV, nTri );
	return bFound;
}


bool IMeshUVBVTree::FindTriangle( IMeshUVBVTree::IMeshUVBVNode * pNode, const Wml::Vector2f & vUV, IMesh::TriangleID & nTriID )
{
	// if node is leaf, test box and then tri
	if ( pNode->IsLeaf() ) {
		if ( IsInside(pNode, vUV.X(), vUV.Y()) ) {

			// ok, hit box, now try the triangle
			IMesh::TriangleID tID = pNode->GetTriangleID();
			Wml::Vector2f vTriUV[3];
			if ( ! m_pMesh->GetTriangleUV(tID, 0, vTriUV) )
				return false;

			Wml::PointInPolygon2f piquery(3, vTriUV);
			if ( piquery.ContainsConvexOrderN(vUV) ) {
				nTriID = tID;
				return true;
			}
		}

	} else { 

		if ( IsInside(pNode, vUV.X(), vUV.Y()) ) {

			// ok, hit this box, drop to children. Have to check and maybe expand first...
			if ( pNode->HasChildren() == false ) 
				ExpandNode( pNode );
			lgASSERT( pNode->pLeft && pNode->pRight );

			bool bLeft = FindTriangle( pNode->pLeft, vUV, nTriID );
			bool bRight = FindTriangle( pNode->pRight, vUV, nTriID );
			if ( bLeft || bRight )
				return true;
		}
	}

	return false;
}




bool IMeshUVBVTree::FindNearestTriangle( const Wml::Vector2f & vUV, Wml::Vector2f & vNearest, IMesh::TriangleID & nTri )
{
	if ( m_pRoot == NULL )
		Initialize();
	if ( ! m_pRoot )
		return false;

	nTri = IMesh::InvalidID;
	float fNearest = std::numeric_limits<float>::max();
	bool bFound = FindNearestTriangle( m_pRoot, vUV, vNearest, fNearest, nTri );
	return bFound;
}

bool IMeshUVBVTree::FindNearestTriangle( IMeshUVBVTree::IMeshUVBVNode * pNode, const Wml::Vector2f & vUV, 
										 Wml::Vector2f & vNearest, float & fNearest, IMesh::TriangleID & nTriID )
{
	float fDistance;

	// if node is leaf, test box and then tri
	if ( pNode->IsLeaf() ) {

		// ok, hit box, now try the triangle
		IMesh::TriangleID tID = pNode->GetTriangleID();
		Wml::Vector2f vTriUV[3];
		if ( ! m_pMesh->GetTriangleUV(tID, 0, vTriUV) )
			return false;

		// check if point is inside triangle...if so, distance is 0
		Wml::PointInPolygon2f piquery(3, vTriUV);
		if ( IsInside(pNode, vUV.X(), vUV.Y()) && piquery.ContainsConvexOrderN(vUV ) ) {
			fNearest = 0;
			vNearest = vUV;
			nTriID = tID;
			return true;

		} else if ( (fDistance = MinDistance(pNode,vUV)) < fNearest ) {

			// ok, hit box, now try the triangle (use 3D triangle dist...)
			Wml::Triangle3f vTri( 
				Wml::Vector3f(vTriUV[0].X(), vTriUV[0].Y(), 0.0f),
				Wml::Vector3f(vTriUV[1].X(), vTriUV[1].Y(), 0.0f),
				Wml::Vector3f(vTriUV[2].X(), vTriUV[2].Y(), 0.0f) );
			Wml::Vector3f vPoint( vUV.X(), vUV.Y(), 0.0f );
			Wml::DistVector3Triangle3f dquery(vPoint, vTri);
			float fTriDist = dquery.Get();
			if ( fTriDist < fNearest ) {
				Wml::Vector3f vNearest3 = dquery.GetClosestPoint1();
				vNearest = Wml::Vector2f( vNearest3.X(), vNearest3.Y() );
				nTriID = pNode->GetTriangleID();
				return true;
			}
		}

	} else { 

		if ( (fDistance = MinDistance(pNode,vUV)) < fNearest ) {

			// ok, hit this box, drop to children. Have to check and maybe expand first...
			if ( pNode->HasChildren() == false ) 
				ExpandNode( pNode );
			lgASSERT( pNode->pLeft && pNode->pRight );

			bool bLeft = FindNearestTriangle( pNode->pLeft, vUV, vNearest, fNearest, nTriID );
			bool bRight = FindNearestTriangle( pNode->pRight, vUV, vNearest, fNearest, nTriID );
			if ( bLeft || bRight )
				return true;
		}
	}

	return false;
}




void IMeshUVBVTree::Clear()
{
	m_vTriangles.resize(0);
	m_vNodePool.ClearAll();
	m_pRoot = NULL;
	m_nNodeIDGen = 1;
	m_nMaxTriangle = 0;
}

IMeshUVBVTree::IMeshUVBVNode * IMeshUVBVTree::GetNewNode()
{
	IMeshUVBVNode * pNode = m_vNodePool.Allocate();
	pNode->ID = m_nNodeIDGen++;
	pNode->pLeft = NULL;
	pNode->pRight = NULL;
	pNode->SetTriangleID(0);
	return pNode;
}

void IMeshUVBVTree::Initialize()
{
	Clear();

	if ( ! m_pMesh->HasUVSet(0) )
		DebugBreak();
	IMesh::UVSet & uvset = m_pMesh->GetUVSet(0);

	m_pRoot = GetNewNode();

	// hacky, should only resize as much as we need...
	m_vTriangles.resize(0);

	// insert triangles w/ UV's
	unsigned int nIndex = 0;
	IMesh::VertexID nTri[3];
	IMesh::ITriIterator curt(m_pMesh->BeginITriangles()), endt(m_pMesh->EndITriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt;  ++curt;
		m_pMesh->GetTriangle( tID, nTri );
		bool b1 = uvset.HasUV(nTri[0]);
		bool b2 = uvset.HasUV(nTri[1]);
		bool b3 = uvset.HasUV(nTri[2]);
		if ( b1 && b2 && b3  ) {
			TriangleEntry entry;
			entry.nNodeID = m_pRoot->ID;
			entry.triID = tID;
			entry.nJump = 1;
			m_vTriangles.push_back(entry);
			++nIndex;
		}
	}
	

	m_nMaxTriangle = (unsigned int)m_vTriangles.size();
	if ( m_nMaxTriangle == 0 )
		return;

	m_pRoot->SetIndex(0);
	ComputeBox(m_pRoot);
}


void IMeshUVBVTree::ComputeBox( IMeshUVBVNode * pNode )
{
	Wml::Vector2f vTriUV[3];

	if ( pNode->IsLeaf() ) {
		m_pMesh->GetTriangleUV( pNode->GetTriangleID(), 0, vTriUV );
		pNode->Box = Wml::AxisAlignedBox2f(
			vTriUV[0].X(), vTriUV[0].X(), vTriUV[0].Y(), vTriUV[0].Y() );
		for ( int j = 1; j < 3; ++j )
			pNode->Union( vTriUV[j].X(), vTriUV[j].Y() );

	} else { 
		unsigned int nIndex = pNode->GetIndex();

		// initialize first box
		m_pMesh->GetTriangleUV( m_vTriangles[nIndex].triID, 0, vTriUV );
		pNode->Box = Wml::AxisAlignedBox2f(
			vTriUV[0].X(), vTriUV[0].X(), vTriUV[0].Y(), vTriUV[0].Y() );
		for ( int j = 1; j < 3; ++j )
			pNode->Union( vTriUV[j].X(), vTriUV[j].Y() );
		nIndex = GetNextEntryIdx(pNode, nIndex);

		// iterate over rest of boxes
		while ( nIndex < m_nMaxTriangle ) {
			m_pMesh->GetTriangleUV( m_vTriangles[nIndex].triID, 0, vTriUV );
			for ( int j = 0; j < 3; ++j )
				pNode->Union( vTriUV[j].X(), vTriUV[j].Y() );
			nIndex = GetNextEntryIdx(pNode, nIndex);
		}
	}
}



void IMeshUVBVTree::ExpandNode( IMeshUVBVTree::IMeshUVBVNode * pNode )
{
	// have to split - will have 2 children, no matter what
	IMeshUVBVNode * pLeft = GetNewNode();    pNode->pLeft = pLeft;
	IMeshUVBVNode * pRight = GetNewNode();   pNode->pRight = pRight;
		
	// find axis means
	Wml::Vector2f vMeans( Wml::Vector2f::ZERO );
	Wml::Vector2f vTriUV[3];

	unsigned int nCount = 0;
	unsigned int nIndex = pNode->GetIndex();
	while ( nIndex < m_nMaxTriangle ) {
		if (! m_pMesh->GetTriangleUV( m_vTriangles[nIndex].triID, 0, vTriUV ) )
			DebugBreak();
		vMeans += (1.0f/3.0f) * Wml::Vector2f( vTriUV[0] + vTriUV[1] + vTriUV[2] );
		++nCount;
		nIndex = GetNextEntryIdx(pNode, nIndex);
	}
	vMeans *= 1.0f / (float)nCount;

	// split box on mean of largest axis
	float fWidth = pNode->Box.Max[0] - pNode->Box.Min[0];
	float fHeight = pNode->Box.Max[1] - pNode->Box.Min[1];
	int nSplit = (fWidth > fHeight) ? 0 : 1;

	//_RMSInfo("ExpandNode:  %d tris, Bounds: %f-%f %f-%f   Mean: %f %f   Split: %d\n",
	//	nCount, pNode->Box.Min[0], pNode->Box.Min[0], pNode->Box.Max[1],  pNode->Box.Min[1],
	//	vMeans.X(), vMeans.Y(), nSplit );

	//if ( ! Wml::Contained(pNode->Box, vMeans.X(), vMeans.Y() ) )
	//	DebugBreak();

	unsigned int nLastLeft = -1;
	unsigned int nLastRight = -1;
	unsigned int nLeftCount = 0;
	unsigned int nRightCount = 0;

	// split tris between left and right boxes
	nIndex = pNode->GetIndex();
	while ( nIndex < m_nMaxTriangle ) {
		unsigned int nNextIndex = GetNextEntryIdx(pNode, nIndex);

		m_pMesh->GetTriangleUV( m_vTriangles[nIndex].triID, 0, vTriUV );
		Wml::Vector2f vCentroid( vTriUV[0] + vTriUV[1] + vTriUV[2] );
		vCentroid *= (1.0f / 3.0f);

		if ( vCentroid[nSplit] < vMeans[nSplit] ) {		// assign to left
			m_vTriangles[nIndex].nNodeID = pLeft->ID;
			if ( nLastLeft == -1 ) {
				pLeft->SetIndex( nIndex );
			} else
				SetJump(nLastLeft, nIndex);
			nLastLeft = nIndex;
			++nLeftCount;

		} else {		// assign to right
			m_vTriangles[nIndex].nNodeID = pRight->ID;
			if ( nLastRight == -1 ) {
				pRight->SetIndex( nIndex );
			} else
				SetJump(nLastRight, nIndex);
			nLastRight = nIndex;
			++nRightCount;
		}

		nIndex = nNextIndex;
	}


	// handle bad case where some child ended up with no nodes
	if ( nLeftCount == 0 || nRightCount == 0 ) {
		IMeshUVBVNode * pUseNode = (nLeftCount == 0) ? pRight : pLeft;
		int nLastLeft = -1;
		int nLastRight = -1;
		nLeftCount = nRightCount = 0;

		bool bLeft = true;
		nIndex = pUseNode->GetIndex();
		while ( nIndex < m_nMaxTriangle ) {	
			unsigned int nNextIndex = GetNextEntryIdx(pUseNode, nIndex);

			if ( bLeft ) {		// assign to left
				m_vTriangles[nIndex].nNodeID = pLeft->ID;
				if ( nLastLeft == -1 ) {
					pLeft->SetIndex( nIndex );
				} else
					SetJump(nLastLeft, nIndex);
				nLastLeft = nIndex;
				++nLeftCount;

			} else {		// assign to right
				m_vTriangles[nIndex].nNodeID = pRight->ID;
				if ( nLastRight == -1 ) {
					pRight->SetIndex( nIndex );
					nLastRight = nIndex;
				} else
					SetJump(nLastRight, nIndex);
				nLastRight = nIndex;
				++nRightCount;
			}

			nIndex = nNextIndex;
			bLeft = ! bLeft;
		}
	}

	lgASSERT( nLeftCount != 0 && nRightCount != 0 );
	if ( nLeftCount == 0 || nRightCount == 0 )
		DebugBreak();
	
	
	// set leaf flags
	if ( nLeftCount == 1 ) {
		pLeft->SetTriangleID( m_vTriangles[pLeft->GetIndex()].triID );
		lgASSERT(pLeft->IsLeaf());
	}
	if ( nRightCount == 1 ) {
		pRight->SetTriangleID( m_vTriangles[pRight->GetIndex()].triID );
		lgASSERT(pRight->IsLeaf());
	}

	ComputeBox( pLeft );
	ComputeBox( pRight );
}


void IMeshUVBVTree::ExpandAll()
{
	if ( m_pRoot == NULL )
		Initialize();
	if ( m_pRoot )
		ExpandAll(m_pRoot);
}

void IMeshUVBVTree::ExpandAll( IMeshUVBVTree::IMeshUVBVNode * pNode )
{
	if ( ! pNode->IsLeaf() ) {
		ExpandNode(pNode);
		ExpandAll(pNode->pLeft);
		ExpandAll(pNode->pRight);
	}
}





bool IMeshUVBVTree::IsInside( IMeshUVBVNode * pNode, float fX,float fY )
{
	Wml::AxisAlignedBox2f & box = pNode->Box;
	return ( box.Min[0] <= fX && fX < box.Max[0] &&
			 box.Min[1] <= fY && fY < box.Max[1] );
}


float IMeshUVBVTree::MinDistance( IMeshUVBVNode * pNode, const Wml::Vector2f & vPoint )
{
	Wml::AxisAlignedBox2f & box = pNode->Box;
	float fDist[2] = {0,0};
	bool bIn[2] = {false,false};

	for ( int k = 0; k < 2; ++k ) {
		if (vPoint[k] > box.Max[k])
			fDist[k] = vPoint[k]-box.Max[k];
		else if ( vPoint[k] < box.Min[k] )
			fDist[k] = box.Min[k] - vPoint[k];
		else
			bIn[k] = true;
	}

    if ( bIn[0] && bIn[1] )
		return 0.0f;
	else
		return (float)sqrt( fDist[0]*fDist[0] + fDist[1]*fDist[1] );
}
