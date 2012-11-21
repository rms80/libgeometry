// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "MeshUtils.h"

#include <VectorUtil.h>
#include <DijkstraFrontProp.h>

#include "rmsdebug.h"
#include "rmsprofile.h"

using namespace rms;

MeshUtils::MeshUtils(void)
{
}

MeshUtils::~MeshUtils(void)
{
}


bool MeshUtils::MergeVertices( rms::VFTriangleMesh & mesh, float fThreshold, const std::vector<float> * pImportanceMap, rms::MeshSelection * pSelection )
{
	std::set<IMesh::VertexID> vUsed;
	std::vector<IMesh::EdgeID> vMerge;

	fThreshold *= fThreshold;

	// find edges that are too short
	VFTriangleMesh::edge_iterator cure(mesh.BeginEdges()), ende(mesh.EndEdges());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if ( mesh.IsBoundaryEdge(eID) )
			continue;

		IMesh::VertexID nEdge[2], nTri[3];
		mesh.GetEdge(eID, nEdge, nTri);
		if ( pSelection && 	 ( pSelection->Vertices().find(nEdge[0]) == pSelection->Vertices().end() ||
							   pSelection->Vertices().find(nEdge[1]) == pSelection->Vertices().end() )	   )
			   continue;


		Wml::Vector3f vEdge[2];
		mesh.GetEdge(eID, vEdge);
		float fLength = (vEdge[0]-vEdge[1]).SquaredLength();

		if ( pImportanceMap ) {
			float fImportance = 0.5f * ( (*pImportanceMap)[nEdge[0]] + (*pImportanceMap)[nEdge[1]] );
			fLength *= fImportance;
		}

		if ( fLength < fThreshold ) {
			IMesh::VertexID v[2];   IMesh::TriangleID t[2];
			mesh.GetEdge(eID, v, t);
			if ( mesh.IsBoundaryVertex(v[0]) && mesh.IsBoundaryVertex(v[1]) )
				continue;

			IMesh::VertexID nTri[6];
			mesh.GetTriangle(t[0],&nTri[0]);
			mesh.GetTriangle(t[1],&nTri[3]);
			bool bOK = true;
			for ( int k = 0; k < 6; ++k ) {
				if ( vUsed.find(nTri[k]) != vUsed.end() ) 
					bOK = false;
			}
			if ( bOK ) {
				vMerge.push_back(eID);
				for ( int k = 0; k < 6; ++k )
					vUsed.insert(nTri[k]);
			}
		}
	}


	// collapse edges
	size_t nCollapse = vMerge.size();
	int nReallyCollapsed = 0;
	for ( unsigned int i = 0; i < nCollapse; ++i ) {
		if ( mesh.CollapseEdge( vMerge[i] ) ) {
			++nReallyCollapsed;
		}
	}
	//_RMSInfo("Collapsed %d edges (of %d possibilities)\n", nReallyCollapsed, nCollapse);

	return (nReallyCollapsed > 0);
}


bool MeshUtils::CollapseSlivers( VFTriangleMesh & mesh, float fDegreeThreshold )
{
	std::vector<IMesh::TriangleID> vSlivers;

	// find sliver triangles
	float fThresh = fDegreeThreshold * Wml::Mathf::DEG_TO_RAD;
	VFTriangleMesh::triangle_iterator curt(mesh.BeginTriangles()), endt(mesh.EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		
		Wml::Vector3f nTri[3];
		mesh.GetTriangle(tID, nTri);
		float fMinAngle = std::numeric_limits<float>::max();
		for ( int k = 0; k < 3; ++k ) {
			Wml::Vector3f e1( nTri[(k+1)%3] - nTri[k] );		e1.Normalize();
			Wml::Vector3f e2( nTri[(k+2)%3] - nTri[k] );		e2.Normalize();
			float fAngle = rms::VectorAngle(e1, e2);
			if ( fAngle < fMinAngle )
				fMinAngle = fAngle;
		}
		if ( fMinAngle < fThresh )
			vSlivers.push_back(tID);
	}

	std::set<IMesh::EdgeID> vEdges;
	size_t nSlivers = vSlivers.size();
	for ( unsigned int k = 0; k < nSlivers; ++k ) {
		IMesh::VertexID nTri[3];
		mesh.GetTriangle( vSlivers[k], nTri );
		IMesh::EdgeID edges[3];
		edges[0] = mesh.FindEdge( nTri[0], nTri[1] );
		edges[1] = mesh.FindEdge( nTri[1], nTri[2] );
		edges[2] = mesh.FindEdge( nTri[2], nTri[0] );

		Wml::Vector3f vTri[3];
		mesh.GetTriangle( vSlivers[k], vTri );
		float vLen[3];
		vLen[0] = mesh.IsBoundaryEdge(edges[0]) ? std::numeric_limits<float>::max() : (vTri[1]-vTri[0]).SquaredLength();
		vLen[1] = mesh.IsBoundaryEdge(edges[1]) ? std::numeric_limits<float>::max() : (vTri[2]-vTri[1]).SquaredLength();
		vLen[2] = mesh.IsBoundaryEdge(edges[2]) ? std::numeric_limits<float>::max() : (vTri[0]-vTri[2]).SquaredLength();
		int nMin = 0;
		if ( vLen[1] < vLen[nMin] ) nMin = 1;
		if ( vLen[2] < vLen[nMin] ) nMin = 2;

		if ( mesh.IsBoundaryEdge(edges[nMin]) )
			lgBreakToDebugger();
		vEdges.insert(edges[nMin]);
	}

	int nReallyCollapsed = 0;
	std::set<IMesh::TriangleID>::iterator cure(vEdges.begin()), ende(vEdges.end());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if ( mesh.CollapseEdge( eID ) ) {
			++nReallyCollapsed;
		}
	}

	_RMSInfo("[MeshUtils::CollapseSlivers]  collapsed %d of %d\n", nReallyCollapsed, vEdges.size());
	return (nReallyCollapsed > 0);
}





bool MeshUtils::CollapseTipFaces( rms::VFTriangleMesh & mesh, bool bAll )
{
	bool bCollapsedAny = false;

	std::set<IMesh::EdgeID> vStartBoundaryEdges;
	VFTriangleMesh::edge_iterator cure(mesh.BeginEdges()), ende(mesh.EndEdges());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if ( mesh.IsBoundaryEdge(eID) )
			vStartBoundaryEdges.insert(eID);
	}

	bool bCollapsed = true;
	while ( bCollapsed ) {
		bCollapsed = false;

		std::vector<IMesh::VertexID> vTips;
		std::vector<IMesh::EdgeID> vEdges;

		VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
		while ( curv != endv ) {
			IMesh::VertexID vID = *curv++;

			vEdges.resize(0);
			
			IMesh::VtxNbrItr itr;
			itr.vID = vID;
			mesh.BeginVtxEdges(itr);
			IMesh::EdgeID eID = mesh.GetNextVtxEdges(itr);
			while ( eID != IMesh::InvalidID ) {
				vEdges.push_back(eID);
				eID = mesh.GetNextVtxEdges(itr);
			}

			size_t nEdges = vEdges.size();
			if ( nEdges != 3 )
				continue;
			//if ( mesh.IsBoundaryEdge(vEdges[0]) || mesh.IsBoundaryEdge(vEdges[1]) || mesh.IsBoundaryEdge(vEdges[2]) )
			//	continue;
			if ( vStartBoundaryEdges.find(vEdges[0]) != vStartBoundaryEdges.end() ||
				 vStartBoundaryEdges.find(vEdges[1]) != vStartBoundaryEdges.end() ||
				 vStartBoundaryEdges.find(vEdges[2]) != vStartBoundaryEdges.end() )
				 continue;

			vTips.push_back(vID);
		}

		size_t nTips = vTips.size();
		int nClipped = 0;
		for ( unsigned int i = 0; i < nTips; ++i ) {
			IMesh::VertexID vID = vTips[i];
			if ( ! mesh.IsVertex(vID) )
				continue;

			std::set<IMesh::VertexID> vNbrSet;
			std::vector<IMesh::VertexID> vNbrs;
			std::vector<IMesh::TriangleID> vTris;

			IMesh::VertexID nVerts[3];
			IMesh::VtxNbrItr itr;
			itr.vID = vID;
			mesh.BeginVtxTriangles(itr);
			IMesh::TriangleID tID = mesh.GetNextVtxTriangle(itr);
			while ( tID != IMesh::InvalidID ) {
				mesh.GetTriangle(tID, nVerts);
				for ( int k = 0; k < 3; ++k ) {
					if ( nVerts[k] == vID )
						continue;
					if ( vNbrSet.insert(nVerts[k]).second == false )
						vNbrs.push_back(nVerts[k]);
				}

				vTris.push_back(tID);
				tID = mesh.GetNextVtxTriangle(itr);
			}

			// sanity check
			if ( vNbrs.size() != 3 || vTris.size() != 3 ) {
				//lgBreakToDebugger();
				continue;
			}

			// have to do this to make sure that RemoveVertex does not result in neighbours
			// on boundary being deleted
			for ( unsigned int k = 0; k < vNbrs.size(); ++k )
				mesh.HACK_ManuallyIncrementReferenceCount(vNbrs[k]);

			mesh.RemoveVertex( vID );

			// check if we should add in face (would create invalid mesh if 
			// we have a tetrahedra
			std::vector<IMesh::VertexID> nNbrVerts(3);
			std::sort( vNbrs.begin(), vNbrs.end() );
			IMesh::VtxNbrItr itr2(vNbrs[0]);
			mesh.BeginVtxTriangles(itr2);
			tID = mesh.GetNextVtxTriangle(itr2);
			bool bSkipTri = false;
			while ( tID != IMesh::InvalidID && ! bSkipTri) {
				mesh.GetTriangle(tID, &nNbrVerts[0]);
				std::sort(nNbrVerts.begin(), nNbrVerts.end());
				if ( vNbrs[0] == nNbrVerts[0] && vNbrs[1] == nNbrVerts[1] && vNbrs[2] == nNbrVerts[2] )
					bSkipTri = true;
				tID = mesh.GetNextVtxTriangle(itr2);
			}
			if ( ! bSkipTri ) {

				// hacky bit to maintain vertex ordering
				IMesh::VertexID vFirst, vSecond, vThird;
				for ( int j = 0; j < 3; ++j ) {
					if ( nVerts[j] == vID ) {
						vFirst = nVerts[(j+1)%3];  vSecond = nVerts[(j+2)%3];
					}
				}
				for ( int j = 0; j < 3; ++j ) {
					if ( vNbrs[j] != vFirst && vNbrs[j] != vSecond )
						vThird = vNbrs[j];
				}
				mesh.AppendTriangle( vFirst, vSecond, vThird );

			} else {

				// might have created a fin - get rid of it
				for ( int k = 0; k < 3; ++k ) {
					if ( mesh.GetEdgeCount(vNbrs[k]) == 2 )
						mesh.RemoveVertex(vNbrs[k]);
				}

				// TODO: this removal might in turn create more fins - really ought to keep going...
			}

			for ( unsigned int k = 0; k < vNbrs.size(); ++k )
				mesh.HACK_ManuallyDecrementReferenceCount(vNbrs[k]);

			++nClipped;
			bCollapsedAny = true;
		}

		//_RMSInfo("Clipped %d tips (of %d possible)\n", nClipped, nTips);
		bCollapsed = nClipped > 0 && bAll;
	}

	return bCollapsedAny;
}




bool MeshUtils::CollapseFinFaces( rms::VFTriangleMesh & mesh, bool bAll )
{
	bool bCollapsedAny = false;

	std::set<IMesh::EdgeID> vStartBoundaryEdges;
	VFTriangleMesh::edge_iterator cure(mesh.BeginEdges()), ende(mesh.EndEdges());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if ( mesh.IsBoundaryEdge(eID) )
			vStartBoundaryEdges.insert(eID);
	}

	bool bCollapsed = true;
	while ( bCollapsed ) {
		bCollapsed = false;

		std::vector<IMesh::VertexID> vFins;
		std::vector<IMesh::EdgeID> vEdges;

		VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
		while ( curv != endv ) {
			IMesh::VertexID vID = *curv++;

			vEdges.resize(0);
			
			IMesh::VtxNbrItr itr;
			itr.vID = vID;
			mesh.BeginVtxEdges(itr);
			IMesh::EdgeID eID = mesh.GetNextVtxEdges(itr);
			while ( eID != IMesh::InvalidID ) {
				vEdges.push_back(eID);
				eID = mesh.GetNextVtxEdges(itr);
			}

			size_t nEdges = vEdges.size();
			if ( nEdges != 2 )
				continue;
//			if ( mesh.IsBoundaryEdge(vEdges[0]) || mesh.IsBoundaryEdge(vEdges[1]) )
			if ( vStartBoundaryEdges.find(vEdges[0]) != vStartBoundaryEdges.end() ||
				 vStartBoundaryEdges.find(vEdges[1]) != vStartBoundaryEdges.end() )
				continue;

			vFins.push_back(vID);
		}

		size_t nFins = vFins.size();
		int nClipped = 0;
		for ( unsigned int i = 0; i < nFins; ++i ) {
			IMesh::VertexID vID = vFins[i];
			mesh.RemoveVertex(vID);
			++nClipped;
			bCollapsedAny = true;
		}

		//_RMSInfo("Clipped %d tips (of %d possible)\n", nClipped, nTips);
		bCollapsed = nClipped > 0 && bAll;
	}

	return bCollapsedAny;
}







bool MeshUtils::CheckForFins( VFTriangleMesh & mesh )
{
	int nFins = 0;
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;

		if ( ! mesh.IsBoundaryVertex(vID) && mesh.GetEdgeCount(vID) == 2 )
			++nFins;
	}
	if ( nFins > 0 )
		_RMSInfo("Found %d fins!\n", nFins);
	return (nFins > 0);
}

bool MeshUtils::CheckMeshValidity( VFTriangleMesh & mesh, bool bTopologyChecks )
{
	bool bOK = true;

	// check verts
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		if ( ! mesh.IsVertex(vID) ) {
			bOK = false; lgBreakToDebugger();
		}

		// do nbr iters
		VFTriangleMesh::VtxNbrItr triItr(vID);
		mesh.BeginVtxTriangles(triItr);
		IMesh::TriangleID tID = mesh.GetNextVtxTriangle(triItr);
		while ( tID != IMesh::InvalidID ) {
			if ( ! mesh.IsTriangle(tID) ) {
				bOK = false; lgBreakToDebugger();
			}
			tID = mesh.GetNextVtxTriangle(triItr);
		}

		VFTriangleMesh::VtxNbrItr edgeItr(vID);
		mesh.BeginVtxEdges(edgeItr);
		IMesh::EdgeID eID = mesh.GetNextVtxEdges(edgeItr);
		while ( eID != IMesh::InvalidID ) {
			if ( ! mesh.IsEdge(eID) ) {
				bOK = false; lgBreakToDebugger();
			}
			eID = mesh.GetNextVtxEdges(edgeItr);
		}

		if ( bTopologyChecks ) {
			bool bIsBoundary = mesh.IsBoundaryVertex(vID);

			if ( ! bIsBoundary && mesh.GetTriangleCount(vID) < 3 ) {
				bOK = false; lgBreakToDebugger();
			}

			if ( ! bIsBoundary && mesh.GetEdgeCount(vID) < 3 ) {
				bOK = false; lgBreakToDebugger();
			}
		}
	}


	// check that each tri vert is a real vert
	VFTriangleMesh::triangle_iterator curt(mesh.BeginTriangles()), endt(mesh.EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		if (! mesh.IsTriangle(tID) ) {
			bOK = false; lgBreakToDebugger();
		}
		IMesh::VertexID nTri[3];
		mesh.GetTriangle(tID, nTri);
		for ( int j = 0; j < 3; ++j ) {
			if ( ! mesh.IsVertex( nTri[j]) ) {
				bOK = false; lgBreakToDebugger();
			}
		}

		Wml::Vector3f vVerts[3];
		mesh.GetTriangle(tID, vVerts);
		float fArea = Area(vVerts[0], vVerts[1], vVerts[2]);
		if ( fArea < 0.00001f ) {
			bOK = false; lgBreakToDebugger();
		}
	}

	// check that each edge vert is a real vert
	VFTriangleMesh::edge_iterator cure(mesh.BeginEdges()), ende(mesh.EndEdges());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if (! mesh.IsEdge(eID) ) {
			bOK = false; lgBreakToDebugger();
		}
		IMesh::VertexID nVtx[2];
		IMesh::TriangleID nTri[2];
		mesh.GetEdge(eID, nVtx, nTri);
		for ( int j = 0; j < 2; ++j ) {
			if ( ! mesh.IsVertex( nVtx[j]) ) {
				bOK = false; lgBreakToDebugger();
			}
			if ( nTri[j] != IMesh::InvalidID && ! mesh.IsTriangle( nTri[j]) ) {
				bOK = false; lgBreakToDebugger();
			}
		}
	}

	return bOK;
}





void MeshUtils::CopyMesh( IMesh * pFrom, IMesh * pTo, VertexMap * pVtxMap, TriangleMap * pFaceMap, bool bCompact )
{
	pTo->Clear(false);

	VertexMap vLocalVertMap(true);
	VertexMap & VMap = (pVtxMap != NULL) ? *pVtxMap : vLocalVertMap;
	VMap.Resize(pFrom->GetMaxVertexID(), pFrom->GetMaxVertexID());

	std::set<IMesh::VertexID> vKeep;
	if ( bCompact ) {
		IMesh::VertexID nTri[3];
		IMesh::ITriIterator curt(pFrom->BeginITriangles()), endt(pFrom->EndITriangles());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt;  ++curt;
			pFrom->GetTriangle(tID, nTri);
			vKeep.insert(nTri[0]);		vKeep.insert(nTri[1]);		vKeep.insert(nTri[2]);  
		}
	}

	Wml::Vector3f vVertex, vNormal;
	IMesh::IVtxIterator curv(pFrom->BeginIVertices()), endv(pFrom->EndIVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv;  ++curv;
		if ( bCompact && vKeep.find(vID) == vKeep.end() ) {
			VMap.SetMap(vID, IMesh::InvalidID);
			continue;
		}
		pFrom->GetVertex(vID, vVertex, &vNormal );
		IMesh::VertexID vNewID = pTo->AppendVertex(vVertex, &vNormal);
		VMap.SetMap(vID, vNewID);
	}

	if ( pFaceMap )
		pFaceMap->Resize( pFrom->GetMaxTriangleID(), pFrom->GetMaxTriangleID() );
	IMesh::VertexID nTri[3];
	IMesh::ITriIterator curt(pFrom->BeginITriangles()), endt(pFrom->EndITriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt;  ++curt;
		pFrom->GetTriangle(tID, nTri);
		for ( int j = 0; j < 3; ++j )
			nTri[j] = VMap.GetNew(nTri[j]);
		IMesh::TriangleID tNewID = pTo->AppendTriangle( nTri[0], nTri[1], nTri[2] );
		if ( pFaceMap )
			pFaceMap->SetMap(tID, tNewID);
	}
}



void MeshUtils::GetSubMesh( rms::VFTriangleMesh & original, 
							rms::VFTriangleMesh & submesh, 
							const BitSet & vTris,
							VertexMap * pVtxMap,
							TriangleMap * pFaceMap,
							bool bCopyUVs )
{
	std::vector<IMesh::TriangleID> vTriV;
	size_t nCount = vTris.size();
	for ( unsigned int k = 0; k < nCount; ++k )
		if ( vTris[k] )
			vTriV.push_back(k);
	GetSubMesh( original, submesh, vTriV, pVtxMap, pFaceMap, bCopyUVs);
}


void MeshUtils::GetSubMesh( rms::VFTriangleMesh & original, 
							rms::VFTriangleMesh & submesh, 
							const std::vector<rms::IMesh::TriangleID> & vTris,
							VertexMap * pVtxMap,
							TriangleMap * pFaceMap,
							bool bCopyUVs )
{
	static bool bVerbose = false;
	if ( bVerbose ) _RMSTUNE_start(2);

	submesh.Clear(false);

	// make list of verts to save
	IMesh::VertexID nTri[3], nNewTri[3];
	IMesh::VertexID nMaxVID = original.GetMaxVertexID();
	std::vector<bool> vVerts( nMaxVID, false );
	size_t nCount = vTris.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		IMesh::TriangleID tID = vTris[i]; 
		original.GetTriangle(tID, nTri);
		vVerts[nTri[0]] = true;   vVerts[nTri[1]] = true;  vVerts[nTri[2]] = true;
	}

	if (bVerbose) {
		_RMSTUNE_end(2);
		_RMSInfo("MeshUtils::GetSubMesh - make vert list - %f\n", _RMSTUNE_time(2));
	}

	VertexMap * VMap = pVtxMap;
	bool bDeleteVMap = false;
	if ( VMap == NULL ) {
		if ( bVerbose ) _RMSTUNE_start(2);   
		bool bUseSparse = (original.GetMaxVertexID() > 1024) &&  ( vTris.size() < original.GetMaxTriangleID()/8 );
		VMap = new VertexMap(bUseSparse);
		if ( bVerbose ) {
			_RMSTUNE_end(2);
			_RMSInfo("MeshUtils::GetSubMesh - create local vertmap - %f\n", _RMSTUNE_time(2));
		}
		bDeleteVMap = true;
	}

	// add verts
	if ( bVerbose ) _RMSTUNE_start(2);
	VMap->Resize((unsigned int)original.GetMaxVertexID(), (unsigned int)vVerts.size());
	for ( unsigned int vID = 0; vID < nMaxVID; ++vID ) {
		if ( ! vVerts[vID] )
			continue;
		Wml::Vector3f vVertex, vNormal;
		original.GetVertex(vID, vVertex, &vNormal);
		IMesh::VertexID vNewID = submesh.AppendVertex(vVertex, &vNormal);
		VMap->SetMap( vID, vNewID );
	}
	if (bVerbose ) {
		_RMSTUNE_end(2);
		_RMSInfo("MeshUtils::GetSubMesh - add verts - %f\n", _RMSTUNE_time(2));
	}

	if ( bCopyUVs && original.HasUVSet(0)) {
		if ( bVerbose ) _RMSTUNE_start(2);

		if (! submesh.HasUVSet(0) )
			submesh.AppendUVSet();
		submesh.InitializeUVSet(0);
		for ( unsigned int vID = 0; vID < nMaxVID; ++vID ) {
			if ( ! vVerts[vID] )
				continue;
			Wml::Vector2f vUV;
			if ( original.GetUV(vID, 0, vUV) ) {
				IMesh::VertexID vNewID = VMap->GetNew(vID);
				submesh.AddUV(vNewID, 0, vUV);
			}
		}

		if (bVerbose ) {
			_RMSTUNE_end(2);
			_RMSInfo("MeshUtils::GetSubMesh - copy UVs - %f\n", _RMSTUNE_time(2));
		}
	} 





	// add tris
	if ( bVerbose ) _RMSTUNE_start(2);
	if ( pFaceMap )
		pFaceMap->Resize(original.GetMaxTriangleID(), (unsigned int)nCount);
	for ( unsigned int i = 0; i < nCount; ++i ) {
		IMesh::TriangleID tID = vTris[i]; 
		original.GetTriangle(tID, nTri);
		for ( int k = 0; k < 3; ++k )
			nNewTri[k] = VMap->GetNew( nTri[k] );
		IMesh::TriangleID tNewID = submesh.AppendTriangle( nNewTri[0], nNewTri[1], nNewTri[2] );
		if ( pFaceMap )
			pFaceMap->SetMap(tID, tNewID);
	}
	if (bVerbose ) {
		_RMSTUNE_end(2);
		_RMSInfo("MeshUtils::GetSubMesh - add tris - %f\n", _RMSTUNE_time(2));
	}

	if ( bDeleteVMap )
		delete VMap;
}




void MeshUtils::SetMeshXYZtoUV( VFTriangleMesh & mesh, const Wml::Vector3f & vSetNormal, int nCoordMap[2],
							   std::vector<Wml::Vector3f> * pvXYZPositions, std::vector<Wml::Vector3f> * pvXYZNormals )
{
	if ( pvXYZPositions )
		pvXYZPositions->resize( mesh.GetMaxVertexID() );
	if ( pvXYZNormals )
		pvXYZNormals->resize( mesh.GetMaxVertexID() );

	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		Wml::Vector3f vVertex, vNormal;  Wml::Vector2f vUV;
		mesh.GetVertex(vID, vVertex, &vNormal);
		if ( pvXYZPositions )
			(*pvXYZPositions)[vID] = vVertex;
		if ( pvXYZNormals ) 
			(*pvXYZNormals)[vID] = vNormal;
		if ( ! mesh.GetUV(vID, 0, vUV) )
			vUV = Wml::Vector2f(-1, -1);
		vVertex = Wml::Vector3f::ZERO;
		vVertex[ nCoordMap[0] ] = vUV.X();
		vVertex[ nCoordMap[1] ] = vUV.Y();
		mesh.SetVertex(vID, vVertex, & vSetNormal);
	}	
}


void MeshUtils::CopyMeshXYZtoUV( VFTriangleMesh & mesh, int nCoordMap[2], IMesh::UVSetID nSetID )
{
	while ( ! mesh.HasUVSet(nSetID) )
		mesh.AppendUVSet();
	mesh.InitializeUVSet(nSetID);

	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		Wml::Vector3f vXYZ;
		mesh.GetVertex(vID, vXYZ);
		mesh.SetUV(vID, nSetID, Wml::Vector2f( vXYZ[nCoordMap[0]], vXYZ[nCoordMap[1]] ) );
	}
}



void MeshUtils::CopyUVs( const VFTriangleMesh & from, VFTriangleMesh & to, IMesh::UVSetID nFromSetID, IMesh::UVSetID nToSetID,
								bool bBoundaryOnly )
{
	if ( ! to.HasUVSet(nToSetID) ) {
		while ( to.AppendUVSet() != nToSetID )
			;
		to.InitializeUVSet( nToSetID );
	}
	to.ClearUVSet( nToSetID );

	Wml::Vector2f vUV;
	VFTriangleMesh::vertex_iterator curv(from.BeginVertices()), endv(from.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		if ( bBoundaryOnly && ! from.IsBoundaryVertex(vID) )
			continue;
		from.GetUV(vID, nFromSetID, vUV);
		to.SetUV(vID, nToSetID, vUV);
	}
}


void MeshUtils::CopyUVs( IMesh::UVSet & from, IMesh::UVSet & to, rms::VertexMap * pVtxMap )
{
	SparseArray<Wml::Vector2f>::iterator cur(from.UV().begin()), end(from.UV().end());
	while (cur != end) {
		IMesh::VertexID vID = cur.index();
		Wml::Vector2f vUV = *cur++;
		if ( pVtxMap )
			vID = pVtxMap->GetNew(vID);
		if ( vID == IMesh::InvalidID )		// no longer in map!
			continue;
		to.SetUV(vID, vUV);
	}
}




void MeshUtils::CopyUVs( const IMesh & from, VFTriangleMesh & to, IMesh::UVSetID nFromSetID, IMesh::UVSetID nToSetID,
								const VertexMap & VMap, bool bBoundaryOnly )
{
	if ( ! to.HasUVSet(nToSetID) ) {
		while ( to.AppendUVSet() != nToSetID )
			;
	}
	to.ClearUVSet( nToSetID );
	to.InitializeUVSet( nToSetID );

	Wml::Vector2f vUV;
	IMesh::IVtxIterator curv(from.BeginIVertices()), endv(from.EndIVertices());
	while ( curv != endv ) {
		IMesh::VertexID vFromID = *curv; curv++;
		IMesh::VertexID vToID = VMap.GetNew(vFromID);

		if ( bBoundaryOnly && ! from.IsBoundaryVertex(vToID) )
			continue;
		if ( from.GetUV(vFromID, nFromSetID, vUV) ) 
			to.SetUV(vToID, nToSetID, vUV);
	}
}





void MeshUtils::CopyBoundaryUVs( VFTriangleMesh & from, VFTriangleMesh & to, IMesh::UVSetID nFromSetID, IMesh::UVSetID nToSetID,
		BoundaryLoopMap & bmap )
{
	if ( ! to.HasUVSet(nToSetID) ) {
		while ( to.AppendUVSet() != nToSetID )
			;
		to.InitializeUVSet( nToSetID );
	}
	to.ClearUVSet( nToSetID );

	Wml::Vector2f vUV;
	size_t nCount = bmap.vFrom.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		IMesh::VertexID vFromID = bmap.vFrom[i];
		IMesh::VertexID vToID = bmap.vTo[i];
		from.GetUV( vFromID, nFromSetID, vUV );
		to.SetUV( vToID, nToSetID, vUV );
	}
}




void MeshUtils::MergeBoundaryLoops( rms::VFTriangleMesh & mesh,
		const std::vector<IMesh::VertexID> & vFrom, const std::vector<IMesh::VertexID> & vTo  )
{
	size_t nCount = vFrom.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		IMesh::VertexID vFromID = vFrom[i];
		IMesh::VertexID vToID = vTo[i];
		mesh.Weld(vToID, vFromID);
	}
}


void MeshUtils::MergeAllBoundaryLoops( VFTriangleMesh & mesh, std::vector< std::vector<IMesh::VertexID> > * vToLoops, float fDistThresh )
{
	// have to iterate because IDs change after each merge!!
	bool bDone = false;
	while (! bDone ) {
		std::vector< std::vector< IMesh::VertexID > > vLoops;
		FindBoundaryLoops( mesh, vLoops );

		std::vector< BoundaryLoopMap > vMaps;
		FindBoundaryLoopMaps( mesh, vLoops, mesh, vLoops, vMaps, fDistThresh );

		if ( vMaps.size() == 0 ) {
			bDone = true;
		} else {
			if ( vToLoops != NULL )
				vToLoops->push_back( vMaps[0].vTo );
			MergeBoundaryLoops( mesh, vMaps[0].vFrom, vMaps[0].vTo );
		}
	}

}




void MeshUtils::FindBoundaryLoopMaps(
		IMesh & mesh1, const std::vector< std::vector< IMesh::VertexID > > & vLoops1,
		IMesh & mesh2, const std::vector< std::vector< IMesh::VertexID > > & vLoops2,
		std::vector< BoundaryLoopMap > & vMaps, float fDistThresh )
{
	fDistThresh *= fDistThresh; 

	bool bSameMesh = ( (&mesh1) == (&mesh2) );

	size_t nCount1 = vLoops1.size();
	size_t nCount2 = vLoops2.size();
	for ( unsigned int n1 = 0; n1 < nCount1; ++n1 ) {
	for ( unsigned int n2 = 0; n2 < nCount2; ++n2 ) {
		if ( bSameMesh && n1 == n2 )
			continue;
		if ( vLoops1[n1].size() != vLoops2[n2].size() ) 
			continue;

		const std::vector< IMesh::VertexID > & vLoopA = vLoops1[n1];
		size_t nCountA = vLoopA.size();
		const std::vector< IMesh::VertexID > & vLoopB = vLoops2[n2];
		size_t nCountB = vLoopB.size();

		// find vertex w/ zero-distance in both loops
		Wml::Vector3f vA, vB; 
		IMesh::VertexID vIDA = vLoopA[0];
		mesh1.GetVertex(vIDA, vA);
		IMesh::VertexID vIDB = IMesh::InvalidID;
		int iLoopB = 0;
		float fMinDist = 9999999.0f;
		for ( unsigned int i = 0; i < nCountB; ++i ) {
			mesh2.GetVertex( vLoopB[i], vB );
			float fLenSqr = (vA-vB).SquaredLength();
			if ( fLenSqr < fMinDist && fLenSqr < fDistThresh ) {
				vIDB = vLoopB[i];
				iLoopB = i;
				fMinDist = fLenSqr;
			}
		}
		if ( vIDB == IMesh::InvalidID )
			continue;
	
		// ok got one - now generate correspondence
		BoundaryLoopMap bmap;
		bmap.nLoop1Index = n1;
		bmap.nLoop2Index = n2;
		bmap.vFrom = vLoopA;

		bmap.vTo.push_back(vIDB);

		// figure out if B is going in same or opposite direction
		bool bFailed = false;
		iLoopB = (iLoopB + 1) % (int)nCountB;
		mesh2.GetVertex( vLoopB[ iLoopB ], vB );
		mesh1.GetVertex( vLoopA[1], vA );
		if ( (vA-vB).SquaredLength() < fDistThresh ) {
			while ( bmap.vTo.size() < nCountA ) {
				bmap.vTo.push_back( vLoopB[iLoopB] );
				iLoopB = (iLoopB + 1) % (int)nCountB;
			}

		} else {
			mesh1.GetVertex( vLoopA.back(), vA );
			if ( (vA-vB).SquaredLength() < fDistThresh ) {
				iLoopB -= 2;
				if ( iLoopB < 0 ) 
					iLoopB = (int)nCountB + iLoopB;
				while ( bmap.vTo.size() < nCountA ) {
					bmap.vTo.push_back( vLoopB[iLoopB] );
					iLoopB--;
					if ( iLoopB < 0 ) 
						iLoopB = (int)nCountB + iLoopB;
				}
			} else
				bFailed = true;
		}

		if ( ! bFailed )
			vMaps.push_back( bmap );
		else
			_RMSInfo("Correspondence search failed in MeshUtils::GetBoundaryLoopMaps\n");
	}
	}
}

void MeshUtils::FindBoundaryLoops( const rms::VFTriangleMesh & mesh, std::vector< std::vector< rms::IMesh::TriangleID > > & vLoops )
{
	std::set< IMesh::EdgeID > vBEdges;

	VFTriangleMesh::edge_iterator cure(mesh.BeginEdges()), ende(mesh.EndEdges());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if ( mesh.IsBoundaryEdge(eID) )
			vBEdges.insert(eID); 
	}

	// ok start loop at first edge
	while ( ! vBEdges.empty() ) {
		IMesh::EdgeID eID = *vBEdges.begin();
		vBEdges.erase(eID);

		IMesh::VertexID nVerts[2];   IMesh::TriangleID nTris[2];
		mesh.GetEdge(eID, nVerts, nTris);

		// make sure boundary loops wind in a consistent direction
		int nTri = (nTris[0] != IMesh::InvalidID) ? 0 : 1;
		IMesh::VertexID vTri[3];
		mesh.GetTriangle(nTris[nTri], vTri);
		bool bFlip = true;
		for ( int k = 0; k < 3; ++k ) {
			if ( vTri[k] == nVerts[0] && vTri[(k+1)%3] == nVerts[1] )
				bFlip = false;
		}

		std::vector<IMesh::VertexID> vLoop;
		if ( bFlip ) {
			vLoop.push_back( nVerts[1] );  
			vLoop.push_back( nVerts[0] );
		} else {
			vLoop.push_back( nVerts[0] );  
			vLoop.push_back( nVerts[1] );
		}
		IMesh::VertexID nLastVID = vLoop.back();

		int nIters = 0;
		const int nWhyIsThisFailing = 100000;
		while ( nLastVID != vLoop.front() && nIters++ < nWhyIsThisFailing ) {

			VFTriangleMesh::VtxNbrItr itr(nLastVID);
			mesh.BeginVtxEdges(itr);
			eID = mesh.GetNextVtxEdges(itr);
			while (eID != IMesh::InvalidID) {
				if ( vBEdges.find(eID) != vBEdges.end() ) {
					mesh.GetEdge(eID, nVerts, nTris);
					nLastVID = (nVerts[0] == nLastVID) ? nVerts[1] : nVerts[0];
					if ( nLastVID != vLoop.front() )
						vLoop.push_back( nLastVID );
					vBEdges.erase(eID);
					eID = IMesh::InvalidID;		
				} else
					eID = mesh.GetNextVtxEdges(itr);
			}
		}

		if ( nIters != nWhyIsThisFailing ) 
			vLoops.push_back(vLoop);
	}
}


void MeshUtils::FindBoundaryEdges( VFTriangleMesh & mesh, std::vector< std::pair<IMesh::VertexID,IMesh::VertexID> > & vEdges)
{
	VFTriangleMesh::edge_iterator cure(mesh.BeginEdges()), ende(mesh.EndEdges());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if ( mesh.IsBoundaryEdge(eID) ) {
			IMesh::VertexID edgeV[2];  IMesh::TriangleID edgeT[2];
			mesh.GetEdge(eID, edgeV, edgeT);
			vEdges.push_back( std::pair<IMesh::VertexID,IMesh::VertexID>( edgeV[0], edgeV[1] ) );
		}
	}
}





bool MeshUtils::VertexOneRing( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vOneRing, bool bOrdered, bool * bClosed )
{
	return mesh.VertexOneRing(vID, vOneRing, bOrdered, bClosed);
}



//! bClosed flag is only set if bOrdered is true
bool MeshUtils::TriangleOneRing( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::TriangleID> & vOneRing, bool bOrdered , bool * bClosed  )
{
	return mesh.TriangleOneRing(vID, vOneRing, bOrdered, bClosed);
}


bool MeshUtils::GetTriVerts( IMesh::VertexID nTri[3], IMesh::VertexID vID, IMesh::VertexID & other1, IMesh::VertexID & other2 )
{
	if ( nTri[0] == vID ) {
		other1 = nTri[1];
		other2 = nTri[2];
	} else if ( nTri[1] == vID ) {
		other1 = nTri[0];
		other2 = nTri[2];
	} else if ( nTri[2] == vID ) {
		other1 = nTri[0];
		other2 = nTri[1];
	} else
		return false;
	return true;
}







void MeshUtils::CotangentWeights( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vOneRing, std::vector<float> & vWeights, bool bNormalize  )
{
	Wml::Vector3f vi,vj,vo;
	mesh.GetVertex(vID, vi);

	size_t nNbrs = vOneRing.size();
	vWeights.resize(nNbrs);

	float fWeightSum = 0;
	for ( unsigned int j = 0; j < nNbrs; ++j ) {
		mesh.GetVertex(vOneRing[j], vj);
		IMesh::EdgeID eID = mesh.FindEdge(vID, vOneRing[j]);
		IMesh::VertexID vEdgeV[2];
		mesh.FindNeighboursEV(eID, vEdgeV);

		float dCotSum = 0;
		for ( int k = 0; k < 2; ++k ) {
			if ( vEdgeV[k] == IMesh::InvalidID )
				continue;
			mesh.GetVertex(vEdgeV[k], vo);
			dCotSum += rms::VectorCot(vi-vo, vj-vo);
		}
		vWeights[j] = dCotSum / 2;
		fWeightSum += dCotSum / 2;
	}

	if ( bNormalize ) {
		for ( unsigned int k = 0; k < nNbrs; ++k )
			vWeights[k] /= fWeightSum;
	}
}


void MeshUtils::UniformWeights( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vOneRing, std::vector<float> & vWeights, bool bNormalize  )
{
	Wml::Vector3f vi,vj,vo;
	mesh.GetVertex(vID, vi);

	size_t nNbrs = vOneRing.size();
	vWeights.resize(nNbrs);

	float fWeightSum = 0;
	for ( unsigned int j = 0; j < nNbrs; ++j ) {
		vWeights[j] = 1;
		fWeightSum += vWeights[j];
	}

	if ( bNormalize ) {
		for ( unsigned int k = 0; k < nNbrs; ++k )
			vWeights[k] /= fWeightSum;
	}
}


float MeshUtils::VertexArea_Mixed( VFTriangleMesh & mesh, IMesh::VertexID vID )
{
	std::vector<IMesh::TriangleID> vTriRing;
	mesh.TriangleOneRing(vID, vTriRing);

	float A = 0.0f;
	Wml::Vector3f P( mesh.GetVertex(vID) );

	size_t nTris = vTriRing.size();
	for ( unsigned int ti = 0; ti < nTris; ++ti ) {
		IMesh::VertexID nTri[3];
		mesh.GetTriangle( vTriRing[ti], nTri );
		IMesh::VertexID nOther1, nOther2;
		if ( ! PickTriVerts(nTri, vID, nOther1, nOther2) )
			lgBreakToDebugger();
		Wml::Vector3f Q( mesh.GetVertex(nOther1) );
		Wml::Vector3f R( mesh.GetVertex(nOther2) );

		if ( IsObtuse(P,Q,R) ) {
			float fArea = (float)fabs( rms::Area( P, Q, R ) );
			float fAngle = VectorAngle( Normalize(Q-P), Normalize(R-P) );
			if ( fAngle > Wml::Mathf::HALF_PI )
				A += fArea / 2;
			else
				A += fArea / 4;
		} else {
			float fCotAngleQ = VectorCot( Normalize(P-Q), Normalize(R-Q) );
			float fCotAngleR = VectorCot( Normalize(P-R), Normalize(Q-R) );
			A += (1.0f/8.0f) * ( (P-R).SquaredLength() * fCotAngleQ  +  (P-Q).SquaredLength() * fCotAngleR );
		}
	}
	return A;
}





Wml::Vector3f MeshUtils::MeshLaplacian( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vOneRing, std::vector<float> & vWeights )
{
	Wml::Vector3f vCenter, vNbr;
	mesh.GetVertex(vID, vCenter);
	size_t nNbrs = vOneRing.size();
	Wml::Vector3f vLaplacian(Wml::Vector3f::ZERO);
	for ( unsigned int i = 0; i < nNbrs; ++i ) {
		mesh.GetVertex(vOneRing[i], vNbr);
		vLaplacian += vWeights[i] * ( vNbr - vCenter );
	}
	return vLaplacian;
}


void MeshUtils::GetEdgeLengthStats( IMesh * pMesh, float & fMin, float & fMax, float & fAvg )
{
	fMax = std::numeric_limits<float>::min();
	fMin = std::numeric_limits<float>::max();
	fAvg = 0;
	
	int nCount = 0;
	IMesh::ITriIterator curt( pMesh->BeginITriangles()), endt( pMesh->EndITriangles());
	while ( curt != endt ) {
		IMesh::TriangleID nID = *curt;
		curt++;
		++nCount;

		Wml::Vector3f vVertices[3];
		pMesh->GetTriangle(nID, vVertices);

		for ( int i = 0; i < 3; ++i ) {
			float fLen = (vVertices[i] - vVertices[(i+1)%3]).Length();
			if ( fLen > fMax )
				fMax = fLen;
			if ( fLen < fMin )
				fMin = fLen;
			fAvg += fLen / 3.0f;
		}
	}
	fAvg /= (float)nCount;
}


void MeshUtils::GetEdgeLengthStats( VFTriangleMesh & mesh, MeshSelection & selection, float & fMin, float & fMax, float & fAverage )
{	
	fMax = 0.0f;
	fMin = std::numeric_limits<float>::max();
	double fAverageEdge = 0.0f;
	Wml::Vector3f vVertices[2];

	int nCount = 0;
	VFTriangleMesh::edge_iterator cure( mesh.BeginEdges() ), ende( mesh.EndEdges() );
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;

		IMesh::VertexID nEdge[2], nTri[3];
		mesh.GetEdge(eID, nEdge, nTri);
		if ( selection.Vertices().find(nEdge[0]) == selection.Vertices().end() ||
			 selection.Vertices().find(nEdge[1]) == selection.Vertices().end() )
			   continue;

		mesh.GetEdge(eID, vVertices);
		float fLen = (vVertices[0] - vVertices[1]).Length();
		if ( fLen < fMin )
			fMin = fLen;
		if ( fLen > fMax )
			fMax = fLen;
		fAverageEdge += fLen;
		++nCount;
	}

	fAverage = (float)(fAverageEdge / (double)nCount);		
}



void MeshUtils::GetBoundaryEdgeStats( VFTriangleMesh & mesh, float & fMin, float & fMax, float & fAverage )
{
	fMax = 0.0f;
	fMin = std::numeric_limits<float>::max();
	double fAverageEdge = 0.0f;
	Wml::Vector3f vVertices[2];

	int nCount = 0;
	VFTriangleMesh::edge_iterator cure( mesh.BeginEdges() ), ende( mesh.EndEdges() );
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if ( ! mesh.IsBoundaryEdge(eID) )
			continue;
		
		mesh.GetEdge(eID, vVertices);
		float fLen = (vVertices[0] - vVertices[1]).Length();
		if ( fLen < fMin )
			fMin = fLen;
		if ( fLen > fMax )
			fMax = fLen;
		fAverageEdge += fLen;
		++nCount;
	}

	fAverage = (float)(fAverageEdge / (double)nCount);	
}


void MeshUtils::GetBoundaryBoundingBox( VFTriangleMesh & mesh, Wml::AxisAlignedBox3f & bounds )
{
	bool bInitialized = false;
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		if ( ! mesh.IsBoundaryVertex(vID) )
			continue;
		Wml::Vector3f v;
		mesh.GetVertex(vID, v);
		if ( ! bInitialized ) {
			bounds = Wml::AxisAlignedBox3f(v.X(),v.X(), v.Y(),v.Y(), v.Z(),v.Z());
			bInitialized = true;
		} else {
			rms::Union(bounds, v);
		}
	}
}

void MeshUtils::GetBoundaryUVBoundingBox( VFTriangleMesh & mesh, Wml::AxisAlignedBox2f & bounds, IMesh::UVSetID nSetID )
{
	bool bInitialized = false;
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		if ( ! mesh.IsBoundaryVertex(vID) )
			continue;
		Wml::Vector2f v;
		if ( ! mesh.GetUV(vID, nSetID, v) )
			continue;
		if ( ! bInitialized ) {
			bounds = Wml::AxisAlignedBox2f(v.X(),v.X(), v.Y(),v.Y());
			bInitialized = true;
		} else {
			rms::Union(bounds, v);
		}
	}
}




void MeshUtils::DrawBoundaryEdges( rms::VFTriangleMesh & mesh, const Wml::Vector3f & vColor )
{
	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glColor3f(vColor.X(), vColor.Y(), vColor.Z());
	glLineWidth(5.0f);

	glBegin(GL_LINES);

	VFTriangleMesh::edge_iterator cure(mesh.BeginEdges()), ende(mesh.EndEdges());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		if ( ! mesh.IsBoundaryEdge(eID) )
			continue;

		Wml::Vector3f vVerts[2];
		mesh.GetEdge(eID, vVerts);
		glVertex3fv(vVerts[0]);
		glVertex3fv(vVerts[1]);
	}

	glEnd();
	glPopAttrib();
}




void MeshUtils::DrawFrontEdges( rms::VFTriangleMesh & mesh )
{
	if ( ! mesh.HasScalarSet(0))
		return;

	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(4.0f);
	glDepthFunc(GL_LEQUAL);

	glColor4f(0.0f,0.0f,1.0f,1.0f);

	glBegin(GL_LINES);

	VFTriangleMesh::edge_iterator cure(mesh.BeginEdges()), ende(mesh.EndEdges());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		IMesh::VertexID nVerts[2], nTris[2];
		mesh.GetEdge(eID, nVerts, nTris);
		IMesh::VertexID nTriVerts[3];
		mesh.GetTriangle(nTris[0],nTriVerts);
		bool bFront1 = true;    float fScalar = 0;
		for ( int k = 0; k < 3; ++k )
			bFront1 = bFront1 && mesh.GetScalar(nTriVerts[k], 0, fScalar) && fScalar < 1.0f;
		mesh.GetTriangle(nTris[1],nTriVerts);
		bool bFront2 = true;
		for ( int k = 0; k < 3; ++k )
			bFront2 = bFront2 && mesh.GetScalar(nTriVerts[k], 0, fScalar) && fScalar < 1.0f;
		bool bFrontEdge = bFront1 ^ bFront2;

		if ( bFrontEdge ) {
			Wml::Vector3f vEdge[2];
			mesh.GetEdge(eID, vEdge);
			glVertex3fv(vEdge[0]);
			glVertex3fv(vEdge[1]);
		}
	}

	glEnd();

	glPopAttrib();
}



bool MeshUtils::PickTriVerts( IMesh::VertexID nTri[3], IMesh::VertexID vOpp, IMesh::VertexID & vOther1, IMesh::VertexID & vOther2 )
{
	if ( nTri[0] == vOpp ) {
		vOther1 = nTri[1];
		vOther2 = nTri[2];
		return true;
	} else if ( nTri[1] == vOpp ) {
		vOther1 = nTri[2];
		vOther2 = nTri[0];
		return true;
	} else if ( nTri[2] == vOpp ) {
		vOther1 = nTri[0]; 
		vOther2 = nTri[1];
		return true;
	} else
		return false;
}





// recursive K-ring iteration - repeats itself lots of times, need to be smarter...
//static void InsertKRing( VFTriangleMesh * pMesh, IMesh::VertexID vID, std::set<IMesh::VertexID> & vDone, int K )
//{
//	if ( K == 0 )
//		return;
//
//	VFTriangleMesh::VtxNbrItr itr(vID);
//	pMesh->BeginVtxEdges(itr);
//	IMesh::EdgeID eID = pMesh->GetNextVtxEdges(itr);
//	while ( eID != IMesh::InvalidID ) {
//		IMesh::VertexID nVtx[2], nTri[2];
//		pMesh->GetEdge( eID, nVtx, nTri );
//		IMesh::VertexID vOtherID = (nVtx[0] == vID) ? nVtx[1] : nVtx[0];
//		vDone.insert(vOtherID);
//
//		// not efficient! 
//		InsertKRing(pMesh, vOtherID, vDone, K-1);
//
//		eID = pMesh->GetNextVtxEdges(itr);
//	}
//}


Wml::Vector3f MeshUtils::GetAverageNormal( VFTriangleMesh & mesh, IMesh::VertexID vID, unsigned int nKRings )
{
	Wml::Vector3f vVertex, vNormal, vAvgNormal(Wml::Vector3f::ZERO);
	float fWeightSum = 0.0f;

	VFTriangleMesh::VtxNbrItr itr(vID);
	mesh.BeginVtxEdges(itr);
	IMesh::EdgeID eID = mesh.GetNextVtxEdges(itr);
	while ( eID != IMesh::InvalidID ) {
		IMesh::VertexID nVtx[2], nTri[2];
		mesh.GetEdge( eID, nVtx, nTri );
		IMesh::VertexID vOtherID = (nVtx[0] == vID) ? nVtx[1] : nVtx[0];

		mesh.GetVertex(vOtherID, vVertex, &vNormal);
		vAvgNormal += vNormal;
		fWeightSum += 1.0f;

		eID = mesh.GetNextVtxEdges(itr);
	}	

	vAvgNormal *= 1.0f / fWeightSum;
	vAvgNormal.Normalize();

	return vAvgNormal;
}


void MeshUtils::SmoothNormals( VFTriangleMesh & mesh )
{
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		Wml::Vector3f vSmoothNorm = GetAverageNormal(mesh, vID);
		mesh.SetNormal(vID, vSmoothNorm);
	}
}



Wml::Vector3f MeshUtils::EstimateNormal( VFTriangleMesh & mesh, IMesh::VertexID vID, NormalEstMode eMode )
{
	Wml::Vector3f vEstimate(Wml::Vector3f::ZERO), vTri[3], vNormal;

	IMesh::VtxNbrItr itr(vID);
	mesh.BeginVtxTriangles(itr);
	IMesh::TriangleID tID = mesh.GetNextVtxTriangle(itr);
	while ( tID != IMesh::InvalidID ) {
		mesh.GetTriangle(tID, vTri);

		if ( eMode == AreaWeightedFaceAvg ) {
			float fWeight;
			vNormal = Normal(vTri[0], vTri[1], vTri[2], &fWeight );
			vEstimate += fWeight * vNormal;
		} else if ( eMode == UniformFaceAvg ) {
			vNormal = Normal(vTri[0], vTri[1], vTri[2]);
			vEstimate += vNormal;
		}

		tID = mesh.GetNextVtxTriangle(itr);
	}

	vEstimate.Normalize();
	return vEstimate;
}


void MeshUtils::EstimateNormals( VFTriangleMesh & mesh, NormalEstMode eMode, bool bSkipBoundary, Wml::Vector3f * pBuffer )
{
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		if ( bSkipBoundary && mesh.IsBoundaryVertex(vID) )
			continue;
		Wml::Vector3f vEstimate = EstimateNormal(mesh, vID, eMode);
		if ( pBuffer )
			pBuffer[vID] = vEstimate;
		else
			mesh.SetNormal(vID, vEstimate);
	}	
}



Wml::Vector3f MeshUtils::FaceNormal( VFTriangleMesh & mesh, IMesh::TriangleID tID )
{
	// [TODO] should pick most robust set of edges here...

	Wml::Vector3f vTri[3];
	mesh.GetTriangle(tID, vTri);
	Wml::Vector3f vEdge1(vTri[1] - vTri[0]);					vEdge1.Normalize();
	Wml::Vector3f vEdge2(vTri[2] - vTri[0]);					vEdge2.Normalize();
	Wml::Vector3f vFaceNormal( vEdge1.Cross(vEdge2) );
	vFaceNormal.Normalize();
	return vFaceNormal;
}


Wml::Vector3f MeshUtils::FaceInterpNormal( IMesh * pMesh, IMesh::TriangleID tID, const Wml::Vector3f & vTriPoint )
{
	Wml::Vector3f vTri[3], vNorm[3];
	pMesh->GetTriangle( tID, vTri, vNorm );
	return rms::InterpNormal(vTri[0],vTri[1],vTri[2], vNorm[0], vNorm[1], vNorm[2], vTriPoint);
}



Wml::Vector3f MeshUtils::Centroid( const VFTriangleMesh & mesh )
{
	Wml::Vector3f vVertex, vCentroid(Wml::Vector3f::ZERO);
	unsigned int nCount = 0;
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		mesh.GetVertex( vID, vVertex );
		vCentroid += vVertex;
		++nCount;
	}
	return vCentroid / (float)nCount;
}



void MeshUtils::ScaleMesh( VFTriangleMesh & mesh, const Wml::Vector3f & vScale, const Wml::Vector3f & vCenter )
{
	// TODO: handle normals properly...

	Wml::Vector3f vVertex, vNormal;
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;

		mesh.GetVertex( vID, vVertex, &vNormal );
		vVertex -= vCenter;
		vVertex.X() *= vScale.X();  vVertex.Y() *= vScale.Y();   vVertex.Z() *= vScale.Z();
		vVertex += vCenter;

		mesh.SetVertex( vID, vVertex, &vNormal );
	}
}


void MeshUtils::TranslateMesh( VFTriangleMesh & mesh, const Wml::Vector3f & vTranslate )
{
	Wml::Vector3f vVertex;
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		mesh.GetVertex( vID, vVertex );
		vVertex += vTranslate;
		mesh.SetVertex( vID, vVertex );
	}
}

void MeshUtils::RotateMesh( VFTriangleMesh & mesh, const Wml::Matrix3f & mRotate, const Wml::Vector3f & vCenter )
{
	Wml::Vector3f vVertex, vNormal;
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;

		mesh.GetVertex( vID, vVertex, &vNormal );
		vVertex -= vCenter;
		vVertex = mRotate * vVertex;
		vVertex += vCenter;
		vNormal = mRotate * vNormal;

		mesh.SetVertex( vID, vVertex, &vNormal );
	}
}


void MeshUtils::InvertMesh( VFTriangleMesh & mesh )
{
	VFTriangleMesh::triangle_iterator curt(mesh.BeginTriangles()), endt(mesh.EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		IMesh::VertexID nTri[3];
		mesh.GetTriangle(tID, nTri);
		for ( int j = 0; j < 3; ++j )
			mesh.HACK_ManuallyIncrementReferenceCount(nTri[j]);
		mesh.SetTriangle(tID, nTri[1], nTri[0], nTri[2]);
		for ( int j = 0; j < 3; ++j )
			mesh.HACK_ManuallyDecrementReferenceCount(nTri[j]);
	}
}



float MeshUtils::LoopLength( const VFTriangleMesh & mesh, const std::vector<IMesh::VertexID> & vLoop )
{
	float fLength = 0.0f;
	size_t nCount = vLoop.size();
	Wml::Vector3f v1,v2;
	mesh.GetVertex(vLoop[0], v1);
	for ( unsigned int i = 1; i < nCount; ++i ) {
		mesh.GetVertex(vLoop[(i+1)%nCount], v2);
		fLength += (v1-v2).Length();
		v1 = v2;
	}
	return fLength;
}
float MeshUtils::LoopLengthUV( const VFTriangleMesh & mesh, const std::vector<IMesh::VertexID> & vLoop, IMesh::UVSetID nSetID )
{
	float fLength = 0.0f;
	size_t nCount = vLoop.size();
	Wml::Vector2f v1,v2;
	if (! mesh.GetUV(vLoop[0], nSetID, v1) )
		return -1.0f;
	for ( unsigned int i = 1; i < nCount; ++i ) {
		if ( ! mesh.GetUV(vLoop[(i+1)%nCount], nSetID, v2) )
			return -1.0f;
		fLength += (v1-v2).Length();
		v1 = v2;
	}
	return fLength;
}



void MeshUtils::ScaleMeshByLoop( VFTriangleMesh & mesh, const std::vector<IMesh::VertexID> & vLoop, float fTargetPerimeter )
{
	float fLength = LoopLength(mesh, vLoop);
	float fScale = fTargetPerimeter / fLength;
	ScaleMesh(mesh, Wml::Vector3f(fScale,fScale,fScale));
}



float MeshUtils::MeshArea( VFTriangleMesh & mesh )
{
	float fSum = 0.0f;
	VFTriangleMesh::triangle_iterator curt(mesh.BeginTriangles()), endt(mesh.EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		Wml::Vector3f vTri[3];
		mesh.GetTriangle(tID, vTri);
		fSum += Area( vTri[0], vTri[1], vTri[2] );
	}
	return fSum;
}

float MeshUtils::MeshUVArea( VFTriangleMesh & mesh, IMesh::UVSetID nUVSet )
{
	float fSum = 0.0f;
	VFTriangleMesh::triangle_iterator curt(mesh.BeginTriangles()), endt(mesh.EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		Wml::Vector2f vTri[3];
		if ( mesh.GetTriangleUV(tID, nUVSet, vTri) )
			fSum += Area( vTri[0], vTri[1], vTri[2] );
	}
	return fSum;
}



void MeshUtils::ScaleMeshUV( VFTriangleMesh & mesh, IMesh::UVSetID nSetID, const Wml::Vector2f & vScale, const Wml::Vector2f & vCenter )
{
	if ( ! mesh.HasUVSet(nSetID) )
		return;

	IMesh::UVSet & uvset = mesh.GetUVSet(nSetID);

	Wml::Vector2f vUV;
	VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;

		if ( mesh.GetUV(vID, nSetID, vUV) ) {
			vUV -= vCenter;
			vUV.X() *= vScale.X();  vUV.Y() *= vScale.Y();
			vUV += vCenter;
			mesh.SetUV(vID, nSetID, vUV);
		}
	}
}




IMesh::VertexID MeshUtils::FindGeodesicCenter( VFTriangleMesh & mesh, float * pMaxDistance )
{
	std::vector<std::vector<IMesh::VertexID> > vBoundaryLoops;
	FindBoundaryLoops(mesh, vBoundaryLoops);
	if ( vBoundaryLoops.size() != 1 )
		return false;
	std::vector<IMesh::VertexID> & vLoop = vBoundaryLoops[0];
	

	rms::MeshOneRingSource oneringSource(&mesh, false);
	rms::MeshPositionSource pointSource(&mesh);
	DijkstraFrontProp<IMesh::VertexID> prop;
	prop.SetSource(&pointSource, &oneringSource);
	prop.AppendStartValues(vLoop, 0);
	prop.FindAllDistances();

	if ( pMaxDistance )
		*pMaxDistance = prop.GetMaxDistance();


	return prop.GetOrder().back();
}




// vComponents is vector of size mesh->GetMaxTriangleID() where value is -1 if tID is not in vTris, otherwise component number < nComponents
void MeshUtils::FindConnectedComponents(IMesh * pMesh, const std::set<IMesh::TriangleID> & vTris, std::vector<int> & vComponents, int & nComponents, int & nLargest)
{
	// [TODO optimization]
	//  - don't use set for internal stack (actually it is a queue...)
	//      - could do recursion, I suppose, but it might get very deep...
	//  - can encode vAvailable inside vComponents (set to -2 or something)

	size_t nCount = pMesh->GetMaxTriangleID();
	vComponents.resize(nCount);
	vComponents.resize(0);  vComponents.resize(nCount, -1);

	std::vector<bool> vAvailable(nCount,false);
	std::set<IMesh::TriangleID>::const_iterator curt(vTris.begin()), endt(vTris.end());
	int nTriCount = 0;
	while ( curt != endt ) {
		vAvailable[ (*curt++) ] = true;
		++nTriCount;
	}

	int nCurComponent = 0;
	nLargest = 0;  int nLargestSize = 0;
	while ( nTriCount > 0 ) {

		std::set<IMesh::TriangleID> vStack;
		IMesh::TriangleID tStart = IMesh::InvalidID;
		for ( unsigned int k = 0; k < nCount && tStart == IMesh::InvalidID; ++k ) {
			if ( vAvailable[k] )
				tStart = k;
		}
		if ( tStart == IMesh::InvalidID )
			lgBreakToDebugger();

		vStack.insert(tStart);
		vAvailable[tStart] = false;
		int nComponentSize = 0;

		do {
			IMesh::TriangleID tCur = *vStack.begin();
			vComponents[tCur] = nCurComponent;
			--nTriCount;
			++nComponentSize;
			vStack.erase(tCur);
			IMesh::TriangleID tNbrs[3];
			pMesh->FindNeighbours(tCur,tNbrs);
			for ( int k = 0; k < 3; ++k ) {
				if ( tNbrs[k] != IMesh::InvalidID && vAvailable[tNbrs[k]] ) {
					vStack.insert( tNbrs[k] );
					vAvailable[tNbrs[k]] = false;
				}
			}
		} while (! vStack.empty() );
	
		if ( nComponentSize > nLargestSize ) {
			nLargestSize = nComponentSize;
			nLargest = nCurComponent;
		}
		++nCurComponent;
	}
	nComponents = nCurComponent;
}





