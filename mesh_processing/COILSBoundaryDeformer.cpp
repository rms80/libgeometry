// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "COILSBoundaryDeformer.h"

#include <opengl.h>

#include <set>
#include <vector>
#include <limits>
#include <algorithm>

#include <DijkstraFrontProp.h>
#include <MeshUtils.h>
#include <MeshSmoother.h>
#include <MatrixBlender.h>
#include <MeshFunction.h>
#include <VFMeshRenderer.h>
#include <VectorUtil.h>
#include <Frame.h>

#include "rmsdebug.h"
#include "rmsprofile.h"

#include <Wm4Segment3.h>
#include <Wm4DistVector3Segment3.h>


using namespace rms;

COILSBoundaryDeformer::COILSBoundaryDeformer()
	: m_meshDistCache(false), m_combiner(rms::MatrixBlender::AverageMatrix)
{
	m_eEncodeMode = UpwindThreshold;
	m_bUseUpwindGeodesicDistances = false;

	m_fDistanceWeightPower = 2.0f;

	m_fOptimizedBoundaryAngle = 0.0f;
	m_fBoundaryRotateAngle = 0.0f;
	m_bOptimizeBoundaryFrames = false;
	m_bUseOptimalBoundaryFrames = false;
	m_bDoPerVertexOptimization = true;

	m_fGlobalScale = 1.0f;

	m_fUpwindThreshold = 2.1f;

	m_fUpwindThresholdScale = 1.0f;

	m_bEnableUpwindCorrection = true;
	m_bEnableDensityCorrection = true;
	m_bDeferredWeightInvalidationPending =  false;

	m_pMesh = NULL;
	m_meshDistCache.SetCalculator(this);

	m_nUseBoundaryLoop = -1;

	m_pOriginalMesh = NULL;
}

COILSBoundaryDeformer::~COILSBoundaryDeformer(void)
{
}


void COILSBoundaryDeformer::Reset()
{
	m_vOrigMesh.resize(0);
	m_vLayers.resize(0);

	m_vGlobalOrdering.resize(0);
	m_vLayerMap.clear();
	m_vGlobalUpwindSet.clear();

	m_vOneRingAreas.resize(0);

	m_fDistanceWeightPower = 2.0f;

	m_fGlobalScale = 1.0f;

	m_nUseBoundaryLoop = -1;
	m_fBoundaryRotateAngle = 0.0f;
	m_fOptimizedBoundaryAngle = 0.0f;
	m_vOptimizedBoundaryAngles.resize(0);

	if ( m_bEnableDensityCorrection ) {
		//if ( m_pMesh->GetVertexCount() > 3000 )
		//	m_meshDistCache.SetUseSparse(true);
		//else
		//	m_meshDistCache.SetUseSparse(false);
		m_meshDistCache.SetUseSparse(true);
		m_meshDistCache.Resize( m_pMesh->GetVertexCount() );
	}

	// re-seed RNG for randomized mode (w/ consistent seed...)
	srand(3167731317);
}


/** Boundary normals of pMesh must be set to direction of part base surface. 
    If mesh has more than one boundary loop, use SetUseBoundaryLoop() to indicate which
 **/ 
void COILSBoundaryDeformer::SetMesh( VFTriangleMesh * pMesh )
{
	m_pMesh = pMesh;

	// cannot do this because it ovewrites normals of boundary loop we are going to encode relative to
	// (which are important for some applications)
	//MeshUtils::EstimateNormals(*m_pMesh, rms::MeshUtils::AreaWeightedFaceAvg, true);

//	if ( ! m_vOrigMesh.empty() )
		Reset();

	m_pMesh->GetBoundingBox(m_bounds);

	float fAverage;
	m_pMesh->GetEdgeLengthStats(m_fMinEdgeLength, m_fMaxEdgeLength, fAverage);
	m_fMaxEdgeLength = fAverage;

	// initialize scalar sets
	if ( m_pMesh->HasScalarSet(0) == false )
		IMesh::ScalarSetID scalarID = m_pMesh->AppendScalarSet();
	m_pMesh->InitializeScalarSet(0);

	m_meshPointSource = MeshPositionSource(m_pMesh);

	m_oneringFinder = MeshOneRingSource(m_pMesh, false);
	m_oneringCache.SetSource( &m_oneringFinder );

	m_geonbrFinder = MeshGeoNbrSource(m_pMesh, &m_oneringCache, &m_meshPointSource);
	m_geonbrCache.SetSource( &m_geonbrFinder );

	m_pNeighbourCache = &m_geonbrCache;
	m_dijkstra.SetSource(&m_meshPointSource, m_pNeighbourCache);
}

void COILSBoundaryDeformer::RestoreMesh()
{
	if ( m_vLayers.empty() )
		return;

	// restore verts
	VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		m_pMesh->SetVertex( vID, m_vOrigMesh[vID].vPosition, &m_vOrigMesh[vID].vNormal );
	}
}


void COILSBoundaryDeformer::SetEncodeMode( COILSBoundaryDeformer::EncodeMode eMode )
{ 
	if ( m_eEncodeMode == TwoPass && m_pOriginalMesh) {
		RestoreMesh();
		SetMesh(m_pOriginalMesh);
		m_pOriginalMesh = NULL;
	} 
	m_eEncodeMode = eMode; 
}


void COILSBoundaryDeformer::Reconstruct( )
{
	if ( m_vLayers.size() == 0 )
		return;

	_RMSInfo("[Reconstruct]\n");

	_RMSTUNE_start(1);

	RestoreMesh();

	if ( m_bDeferredWeightInvalidationPending )
		InvalidateAllWeights(false);

	if ( m_bOptimizeBoundaryFrames ) {

		_RMSTUNE_start(5);

		float fSavedAngle = m_fBoundaryRotateAngle;
		m_fBoundaryRotateAngle = 0.0f;

		InitializeBoundaryOptimization(m_vLayers[0]);
		OptimizeBoundaryFrames_Gradient(m_vLayers[0], m_bDoPerVertexOptimization);

		m_fBoundaryRotateAngle = fSavedAngle;

		_RMSTUNE_end(5);
		_RMSInfo("  FrameOpt time: %f\n", _RMSTUNE_time(5));
	}

	Decode();

	_RMSTUNE_end(1);
	_RMSInfo("  Total Reconstruct() time: %f\n", _RMSTUNE_time(1));
}



void COILSBoundaryDeformer::OptimizeBoundaryFrames(bool bPerVertexOptimization, float fTimeout)
{
	m_bOptimizeBoundaryFrames = true;
	m_bDoPerVertexOptimization = bPerVertexOptimization;
	bool bSave = m_bUseOptimalBoundaryFrames;
	m_bUseOptimalBoundaryFrames = true;
	m_fOptimizationTimeout = fTimeout;
	Reconstruct();
	m_bOptimizeBoundaryFrames = false;
	m_bUseOptimalBoundaryFrames = bSave;
}



void COILSBoundaryDeformer::Decode(bool bBoundaryNbrsOnly)
{
	bool bTwoPass = ( m_eEncodeMode == TwoPass );
	if ( bTwoPass )
		m_eEncodeMode = UpwindThreshold;

	bool bVerbose = ! bBoundaryNbrsOnly;

	_RMSTUNE_start(2);
	InitializeBoundary(m_vLayers[0]);
	_RMSTUNE_end(2);
	if ( bVerbose ) {
		_RMSInfo("  Decoding...\n");
		_RMSInfo("      InitializeBoundary() time: %f\n", _RMSTUNE_time(2));
	}

	_RMSTUNE_accum_init(2); _RMSTUNE_accum_init(3);
	_RMSTUNE_start(4);
	for ( unsigned int li = 0; li < m_vLayers.size(); ++li ) {
		EncodingLayer & layer = m_vLayers[li];

		std::vector<bool> vFixed( layer.vEncoding.size(), false );
		std::vector<bool> vFixedNormal( layer.vEncoding.size(), false );

		// need to copy boundary frames from some previous layer
		if ( li > 0 ) {
			VertSet::iterator curbv(layer.vBoundaryVertIDs.begin()), endbv(layer.vBoundaryVertIDs.end());
			while ( curbv != endbv ) {
				IMesh::VertexID vID = *curbv++;
				bool bFound = false;
				for ( int k = li-1; !bFound && k >= 0; --k ) {
					if ( m_vLayers[k].vVertices.find(vID) != m_vLayers[k].vVertices.end() ) {
						layer.vEncoding[vID].vFrame = m_vLayers[k].vEncoding[vID].vFrame;
						bFound = true;
					}
				}
				if (! bFound ) DebugBreak();
			}
		}


		float fMaxDist = layer.fMaxBoundaryDist;

		size_t nCount = layer.vInteriorOrder.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {

			// terminate if we have reconstructed everything we need
			if ( bBoundaryNbrsOnly && i > layer.nLastBoundaryNbrIndex )
				goto finished_decode;

			IMesh::VertexID vID = layer.vInteriorOrder[i];
			BoundaryVert vFind(vID);
			if ( layer.vBoundaryVerts.find(vFind) != layer.vBoundaryVerts.end() ) {
				vFixed[vID] = true;
				vFixedNormal[vID] = true;
				continue;
			}

			_RMSTUNE_start(2);
			ValidateWeights(layer, vID, li == 0);
			_RMSTUNE_end(2);   _RMSTUNE_accum(2);

			_RMSTUNE_start(3);
			rms::Frame3f vFrame;
			ReconstructVertFrame( layer, vID, vFrame );
			_RMSTUNE_end(3);   _RMSTUNE_accum(3);

			layer.vEncoding[vID].vFrame = vFrame;
			m_pMesh->SetVertex( vID, vFrame.Origin(), & vFrame.Z() );
			vFixed[vID] = true;
		}

		// have to finish estimating all nbr frames, in case next segment uses them
		for ( unsigned int i = 0; i < nCount; ++i ) {
			IMesh::VertexID vID = layer.vInteriorOrder[i];
			if (! vFixedNormal[vID] ) {
				Wml::Vector3f vNormal = MeshUtils::EstimateNormal(*m_pMesh, vID );
				rms::Frame3f vFrame = layer.vEncoding[vID].vFrame;
				vFrame.AlignZAxis(vNormal);
				layer.vEncoding[vID].vFrame = vFrame;
				m_pMesh->SetNormal( vID, vFrame.Z() );
				vFixedNormal[vID] = true;
			}
		}

	}

	_RMSTUNE_start(5);
	if ( bTwoPass ) {
		m_eEncodeMode = TwoPass;
		Decode_Detail();
	}
	_RMSTUNE_end(5);

	_RMSTUNE_end(4);


finished_decode:
	if ( bTwoPass )
		m_eEncodeMode = TwoPass;

	if ( bVerbose ) {
		_RMSInfo("      ValidateWeights() accum time: %f\n", _RMSTUNE_accum_time(2));
		_RMSInfo("      ReconstructVertFrame() accum time: %f\n", _RMSTUNE_accum_time(3));
		if (bTwoPass )
			_RMSInfo("      Decode_Detail() time: %f\n", _RMSTUNE_time(5));

		_RMSInfo("    Decode time: %f\n", _RMSTUNE_time(4));
	}
}


void COILSBoundaryDeformer::Decode_Detail()
{
	EncodingLayer & baseLayer = m_vLayers[0];

	float fScale = m_fGlobalScale;

	size_t nCount = m_detailLayer.vInteriorOrder.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {

		IMesh::VertexID vID = m_detailLayer.vInteriorOrder[i];
		VertEncoding & enc = m_detailLayer.vEncoding[vID];

		if ( m_detailLayer.vBoundaryVertIDs.find(vID) != m_detailLayer.vBoundaryVertIDs.end() ) {
			Parent & p = enc.vParents[0];
			Wml::Vector3f vVertex;
			m_pMesh->GetVertex( p.vID, vVertex );
			m_pOriginalMesh->SetVertex(vID, vVertex);
			continue;
		}

		_RMSTUNE_start(2);
		ValidateWeights(m_detailLayer, vID, false);
		_RMSTUNE_end(2);   _RMSTUNE_accum(2);

		_RMSTUNE_start(3);
		Wml::Vector3f vOrigin = Wml::Vector3f::ZERO;
		std::vector<Parent> & vParents = enc.vParents;
		size_t nParents = vParents.size();

		// average parent displacement vectors
		float fWeightSum = 0.0f;
		for ( unsigned int i = 0; i < nParents; ++i ) {
			Parent & p = vParents[i];
			float fWeight = p.fWeight;
		
			const rms::Frame3f & vParentFrame = baseLayer.vEncoding[p.vID].vFrame;
			
			const Wml::Vector3f & vOffsetVec = p.vOffsetVector;
			vOrigin += fWeight * ( vParentFrame.Origin() + (vParentFrame.FromFrameMatrix() * vOffsetVec * fScale) );

		}
		_RMSTUNE_end(3);   _RMSTUNE_accum(3);

		m_pOriginalMesh->SetVertex( vID, vOrigin);
	}

	// estimate normals
	for ( unsigned int i = 0; i < nCount; ++i ) {
		IMesh::VertexID vID = m_detailLayer.vInteriorOrder[i];
		Wml::Vector3f vNormal = MeshUtils::EstimateNormal(*m_pOriginalMesh, vID );
		m_pOriginalMesh->SetNormal( vID, vNormal );
	}

}



void COILSBoundaryDeformer::ReconstructVertFrame(EncodingLayer & layer, IMesh::VertexID vID, rms::Frame3f & vFrame )
{
	std::vector<Parent> & vParents = layer.vEncoding[vID].vParents;

	size_t nParents = vParents.size();

	// figure out how much to scale vectors by
	float fScale = m_fGlobalScale;

	Wml::Vector3f vPosition = Wml::Vector3f::ZERO;
	Wml::Matrix3f vOrientation = Wml::Matrix3f::ZERO;

	// average parent displacement vectors
	for ( unsigned int i = 0; i < nParents; ++i ) {
		Parent & p = vParents[i];
		float fWeight = p.fWeight;
		
		const rms::Frame3f & vParentFrame = layer.vEncoding[p.vID].vFrame;

		Wml::Vector3f vPrediction = vParentFrame.Origin() + fScale * (vParentFrame.FromFrameMatrix() * p.vOffsetVector);
		vPosition += fWeight * vPrediction;

		vOrientation += fWeight * vParentFrame.FromFrameMatrix() * p.vOffsetFrame;
	}

	vFrame = rms::Frame3f( vOrientation, vOrientation.Transpose(), vPosition );
	vFrame.ReNormalize(2);
}















bool COILSBoundaryDeformer::Encode() 
{
	Reset();

	_RMSInfo("[Encode]\n");
	_RMSTUNE_start(1);

	bool bOK = true;
	if ( m_eEncodeMode == TwoPass )
		bOK = Encode_Multipass();
	else
		bOK = Encode_Segments();

	// set up global ordering
	for ( unsigned int i = 0; i < m_vLayers.size(); ++i ) {
		EncodingLayer & layer = m_vLayers[i];
		for ( unsigned int k = 0; k < layer.vInteriorOrder.size(); ++k ) {
			IMesh::VertexID vID = layer.vInteriorOrder[k];
			if ( layer.vBoundaryVertIDs.find(vID) != layer.vBoundaryVertIDs.end() )
				continue;
			m_vGlobalOrdering.push_back(vID);
			m_vLayerMap[vID] = i;
		}
	}
	_RMSInfo("      %d Encoded points\n", m_vGlobalOrdering.size());
	_RMSTUNE_end(1);  _RMSInfo("  Encode time: %6.3f\n", _RMSTUNE_time(1) );

	return bOK;
}



bool COILSBoundaryDeformer::Encode_Segments() 
{
	InitializeLayers_Basic();

	bool bOK = true;

	for ( unsigned int i = 0; i < m_vLayers.size(); ++i ) {
																						_RMSTUNE_start(2);
		EncodingLayer & layer = m_vLayers[i];
																						_RMSTUNE_start(3);
		OrderVertices(layer);
																						_RMSTUNE_end(3);  double fOrderVerticesTime = _RMSTUNE_time(3);
		layer.vEncoding.resize( m_pMesh->GetVertexCount() );

		std::set<BoundaryVert>::iterator curv(layer.vBoundaryVerts.begin()), endv(layer.vBoundaryVerts.end());
		while ( curv != endv )

		layer.vBoundaryVertsVec.push_back( const_cast<BoundaryVert*>(&(*curv++))); // :)

		_RMSInfo("    Encoding layer %d with mode %d\n", i, layer.eEncodeMode);
																						_RMSTUNE_start(3);
		bOK = bOK && Encode_Layer(layer);
																						_RMSTUNE_end(3);  double fEncodeLayerTime = _RMSTUNE_time(3);
		// save initial boundary frames for outermost boundary (layer 0)
		if ( i == 0 ) {
			curv = layer.vBoundaryVerts.begin();
			while ( curv != endv ) {
				// BoundaryVert & bv = *curv++;
				UpdateBoundary( curv->vID, curv->vInitial.Origin(), curv->vInitial.Z() );
				curv++;
			}
		}

																						_RMSTUNE_end(2);
		_RMSInfo("      Total time: %6.3f   Order: %6.3f   Encode: %6.3f\n", _RMSTUNE_time(2), fOrderVerticesTime, fEncodeLayerTime);
	}


	return bOK;
}


bool COILSBoundaryDeformer::Encode_Multipass() 
{
	m_pOriginalMesh = m_pMesh;

	_RMSTUNE_start(2);
	GenerateBaseMesh();
	_RMSTUNE_end(2);
	_RMSInfo("    Base mesh generation took %6.3f - reduced from %d to %d\n", _RMSTUNE_time(2), m_pOriginalMesh->GetMaxVertexID(), m_baseMesh.GetMaxVertexID());

	RestoreMesh();
	SetMesh(&m_baseMesh);
	m_baseMeshBVTree.SetMesh(&m_baseMesh);

	// encode base mesh
	m_eEncodeMode = UpwindThreshold;
	bool bOK = Encode_Segments();
	m_eEncodeMode = TwoPass;

	// encode detail mesh
	_RMSInfo("    Encoding detail layer...\n", _RMSTUNE_time(2));
	_RMSTUNE_start(2);
	InitializeLayers_DetailMesh();
	_RMSTUNE_end(2);
	_RMSInfo("    Encoding detail layer took %6.3f\n", _RMSTUNE_time(2));

	
	return bOK;
}





/*
 * This one-ring neighbour iteration constructs a list [parents] of all
 * the neighbouring vertices of [vertid] which are in [knownverts]
 */
class MakeParentListCallback : public IMesh::NeighborTriCallback
{
public:
	MakeParentListCallback( IMesh::VertexID vertid, COILSBoundaryDeformer::VertSet * parents, COILSBoundaryDeformer::VertSet * knownverts, VFTriangleMesh * mesh) {
		vID = vertid;
		pParents = parents;
		pKnownVerts = knownverts;
		pMesh = mesh;
	}
	IMesh::VertexID vID;
	COILSBoundaryDeformer::VertSet * pParents;
	COILSBoundaryDeformer::VertSet * pKnownVerts;
	VFTriangleMesh * pMesh;

	virtual void NextTriangle( IMesh::TriangleID tID ) {
		IMesh::VertexID nTri[3];
		pMesh->GetTriangle(tID, nTri);
		for ( int k = 0; k < 3; ++k ) {
			if ( nTri[k] != vID && 
				 pKnownVerts->find(nTri[k]) != pKnownVerts->end() && 
				 pParents->find(nTri[k]) == pParents->end() ) {
					 pParents->insert(nTri[k]);
			}
		}		
	}
};





bool COILSBoundaryDeformer::Encode_Layer(EncodingLayer & layer)
{
	// we always know boundary vertices...
	VertSet vKnown;
	size_t nIntCount = layer.vInteriorOrder.size();
	for ( unsigned int i = 0; i < nIntCount; ++i ) {
		IMesh::VertexID vID = layer.vInteriorOrder[i];
		if ( layer.vBoundaryVertIDs.find(vID) != layer.vBoundaryVertIDs.end() ) {
			vKnown.insert(vID);
			m_vGlobalUpwindSet.insert(vID);
		}
	}

	float fUpwindThresholdDist = m_fMaxEdgeLength * m_fUpwindThreshold;
	_RMSInfo("      Max boundary distance is %6.3f, upwind thresh distance %6.3f  (maxedgelen %6.3f)\n", layer.fMaxBoundaryDist, fUpwindThresholdDist, m_fMaxEdgeLength);
	unsigned int nCurMinUpwind = 0;
	unsigned int nAvgParentSetSize = 0;

	// pre-initialize orders
	for ( unsigned int i = 0; i < nIntCount; ++i ) {
		IMesh::VertexID vID = layer.vInteriorOrder[i];
		layer.vEncoding[vID].nOrder = i;
	}

	// ok now compute parent information for each vertex
	for ( unsigned int i = 0; i < nIntCount; ++i ) {
		IMesh::VertexID vID = layer.vInteriorOrder[i];
		Wml::Vector3f vVtx, vNormal;
		m_pMesh->GetVertex( vID, vVtx, &vNormal);

		layer.vEncoding[vID].vID = vID;
		layer.vEncoding[vID].nOrder = i;
		layer.vEncoding[vID].fBoundaryGeoDist = layer.vBoundaryDistances[vID];
		layer.vEncoding[vID].bWeightsValid = false;
		layer.vEncoding[vID].bIsBoundary = ! ( layer.vBoundaryVertIDs.find(vID) == layer.vBoundaryVertIDs.end() );

		// set initial frame
		// skip boundary vertices, but copy frame first
		if ( layer.vEncoding[vID].bIsBoundary ) {
			BoundaryVert findme(vID);
			std::set<BoundaryVert>::iterator found(layer.vBoundaryVerts.find(findme));
			layer.vEncoding[vID].vFrame = (*found).vInitial;
			continue;
		}

		bool bIsBdryOneRingNbr = false;
		const std::vector<IMesh::VertexID> & vNbrs = m_oneringCache.GetNeighbours(vID).vNbrs;
		for ( unsigned int k = 0; k < vNbrs.size(); ++k ) {
			if ( layer.vBoundaryVertIDs.find(vNbrs[k]) != layer.vBoundaryVertIDs.end() )
				bIsBdryOneRingNbr = true;
		}

		// find parents
		VertSet parents;
		switch ( layer.eEncodeMode ) {

			case AllBoundary:
				parents = layer.vBoundaryVertIDs;
				break;

			case UpwindThreshold:
				{
					// [RMS] this could be done much more efficiently...
					float fCurBoundaryGeoDist = layer.vEncoding[vID].fBoundaryGeoDist;
					while ( fCurBoundaryGeoDist - layer.vEncoding[layer.vInteriorOrder[nCurMinUpwind]].fBoundaryGeoDist > fUpwindThresholdDist )
						++nCurMinUpwind;		// update min-dist
					size_t nStop = i;
					for ( unsigned int k = nCurMinUpwind; k < nStop; ++k )
						parents.insert( layer.vInteriorOrder[k] );
				}
				break;

			case TwoPass:
				DebugBreak();
				break;
		}


		// set frame by transforming from nearby parent
		//rms::Frame3f vParentFrame( layer.vEncoding[*parents.begin()].vFrame );
		//vParentFrame.AlignZAxis( vNormal );
		//vParentFrame.Origin() = vVtx;
		//layer.vEncoding[vID].vFrame = vParentFrame; 
		layer.vEncoding[vID].vFrame = rms::Frame3f( vVtx, vNormal ); 

		if ( parents.size() == 0 )
			DebugBreak();
		nAvgParentSetSize += (unsigned int)parents.size();

		// make parent set and compute stats so we can sort
		std::set<Parent> vParentSet;
		layer.vEncoding[vID].fMinParentDist = EncodeRelative( layer, vID, parents, vParentSet );

		// now store parent set
		unsigned int nNum = 0;
		std::set<Parent>::iterator curps(vParentSet.begin()), endps(vParentSet.end());
		while ( curps != endps ) {
			Parent & p = const_cast<Parent&>(*curps);
			++curps;
			layer.vEncoding[vID].vParents.push_back(p);
			++nNum;
		}

		ComputeRelativeDensityWeights( layer, vID, layer.vEncoding[vID].vParents );

		// this vertex is now encoded
		vKnown.insert( vID );
		m_vGlobalUpwindSet.insert( vID );
	}

	_RMSInfo("      Average %3.1f verts in parent set (%d in layer, %d in mesh total)\n", (float)nAvgParentSetSize / (float)(nIntCount - layer.vBoundaryVerts.size()), layer.vInteriorOrder.size(), m_pMesh->GetVertexCount());


	// pre-validate weights
	for ( unsigned int i = 0; i < nIntCount; ++i ) {
		ValidateWeights( layer, layer.vInteriorOrder[i], &layer == &m_vLayers[0] );
	}

	return true;
}





typedef std::list<IMesh::VertexID> VertList;






class GetNeighbourSetTriCallback : public IMesh::NeighborTriCallback {
public:
	GetNeighbourSetTriCallback() {}
	VFTriangleMesh * pMesh;
	COILSBoundaryDeformer::VertSet vNbrs;
	std::set<IMesh::TriangleID> vTris;
	virtual void NextTriangle( IMesh::TriangleID tID ) { 
		vTris.insert(tID);
		IMesh::VertexID vTri[3];
		pMesh->GetTriangle(tID, vTri);
		for ( int j = 0; j < 3; ++j )
			vNbrs.insert(vTri[j]);
	}
	void MakeNbrVector( std::vector<IMesh::VertexID> & v, const COILSBoundaryDeformer::VertSet & vSkip ) {
		COILSBoundaryDeformer::VertSet::iterator cur(vNbrs.begin()), end(vNbrs.end());
		while ( cur != end ) {
			if ( vSkip.find(*cur) == vSkip.end() )
				v.push_back(*cur);
			++cur;
		}
	}

};






void COILSBoundaryDeformer::RemoveDuplicates( VertSet & vKeep, VertSet & vFilter )
{
	VertSet::iterator curk(vKeep.begin()), endk(vKeep.end());
	while ( curk != endk ) {
		vFilter.erase( *curk );
		++curk;
	}
}



float COILSBoundaryDeformer::EncodeRelative( EncodingLayer & layer, IMesh::VertexID vID, const VertSet & vParentVIDs, std::set<Parent> & vParents )
{
	if ( vParentVIDs.size() == 0 )
		return 0.0f;

	if ( m_bUseUpwindGeodesicDistances ) {
		m_dijkstra.Reset();

		m_dijkstra.AppendStartValue(vID);
		if ( layer.eEncodeMode == AllBoundary ) {
			m_dijkstra.AppendFilterVerts( m_vGlobalUpwindSet );
			m_dijkstra.AppendTerminateVerts( vParentVIDs );
		} else {
			m_dijkstra.AppendFilterVerts( vParentVIDs );
		}
		m_dijkstra.AppendFilterVert( vID );

		m_dijkstra.FindAllDistances();
	}
	VertEncoding & v = layer.vEncoding[vID];


	float fMinDist = std::numeric_limits<float>::max();
	VertSet::const_iterator curp(vParentVIDs.begin()), endp(vParentVIDs.end());
	while ( curp != endp ) {
		Parent p;
		p.vID = *curp;  ++curp;

		if ( m_bUseUpwindGeodesicDistances ) {
			if ( ! m_dijkstra.GetResults()[p.vID].bFrozen )		// skip disconnected parent verts
				continue;
			p.fDistance = m_dijkstra.GetResults()[p.vID].fMinDist;
		} else 
			p.fDistance = m_meshDistCache.GetValue(vID, p.vID);

		if ( p.fDistance < fMinDist )
			fMinDist = p.fDistance;
		p.fWeight = -1;

		// store frame vectors in parent frame coord system
		rms::Frame3f vParentFrame = layer.vEncoding[p.vID].vFrame;
		rms::Frame3f vRelFrame = layer.vEncoding[vID].vFrame;
		vRelFrame.Rotate( vParentFrame.ToFrameMatrix()  );

		// store origin as offset vector in local frame coords
		Wml::Vector3f vOrigin = vRelFrame.Origin() - vParentFrame.Origin();
		vParentFrame.ToFrameLocal(vOrigin);
		vRelFrame.Origin() = vOrigin;

		p.vOffsetVector = vRelFrame.Origin();
		p.vOffsetFrame = vRelFrame.FromFrameMatrix();

		vParents.insert(p);
	};

	return fMinDist;
}




void COILSBoundaryDeformer::ValidateWeights( EncodingLayer & layer, IMesh::VertexID vID, bool bIsBaseLayer )
{
	if ( layer.vEncoding[vID].bWeightsValid == true )
		return;

	float fWeightSum = 0.0f;
	float fGeoDist = layer.vEncoding[vID].fBoundaryGeoDist;
	float fGeoDeltaFalloffRadius = m_fMinEdgeLength;
	float fMinParentDist = layer.vEncoding[vID].fMinParentDist;

	bool bEnableUpwindCorrection = m_bEnableUpwindCorrection;
	bool bEnableDensityCorrection = m_bEnableDensityCorrection;
	if ( layer.eEncodeMode == TwoPass ) {
		m_bEnableUpwindCorrection = false;
		m_bEnableDensityCorrection = false;
	}

	size_t nParents = layer.vEncoding[vID].vParents.size();

	for ( unsigned int k = 0; k < nParents; ++k ) {
		Parent & p = layer.vEncoding[vID].vParents[k];

		p.fWeight = 1.0f / p.fDistance;
		float fK = (bIsBaseLayer) ? m_fDistanceWeightPower : 2.0f;
		p.fWeight = (float)pow(p.fWeight, fK);


		// add near-upwind factor   (this could be precomputed, no?)
		if ( m_bEnableUpwindCorrection && ! layer.vEncoding[p.vID].bIsBoundary  ) {
			float fParentGeoDist = layer.vEncoding[p.vID].fBoundaryGeoDist;
			float fGeoDelta = fabs(fGeoDist - fParentGeoDist);

			// smooth falloff
			if ( fGeoDelta < fGeoDeltaFalloffRadius ) {
				fGeoDelta /= fGeoDeltaFalloffRadius;
				fGeoDelta = 1.0f - fGeoDelta*fGeoDelta;
				fGeoDelta = 1.0f - fGeoDelta*fGeoDelta*fGeoDelta;
			} else
				fGeoDelta = 1.0f;
			p.fWeight *= fGeoDelta;
		}

		// add density factor
		if ( m_bEnableDensityCorrection ) {
			p.fWeight *= p.fRelativeDensity;
		}

		// smooth falloff for upwind threshold weights...
		if ( layer.eEncodeMode == UpwindThreshold ) {
			float fUpwindThresholdDist = m_fMaxEdgeLength * m_fUpwindThreshold * m_fUpwindThresholdScale;

			float fParentGeoDist = layer.vEncoding[p.vID].fBoundaryGeoDist;
			float fGeoDelta = fabs(fGeoDist - fParentGeoDist);

			fGeoDelta /= fUpwindThresholdDist;
			float fFalloff = 1 - fGeoDelta*fGeoDelta;
			if ( fFalloff < 0 ) 
				fFalloff = 0;
			else
				fFalloff = fFalloff*fFalloff*fFalloff;
			if ( fFalloff < 0 || fFalloff > 1 )
				DebugBreak();
			p.fWeight *= fFalloff;
		}

		fWeightSum += p.fWeight;
	}

	// normalize weights
	layer.vEncoding[vID].fMaxWeight = 0.0f;
	for ( unsigned int k = 0; k < nParents; ++k ) {
		Parent & p = layer.vEncoding[vID].vParents[k];
		p.fWeight /= fWeightSum;
		if ( p.fWeight > layer.vEncoding[vID].fMaxWeight )
			layer.vEncoding[vID].fMaxWeight = p.fWeight;
	}

	layer.vEncoding[vID].bWeightsValid = true;


	if ( layer.eEncodeMode == TwoPass ) {
		m_bEnableUpwindCorrection = bEnableUpwindCorrection;
		m_bEnableDensityCorrection = bEnableDensityCorrection;
	}

}



void COILSBoundaryDeformer::InvalidateAllWeights(bool bDefer)
{
	if ( bDefer ) {
		m_bDeferredWeightInvalidationPending = true;
	} else { 
		for ( unsigned int li = 0; li < m_vLayers.size(); ++li ) {
			EncodingLayer & layer = m_vLayers[li];
			size_t nIntCount = layer.vInteriorOrder.size();
			for ( unsigned int i = 0; i < nIntCount; ++i )
				layer.vEncoding[ layer.vInteriorOrder[i] ].bWeightsValid = false;
		}
		m_bDeferredWeightInvalidationPending = false;
	}
}




struct DJVert {
	IMesh::VertexID vID;
	float fMinDist;
	COILSBoundaryDeformer::VertSet vNbrs;
	bool bFrozen;
	DJVert() { vID = IMesh::InvalidID; fMinDist = 0; bFrozen = false; }

	inline bool operator<( const DJVert & d2 ) const {
		if ( fMinDist < d2.fMinDist || (fMinDist == d2.fMinDist && vID < d2.vID) )
			return true;
		return false;
	}
};
struct DJVertWrapper {
	DJVertWrapper( DJVert  * p ) { pVert = p; }
	DJVert * pVert;
	inline bool operator< ( const DJVertWrapper & v2 ) const {
		if ( pVert->fMinDist < v2.pVert->fMinDist || (pVert->fMinDist == v2.pVert->fMinDist && pVert->vID < v2.pVert->vID) )
			return true;
		return false;
		//return *pVert < *(v2.pVert);
	}
};






void COILSBoundaryDeformer::ComputeRelativeDensityWeights( EncodingLayer & layer, IMesh::VertexID vID, std::vector<Parent> & vParents )
{
	size_t nCount = vParents.size();

	std::set<IMesh::VertexID> vParentSet;
	for ( unsigned int i = 0; i < nCount; ++i )
		vParentSet.insert( vParents[i].vID );


	for ( unsigned int i = 0; i < nCount; ++i ) {

		if ( layer.eEncodeMode != AllBoundary ) {

#if 1
			vParents[i].fRelativeDensity = m_vOneRingAreas[ vParents[i].vID ];
#else
			vParents[i].fRelativeDensity = m_vAvgNbrDists[ vParents[i].vID ];
			vParents[i].fRelativeDensity *= vParents[i].fRelativeDensity;
#endif

		} else {

			// set 
			for ( unsigned int k = 0; k < layer.vBoundaryVertsVec.size(); ++k ) {
				if ( layer.vBoundaryVertsVec[k]->vID == vParents[i].vID )
					vParents[i].fRelativeDensity = layer.vBoundaryVertsVec[k]->fNbrEdgeLength;
			}

		}
	}
}





namespace rms {

/*
 * This one-ring neighbour iteration finds a boundary neighbour of [vertid]
  */
class FindBoundaryNbrCallback : public IMesh::NeighborTriCallback
{
public:
	FindBoundaryNbrCallback( IMesh::VertexID vertid, 
		                     VFTriangleMesh * mesh,
							 std::set<IMesh::VertexID> * boundaryLoop ) {
		vID = vertid;
		pMesh = mesh;
		vNbr = IMesh::InvalidID;
		pBoundaryLoop = boundaryLoop;
	}
	IMesh::VertexID vID;
	IMesh::VertexID vNbr;
	VFTriangleMesh * pMesh;
	std::set<IMesh::VertexID> * pBoundaryLoop;

	virtual void NextTriangle( IMesh::TriangleID tID ) {
		if ( vNbr != IMesh::InvalidID )
			return;
		IMesh::VertexID nTri[3];
		pMesh->GetTriangle(tID, nTri);
		for ( int k = 0; k < 3; ++k ) {
			if ( nTri[k] != vID && (pBoundaryLoop->find(nTri[k]) != pBoundaryLoop->end()) )
				 vNbr = nTri[k];
		}
	}
};

}  // end namespace





void COILSBoundaryDeformer::InitializeLayers_Basic()
{
	// store initial mesh state
	m_vOrigMesh.resize( m_pMesh->GetMaxVertexID() );
	VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		m_pMesh->GetVertex( vID, m_vOrigMesh[vID].vPosition, &m_vOrigMesh[vID].vNormal );
	}

	// compute boundary loops and select largest as loop we are going to ultimately encode relative to
	std::vector< std::vector< IMesh::VertexID > > vBoundaryLoops;
	MeshUtils::FindBoundaryLoops( *m_pMesh, vBoundaryLoops );
	int nLargest = -1;
	_RMSInfo("Largest is %d, use is %d\n", nLargest, m_nUseBoundaryLoop);
	if ( m_nUseBoundaryLoop >= 0 ) {
		nLargest = m_nUseBoundaryLoop;
	} else {
		for ( unsigned int k = 0; k < vBoundaryLoops.size(); ++k ) {
			if ( nLargest == -1 || vBoundaryLoops[k].size() > vBoundaryLoops[nLargest].size() )
				nLargest = k;
		}
	}
	_RMSInfo("Largest is %d, use is %d\n", nLargest, m_nUseBoundaryLoop);
	std::vector<IMesh::VertexID> & vOutermostLoop = vBoundaryLoops[nLargest];
	std::set<IMesh::VertexID> vBoundarySet( vOutermostLoop.begin(), vOutermostLoop.end() );

	// make initial layer that has all vertices
	EncodingLayer newLayer;
	m_vLayers.push_back(newLayer);
	EncodingLayer & base = m_vLayers[0];
	base.eEncodeMode = m_eEncodeMode;

	std::vector<IMesh::VertexID> & vBoundaryLoop = vOutermostLoop;

	// initialize layer vert sets
	base.vInLayer.resize( m_pMesh->GetMaxVertexID(), false );
	curv = m_pMesh->BeginVertices();
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		base.vInLayer[vID] = true;
		base.vVertices.insert(vID);
	}

	// initialize boundary vert list
	base.vBoundaryLoop = vBoundaryLoop;
	size_t nLoopSize = vBoundaryLoop.size();
	for ( unsigned int k = 0; k < nLoopSize; ++k ) {
		IMesh::VertexID vID = vBoundaryLoop[k];
		BoundaryVert v(vID);

		// find nbr vtx in each direction
		v.vNbrIDs[0] = vBoundaryLoop[ (k+1) % nLoopSize ];
		v.vNbrIDs[1] = vBoundaryLoop[ (k == 0) ? nLoopSize-1 : k-1 ];

		// compute sum of nbr lengths
		Wml::Vector3f vNbrVertex, vVertex, vNormal;
		m_pMesh->GetVertex(vID, vVertex, &vNormal);
		v.fNbrEdgeLength = 0.0f;
		for ( int j = 0; j < 2; ++j ) {
			m_pMesh->GetVertex( v.vNbrIDs[j], vNbrVertex );
			v.fNbrEdgeLength += (vNbrVertex-vVertex).Length();
		}

		v.vInitial = rms::Frame3f( vVertex, vNormal ); 

		// need a way to compute frame such that we can get consistent tangent directions 
		// after the boundary is deformed. Currently using direction to one boundary nbr...

		// tangent vector defined by nbr vertex  (use both nbrs?)
		m_pMesh->GetVertex( v.vNbrIDs[0], vNbrVertex );
		Wml::Vector3f vTangent( vNbrVertex - vVertex );
		vTangent.Normalize();
		Wml::Vector3f vTan2( vNormal.Cross(vTangent) );
		vTan2.Normalize();
		vTangent = vTan2.Cross(vNormal);
		vTangent.Normalize();

		v.vInitial = rms::Frame3f(vVertex);
		v.vInitial.SetFrame( vTangent, vTan2, vNormal );
			
		base.vBoundaryVerts.insert( v );
		base.vBoundaryVertIDs.insert( vID );
	}
}





void COILSBoundaryDeformer::GenerateBaseMesh()
{
	static const bool bUseImportance = true;
	static const int nReduceToVertCount = 1024;

	std::vector<float> vImportance;
	if ( bUseImportance ) {
		// run dijkstra propagation to get importance
		m_dijkstra.Reset();
		std::vector< std::vector< IMesh::VertexID > > vBoundaryLoops;
		MeshUtils::FindBoundaryLoops( *m_pMesh, vBoundaryLoops );	
		int nLargest = -1;
		for ( unsigned int k = 0; k < vBoundaryLoops.size(); ++k ) {
			if ( nLargest == -1 || vBoundaryLoops[k].size() > vBoundaryLoops[nLargest].size() )
				nLargest = k;
		}
		std::vector<IMesh::VertexID> & vOutermostLoop = vBoundaryLoops[nLargest];
		for ( unsigned int i = 0; i < vOutermostLoop.size(); ++i )
			m_dijkstra.AppendStartValues( vOutermostLoop, 0.0f );
		m_dijkstra.FindAllDistances();
		const std::vector<DijkstraFrontProp<IMesh::VertexID>::VertInfo> & results =  m_dijkstra.GetResults();

		unsigned int nVertCount = m_pMesh->GetMaxVertexID();
		vImportance = std::vector<float>( nVertCount, 0.0f );
		float fMaxDist = 0.0f;
		for ( unsigned int k = 0; k < nVertCount; ++k ) {
			vImportance[k] = results[k].fMinDist;
			if ( vImportance[k] > fMaxDist )
				fMaxDist = vImportance[k];
		}
		float fThresh = 0.4f;
		for ( unsigned int k = 0; k < nVertCount; ++k ) {
			float fT = vImportance[k] / fMaxDist;
			vImportance[k] = (fT>fThresh)  ?  1.0f  :  1.0f + (fThresh-fT)/fThresh;
			if ( ! _finite(vImportance[k]) )
				vImportance[k] = 1.0f;
		}
	}

	VFTriangleMesh reduced;
	reduced.Copy(*m_pMesh);
	
	float fMin, fMax, fAvg;
	rms::MeshUtils::GetBoundaryEdgeStats(reduced, fMin, fMax, fAvg);
	fAvg *= 1.25f;

	_RMSInfo("   Initial vtx count is %d\n", reduced.GetVertexCount());
	while ( reduced.GetVertexCount() > nReduceToVertCount ) {
		int nIters = 0;   bool bConverged = false;
		while ( ! bConverged && nIters < 10 && reduced.GetVertexCount() > nReduceToVertCount ) {
			++nIters;
			bool bCollapsed = rms::MeshUtils::CollapseTipFaces(reduced);
			bool bMerged = rms::MeshUtils::MergeVertices(reduced, fAvg, (bUseImportance) ? &vImportance : NULL);
			if ( ! bCollapsed && ! bMerged )
				bConverged = true;
			_RMSInfo("     Reduced vtx count is %d\n", reduced.GetVertexCount());
		}

		fAvg *= 1.25f;
	}

	MeshSmoother smoother;
	smoother.SetSurface(&reduced);
	smoother.DoTaubinSmooth(10);

	m_baseMesh.Clear(false);
	m_baseMesh.Copy(reduced);

	// estimate normals on base mesh
	MeshUtils::EstimateNormals(m_baseMesh);

	std::vector< std::vector<IMesh::VertexID> > vOrigLoops;
	MeshUtils::FindBoundaryLoops(*m_pMesh, vOrigLoops );
	std::vector< std::vector<IMesh::VertexID> > vBaseLoops;
	MeshUtils::FindBoundaryLoops(m_baseMesh, vBaseLoops );
	std::vector<MeshUtils::BoundaryLoopMap> vMaps;
	MeshUtils::FindBoundaryLoopMaps(*m_pMesh, vOrigLoops, m_baseMesh, vBaseLoops, vMaps);
	int iLargest = -1;  size_t nLargest = 0;
	for ( int i = 0; i < (int)vMaps.size(); ++i ) {
		size_t nLoopSize = vMaps[i].vFrom.size();
		if ( nLoopSize > nLargest ) {
			nLargest = nLoopSize;
			iLargest = i;
		}
	}
	if ( iLargest == -1 )
		DebugBreak();
	MeshUtils::BoundaryLoopMap & useMap = vMaps[iLargest];
	m_vBoundaryLoopBaseMap.clear();
	for ( unsigned int i = 0; i < useMap.vFrom.size(); ++i ) {
		IMesh::VertexID vOrigID = useMap.vFrom[i];
		IMesh::VertexID vBaseID = useMap.vTo[i];
		m_vBoundaryLoopBaseMap[vOrigID] = vBaseID;
		m_baseMesh.SetNormal( vBaseID, m_pMesh->GetNormal(vOrigID) );
	}

}




void COILSBoundaryDeformer::FindBaseParents( IMesh::VertexID vDetailID, VertSet & parents, VertSet * pKnown )
{
	Wml::Vector3f vVert, vNearest;
	m_pOriginalMesh->GetVertex(vDetailID, vVert);
	IMesh::TriangleID tNearest;
	m_baseMeshBVTree.FindNearest(vVert, vNearest, tNearest);
	IMesh::VertexID nTri[3];
	m_baseMesh.GetTriangle(tNearest, nTri);
	for ( int j = 0; j < 3; ++j ) {
		const std::vector<IMesh::VertexID> & vNbrs = m_oneringCache.GetNeighbours(nTri[j]).vNbrs;
		for ( unsigned int k = 0; k < vNbrs.size(); ++k )
			parents.insert(vNbrs[k]);
	}

}



void COILSBoundaryDeformer::InitializeLayers_DetailMesh()
{
	EncodingLayer & baseLayer = m_vLayers[0];

	/*
	 * initialize detail layer
	 */

	m_detailLayer = EncodingLayer();
	m_detailLayer.eEncodeMode = TwoPass;

	unsigned int nVerts = m_pOriginalMesh->GetMaxVertexID();
	m_detailLayer.vInLayer.resize(nVerts);

	m_vOrigOrigMesh.resize(nVerts);

	// add detail mesh vertices to detail layer
	Wml::Vector3f vVertex, vNearest;
	VFTriangleMesh::vertex_iterator curv(m_pOriginalMesh->BeginVertices()), endv(m_pOriginalMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		m_detailLayer.vInLayer[vID] = true;
		m_detailLayer.vVertices.insert(vID);
		m_detailLayer.vInteriorOrder.push_back(vID);

		m_pOriginalMesh->GetVertex(vID, vVertex);
		m_vOrigOrigMesh[vID].vPosition = vVertex;
		IMesh::VertexID vBaseNearestID;
		
		m_baseMeshBVTree.FindNearestVtx(vVertex, vBaseNearestID);
		m_baseMesh.GetVertex(vBaseNearestID,vNearest);
		float fDist = (vNearest-vVertex).Length();

		if ( fDist < 0.001f && baseLayer.vBoundaryVertIDs.find(vBaseNearestID) != baseLayer.vBoundaryVertIDs.end() )
			m_detailLayer.vBoundaryVertIDs.insert(vID);
	}

	/*
	 * encode detail layer
	 */ 

	m_detailLayer.vEncoding.resize(nVerts);
	unsigned int nAvgParentSetSize = 0;

	// ok now compute parent information for each vertex
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		IMesh::VertexID vID = m_detailLayer.vInteriorOrder[i];
		m_detailLayer.vEncoding[vID].vID = vID;
		m_detailLayer.vEncoding[vID].bWeightsValid = false;
		m_detailLayer.vEncoding[vID].bIsBoundary = ! ( m_detailLayer.vBoundaryVertIDs.find(vID) == m_detailLayer.vBoundaryVertIDs.end() );
		Wml::Vector3f vVertex, vNormal, vParentVertex;
		m_pOriginalMesh->GetVertex(vID, vVertex, &vNormal);

		if ( m_detailLayer.vEncoding[vID].bIsBoundary ) {
			IMesh::VertexID vBaseNearestID;
			m_baseMeshBVTree.FindNearestVtx(vVertex, vBaseNearestID);
			Parent p;
			p.vID = vBaseNearestID;
			p.fDistance = 0;
			p.fWeight = 1.0f;
			m_detailLayer.vEncoding[vID].vParents.push_back(p);

		} else {

			VertSet parents;
			FindBaseParents( vID, parents );
			if ( parents.size() == 0 )
				DebugBreak();
			nAvgParentSetSize += (unsigned int)parents.size();

			// construct frame
			m_detailLayer.vEncoding[vID].vFrame = rms::Frame3f( vVertex, vNormal ); 

			// construct parent set
			std::set<Parent> vParentSet;
			VertSet::const_iterator curp(parents.begin()), endp(parents.end());
			while ( curp != endp ) {
				Parent p;
				p.vID = *curp;  ++curp;
				m_baseMesh.GetVertex(p.vID, vParentVertex);
				p.fDistance = (vVertex - vParentVertex).Length();
				p.fWeight = -1;
				p.fRelativeDensity = 1.0f;

				// store frame vectors in parent frame coord system
				rms::Frame3f vParentFrame = baseLayer.vEncoding[p.vID].vFrame;
				rms::Frame3f vRelFrame = m_detailLayer.vEncoding[vID].vFrame;
				vRelFrame.Rotate( vParentFrame.ToFrameMatrix()  );

				// store origin as offset vector in local frame coords
				Wml::Vector3f vOrigin = vRelFrame.Origin() - vParentFrame.Origin();
				vParentFrame.ToFrameLocal(vOrigin);
				vRelFrame.Origin() = vOrigin;

				p.vOffsetVector = vRelFrame.Origin();
				p.vOffsetFrame = vRelFrame.FromFrameMatrix();

				vParentSet.insert(p);
			}

			// store parent set
			m_detailLayer.vEncoding[vID].vParents = std::vector<Parent>(vParentSet.begin(), vParentSet.end());
		}

	}
	_RMSInfo("      Average %3.1f verts in detail parent set (%d in layer, %d in mesh total)\n", (float)nAvgParentSetSize / (float)(nVerts - m_detailLayer.vBoundaryVertIDs.size()), m_detailLayer.vInteriorOrder.size(), m_pMesh->GetVertexCount());


	// pre-validate weights
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		ValidateWeights( m_detailLayer, m_detailLayer.vInteriorOrder[i], false );
	}

}







typedef std::set<DJVertWrapper> DJQueue;
typedef std::vector<DJVert> DJVector;


class QueueUpdateCallback : public IMesh::NeighborTriCallback
{
public:
	QueueUpdateCallback( DJQueue * pQueue, DJVector * pVerts, DJVert * vtx, VFTriangleMesh * mesh, const std::vector<float> & vertexCosts,
						 float fSmoothingAlpha = 0.0f) 
		: vVertexCosts(vertexCosts)
	{
		m_pQueue = pQueue;
		m_pVerts = pVerts;
		pVtx = vtx;
		pMesh = mesh;
		m_fSmoothingAlpha = fSmoothingAlpha;
	}
	DJQueue * m_pQueue;
	DJVector * m_pVerts;
	VFTriangleMesh * pMesh;
	DJVert * pVtx;
	Wml::Vector3f m_vVtx;
	const std::vector<float> & vVertexCosts;
	float m_fSmoothingAlpha;

	// internal vars...
	IMesh::VertexID nTri[3];
	Wml::Vector3f vTri[3];

	float GetAvgUpwindValue( IMesh::VertexID vID ) {
		std::vector<IMesh::VertexID> vOneRing;
		MeshUtils::VertexOneRing( *pMesh, vID, vOneRing );
		float fAvg = 0.0f;
		int nUpwind = 0;
		for ( unsigned int i = 0; i < vOneRing.size(); ++i ) {
			DJVert & v = (*m_pVerts)[vOneRing[i]];
			if ( v.bFrozen ) {
				fAvg += v.fMinDist;
				nUpwind++;
			}
		}
		if ( nUpwind > 0 )
			return fAvg / (float)nUpwind;
		else
			return 0.0f;
	}

	virtual void BeginTriangles() {
		pMesh->GetVertex(pVtx->vID, m_vVtx);
	}
	virtual void NextTriangle( IMesh::TriangleID tID ) {
		pMesh->GetTriangle(tID, nTri);
		pMesh->GetTriangle(tID, vTri);
		for ( int k = 0; k < 3; ++k ) {

		}		
	}
};






void COILSBoundaryDeformer::OrderVertices( EncodingLayer & layer )
{
	if ( layer.vVertices.empty() )
		DebugBreak();

	unsigned int nMaxVertexID = m_pMesh->GetMaxVertexID();
	DJVector vVertices( nMaxVertexID );

	ComputeOneRingAreas();

	float fUpwindSmoothingAlpha = 0.0f;

	// pre-allocate distances list
	layer.vInteriorOrder.resize(0);
	layer.vBoundaryDistances.resize(0);
	layer.vBoundaryDistances.resize(m_pMesh->GetMaxVertexID());

	// [RMS TODO] should use m_dijkstra here, and restrict propagation to current layer (is doing whole mesh right now...)

	// initialize vertex list
	VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv; ++curv;
		vVertices[vID].vID = vID;
		vVertices[vID].fMinDist = std::numeric_limits<float>::max()/2;
		vVertices[vID].bFrozen = false;

		// construct neighbour set
		const std::vector<IMesh::VertexID> & vNbrs = m_pNeighbourCache->GetNeighbours( vID ).vNbrs;
		for ( unsigned int k = 0; k < vNbrs.size(); ++k )
			vVertices[vID].vNbrs.insert( vNbrs[k] );

		if ( layer.vBoundaryVertIDs.find(vID) != layer.vBoundaryVertIDs.end() ) {
			vVertices[vID].fMinDist = 0;
			vVertices[vID].bFrozen = true;
			layer.vInteriorOrder.push_back( vID );
			layer.vBoundaryDistances[vID] = 0;
		}
	}


	DJQueue vQueue;

	// find vertices connected to boundary (have to initialize these differently)
	std::set<IMesh::VertexID> vBoundaryNbrVerts;
	std::set<IMesh::VertexID>::iterator bcurv( layer.vBoundaryVertIDs.begin()), bendv( layer.vBoundaryVertIDs.end());
	while ( bcurv != bendv ) {
		IMesh::VertexID vID = *bcurv++;
		VertSet::iterator curv(vVertices[vID].vNbrs.begin()), endv(vVertices[vID].vNbrs.end());
		while ( curv != endv ) {
			DJVert & v = vVertices[*curv++];
			if ( v.bFrozen )
				continue;
			vBoundaryNbrVerts.insert( v.vID );
		}
	}

	// initialize all boundary neighbours with distances to nearest point on boundary loop
	size_t nLoopSize = layer.vBoundaryLoop.size();
	std::set<IMesh::VertexID>::iterator curn(vBoundaryNbrVerts.begin()), endn(vBoundaryNbrVerts.end());
	while ( curn != endn ) {
		DJVert & v = vVertices[*curn++];

		float fMinDistSqr = std::numeric_limits<float>::max();
		Wml::Vector3f vVtx, vSeg[2];  
		m_pMesh->GetVertex( v.vID, vVtx );
		for ( unsigned int k = 0; k < nLoopSize; ++k ) {
			m_pMesh->GetVertex( layer.vBoundaryLoop[k], vSeg[0] );
			m_pMesh->GetVertex( layer.vBoundaryLoop[(k+1)%nLoopSize], vSeg[1] );
			vSeg[1] -= vSeg[0];    float fExtent = vSeg[1].Normalize();
			Wml::Segment3f seg(vSeg[0],vSeg[1],fExtent);
			Wml::DistVector3Segment3f dist(vVtx, seg);
			float fDistSqr = dist.GetSquared();
			if ( fDistSqr < fMinDistSqr )
				fMinDistSqr = fDistSqr;
		}
		v.fMinDist = (float)sqrt(fMinDistSqr);
		v.bFrozen = true;

		vQueue.insert( DJVertWrapper( &v ) );
	}

	// ok now pop verts one-by-one
	layer.fMaxBoundaryDist = 0.0f;
	while (! vQueue.empty() ) {
		
		DJVertWrapper remove = * vQueue.begin();
		remove.pVert->bFrozen = true;
		if ( layer.vVertices.find(remove.pVert->vID) != layer.vVertices.end() ) {
			layer.vInteriorOrder.push_back( remove.pVert->vID );
			layer.vBoundaryDistances[remove.pVert->vID] = remove.pVert->fMinDist;
			if ( remove.pVert->fMinDist > layer.fMaxBoundaryDist )
				layer.fMaxBoundaryDist = remove.pVert->fMinDist;
		}

		vQueue.erase( vQueue.begin() );


		// update queue
		IMesh::VertexID vID = remove.pVert->vID;
		Wml::Vector3f vUpdateVtx, vNbrVtx;   
		m_pMesh->GetVertex( vID, vUpdateVtx );
		VertSet::iterator curv(vVertices[vID].vNbrs.begin()), endv(vVertices[vID].vNbrs.end());
		while ( curv != endv ) {
			DJVert & v = vVertices[*curv++];
			if ( v.bFrozen )
				continue;

			m_pMesh->GetVertex( v.vID, vNbrVtx );
			float fDist = vVertices[vID].fMinDist + ( vNbrVtx - vUpdateVtx ).Length();
			if ( fDist < v.fMinDist ) {
				DJQueue::iterator found( vQueue.find( DJVertWrapper(&v) ) );
				if ( found != vQueue.end() ) 
					vQueue.erase(found);
				v.fMinDist = fDist;
				vQueue.insert( DJVertWrapper(&v) );
			}
		}
	}

	// compute index of last point in vInteriorOrder which is connected to boundary
	//  (used to early-halt reconstruction when doing boundary-frame optimization)
	layer.nLastBoundaryNbrIndex = 0;
	size_t nCount = layer.vInteriorOrder.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::IMesh::VertexID vID = layer.vInteriorOrder[i];
		const std::vector<IMesh::VertexID> & vOneRing = m_oneringCache.GetNeighbours(vID).vNbrs;
		bool bBoundaryNbr = false;
		for ( unsigned int k = 0; k < vOneRing.size() && ! bBoundaryNbr; ++k )
			if ( layer.vBoundaryVertIDs.find( vOneRing[k] ) != layer.vBoundaryVertIDs.end() )
				bBoundaryNbr = true;
		if ( bBoundaryNbr )
			layer.nLastBoundaryNbrIndex = i;
	}


	// initialize scalar set for distance   
	// [TODO] move this elsewhere - is happening multiple times...
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::IMesh::VertexID vID = layer.vInteriorOrder[i];
		float fD = layer.vBoundaryDistances[ vID ];
		m_pMesh->SetScalar( vID, 0, fD );
	}

}






void COILSBoundaryDeformer::ComputeOneRingAreas()
{
	// compute scalars for mesh
	m_vOneRingAreas.resize( 0 );   
	m_vOneRingAreas.resize( m_pMesh->GetMaxVertexID(), 0 );

	VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		
		Wml::Vector3f vTri[3];
		m_pMesh->GetTriangle( tID, vTri );
		float fArea = rms::Area( vTri[0], vTri[1], vTri[2] );

		IMesh::VertexID nTri[3];
		m_pMesh->GetTriangle(tID, nTri);
		for ( int k = 0; k < 3; ++k )
			m_vOneRingAreas[ nTri[k] ] += fArea;
	}

	m_vAvgNbrDists.resize(0);
	m_vAvgNbrDists.resize( m_pMesh->GetMaxVertexID(), 0 );

	Wml::Vector3f vVertex, vNbr;
	VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv++;
		m_pMesh->GetVertex(vID,vVertex);

		const std::vector<IMesh::VertexID> & vNbrs = m_pNeighbourCache->GetNeighbours(vID).vNbrs;
		float fMin = 99999.0f, fMax = 0.0f, fAvg = 0.0f;
		for ( unsigned int k = 0; k < vNbrs.size(); ++k ) {
			m_pMesh->GetVertex(vNbrs[k], vNbr);
			float fDist = (vNbr-vVertex).Length();
			if ( fDist < fMin ) fMin = fDist;
			if ( fDist > fMax ) fMax = fDist;
			fAvg += fDist;
		}
		fAvg /= (float)vNbrs.size();
		m_vAvgNbrDists[vID] = fMin;
	}
}



const IMesh::VertexID COILSBoundaryDeformer::GetBoundaryVertID( IMesh::VertexID vExternalID )
{
	if ( m_eEncodeMode == TwoPass )
		return m_vBoundaryLoopBaseMap[vExternalID];
	else
		return vExternalID;
}


const rms::Frame3f & COILSBoundaryDeformer::GetInitialBoundaryFrame(IMesh::VertexID vID)
{
	static const rms::Frame3f INTERNAL_FRAME;

	BoundaryVert findme(vID);
	std::set<BoundaryVert>::iterator found(m_vLayers[0].vBoundaryVerts.find(findme));
	if ( found != m_vLayers[0].vBoundaryVerts.end() )
		return (*found).vInitial;
	return INTERNAL_FRAME;
}

const rms::Frame3f & COILSBoundaryDeformer::GetCurrentBoundaryFrame(IMesh::VertexID vID)
{
	return m_vLayers[0].vEncoding[vID].vFrame;
}




void COILSBoundaryDeformer::UpdateBoundary( IMesh::VertexID vID, const Wml::Vector3f & vNewOrigin,  const Wml::Vector3f & vNewNormal, std::vector<Wml::Triangle3f> * pExteriorTris )
{
	BoundaryVert findme(vID);
	std::set<BoundaryVert>::iterator found(m_vLayers[0].vBoundaryVerts.find(findme));
	if ( found  != m_vLayers[0].vBoundaryVerts.end() ) {
		m_vLayers[0].vEncoding[vID].vFrame.Origin() = vNewOrigin;

		// [RMS] only the Z axis (normal) is actually used during the reconstruction, so it
		//   doesn't actually matter if we align the whole frame, or just set the Z...

		//m_vLayers[0].vEncoding[vID].vFrame.SetFrame(
		//	m_vLayers[0].vEncoding[vID].vFrame.X(), m_vLayers[0].vEncoding[vID].vFrame.Y(), vNewNormal );
		m_vLayers[0].vEncoding[vID].vFrame.AlignZAxis( vNewNormal );


		//save this frame so we can re-initialize boundary if params change
		(*found).vSet = m_vLayers[0].vEncoding[vID].vFrame;

		if ( pExteriorTris )
			(*found).vExteriorTris = *pExteriorTris;
	} else
		DebugBreak();
}


void COILSBoundaryDeformer::InitializeBoundary(EncodingLayer & layer)
{

	size_t nCount = layer.vBoundaryVertsVec.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		const BoundaryVert & bv = *layer.vBoundaryVertsVec[i];

		Wml::Vector3f vNormal = bv.vSet.Z();
		Wml::Vector3f vVertex = bv.vSet.Origin();

		float fX = m_vOrigMesh[bv.vID].vPosition.X();
		float fA = (fX - m_bounds.Min[0]) / (m_bounds.Max[0] - m_bounds.Min[0]);
		if ( fA > 1 || fA < 0 ) DebugBreak();

		// find positions of nbr verts
		Wml::Vector3f vNbrVertex[2];
		for ( int k = 0; k < 2; ++k ) {
			BoundaryVert findme( bv.vNbrIDs[k] );
			std::set<BoundaryVert>::iterator found(layer.vBoundaryVerts.find(findme));
			if ( found != layer.vBoundaryVerts.end() )
				vNbrVertex[k] = (*found).vSet.Origin();
			else
				DebugBreak();
		}

		Wml::Vector3f vTangent( vNbrVertex[0] - vVertex );
		vTangent.Normalize();
		Wml::Vector3f vTan2( vNormal.Cross(vTangent) );
		vTan2.Normalize();
		vTangent = vTan2.Cross(vNormal);
		vTangent.Normalize();

		if ( m_fBoundaryRotateAngle != 0.0f || m_bUseOptimalBoundaryFrames ) {
			Wml::Vector3f vSecant( vNbrVertex[0] - vNbrVertex[1] );
			vSecant.Normalize();

			float fAngle = m_fBoundaryRotateAngle;
			if ( m_bUseOptimalBoundaryFrames )
				fAngle += (m_vOptimizedBoundaryAngles.empty()) ? m_fOptimizedBoundaryAngle : m_vOptimizedBoundaryAngles[i];

			Wml::Matrix3f rotate;
			rotate.FromAxisAngle( vSecant, fAngle );

			vTangent = rotate * vTangent;
			vTan2 = rotate * vTan2;
			vNormal = rotate * vNormal;
		}
		layer.vEncoding[bv.vID].vFrame.SetFrame(vTangent, vTan2, vNormal );

		m_pMesh->SetVertex( bv.vID, layer.vEncoding[bv.vID].vFrame.Origin(), &vNormal );
	}

}




static Wml::Vector3f EstimateNormal( VFTriangleMesh * pMesh, IMesh::VertexID vID, 
									const std::vector<Wml::Triangle3f> & vExteriorTris, 
									 bool bUseAreaWeightedNormal )
{
	// estimate current normal
	Wml::Vector3f vAvgFaceNormal(Wml::Vector3f::ZERO);  
	float fWeightSum = 0;
	IMesh::VtxNbrItr itr(vID);
	pMesh->BeginVtxTriangles(itr);
	IMesh::TriangleID tID = pMesh->GetNextVtxTriangle(itr);
	while ( tID != IMesh::InvalidID ) {
		Wml::Vector3f vTri[3];
		pMesh->GetTriangle(tID, vTri);
		float fArea;
		Wml::Vector3f vFaceNormal = rms::Normal(vTri[0], vTri[1], vTri[2], &fArea);
		if ( bUseAreaWeightedNormal ) {
			vAvgFaceNormal += fArea * vFaceNormal;
			fWeightSum += fArea;
		} else {
			vAvgFaceNormal += vFaceNormal;
			fWeightSum += 1.0f;
		}
		tID = pMesh->GetNextVtxTriangle(itr);
	}

	size_t nExt = vExteriorTris.size();
	for ( unsigned int k = 0; k < nExt; ++k ) {
		float fArea;
		Wml::Vector3f vFaceNormal = rms::Normal(vExteriorTris[k].V[0], vExteriorTris[k].V[1], vExteriorTris[k].V[2], &fArea);
		if ( bUseAreaWeightedNormal ) {
			vAvgFaceNormal += fArea * vFaceNormal;
			fWeightSum += fArea;
		} else {
			vAvgFaceNormal += vFaceNormal;
			fWeightSum += 1.0f;
		}
	}
	vAvgFaceNormal /= fWeightSum;
	vAvgFaceNormal.Normalize();

	return vAvgFaceNormal;
}







void COILSBoundaryDeformer::InitializeBoundaryOptimization(EncodingLayer & layer)
{
	size_t nCount = layer.vBoundaryVertsVec.size();
	m_boundaryOptCache.vSecants.resize(nCount);

	for ( unsigned int i = 0; i < nCount; ++i ) {
		const BoundaryVert & bv = *layer.vBoundaryVertsVec[i];
		IMesh::VertexID vID = bv.vID;

		// find nbr boundary verts and get secant
		Wml::Vector3f vNbrVertex[2];
		for ( int k = 0; k < 2; ++k ) {
			BoundaryVert findme( bv.vNbrIDs[k] );
			std::set<BoundaryVert>::iterator found(layer.vBoundaryVerts.find(findme));
			if ( found != layer.vBoundaryVerts.end() )
				vNbrVertex[k] = (*found).vSet.Origin();
			else
				DebugBreak();
		}
		m_boundaryOptCache.vSecants[i] = Wml::Vector3f(vNbrVertex[0] - vNbrVertex[1] );
		m_boundaryOptCache.vSecants[i].Normalize();
	}

	m_boundaryOptCache.vCurNormals.resize(nCount);

	m_fOptimizedBoundaryAngle = 0.0f;
	m_vOptimizedBoundaryAngles.resize(0);
}



float COILSBoundaryDeformer::BoundaryNormalError(EncodingLayer & layer, IMesh::VertexID vModified)
{
	static const bool bUseAreaWeightedNormal = true;

	size_t nCount = layer.vBoundaryVertsVec.size();

	Decode(true);

	Wml::Vector3f vTargetNormal, vCurNormal;
	float fSum = 0.0f;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		const BoundaryVert & bv = *layer.vBoundaryVertsVec[i];

		vTargetNormal = bv.vSet.Z();

		vCurNormal = EstimateNormal(m_pMesh, bv.vID, bv.vExteriorTris, bUseAreaWeightedNormal );
		m_boundaryOptCache.vCurNormals[i] = vCurNormal;

		float fDot = vTargetNormal.Dot(vCurNormal);
		fSum += 1.0f - fDot;
	}


	return fSum;
}



static void LaplacianSmooth( std::vector<float> & v, float fAlpha, bool bLoop )
{
	size_t nCount = v.size();
	if ( nCount < 3 )
		return;
	for ( unsigned int k = 0; k < nCount; ++k ) {
		float fLeft = 0, fRight = 0;   int nNbrCount = 0;
		if ( bLoop ) {
			fLeft = v[  (k==0) ? nCount-1 : k-1  ];
			fRight = v[ (k+1) % nCount ];
			nNbrCount = 2;
		} else {
			if ( k > 0 ) { fLeft = v[k-1];  ++nNbrCount; }
			if ( k < (unsigned int)nCount-1 ) { fRight = v[k+1];  ++nNbrCount; }
		}
		float fCentroid = (fLeft+fRight) / (float)nNbrCount;
		v[k] = (1-fAlpha)*v[k] + (fAlpha)*fCentroid;
	}
}




void COILSBoundaryDeformer::OptimizeBoundaryFrames_Gradient(EncodingLayer & layer, bool bDoPerVertex)
{
	static const float fEps = 0.001f;

	size_t nCount = layer.vBoundaryVertsVec.size();
	float fErrThresh = nCount * (1-cos(1.0f * Wml::Mathf::DEG_TO_RAD));
	float fErrDeltaThresh = (1-cos(2.0f * Wml::Mathf::DEG_TO_RAD));
	float fStepThresh = 0.001f;

	// pass 1 - do one-angle optimization to get close
	m_fOptimizedBoundaryAngle = 0.0f;

	float fCurError = BoundaryNormalError(layer, IMesh::InvalidID);
	float fErrorDelta = 99999.0f;

	// pass 1 - 1D line search for optimal joint angle

	m_fOptimizedBoundaryAngle += fEps;
	float fNewError = BoundaryNormalError(layer, IMesh::InvalidID);
	float fGradient = (fNewError - fCurError) / fEps;
	m_fOptimizedBoundaryAngle = 0.0f;

	float fStepSize = 0.01f;
	float fStartError = fCurError;
	int nIter = 0, nIterFails = 0;
	int nMaxIters = 50;
	_RMSInfo("    LineIter (%6.3f)", fCurError);
	do { 
		float fSaveAngle = m_fOptimizedBoundaryAngle;
		m_fOptimizedBoundaryAngle -= fStepSize * fGradient;
		float fNewError = BoundaryNormalError(layer, IMesh::InvalidID);
		if ( fNewError < fCurError ) {
			fErrorDelta = fCurError - fNewError;
			fCurError = fNewError;
			fStepSize *= 1.25f;
			nIterFails = 0;
		} else {
			fErrorDelta = 1.0f;
			++nIterFails;
			m_fOptimizedBoundaryAngle = fSaveAngle;
			fStepSize *= 0.5f;
		}
		_RMSInfo(" %6.3f ", -fErrorDelta);
		++nIter;
	} while ( nIter < nMaxIters && fErrorDelta > fErrDeltaThresh && nIterFails < 5 );
	_RMSInfo(" (%6.3f) \n", fCurError);
	_RMSInfo("      1D Angle Opt (%d iters):   start err: %6.3f    end err: %6.3f    delta: %6.3f  stop: %6.3f   stepsize %6.3f\n", nIter, fStartError, fCurError, -(fStartError - fCurError), fErrThresh, fStepSize);


	if ( ! bDoPerVertex  )
		return;


	// pass 2 - do per-vertex optimization

	m_vOptimizedBoundaryAngles.resize(0);
	m_vOptimizedBoundaryAngles.resize(nCount, m_fOptimizedBoundaryAngle);
 
	std::vector<float> vGradient(nCount), vSaveVertexAngles(nCount);
	fErrorDelta = 99999.0f;

	_RMSTUNE_start(6);

	nMaxIters = 25;
	fStepSize = 1.0f;
	nIter = 0;
	while ( nIter < nMaxIters && fStepSize > fStepThresh && fCurError > fErrThresh && fErrorDelta > fErrDeltaThresh ) {
		++nIter;

		// compute approximate numerical gradient  (expensive!)
		for ( unsigned int i = 0; i < nCount; ++i ) {
			float fSave = m_vOptimizedBoundaryAngles[i];
			m_vOptimizedBoundaryAngles[i] += fEps;
			float fNewError = BoundaryNormalError(layer, (*layer.vBoundaryVertsVec[i]).vID  );
			vGradient[i] = (fNewError - fCurError) / fEps;
			m_vOptimizedBoundaryAngles[i] = fSave;
		}

		// do line search in gradient direction
		float fLineStepSize = fStepSize;
		float fPrevError = fCurError;
		int nLineIter = 0;
		int nLineIterFails = 0;
		_RMSInfo("    LineIter (%6.3f)", fCurError);
		do { 
			vSaveVertexAngles = m_vOptimizedBoundaryAngles;
			for ( unsigned int i = 0; i < nCount; ++i ) 
				m_vOptimizedBoundaryAngles[i] -= fLineStepSize * vGradient[i];
			float fNewError = BoundaryNormalError(layer, IMesh::InvalidID);
			if ( fNewError < fCurError ) {
				fErrorDelta = fCurError - fNewError;
				fCurError = fNewError;
				fLineStepSize *= 1.25f;
				nLineIterFails = 0;
			} else {
				fErrorDelta = 1.0f;
				++nLineIterFails;
				m_vOptimizedBoundaryAngles = vSaveVertexAngles;
				fLineStepSize *= 0.5f;
			}
			_RMSInfo(" %6.3f ", -fErrorDelta);
			++nLineIter;
		} while ( nLineIter < 25 && fErrorDelta > fErrDeltaThresh && nLineIterFails < 3 );
		_RMSInfo(" (%6.3f) \n", fCurError);
		fErrorDelta = fPrevError - fCurError;
		_RMSInfo("      Step %d - (%d iters):   prev err: %6.3f    new err: %6.3f    delta: %6.3f  stop: %6.3f   stepsize %6.3f\n", nIter, nLineIter, fPrevError, fCurError, -fErrorDelta, fErrThresh, fStepSize);


		_RMSTUNE_end(6);
		if ( _RMSTUNE_time(6) > m_fOptimizationTimeout ) {
			_RMSInfo("Timed out!\n");
			goto terminate_optimization;
		}
	}

terminate_optimization:
	for ( int k = 0; k < 10; ++k ) 
		LaplacianSmooth(m_vOptimizedBoundaryAngles, 0.2f, true);

}











void COILSBoundaryDeformer::RenderBaseMesh(bool bAddWireframe)
{
	if ( m_eEncodeMode == TwoPass ) {
		glColor3f(0.6f, 1.0f, 0.6f);
		rms::VFMeshRenderer renderer(&m_baseMesh);
		renderer.SetNormalMode( VFMeshRenderer::FaceNormals );

		if ( bAddWireframe ) {
			glEnable(GL_POLYGON_OFFSET_FILL);
			glPolygonOffset(1.0f,1.0f);
		}
		renderer.Render();
		if ( bAddWireframe ) {
			glDisable(GL_POLYGON_OFFSET_FILL);
		}

		if ( bAddWireframe ) {
			glColor3f(0.0f, 0.0f, 0.0f);
			glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT | GL_LINE_BIT);
			glDisable(GL_LIGHTING);
			glLineWidth(3.0f);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			renderer.Render();
			glPopAttrib();
		}		
	}
}


void COILSBoundaryDeformer::DebugRender(int nVertex)
{
	glPushAttrib( GL_ENABLE_BIT | GL_POINT_BIT );
	glDisable(GL_LIGHTING);

	if ( nVertex >= (int)m_vGlobalOrdering.size() )
		nVertex = (int)m_vGlobalOrdering.size()-1;

	// show parent set
	if ( nVertex >= 0 && m_vLayers.size() > 0 && nVertex < (int)m_vGlobalOrdering.size() ) {
		IMesh::VertexID vID = m_vGlobalOrdering[nVertex];
		_RMSInfo("Showing parent set for vert %d (%d in order)\n", vID, nVertex);
		EncodingLayer & layer = m_vLayers[ m_vLayerMap[vID] ];

		Wml::Vector3f vVertex;
		VertEncoding & enc = layer.vEncoding[vID];
	}

	glPopAttrib();
}







bool COILSBoundaryDeformer::IsUpwind(IMesh::VertexID vID, IMesh::VertexID vTestID)
{
	int nLayer1 = m_vLayerMap[vID];
	int nLayer2 = m_vLayerMap[vTestID];
	if ( nLayer1 < nLayer2 )
		return true;
	else if ( nLayer2 < nLayer1 )
		return false;
	else {
		EncodingLayer & layer = m_vLayers[nLayer1];
		return layer.vEncoding[vID].nOrder < layer.vEncoding[vTestID].nOrder;
	}
}






float COILSBoundaryDeformer::GetDistance( unsigned int i1, unsigned int i2 )
{
	Wml::Vector3f v1,v2;
	m_pMesh->GetVertex(i1, v1);
	m_pMesh->GetVertex(i2, v2);
	return ( v1-v2 ).Length();
}











