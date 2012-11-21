// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"

#include <DistanceCache.h>
#include <NeighbourCache.h>
#include <DijkstraFrontProp.h>
#include <MeshUtils.h>
#include <MatrixBlender.h>
#include <Wm4Triangle3.h>


namespace rms {


class  COILSBoundaryDeformer : public DistanceCache::DistanceCalculator
{
public:

	typedef std::set<IMesh::VertexID> VertSet;
	typedef std::set<IMesh::TriangleID> TriSet;

	enum EncodeMode {
		AllBoundary,
		UpwindThreshold,
		TwoPass
	};

	struct Parent {
		IMesh::VertexID vID;

		Wml::Vector3f vOffsetVector;
		Wml::Matrix3f vOffsetFrame;

		float fDistance;
		float fRelativeDensity;

		float fWeight;

		bool operator< (const Parent & p2 ) const {
			if ( fDistance == p2.fDistance )
				return vID < p2.vID;
			else
				return fDistance < p2.fDistance;
		}
	};



	struct VertEncoding {
		VertEncoding() { vID = IMesh::InvalidID; }
		IMesh::VertexID vID;
		rms::Frame3f vFrame;
		bool bWeightsValid;

		unsigned int nOrder;

		float fBoundaryGeoDist;
		bool bIsBoundary;

		std::vector<Parent> vParents;
		float fMaxWeight;
		float fMinParentDist;
	};


	struct BoundaryVert {
		BoundaryVert() { vID = IMesh::InvalidID; }
		BoundaryVert( IMesh::VertexID id ) { vID = id; }

		IMesh::VertexID vID;
		IMesh::VertexID vNbrIDs[2];

		rms::Frame3f vInitial;
		rms::Frame3f vSet;

		float fNbrEdgeLength;

		std::vector<Wml::Triangle3f> vExteriorTris;

		bool operator<(const BoundaryVert & v2) const {
			return vID < v2.vID;
		}
	};


	class EncodingLayer
	{
	public:
		EncodingLayer() {  }

		/* set in ::InitializeLayers() */

		COILSBoundaryDeformer::EncodeMode eEncodeMode;

		//! vertices in layer
		VertSet vVertices;		
		std::vector<bool> vInLayer;						// same size as mesh.GetMaxVertexID()


		//! boundary information
		VertSet vBoundaryVertIDs;
		std::set<BoundaryVert> vBoundaryVerts;
		std::vector<IMesh::VertexID> vBoundaryLoop;


		/* set in ::OrderVertices() */
		std::vector<IMesh::VertexID> vInteriorOrder;
		unsigned int nLastBoundaryNbrIndex;		// index of last vert in vInteriorOrder that is connected to a boundary vertex

		//! distances are indexed by VertexID
		std::vector<float> vBoundaryDistances;
		float fMaxBoundaryDist;


		/* set in ::Encode() */

		std::vector<BoundaryVert *> vBoundaryVertsVec;

		//! ordered by VertexID
		std::vector<VertEncoding> vEncoding;
	};



	COILSBoundaryDeformer();
	~COILSBoundaryDeformer(void);



	void Reset();

	/** Boundary normals of pMesh must be set to direction of part base surface. 
	    If mesh has more than one boundary loop, use SetUseBoundaryLoop() to indicate which
	 **/ 
	void SetMesh( VFTriangleMesh * pMesh );

	void RestoreMesh();
	virtual VFTriangleMesh * GetMesh() { return m_pMesh; }

	void SetUseBoundaryLoop( int nLoop ) { m_nUseBoundaryLoop = nLoop; }

	void SetEncodeMode( EncodeMode eMode );
	EncodeMode GetEncodeMode() { return m_eEncodeMode; }

	virtual const VertSet & GetBoundaryVerts() { return m_vLayers[0].vBoundaryVertIDs; }
	virtual const rms::Frame3f & GetInitialBoundaryFrame(IMesh::VertexID vID);
	virtual const rms::Frame3f & GetCurrentBoundaryFrame(IMesh::VertexID vID);
	virtual void UpdateBoundary( IMesh::VertexID vID, const Wml::Vector3f & vNewOrigin,  const Wml::Vector3f & vNewNormal, std::vector<Wml::Triangle3f> * pExteriorTris = NULL );
	virtual const IMesh::VertexID GetBoundaryVertID( IMesh::VertexID vExternalID );

	virtual void SetBoundaryRotateAngle( float fAngle ) { m_fBoundaryRotateAngle = fAngle; }
	virtual float GetBoundaryRotateAngle() { return m_fBoundaryRotateAngle; }

	void SetUseUpwindGeodesicDistances( bool bEnable ) { m_bUseUpwindGeodesicDistances = bEnable; }
	bool GetUseUpwindGeodesicDistances() { return m_bUseUpwindGeodesicDistances; }


	void SetDistanceWeightPower( float fPower ) { m_fDistanceWeightPower = fPower; InvalidateAllWeights(); }
	float GetDistanceWeightPower() { return m_fDistanceWeightPower; }

	void SetEnableUpwindCorrection( bool bEnable ) { m_bEnableUpwindCorrection = bEnable; InvalidateAllWeights(); }
	bool GetEnableUpwindCorrection() { return m_bEnableUpwindCorrection; }

	void SetUpwindThresholdScale( float fScale ) { m_fUpwindThresholdScale = fScale; InvalidateAllWeights(); }
	float GetUpwindThresholdScale() { return m_fUpwindThresholdScale; }

	void SetEnableDensityCorrection( bool bEnable ) { m_bEnableDensityCorrection = bEnable; InvalidateAllWeights(); }
	bool GetEnableDensityCorrection() { return m_bEnableDensityCorrection; }


	void SetGlobalScale( float fScale ) { m_fGlobalScale = fScale; }
	float GetGlobalScale() { return m_fGlobalScale; }


	void SetUseOptimalBoundaryFrames( bool bEnable ) { m_bUseOptimalBoundaryFrames = bEnable; }
	bool GetUseOptimalBoundaryFrames() { return m_bUseOptimalBoundaryFrames; }

	virtual bool Encode();

	virtual void Reconstruct();

	void OptimizeBoundaryFrames(bool bPerVertexOptimization = true, float fTimeout = 0.25);

	VFTriangleMesh & BaseMesh() { return m_baseMesh; }

	void DebugRender(int nVertex = -1);
	void RenderBaseMesh(bool bAddWireframe);


protected:
	VFTriangleMesh * m_pMesh;
	Wml::AxisAlignedBox3f m_bounds;
	float m_fMinEdgeLength;
	float m_fMaxEdgeLength;
	int m_nUseBoundaryLoop;

	EncodeMode m_eEncodeMode;
	bool m_bUseUpwindGeodesicDistances;

	float m_fDistanceWeightPower;

	float m_fUpwindThreshold;					// multiplied by max-edge-length to determine upwind falloff range
	float m_fUpwindThresholdScale;				// scale factor applied to UpwindThreshold when computing threshold weights

	float m_fBoundaryRotateAngle;

	bool m_bOptimizeBoundaryFrames;
	bool m_bUseOptimalBoundaryFrames;
	bool m_bDoPerVertexOptimization;
	float m_fOptimizedBoundaryAngle;		    // automatically-computed optimized rotation angle 
	float m_fOptimizationTimeout;
	std::vector<float> m_vOptimizedBoundaryAngles;
	struct BoundaryOptCache {
		std::vector<Wml::Vector3f> vSecants;
		std::vector<Wml::Vector3f> vCurNormals;
	};
	BoundaryOptCache m_boundaryOptCache;
	void InitializeBoundaryOptimization(EncodingLayer & layer);
	float BoundaryNormalError(EncodingLayer & layer, IMesh::VertexID vModified = IMesh::InvalidID);
	void OptimizeBoundaryFrames_Gradient(EncodingLayer & layer, bool bDoPerVertex);

	float m_fGlobalScale;

	bool m_bDeferredWeightInvalidationPending;

	struct VtxState {
		Wml::Vector3f vPosition;
		Wml::Vector3f vNormal;
	};
	std::vector<VtxState> m_vOrigMesh;


	std::vector<EncodingLayer> m_vLayers;

	std::vector<IMesh::VertexID> m_vGlobalOrdering;
	std::map<IMesh::VertexID, int> m_vLayerMap;
	std::set<IMesh::VertexID> m_vGlobalUpwindSet;


	bool IsUpwind(IMesh::VertexID vID, IMesh::VertexID vTestID);


	std::vector<float> m_vOneRingAreas;
	std::vector<float> m_vAvgNbrDists;
	void ComputeOneRingAreas();			// areas of one-rings of each vertex

	void InitializeLayers_Basic();

	void OrderVertices( EncodingLayer & layer );

	void ComputeRelativeDensityWeights( EncodingLayer & layer, IMesh::VertexID vID, std::vector<Parent> & vParents );

	bool Encode_Segments();
	bool Encode_Multipass();
	bool Encode_Layer(EncodingLayer & layer);

	//! remove any members of vFilter which are also in vKeep
	void RemoveDuplicates( VertSet & vKeep, VertSet & vFilter );
	float EncodeRelative( EncodingLayer & layer, IMesh::VertexID vID, const VertSet & vParentVIDs, std::set<Parent> & vParents );

	void ValidateWeights( EncodingLayer & layer, IMesh::VertexID vID, bool bIsBaseLayer = false );
	void InvalidateAllWeights(bool bDefer = true);

	void Decode(bool bBoundaryNbrsOnly = false);
	void Decode_Detail();

	void InitializeBoundary(EncodingLayer & layer);
	void ReconstructVertFrame(EncodingLayer & layer, IMesh::VertexID vID, rms::Frame3f & vFrame );


	// need to access inner classes
	friend class MakeParentListCallback;

	bool m_bEnableUpwindCorrection;
	bool m_bEnableDensityCorrection;
	DistanceCache m_meshDistCache;
	virtual float GetDistance( unsigned int i1, unsigned int i2 );


	rms::MeshPositionSource m_meshPointSource;

	rms::MeshOneRingSource m_oneringFinder;
	NeighbourCache<IMesh::VertexID> m_oneringCache;

	rms::MeshGeoNbrSource m_geonbrFinder;
	NeighbourCache<IMesh::VertexID> m_geonbrCache;

	NeighbourCache<IMesh::VertexID> * m_pNeighbourCache;	//! points to either m_oneringCache   or   m_geonbrCache

	//! just use one, to avoid repeated memory allocation. Make sure you call Reset() before using this
	rms::DijkstraFrontProp<IMesh::VertexID> m_dijkstra;

	rms::MatrixBlender m_combiner;

	//! multiresolution encoding support
	rms::VFTriangleMesh * m_pOriginalMesh;
	std::vector<VtxState> m_vOrigOrigMesh;
	rms::VFTriangleMesh m_baseMesh;
	rms::IMeshBVTree m_baseMeshBVTree;
	EncodingLayer m_detailLayer;
	std::map<IMesh::VertexID, IMesh::VertexID> m_vBoundaryLoopBaseMap;

	void GenerateBaseMesh();
	void InitializeLayers_DetailMesh();

	void FindBaseParents( IMesh::VertexID vDetailID, VertSet & parents, VertSet * pKnown = NULL );

};





}  // end namespace

