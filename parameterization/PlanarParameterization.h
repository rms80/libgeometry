// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <VFTriangleMesh.h>
#include <ExpMapGenerator.h>

// predecl to avoid include
namespace rmssolver {
	class SparseSymmetricEigenSolver;
	class SparseSymmetricMatrixSolver;
};

namespace rms {


class PlanarParameterization
{
public:
	PlanarParameterization(void);
	~PlanarParameterization(void);

	enum EmbeddingType {
		UniformWeights,
		InverseDistance,
		FloaterShapePreserving,
		GeodesicOneRing,
		Optimal3DOneRing,
		Optimal2DOneRing,
		Optimal2DExpMap,
		Optimal3DExpMap,
		ExpMapLLE,
		StandardLLE,
		DiscreteConformal,
		DiscreteAuthalic,
		MixedConformalAuthalic,
		DiscreteNaturalConformal
	};
	EmbeddingType GetEmbeddingType( ) { return m_eEmbedType; }
	void SetEmbeddingType( EmbeddingType eType );

	void SetMesh( rms::VFTriangleMesh * pMesh, rms::ExpMapGenerator * pExpMap );
	void Invalidate();

	enum BoundaryMapType {
		ProjectBoundary,
		CircleBoundary,
		SquareBoundary,
		NaturalConformalBoundary,
		LLEBoundary,
		BoundaryLLEBoundary
	};
	void SetBoundaryMapType( BoundaryMapType eType ) { m_eBoundaryMap = eType; }
	BoundaryMapType GetBoundaryMapType() { return m_eBoundaryMap; }

	bool Compute();

	bool Compute_LLE_Boundary();

	void SetUseFixedGeodesicNbrhood(bool bUseFixed);
	bool GetUseFixedGeodesicNbrhood() { return m_bUseFixedGeoNbrhoodSize; }

	int GetGeodesicNbrhoodSize() { return m_nGeoNbrhoodSize; }
	void SetGeodesicNbrhoodSize( int nNbrs );
	
	float SuggestGeodesicNbrhoodRadiusSize();
	float GetGeodesicNbrhoodRadiusSize() { return m_fGeoNbrDistance; }
	void SetGeodesicNbrhoodRadiusSize(float fRadius);

	float GetMixedDCDAConformalWeight() { return m_fMixedDCDCConformalWeight; }
	void SetMixedDCDAConformalWeight(float fVal);

	void SetScaleUVs( bool bEnable, float fScaleFactor = 0 ) { m_bScaleUVs = bEnable;  if(fScaleFactor != 0) m_fUVScaleFactor = fScaleFactor; }

protected:
	rms::VFTriangleMesh * m_pMesh;
	rms::ExpMapGenerator * m_pExpMap;

	bool m_bScaleUVs;
	float m_fUVScaleFactor;

	EmbeddingType m_eEmbedType;
	BoundaryMapType m_eBoundaryMap;

	bool m_bComputeNeighbourExpMaps;
	void PrecomputeMeshData();

	bool m_bUseFixedGeoNbrhoodSize;
	unsigned int m_nGeoNbrhoodSize;
	unsigned int m_nCurMaxGeoNbrs;

	float m_fGeoNbrDistance;
	float m_fCurMinMaxGeoNbrDistance;

	bool m_bGeodesicNbrhoodsValid;
	void ValidateGeodesicNbrhoods();
	void InvalidateGeodesicNbrhoods() { m_bGeodesicNbrhoodsValid = false; }

	//! conformal weight for mixed DiscreteConformal/DiscreteAuthalic (other weight is 1-this)
	float m_fMixedDCDCConformalWeight;

	struct VertexAngles {
		float fDistSquared;		// this is a dupe of fDistances3D vector I think...maybe can remove...
		float fAlpha;		float fBeta;
		float fGamma;		float fDelta;
	};

	struct TriangleAngles {
		unsigned int nJ;
		unsigned int nK;
		float fAlpha;		// goes with J
		float fBeta;		// goes with K
	};

	class NeighbourSet {
	public:
		rms::IMesh::VertexID vID;
		std::vector< rms::IMesh::VertexID > vNbrs;

		// number of neighbours to consider in algorithms (sometimes less than actual count...)
		int nUseNbrs;

		std::vector< float > fDistances3D;
		std::vector< float > fWeights;

		// used for local expmaps
		std::vector< Wml::Vector2f > vLocalUVs;

		// these values are used for shape-preserving weights
		std::vector< Wml::Vector2f > vFlattened;
		std::vector<unsigned int> vIntersectEdge;
		std::vector<Wml::Vector2f> vIntersect2D;
		std::vector<Wml::Vector3f> vIntersect3D;

		// used for discrete conformal/authalic maps
		std::vector<VertexAngles> vAngles;
		std::vector<TriangleAngles> vTriAngles;

		virtual void Clear() { 
			vNbrs.resize(0); fDistances3D.resize(0); fWeights.resize(0); 
			vLocalUVs.resize(0); 
			vFlattened.resize(0); vIntersectEdge.resize(0); vIntersect2D.resize(0); vIntersect3D.resize(0); 
			vAngles.resize(0); vTriAngles.resize(0);
		}
	};

	void MakeOneRingNeighbourSet( rms::IMesh::VertexID vID, NeighbourSet & nbrs );
	void MakeGeoDistNeighbourSet( rms::IMesh::VertexID vID, NeighbourSet & nbrs );

	enum NeighbourhoodType {
		OneRing,
		ExpMap
	};

	struct MeshVertex {
		rms::IMesh::VertexID vID;
		NeighbourSet m_OneRingNbrs;
		NeighbourSet m_ExpMapNbrs;

		NeighbourSet & GetNeighbourSet( NeighbourhoodType eType ) {
			if ( eType == ExpMap ) return m_ExpMapNbrs;
			return m_OneRingNbrs;
		}
	};
	std::vector<MeshVertex> m_vVertInfo;
	std::map<rms::IMesh::VertexID, unsigned int> m_vVertMap;


	struct Edge {
		rms::IMesh::VertexID v1;
		rms::IMesh::VertexID v2;
		bool operator<( const Edge & e2 ) const {
			if ( v1 < e2.v1 )						return true;
			else if ( v1 == e2.v1 && v2 < e2.v2)	return true;
			else									return false;	}
	};


	struct BoundaryLoop {
		std::vector<unsigned int> vVerts;
		std::vector<Wml::Vector2f> vUVs;
		std::map< int, std::vector<Wml::Vector2f> > vUVCache;
		void Cache(int id) { vUVCache[id] = vUVs; }
		bool IsCached(int id) { return vUVCache.find(id) != vUVCache.end(); }
		void UseCache(int id) { vUVs = vUVCache[id]; }
	};
	struct BoundaryInfo {
		std::vector<BoundaryLoop> vBoundaryLoops;
		void Clear() {  vBoundaryLoops.resize(0); }
		void Cache(int id) { for ( int i = 0; i < (int)vBoundaryLoops.size(); ++i ) vBoundaryLoops[i].Cache(id); }
		bool IsCached(int id) { return vBoundaryLoops[0].IsCached(id); }
		void UseCache(int id) { for ( int i = 0; i < (int)vBoundaryLoops.size(); ++i ) vBoundaryLoops[i].UseCache(id); }
	};
	BoundaryInfo m_boundaryInfo;
	void ComputeBoundaryInfo(BoundaryInfo & info);

	void ProjectBoundaryLoop( BoundaryLoop & loop );
	void MapBoundaryLoopToCircle( BoundaryLoop & loop );
	void MapBoundaryLoopToUnitSquare( BoundaryLoop & loop );
	void InitializeBoundaries_Conformal( std::vector<BoundaryLoop> & vBoundaryLoops );
	void InitializeBoundaries_LLE( std::vector<BoundaryLoop> & vBoundaryLoops );
	void InitializeBoundary_BoundaryLLE( BoundaryLoop & loop );



	void ComputeWeights_Uniform( NeighbourSet & nbrs );
	void ComputeWeights_InvDist( NeighbourSet & nbrs );
	void ComputeWeights_ShapePreserving( NeighbourSet & nbrs );
	void ComputeWeights_Geodesic( NeighbourSet & nbrs );
	void ComputeWeights_Optimal3D( NeighbourSet & nbrs );

	// TODO: optionally symmetrize weights after computation (??)
	void ComputeWeights_Optimal2D( NeighbourSet & nbrs );

	float GetReconstructionError_3D( NeighbourSet & nbrs );
	float GetReconstructionError_2D( NeighbourSet & nbrs );


	struct Constraint {
		int nVertex;
		Wml::Vector2f vConstraint;
		Constraint(int nvert, const Wml::Vector2f & v ) { nVertex = nvert; vConstraint = v; }
	};
	//! for now, just pins two vertices to fix rotate/translate in DAP/DCP
	void MakeBoundaryConstraints( std::vector<Constraint> & vConstraints );

	// [TODO] this can go away as soon as we fix optimal weight computations...
	struct ParamSystem {
		std::vector<double> vMatrix;
		std::vector<double> vRHS;
		std::vector<double> vSolution;
		unsigned int nDim;
		void Resize(unsigned int dim) 
			{ nDim = dim; vMatrix.resize(nDim*nDim,0); vRHS.resize(nDim,0); }
		void SetMatrixLAPACK( int r, int c, double val ) 
			{ lgASSERT((unsigned int)c<nDim&&(unsigned int)r<nDim); vMatrix[ c*nDim + r ] = val; }
		void AddMatrixLAPACK( int r, int c, double val ) 
			{ lgASSERT((unsigned int)c<nDim&&(unsigned int)r<nDim); vMatrix[ c*nDim + r ] += val; }
		double GetMatrixLAPACK( int r, int c )
			{ lgASSERT((unsigned int)c<nDim&&(unsigned int)r<nDim); return vMatrix[ c*nDim + r ]; }
		void SetRHS( int r, double val )
			{ lgASSERT((unsigned int)r<nDim); vRHS[r] = val; }
	};


	bool Solve_FixBoundary( std::vector<double> & vU, std::vector<double> & vV, NeighbourhoodType eNbrType);

	// driver functions
	bool Parameterize_OneRing();
	bool Parameterize_OneRing_Intrinsic();
	bool Parameterize_ExpMap();
	bool Parameterize_LLE();

	bool EmbedBoundary();

	void SetMeshUVs(const double * pValues);
	void SetMeshUVs(const double * pValuesU, const double * pValuesV);

	// analysis functions
	void FindReconstructionError(NeighbourhoodType eNbrType);
	void ComputeMeshOneRingStretch();
};



}   // end namespace rms