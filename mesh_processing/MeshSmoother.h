// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <VFTriangleMesh.h>
#include <IMeshBVTree.h>
#include <Frame.h>
#include <MeshSelection.h>

namespace rms {

class MeshSmoother
{
public:
	MeshSmoother(void);
	~MeshSmoother(void);

	void SetSurface(rms::VFTriangleMesh * pMesh);
	void SetMask(rms::MeshSelection & selection);

	enum WeightType {
		WeightsUniform,
		WeightsCotangent
	};
	WeightType GetWeightType() const { return m_eWeightType; }
	void SetWeightType( WeightType eType ) { m_eWeightType = eType; }		

	// this applies more aggressive smoothing to vertices w/ a longer laplacian
	// vector. It gives rough approximation of curvature flow (ie protrusions
	// shrink down before the underlying shape changes siginificantly...
	void DoAdaptiveLaplacianSmooth(int nPasses, float fMaxLambda = 1.0f);

	void DoLaplacianSmooth(int nPasses, float fLambda = 1.0f);

	// making K smaller produces shrinkage, larger produces growth   (same w/ lambda)
	void DoTaubinSmooth(int nPasses, float fKpb = 0.1f, float fLambda = 0.6307);


protected:
	rms::VFTriangleMesh * m_pMesh;

	WeightType m_eWeightType;

	std::set<rms::IMesh::TriangleID> m_vMaskTris;
	std::set<rms::IMesh::VertexID> m_vMaskVerts;

	Wml::AxisAlignedBox3f m_bounds;
	float m_fAvgEdgeLength;

	struct Neighbour {
		rms::IMesh::VertexID vID;
		Wml::Vector3f vDelta;
		float fWeight;
	};
	struct Vertex {
		rms::IMesh::VertexID vID;
		bool bIsBoundary;

		Wml::Vector3f vVertex;
		float fLaplacianLenSqr;
		std::vector<IMesh::VertexID> vNbrs;
		std::vector<float> vWeights;
		std::vector<Wml::Vector3f> vDeltas;
		bool operator<( const Vertex & v2 ) const
			{ return vID < v2.vID; }
	};

	std::vector<Vertex> m_vVerts;
	float m_fMaxLaplacianLenSqr;
	void Initialize();

	void UpdateWeights();	

};


} // end namespace rms