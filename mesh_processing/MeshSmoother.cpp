// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "MeshSmoother.h"

#include <limits>
#include <MeshUtils.h>

using namespace rms;


MeshSmoother::MeshSmoother(void)
{
	m_pMesh = NULL;
	m_eWeightType = WeightsUniform;
}

MeshSmoother::~MeshSmoother(void)
{
}

void MeshSmoother::SetSurface(rms::VFTriangleMesh * pMesh)
{
	m_pMesh = pMesh;

	m_pMesh->GetBoundingBox(m_bounds);
	float fMin = 0, fMax = 0;
	m_pMesh->GetEdgeLengthStats(fMin, fMax, m_fAvgEdgeLength);

	Initialize();
}

void MeshSmoother::SetMask(rms::MeshSelection & selection)
{
	//! make sure verts are selected for faces
	selection.SelectFaceVertices();

	m_vMaskVerts = selection.Vertices();
	m_vMaskTris = selection.Triangles();

	Initialize();
}



void MeshSmoother::DoAdaptiveLaplacianSmooth(int nPasses, float fMaxLambda)
{
	for ( int pi = 0; pi < nPasses; ++pi ) {

		size_t nCount = m_vVerts.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			Vertex & v = m_vVerts[i];
			if ( v.bIsBoundary )
				continue;

			Wml::Vector3f vDelta = Wml::Vector3f::ZERO;

			size_t nNbrs = v.vNbrs.size();
			for ( unsigned int k = 0; k < nNbrs; ++k )
				vDelta += v.vWeights[k] * v.vDeltas[k];

			float fLenFactor = v.fLaplacianLenSqr / m_fMaxLaplacianLenSqr;
			float fLambda = fMaxLambda * fLenFactor;

			v.vVertex += fLambda * vDelta;

			m_pMesh->SetVertex( v.vID, v.vVertex );
		}
	}
}



void MeshSmoother::DoLaplacianSmooth(int nPasses, float fLambda)
{
	for ( int pi = 0; pi < nPasses; ++pi ) {

		UpdateWeights();

		size_t nCount = m_vVerts.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			Vertex & v = m_vVerts[i];
			if ( v.bIsBoundary )
				continue;

			Wml::Vector3f vDelta = Wml::Vector3f::ZERO;

			size_t nNbrs = v.vNbrs.size();
			for ( unsigned int k = 0; k < nNbrs; ++k )
				vDelta += v.vWeights[k] * v.vDeltas[k];

			v.vVertex += fLambda * vDelta;

			m_pMesh->SetVertex( v.vID, v.vVertex );
		}
	}
}



void MeshSmoother::DoTaubinSmooth(int nPasses, float fKpb, float fLambda)
{
	float fMu = fLambda / (fLambda*fKpb - 1);
		
	for ( int pi = 0; pi < nPasses; ++pi ) {

		UpdateWeights();

		size_t nCount = m_vVerts.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			Vertex & v = m_vVerts[i];
			if ( v.bIsBoundary )
				continue;

			Wml::Vector3f vDelta = Wml::Vector3f::ZERO;

			size_t nNbrs = v.vNbrs.size();
			for ( unsigned int k = 0; k < nNbrs; ++k )
				vDelta += v.vWeights[k] * v.vDeltas[k];

			v.vVertex += fLambda * vDelta;

			m_pMesh->SetVertex( v.vID, v.vVertex );
		}

		UpdateWeights();

		for ( unsigned int i = 0; i < nCount; ++i ) {
			Vertex & v = m_vVerts[i];
			if ( v.bIsBoundary )
				continue;

			Wml::Vector3f vDelta = Wml::Vector3f::ZERO;

			size_t nNbrs = v.vNbrs.size();
			for ( unsigned int k = 0; k < nNbrs; ++k )
				vDelta += v.vWeights[k] * v.vDeltas[k];

			v.vVertex += fMu * vDelta;

			m_pMesh->SetVertex( v.vID, v.vVertex );
		}

	}
}





void MeshSmoother::Initialize()
{
	m_vVerts.clear();

	VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
	while ( curv != endv ) {
		Vertex v;
		v.vID = *curv++;

		if ( ! m_vMaskVerts.empty() && m_vMaskVerts.find(v.vID) == m_vMaskVerts.end() )
			continue;

		if ( m_vMaskVerts.empty() ) {
			v.bIsBoundary = m_pMesh->IsBoundaryVertex(v.vID);
		} else {
			VFTriangleMesh::VtxNbrItr itr(v.vID);
			m_pMesh->BeginVtxTriangles(itr);
			if ( m_pMesh->IsBoundaryVertex(v.vID) ) {
				v.bIsBoundary = true;
			} else {
				v.bIsBoundary = false;
				IMesh::TriangleID tID = m_pMesh->GetNextVtxTriangle(itr);
				while ( tID != IMesh::InvalidID ) {
					v.bIsBoundary = v.bIsBoundary && (m_vMaskTris.find(tID) != m_vMaskTris.end());
					tID = m_pMesh->GetNextVtxTriangle(itr);
				}
			}
		}

		m_vVerts.push_back(v);
	}

	UpdateWeights();
}


void MeshSmoother::UpdateWeights()
{
	srand((unsigned int)time(NULL));

	m_fMaxLaplacianLenSqr = 0.0f;

	size_t nCount = m_vVerts.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		Vertex & v = m_vVerts[i];

		// update vert position
		m_pMesh->GetVertex( v.vID, v.vVertex );

		if ( v.bIsBoundary )
			continue;

		if ( v.vNbrs.size() == 0 )
			m_pMesh->VertexOneRing(v.vID, v.vNbrs);

		switch (m_eWeightType) {
			case WeightsCotangent:
				MeshUtils::CotangentWeights(*m_pMesh, v.vID, v.vNbrs, v.vWeights, true);
				break;
			case WeightsUniform:
			default:
				MeshUtils::UniformWeights(*m_pMesh, v.vID, v.vNbrs, v.vWeights, true);
				break;
		}

		size_t nNbrs = v.vNbrs.size();
		v.vDeltas.resize( nNbrs );
		Wml::Vector3f vCentroid = Wml::Vector3f::ZERO;
		for ( unsigned int k = 0; k < nNbrs; ++k ) {
			v.vDeltas[k] = Wml::Vector3f(m_pMesh->GetVertex(v.vNbrs[k]));
			vCentroid += v.vWeights[k] * v.vDeltas[k];
			v.vDeltas[k] -= v.vVertex;
		}

		v.fLaplacianLenSqr = (v.vVertex - vCentroid).SquaredLength();
		if ( v.fLaplacianLenSqr > m_fMaxLaplacianLenSqr )
			m_fMaxLaplacianLenSqr = v.fLaplacianLenSqr;
	}

}