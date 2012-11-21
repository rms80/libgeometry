// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "VertexWeights.h"
#include "VectorUtil.h"

#include "rmsdebug.h"

using namespace rms;




void VertexWeights::Uniform( VFTriangleMesh & mesh, IMesh::VertexID vID, 
								  std::vector<IMesh::VertexID> & vNeighbourhood, 
								  std::vector<float> & vWeights, bool bNormalize  )
{
	size_t nNbrs = vNeighbourhood.size();
	vWeights.resize(nNbrs);
	for ( unsigned int k = 0; k < nNbrs; ++k )
		vWeights[k] = 1.0f;
	
	if ( bNormalize ) {
		float fWeightSum = (float)nNbrs;
		for ( unsigned int k = 0; k < nNbrs; ++k )
			vWeights[k] /= fWeightSum;
	}
}



void VertexWeights::InverseDistance( VFTriangleMesh & mesh, IMesh::VertexID vID, 
								     std::vector<IMesh::VertexID> & vNeighbourhood, 
								     std::vector<float> & vWeights, float fPow, float fEps, bool bNormalize  )
{
	Wml::Vector3f vVtx, vNbr;
	mesh.GetVertex(vID, vVtx);

	float fWeightSum = 0.0f;
	size_t nNbrs = vNeighbourhood.size();
	vWeights.resize(nNbrs);
	for ( unsigned int k = 0; k < nNbrs; ++k ) {
		mesh.GetVertex( vNeighbourhood[k], vNbr );
		float fDist = (vNbr - vVtx).Length();
		vWeights[k] = 1.0f / (fEps + pow(fDist,fPow));
		fWeightSum += vWeights[k];
	}
	
	if ( bNormalize ) {
		float fWeightSum = (float)nNbrs;
		for ( unsigned int k = 0; k < nNbrs; ++k )
			vWeights[k] /= fWeightSum;
	}
}



//! assumes one-ring is ordered
void VertexWeights::Cotangent( VFTriangleMesh & mesh, IMesh::VertexID vID, 
								  std::vector<IMesh::VertexID> & vOneRing, 
								  std::vector<float> & vWeights, bool bNormalize  )
{
	Wml::Vector3f vVtx, vOpp, vPrev, vNext;

	size_t nNbrs = vOneRing.size();
	mesh.GetVertex(vID, vVtx);
	mesh.GetVertex(vOneRing[0], vOpp);
	mesh.GetVertex(vOneRing[nNbrs-1], vPrev);

	vWeights.resize(nNbrs);
	float fWeightSum = 0.0f;
	for ( unsigned int k = 0; k < nNbrs; ++k ) {
		IMesh::VertexID nNext = vOneRing[(k+1)%nNbrs];
		mesh.GetVertex(nNext, vNext);

		Wml::Vector3f a1(vVtx - vPrev);   a1.Normalize();
		Wml::Vector3f a2(vOpp - vPrev);   a2.Normalize();
		float fADot = a1.Dot(a2);
		double fAlpha = acos( rms::Clamp(fADot, -1.0f, 1.0f) );

		Wml::Vector3f b1(vVtx - vNext);   b1.Normalize();
		Wml::Vector3f b2(vOpp - vNext);   b2.Normalize();
		float fBDot = b1.Dot(b2);
		double fBeta = acos( rms::Clamp(fBDot, -1.0f, 1.0f) );

		vWeights[k] = (float)( 1/tan(fAlpha) + 1/tan(fBeta) );
		if ( ! _finite(vWeights[k]) )
			_RMSInfo("MeshUtils::CotangentWeights():: non-finite weight at vertex %d [%d/%d]  alpha d/a: %f/%f   beta d/a: %f/%f\n", vOneRing[k], k, nNbrs, fADot,fAlpha,  fBDot, fBeta);
		fWeightSum += vWeights[k];

		vPrev = vOpp;
		vOpp = vNext;
	}

	if ( bNormalize ) {
		for ( unsigned int k = 0; k < nNbrs; ++k )
			vWeights[k] /= fWeightSum;
	}
}

