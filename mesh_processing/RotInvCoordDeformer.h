// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include "IDeformer.h"
#include <VFTriangleMesh.h>
#include <Frame.h>


// predecl to avoid include
namespace gsi {
	class SparseLinearSystem;
	class Solver_TAUCS;
	class SparseMatrix;
	class Vector;
};


namespace rms {

class RotInvCoordDeformer : public IMeshDeformer
{
public:
	RotInvCoordDeformer();
	~RotInvCoordDeformer();

	virtual void SetMesh(rms::VFTriangleMesh * pMesh);
	
	virtual void AddBoundaryConstraints(float fWeight = 1.0f);

	virtual void ClearConstraints() { m_vPosConstraints.resize(0); m_vRotConstraints.resize(0); }

	virtual void UpdatePositionConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition, float fWeight );
	virtual void UpdateOrientationConstraint( IMesh::VertexID vID, const rms::Frame3f & vFrame, float fWeight );

	virtual rms::Frame3f GetCurrentFrame( IMesh::VertexID vID );
	rms::Frame3f GetSolvedFrame( IMesh::VertexID vID );

	void SetVertexWeight( IMesh::VertexID vID, float fWeight );
	void SetVertexScale( IMesh::VertexID vID, float fScale );

	virtual void Solve();

	float GetLaplacianError();

	virtual void DebugRender();

protected:
	rms::VFTriangleMesh * m_pMesh;

	struct VtxInfo {
		IMesh::VertexID vID;
		std::vector<IMesh::VertexID> vNbrs;
		std::vector<float> vNbrWeights;
		float fVertexArea;
		float fVertexWeight;
		float fVertexScale;

		int nTangentNbr;					// nbr edge that is used to compute x axis of tangent frame
		rms::Frame3f vFrame;				// original tangent frame
		Wml::Vector3f vFrameLaplacian;		// laplacian vector encoded in original frame

		rms::Frame3f vTransFrame;			// current frame (solved for)
		Wml::Vector3f vLaplacian;			// current laplacian


		VtxInfo() { }
	};
	std::vector<VtxInfo> m_vVertices;
	unsigned int m_nEdges;
	float m_fAvgVtxArea;
	void ComputeWeights();


	struct RotConstraint {
		IMesh::VertexID vID;
		rms::Frame3f vFrame;
		float fWeight;
	};
	std::vector<RotConstraint> m_vRotConstraints;


	struct PosConstraint {
		IMesh::VertexID vID;
		Wml::Vector3f vPosition;
		float fWeight;
	};
	std::vector<PosConstraint> m_vPosConstraints;


	// system and solver for orientation (frames)
	gsi::SparseLinearSystem * m_pSystemRot;
	gsi::SparseLinearSystem * GetSystemRot();
	gsi::Solver_TAUCS * m_pSolverRot;
	gsi::Solver_TAUCS * GetSolverRot();


	// system and solver for positions (laplacian)
	gsi::SparseLinearSystem * m_pSystemPos;
	gsi::SparseLinearSystem * GetSystemPos();
	gsi::Solver_TAUCS * m_pSolverPos;
	gsi::Solver_TAUCS * GetSolverPos();

	// laplacian matrix for positions
	gsi::SparseMatrix * m_pLs;
	gsi::SparseMatrix * m_pM;
	gsi::Vector       * m_pRHSPos;

	bool m_bMatricesValid;
	void UpdateMatrices();

	void UpdateRHSRot();
	void UpdateRHSPos();
};



}   // end namespace rms