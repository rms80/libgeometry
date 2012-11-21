// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include "IDeformer.h"
#include <VFTriangleMesh.h>
#include <Wm4GMatrix.h>


// predecl to avoid include
namespace gsi {
	class SparseLinearSystem;
	class Solver_TAUCS;
	class SparseMatrix;
};


namespace rms {

class LaplacianDeformer : public IMeshDeformer
{
public:
	LaplacianDeformer();

	virtual void SetMesh(rms::VFTriangleMesh * pMesh);
	
	virtual void AddBoundaryConstraints(float fWeight = 1.0f);

	virtual void ClearConstraints() { m_vConstraints.resize(0); }
	
	virtual void UpdatePositionConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition, float fWeight ) 
		{ UpdateConstraint(vID, vPosition, fWeight); }

	void UpdateConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition, float fWeight );

	virtual void UpdateOrientationConstraint( IMesh::VertexID vID, const rms::Frame3f & vFrame, float fWeight )
		{ }		// no orientation constraint support...

	
	virtual rms::Frame3f GetCurrentFrame( IMesh::VertexID vID );


	void SetTargetFrames(VFTriangleMesh * pMesh);
	void ZeroLaplacianVectors(bool bInteriorOnly = false);
	void SetLaplacianVector( IMesh::VertexID vID, const Wml::Vector3f & vLaplacian );

	virtual void Solve() { Solve(false); }
	void Solve(bool bUseTargetNormals);

	void Solve_Lipman04(int nIterations = 3);

	void PostProcess_SnapConstraints();

	virtual void DebugRender();

protected:
	rms::VFTriangleMesh * m_pMesh;

	struct VtxInfo {
		std::vector<IMesh::VertexID> vNbrs;
		std::vector<float> vNbrWeights;

		Wml::Vector3f vLaplacian;

		Wml::Vector3f vSaveLaplacian;

		int nTangentNbr;
		Wml::Vector3f vFrameLaplacian;

		bool bHasTargetNormal;
		Wml::Vector3f vTargetNormal, vTargetTangentDir;
		float fTargetScale;

		VtxInfo() { bHasTargetNormal = false; }
	};
	std::vector<VtxInfo> m_vVertices;
	void ComputeWeights();

	struct Constraint {
		IMesh::VertexID vID;
		Wml::Vector3f vPosition;
		float fWeight;
	};
	std::vector<Constraint> m_vConstraints;


	gsi::SparseLinearSystem * m_pSystemM;
	gsi::SparseLinearSystem * GetSystem();

	gsi::Solver_TAUCS * m_pSolver;
	gsi::Solver_TAUCS * GetSolver();

	gsi::SparseMatrix * m_pLs;


	Wml::GMatrixd m_MTM;
	Wml::GMatrixd m_MT;
	Wml::GVectord m_RHS[3], m_MTRHS[3];
	bool m_bMatricesValid;
	void UpdateMatrices();

	void UpdateRHS(bool bUseTargetNormals, bool bUseTargetTangents, bool bEstimateNormals);

	
	Wml::Vector3f ToNbrFrame( IMesh::VertexID vID, const Wml::Vector3f & v);
	Wml::Vector3f FromNbrFrame( IMesh::VertexID vID, const Wml::Vector3f & v, Wml::Vector3f * pNormalDir = NULL, Wml::Vector3f * pTangentDir = NULL );
};



}   // end namespace rms