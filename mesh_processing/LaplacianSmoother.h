// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include <VFTriangleMesh.h>
#include <Wm4GMatrix.h>


// predecl to avoid include
namespace gsi {
	class SparseLinearSystem;
	class Solver_TAUCS;
	class SparseMatrix;
	class Vector;
};


namespace rms {

class LaplacianSmoother
{
public:
	LaplacianSmoother();
	~LaplacianSmoother();

	enum WeightMode {
		Weights_Uniform,
		Weights_Cotan
	};
	void SetWeightMode( WeightMode eMode ) { m_eWeightMode = eMode; m_bWeightsValid = false; m_bSolutionValid = false; }
	WeightMode GetWeightMode() { return m_eWeightMode; }

	enum VertexAreaMode {
		Area_One,
		Area_Mixed
	};
	void SetVertexAreaMode( VertexAreaMode eMode ) { m_eVertexAreaMode = eMode; m_bWeightsValid = false; m_bSolutionValid = false; }
	VertexAreaMode GetVertexAreaMode() { return m_eVertexAreaMode; }



	void SetMesh(rms::VFTriangleMesh * pMesh);
	void SetROI( const std::vector<IMesh::VertexID> & vROI );
	
	enum ConstraintType {
		CType_SoftBoundary,
		CType_SoftBoundary_Ring0,
		CType_SoftInterior,
		CType_Unspecified,
		CType_Unconstrained
	};

	void AddBoundaryConstraints(float fWeight = 1.0f);
	void AddSoftBoundaryConstraints(float fWeight = 1.0f, int nRings = 3, bool bBlendWeight = true);
	void AddAllInteriorConstraints(float fWeight = 1.0f);

	void ClearConstraints() { m_vConstraints.resize(0); }
	ConstraintType GetConstraint( IMesh::VertexID vID );

	//! constraint type overwrites existing unless passed as CType_Unspecified
	void UpdateConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition, float fWeight, ConstraintType eType = CType_Unspecified );

	//! only updates constraint position if it exists, doesn't change weight
	void UpdateConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition );


	//! scaling factor for laplacian vectors. Default is 0 (membrane solution). Set to > 1 to exaggerate details
	float GetLaplacianVectorScale() { return m_fLaplacianVectorScale; }
	void SetLaplacianVectorScale(float fValue) { m_fLaplacianVectorScale = fValue; m_bSolutionValid = false; }

	//! weighting factor between laplacian vectors and interior soft constraints
	float GetInteriorConstraintWeightScale() { return m_fInteriorConstraintWeightScale; }
	void SetInteriorConstraintWeightScale(float fSet) { m_fInteriorConstraintWeightScale = fSet;  m_bSolverValid = false;  m_bSolutionValid = false; }

	bool Solve();

	void UpdateConstraintsFromMesh();
	void SnapRing0BoundaryConstraints();

	void Render();

protected:
	rms::VFTriangleMesh * m_pMesh;

	WeightMode m_eWeightMode;
	VertexAreaMode m_eVertexAreaMode;

	std::set<IMesh::VertexID> m_vROI;
	VertexMap m_vMap;
	std::set<unsigned int> m_vROIBoundary;		// might be faster to use bitmask for this?
	size_t m_nROISize;

	struct VtxInfo {
		Wml::Vector3f vOrigPosition;
		std::vector<IMesh::VertexID> vNbrs;
		std::vector<float> vNbrWeights;
		float vVtxArea;

		Wml::Vector3f vCurLaplacian;
		Wml::Vector3f vMeshLaplacian;
	};
	std::vector<VtxInfo> m_vVertices;
	float m_fAvgVtxArea;
	bool m_bWeightsValid;
	unsigned int m_nEdges;
	void ValidateWeights();

	struct Constraint {
		ConstraintType eType;
		IMesh::VertexID vID;
		unsigned int nIndex;
		Wml::Vector3f vPosition;
		float fWeight;
	};
	std::vector<Constraint> m_vConstraints;


	float m_fLaplacianVectorScale;		
	float m_fInteriorConstraintWeightScale;


	gsi::SparseLinearSystem * m_pSystemM;
	gsi::SparseLinearSystem * GetSystem();

	gsi::Solver_TAUCS * m_pSolver;
	gsi::Solver_TAUCS * GetSolver();

	gsi::SparseMatrix * m_pLs;
	gsi::SparseMatrix * m_pM;
	gsi::SparseMatrix * m_pSystem;
	gsi::Vector       * m_pRHS;

	bool m_bMatricesValid;
	bool m_bSolverValid;
	bool m_bSolutionValid;

	void UpdateSytemMatrix_ThinPlate();
	void UpdateSolver_ThinPlate();
	void UpdateRHS_ThinPlate();


	void UpdateMatrices_Shell();
	void UpdateRHS_Shell();

};



}   // end namespace rms