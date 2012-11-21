// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#pragma once

#include "config.h"
#include <vector>
#include "PolyLoop3.h"


// predecl to avoid include
namespace gsi {
	class SparseLinearSystem;
	class Solver_TAUCS;
	class SparseMatrix;
};


namespace rms {

class LaplacianCurveDeformer
{
public:
	LaplacianCurveDeformer();

	virtual void SetCurve(rms::PolyLoop3f * pCurve);
	
	virtual void AddBoundaryConstraints(float fWeight = 1.0f);

	virtual void ClearConstraints() { m_vConstraints.resize(0); }
	
	virtual void UpdatePositionConstraint( IPolyCurve::VertexID vID, const Wml::Vector3f & vPosition, float fWeight );

	float GetLaplacianScale() { return m_fLaplacianScale; }
	void SetLaplacianScale( float fScale ) { m_fLaplacianScale = fScale; }

	virtual void Solve();

	virtual void DebugRender();

protected:
	rms::PolyLoop3f * m_pCurve;

	struct VtxInfo {
		std::vector<IPolyCurve::VertexID> vNbrs;
		std::vector<float> vNbrWeights;

		Wml::Vector3f vLaplacian;

		VtxInfo() {  }
	};
	std::vector<VtxInfo> m_vVertices;
	void ComputeWeights();

	struct Constraint {
		IPolyCurve::VertexID vID;
		Wml::Vector3f vPosition;
		float fWeight;
	};
	std::vector<Constraint> m_vConstraints;

	float m_fLaplacianScale;

	gsi::SparseLinearSystem * m_pSystemM;
	gsi::SparseLinearSystem * GetSystem();

	gsi::Solver_TAUCS * m_pSolver;
	gsi::Solver_TAUCS * GetSolver();

	gsi::SparseMatrix * m_pLs;

	bool m_bMatricesValid;
	void UpdateMatrices();

	void UpdateRHS();
};



}   // end namespace rms