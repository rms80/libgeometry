// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include <VFTriangleMesh.h>
#include <Frame.h>

namespace rms {

class IMeshDeformer
{
public:
	IMeshDeformer() { 
		m_fGlobalScale = 1.0f;
	}

	virtual void SetMesh(rms::VFTriangleMesh * pMesh) = 0;
	
	virtual void AddBoundaryConstraints(float fWeight) = 0;
	virtual void ClearConstraints() = 0;
	virtual void UpdatePositionConstraint( IMesh::VertexID vID, const Wml::Vector3f & vPosition, float fWeight ) = 0;
	virtual void UpdateOrientationConstraint( IMesh::VertexID vID, const rms::Frame3f & vFrame, float fWeight ) = 0;

	virtual float GlobalScale() const { return m_fGlobalScale; }
	virtual void SetGlobalScale(float fGlobalScale) { m_fGlobalScale = fGlobalScale; }

	virtual rms::Frame3f GetCurrentFrame( IMesh::VertexID vID ) = 0;

	virtual void Solve() = 0;

	virtual void DebugRender() {}

protected:
	float m_fGlobalScale;
};



}   // end namespace rms