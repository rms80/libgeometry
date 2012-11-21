// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include <VFTriangleMesh.h>


namespace rms {

class MeshCurvature
{
public:
	MeshCurvature();

	enum MeanCurvatureMode {
		MeanCurvature_Normal,				// calls MeanCurvature_NormalSK01()			
	};

	/*
	 * generic interface, stores mean-curvature values internally, uses caches when possible
	 */

	void MeanCurvature( MeanCurvatureMode eMode, VFTriangleMesh * pMesh, const std::vector<IMesh::VertexID> & vSelection );
	float MeanCurvature( MeanCurvatureMode eMode, VFTriangleMesh * pMesh, IMesh::VertexID vID);



	/*
	 * direct computation (no caching or storage)
	 */

	//! Schneider & Kobbelt 01 version of Moreton & Sequin 92
	float MeanCurvature_NormalSK01( VFTriangleMesh * pMesh, IMesh::VertexID vID );


protected:
	rms::VFTriangleMesh * m_pMesh;

	std::vector<float> m_vH;
};



}   // end namespace rms