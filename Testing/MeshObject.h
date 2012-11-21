// Ryan Schmidt - rms@unknownroad.com
// Copyright (c) 2006-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt


// #pragma once

#include <VFTriangleMesh.h>
#include <VFMeshRenderer.h>
#include <ExpMapGenerator.h>
#include <IMeshBVTree.h>
#include "Texture.h"

class MeshObject
{
public:
	MeshObject(void);
	~MeshObject(void);

	void ReadMeshOBJ( const char * pFilename );
	void SetMesh( rms::VFTriangleMesh & mesh );
	void NotifyMeshModified();

	void Render(bool bWireframe = false, bool bFlatShading = false, 
				bool bUseScalarColors = false, 
				rms::VFMeshRenderer::ColorTransferMode eTransferMode = rms::VFMeshRenderer::ScaledToUnit,
				int nScalarSet = 0);
	void RenderParamMesh();

	rms::VFTriangleMesh & GetMesh() { return m_mesh; }
	rms::IMeshBVTree & GetBVTree() { return m_bvTree; }

	float MaxEdgeLength() { return m_fMaxEdgeLength; }
	float MinEdgeLength() { return m_fMinEdgeLength; }
	float AverageEdgeLength() { return m_fAvgEdgeLength; }

	bool FindIntersection( Wml::Ray3f & vRay, Wml::Vector3f & vHit, Wml::Vector3f & vHitNormal );

	bool FindHitFrame( Wml::Ray3f & vRay, rms::Frame3f & vFrame );
	bool FindNearestFrame( const Wml::Vector3f & vPoint, rms::Frame3f & vNearestFrame );

	void SetComputeExpMap(bool bEnable);
	bool GetComputeExpMap() { return m_bComputeExpMap; }
	rms::Frame3f & ExpMapFrame() { return m_vSeedFrame; }
	void MoveExpMap( rms::Frame3f & vFrame );
	void ScaleExpMap( float fScale );
	void RotateExpMap( float fRotate );
	bool ValidateExpMap(bool bForce = false);
	void SetDecalRadius( float fRadius );

	void ProjectPoints();

	void RemeshDecal();

	void DoRandomEdgeFlips(int nCount);
	void AddNoise(float fConstant);
	void Smooth();

	void SaveCurrent();
	void RestoreOriginal();

	rms::ExpMapGenerator * GetExpMapGen() { return & m_expmapgen; }


	void ExportCSV_Edges(const char * pFilename);

protected:
	rms::VFTriangleMesh m_mesh;
	rms::IMeshBVTree m_bvTree;

	float m_fMinEdgeLength, m_fMaxEdgeLength, m_fAvgEdgeLength;

	rms::Frame3f m_vSeedFrame;
	float m_fDecalRadius;
	float m_fBoundaryWidth;

	rms::ExpMapGenerator m_expmapgen;
	bool m_bExpMapValid;
	bool m_bComputeExpMap;

	std::vector<Wml::Vector3f> m_vProjPoints;

	std::vector<Wml::Vector3f> m_vOrigNormals;
	std::vector<Wml::Vector3f> m_vOrigVerts;

	Texture m_texture;
};
