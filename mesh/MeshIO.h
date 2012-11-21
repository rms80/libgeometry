// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include "config.h"
#include <VFTriangleMesh.h>
#include <MeshPolygons.h>
#include <GSurface.h>
#include <string>

namespace rms {

class MeshIO
{
public:
	enum MeshFormats {
		Format_OBJ,
		Format_OFF,
		Format_STL,
		Format_COLLADA,
		Format_Unknown
	};

	MeshIO(const char * pFilename, VFTriangleMesh *pMesh, MeshPolygons *pPolygonSets = NULL );
	MeshIO(const char * pFilename, GSurface * pSurface );
	MeshIO(const char * pFilename, const GSurface * pSurface );

	//! if this flag is set, we write the GSurface/MeshPolygons uvs, instead of the VFTriangleMesh UVSet
	void SetWritePerPolygonUVs( bool bEnable ) { m_bWritePerPolygonUVs = bEnable; }
	bool GetWritePerPolygonUVs( ) { return m_bWritePerPolygonUVs; }
	

	MeshFormats Format() const { return m_eFormat; }

	bool Read();
	bool Write();

	const std::string & Filename() const { return m_filename; }
	const std::string & GetLastError() const { return m_errstring; }

protected:
	std::string m_filename;
	std::string m_filenameNoSuffix;
	std::string m_errstring;

	MeshFormats m_eFormat;
	void DetermineFormat();

	bool m_bWritePerPolygonUVs;

	GSurface * m_pSurface;
	VFTriangleMesh * m_pMesh;
	MeshPolygons * m_pPolygonSets;

	const GSurface * m_pWriteOnlySurface;
	const VFTriangleMesh * m_pWriteOnlyMesh;
	const MeshPolygons * m_pWriteOnlyPolygons;

	bool Read_OBJ();
	bool Write_OBJ();

	bool Read_OFF();

	bool Write_STL();

	bool Write_COLLADA();
};

}  // end namespace rms
