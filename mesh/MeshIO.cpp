// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "MeshIO.h"

#include <fstream>
#include <strstream>
#include <functional>
#include <Meshutils.h>

using namespace rms;

MeshIO::MeshIO(const char * pFilename, VFTriangleMesh *pMesh, MeshPolygons *pPolygonSets)
{
	m_filename = std::string(pFilename);

	DetermineFormat();

	m_pSurface = new GSurface(pMesh, pPolygonSets);
	m_pMesh = &m_pSurface->Mesh();
	m_pPolygonSets = &m_pSurface->Polygons();
	m_pWriteOnlySurface = NULL;
	m_pWriteOnlyMesh = NULL;
	m_pWriteOnlyPolygons = NULL;

	m_bWritePerPolygonUVs = false;
}

MeshIO::MeshIO(const char * pFilename, GSurface * pSurface )
{
	m_filename = std::string(pFilename);

	DetermineFormat();

	m_pSurface = pSurface;
	m_pMesh = &m_pSurface->Mesh();
	m_pPolygonSets = &m_pSurface->Polygons();
	m_pWriteOnlySurface = NULL;
	m_pWriteOnlyMesh = NULL;
	m_pWriteOnlyPolygons = NULL;

	m_bWritePerPolygonUVs = false;
}

MeshIO::MeshIO(const char * pFilename, const GSurface * pSurface )
{
	m_filename = std::string(pFilename);

	DetermineFormat();

	m_pSurface = NULL;
	m_pMesh = NULL;
	m_pPolygonSets = NULL;
	m_pWriteOnlySurface = pSurface;
	m_pWriteOnlyMesh = &m_pWriteOnlySurface->Mesh();
	m_pWriteOnlyPolygons = &m_pWriteOnlySurface->Polygons();

	m_bWritePerPolygonUVs = false;
}


void MeshIO::DetermineFormat()
{
	size_t nIndex = m_filename.find_last_of('.');
	std::string extension = m_filename.substr(nIndex);

	size_t nFileStart = m_filename.find_last_of('\\');
	if ( nFileStart == std::string::npos )
		m_filenameNoSuffix = m_filename.substr(0, nIndex);
	else
		m_filenameNoSuffix = m_filename.substr(nFileStart+1, nIndex-nFileStart-1);

	std::transform(extension.begin(), extension.end(), extension.begin(), tolower);

	if ( extension == std::string(".obj") )
		m_eFormat = Format_OBJ;
	else if ( extension == std::string(".off") )
		m_eFormat = Format_OFF;
	else if ( extension == std::string(".stl") )
		m_eFormat = Format_STL;
	else if ( extension == std::string(".dae") )
		m_eFormat = Format_COLLADA;
	else
		m_eFormat = Format_Unknown;
}



bool MeshIO::Read()
{
	switch ( m_eFormat ) {
		case Format_OBJ:
			return Read_OBJ();
		case Format_OFF:
			return Read_OFF();
		default:
			m_errstring = std::string("Unrecognized format ") + m_filename;
			std::cerr << m_errstring << std::endl;
			return false;
	}
}


struct tri_uv {
	int vIndex[3];
};
bool MeshIO::Read_OBJ()
{
	m_pMesh->Clear(false);

	std::ifstream in(m_filename.c_str());
	if(!in){
		m_errstring = std::string("Cannot open file ") + m_filename;
		std::cerr << m_errstring << std::endl;
		return false;
	}

	std::string command, facevtx;
	unsigned int tv,tn,tt;
	char linebuf[1024];
	std::vector<IMesh::VertexID> vv, vn, vt;

	Wml::Vector3f fvec = Wml::Vector3f::ZERO;

	bool bHasTextures = false;
	bool bHasNormals = false;
	bool bInitializedUVSet = false;

	// need to save normals separately and then match to vertices (maya "optimizes" the mesh...argh!)
	std::map<int, int> vUVsFromPolygons;
	UVList & vUVs = m_pSurface->UV();
	NormalList & vNormals = m_pSurface->Normals();

	while(in){
		in >> command;
		if(!in)
			continue;
		switch(command.c_str()[0]){

		case '#':				// comment
			in.getline(linebuf, 1023, '\n');
			break;

		case 'm':				// mtllib (?)
			in.getline(linebuf, 1023, '\n');
			break;

		case 'v':    
			in >> fvec[0] >> fvec[1];
			if(!in)
				continue;
			switch(command.c_str()[1]){
			case '\0':  // vertex
				in >> fvec[2];
				m_pMesh->AppendVertex(fvec);
				break;
			case 'n': // vertex normal
				bHasNormals = true;
				in >> fvec[2];
				fvec.Normalize();
				vNormals.push_back( fvec );
				break;
			case 't':
				if ( ! bHasTextures ) 
					bHasTextures = true;
				vUVs.push_back( Wml::Vector2f(fvec) );
				break;
			default:
				std::string err("Got unknown OBJ command ");
				err += command;
				std::cerr << err << std::endl;
			}
		break;

		case 'f': {
			in.getline(linebuf, 1023, '\n');
			vv.resize(0);  vn.resize(0);  vt.resize(0);

			std::istrstream linein(linebuf);
			while ( linein ) {
				linein >> facevtx;
				size_t nSlash1 = facevtx.find_first_of('/');
				size_t nSlash2 = facevtx.find_last_of('/');
				tv = atoi( facevtx.substr(0, nSlash1).c_str() ) - 1;
				if ( ! vv.empty() && tv == vv.back() )
					break;
				if ( nSlash1 == std::string::npos ) {
					vv.push_back(tv);
					vn.push_back(IMesh::InvalidID);
					vt.push_back(IMesh::InvalidID);
				} else if ( nSlash2 == nSlash1 ) {
					tt = atoi( facevtx.substr(nSlash1+1).c_str() ) - 1;
					vv.push_back(tv);
					vn.push_back(IMesh::InvalidID);
					vt.push_back(tt);
				} else if ( nSlash2 == nSlash1+1 ) {
					tn = atoi( facevtx.substr(nSlash2+1).c_str() ) - 1;
					vv.push_back(tv);
					vn.push_back(tn);
					vt.push_back(IMesh::InvalidID);
				} else {
					tt = atoi( facevtx.substr(nSlash1+1,nSlash2).c_str() ) - 1;
					tn = atoi( facevtx.substr(nSlash2+1).c_str() ) - 1;
					vv.push_back(tv);
					vn.push_back(tn);
					vt.push_back(tt);
				}
			}		
			size_t nVerts = vv.size();
			if ( nVerts < 2 ) {
				std::cerr << "face has less than 2 vertices" << std::endl;
				continue;
			}

			// make set even for polygons that are triangles
			if ( m_pPolygonSets ) {
				MeshPolygons::PolygonID nPolyID = m_pPolygonSets->CreatePolygon((unsigned int)nVerts-2);
				for ( unsigned int k = 1; k < nVerts-1; ++k ) {
					IMesh::TriangleID tID = m_pMesh->AppendTriangle(vv[0], vv[k], vv[k+1]);
					m_pPolygonSets->AppendTriangle(nPolyID, tID);
				}
				m_pPolygonSets->SetBoundary(nPolyID, vv, &vt);
			} else {
				for ( unsigned int k = 1; k < nVerts-1; ++k ) {
					m_pMesh->AppendTriangle(vv[0], vv[k], vv[k+1]);
				}
			}

			// set proper normals
			if ( bHasNormals ) {
				for ( unsigned int k = 0; k < nVerts; ++k ) 
					m_pMesh->SetNormal( vv[k], vNormals[vn[k]] );
			}

			if ( bHasTextures ) {
				for ( unsigned int k = 0; k < nVerts; ++k )
					vUVsFromPolygons[ vv[k] ] = vt[k];
			}

		} break;

		default:
			in.getline(linebuf, 1023, '\n');
		}
	}

	// set uv's from polygons
	if ( ! vUVsFromPolygons.empty() ) {
		if ( ! bInitializedUVSet ) {
			m_pMesh->AppendUVSet();
			m_pMesh->InitializeUVSet(0);
			bInitializedUVSet = true;
		}
		std::map<int, int>::iterator curvtx(vUVsFromPolygons.begin()), endvtx(vUVsFromPolygons.end());
		while ( curvtx != endvtx ) {
			int nVtxIdx = curvtx->first;
			int nUVIdx = curvtx->second;  curvtx++;
			if ( m_pMesh->IsVertex(nVtxIdx) && nUVIdx >= 0 && nUVIdx < (int)vUVs.size() )
				m_pMesh->SetUV( nVtxIdx, 0, vUVs[nUVIdx] );
		}
	}


	// if we have no triangles, assume 1-1 ordered matches
	unsigned int nVerts = m_pMesh->GetVertexCount();
	if ( m_pMesh->GetTriangleCount() == 0 ) {
		if ( vNormals.size() == nVerts ) {
			for ( unsigned int k = 0; k < nVerts; ++k )
				m_pMesh->SetNormal( k, vNormals[k] );
		}
		if ( vUVs.size() == nVerts ) {
			if ( ! bInitializedUVSet ) {
				m_pMesh->AppendUVSet();
				m_pMesh->InitializeUVSet(0);
				bInitializedUVSet = true;
			}
			for ( unsigned int k = 0; k < nVerts; ++k )
				m_pMesh->SetUV( k, 0, vUVs[k] );
		}

	}

	return true;	
}



bool MeshIO::Read_OFF()
{
	m_pMesh->Clear(false);

	std::ifstream in(m_filename.c_str());
	if(!in){
		m_errstring = std::string("Cannot open file ") + m_filename;
		std::cerr << m_errstring << std::endl;
		return false;
	}

	std::string formatLine;
	in >> formatLine;
	if ( formatLine != std::string("OFF") )
		return false;

	int numV, numF, numE;
	in >> numV >> numF >> numE;
	if ( ! in )
		return false;

	for ( int vi = 0; vi < numV; ++vi )  {
		float v[3];
		in >> v[0] >> v[1] >> v[2];
		m_pMesh->AppendVertex(v);
	} 
	if (!in)
		return false;

	std::vector<IMesh::VertexID> vv;
	for ( int fi = 0; fi < numF; ++fi ) {
		int nVerts, vert;
		in >> nVerts;
		vv.resize(0);
		for ( int vi = 0; vi < nVerts; ++vi ) {
			in >> vert;
			vv.push_back(vert);
		}

		for ( int vi = 0; vi < nVerts; ++vi ) {
			if ( ! m_pMesh->IsVertex(vv[vi]) ) {
				lgBreakToDebugger();
				return false;
			}
		}



		// make set even for polygons that are triangles
		if ( m_pPolygonSets ) {
			MeshPolygons::PolygonID nPolyID = m_pPolygonSets->CreatePolygon((unsigned int)nVerts-2);
			for ( int k = 1; k < nVerts-1; ++k ) {
				IMesh::TriangleID tID = m_pMesh->AppendTriangle(vv[0], vv[k], vv[k+1]);
				m_pPolygonSets->AppendTriangle(nPolyID, tID);
			}
			m_pPolygonSets->SetBoundary(nPolyID, vv);
		} else {
			for ( int k = 1; k < nVerts-1; ++k ) {
				m_pMesh->AppendTriangle(vv[0], vv[k], vv[k+1]);
			}
		}
	}
	if ( ! in )
		return false;
	in.close();

	// compute normals, as OFF doesn't store them
	rms::MeshUtils::EstimateNormals(*m_pMesh);

	return true;	
}





bool MeshIO::Write()
{
	switch ( m_eFormat ) {
		case Format_OBJ:
			return Write_OBJ();
		case Format_STL:
			return Write_STL();
		case Format_COLLADA:
			return Write_COLLADA();
		default:
			m_errstring = std::string("Unrecognized format ") + m_filename;
			std::cerr << m_errstring << std::endl;
			return false;
	}
}


struct Face {
	std::vector<IMesh::VertexID> vFace;
	std::vector<unsigned int> vFaceUV;
};

bool MeshIO::Write_OBJ( )
{
	const GSurface * pWriteSurface = (m_pWriteOnlySurface) ? m_pWriteOnlySurface : m_pSurface;
	const VFTriangleMesh * pWriteMesh = (m_pWriteOnlyMesh) ? m_pWriteOnlyMesh : m_pMesh;
	const MeshPolygons * pWritePolygons = (m_pWriteOnlyPolygons) ? m_pWriteOnlyPolygons : m_pPolygonSets;

	std::ofstream out(m_filename.c_str());
	if (!out) {
		m_errstring = std::string("Cannot open file ") + m_filename;
		std::cerr << m_errstring << std::endl;
		return false;
	}

	bool bUsePerVertexTexCoords = (pWriteMesh->HasUVSet(0) && !m_bWritePerPolygonUVs);

	// if we are writing per-vertex UVs, force some UV-value to be set for each vertex  (why??)
	std::vector<Wml::Vector2f> vUV;
	if ( bUsePerVertexTexCoords ) {
		vUV.resize(pWriteMesh->GetMaxVertexID());
		VFTriangleMesh::vertex_iterator curv(pWriteMesh->BeginVertices()), endv(pWriteMesh->EndVertices());
		while ( curv != endv ) {
			IMesh::VertexID vID = *curv;  ++curv;
			if ( ! pWriteMesh->GetUV(vID, 0, vUV[vID]) )
				vUV[vID] = Wml::Vector2f(-5.0f,-5.0f);		// set 'invalid' UV value (?)
		}
	}

	// write vertex/normal/texcoord records
	VertexMap vMap;
	vMap.Resize(pWriteMesh->GetMaxVertexID(), pWriteMesh->GetMaxVertexID());
	unsigned int nCounter = 0;
	VFTriangleMesh::vertex_iterator curv(pWriteMesh->BeginVertices()), endv(pWriteMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vID = *curv;  ++curv;
		vMap.SetMap(vID, nCounter++);

		Wml::Vector3f vert, norm;
		pWriteMesh->GetVertex(vID, vert, &norm);

		out << "v " << vert.X() << " " << vert.Y() << " " << vert.Z() << std::endl;
		out << "vn " << norm.X() << " " << norm.Y() << " " << norm.Z() << std::endl; 

		if ( bUsePerVertexTexCoords )
			out << "vt " << vUV[vID].X() << " " << vUV[vID].Y() << std::endl; 
	}

	// if we are writing polygon UVs, write them all at end
	if ( m_bWritePerPolygonUVs ) {
		size_t nCount = pWriteSurface->UV().size();
		for ( unsigned int k = 0; k < nCount; ++k ) {
			out << "vt " << pWriteSurface->UV()[k].X() << " " << pWriteSurface->UV()[k].Y() << std::endl; 
		}
	}

	// construct face lists
	std::vector<Face> vFaces;

	if ( pWritePolygons ) {
		MeshPolygons::id_iterator curf(pWritePolygons->begin()), endf(pWritePolygons->end());
		while ( curf != endf ) {
			MeshPolygons::PolygonID sID = *curf++;
			Face f;
			f.vFace = pWritePolygons->GetBoundary(sID);
			if ( m_bWritePerPolygonUVs )
				f.vFaceUV = pWritePolygons->GetBoundaryUV(sID);
			vFaces.push_back(f);
		}

	} else {
		VFTriangleMesh::triangle_iterator curt(pWriteMesh->BeginTriangles()), endt(pWriteMesh->EndTriangles());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt; ++curt;
			Face f;
			f.vFace.resize(3);
			pWriteMesh->GetTriangle(tID, &f.vFace[0]);
			vFaces.push_back(f);
		}
	}

	// write faces
	size_t nFaces = vFaces.size();
	for ( unsigned int i = 0; i < nFaces; ++i ) {
		Face & f = vFaces[i];
		size_t nV = f.vFace.size();
		size_t nU = f.vFaceUV.size();
		out << "f ";
		for ( unsigned int j = 0; j < nV; ++j ) {
			int nIndex = vMap.GetNew(f.vFace[j]) + 1;			// OBJ indices start at 1

			if ( bUsePerVertexTexCoords )
				out << nIndex << "/" << nIndex << "/" << nIndex << " ";
			else if ( m_bWritePerPolygonUVs && nV == nU )
				out << nIndex << "/" << (f.vFaceUV[j]+1) << "/" << nIndex << " ";
			else
				out << nIndex << "//" << nIndex << " ";
		}
		out << std::endl;
	}

	out.close();

	m_errstring = std::string("no error");

	return true;
}




bool MeshIO::Write_STL()
{
	const VFTriangleMesh * pWriteMesh = (m_pWriteOnlyMesh) ? m_pWriteOnlyMesh : m_pMesh;
	const MeshPolygons * pWritePolygons = (m_pWriteOnlyPolygons) ? m_pWriteOnlyPolygons : m_pPolygonSets;

	std::ofstream out(m_filename.c_str());
	if (!out) {
		m_errstring = std::string("Cannot open file ") + m_filename;
		std::cerr << m_errstring << std::endl;
		return false;
	}

	out << "solid " << m_filenameNoSuffix << std::endl;



	// construct face lists
	std::vector<Face> vFaces;
	std::vector<Wml::Vector3f> vFaceNormals;

	if ( pWritePolygons ) {
		MeshPolygons::id_iterator curf(pWritePolygons->begin()), endf(pWritePolygons->end());
		while ( curf != endf ) {
			MeshPolygons::PolygonID sID = *curf++;
			Face f;
			f.vFace = pWritePolygons->GetBoundary(sID);
			vFaces.push_back(f);
			vFaceNormals.push_back( Wml::Vector3f(0,0,0) );
		}

	} else {
		VFTriangleMesh::triangle_iterator curt(pWriteMesh->BeginTriangles()), endt(pWriteMesh->EndTriangles());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt; ++curt;
			Face f;
			f.vFace.resize(3);
			pWriteMesh->GetTriangle(tID, &f.vFace[0]);
			vFaces.push_back(f);
			Wml::Vector3f vTri[3];
			pWriteMesh->GetTriangle(tID, vTri);
			vFaceNormals.push_back( Wml::Vector3f(0,0,0) );
		}
	}

	// write faces
	size_t nFaces = vFaces.size();
	for ( unsigned int i = 0; i < nFaces; ++i ) {
		Face & f = vFaces[i];
		size_t nV = f.vFace.size();
		Wml::Vector3f & vFaceNormal = vFaceNormals[i];

		out << "  facet normal " << vFaceNormal.X() << " " << vFaceNormal.Y() << " " << vFaceNormal.Z() << std::endl;
		out << "    outer loop" << std::endl;
		for ( unsigned int i = 0; i < nV; ++i ) {
			Wml::Vector3f v( pWriteMesh->GetVertex( f.vFace[i] ) );
			out << "      vertex " << v.X() << " " << v.Y() << " " << v.Z() << std::endl;
		}
		out << "    endloop" << std::endl;
		out << "  endfacet" << std::endl;
	}


	out << "endsolid " << m_filenameNoSuffix << std::endl;

	out.close();

	m_errstring = std::string("no error");
	return true;
}



bool MeshIO::Write_COLLADA()
{
	const VFTriangleMesh * pWriteMesh = (m_pWriteOnlyMesh) ? m_pWriteOnlyMesh : m_pMesh;
	const MeshPolygons * pWritePolygons = (m_pWriteOnlyPolygons) ? m_pWriteOnlyPolygons : m_pPolygonSets;

	std::ofstream out(m_filename.c_str());
	if (!out) {
		m_errstring = std::string("Cannot open file ") + m_filename;
		std::cerr << m_errstring << std::endl;
		return false;
	}


	out << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << std::endl;
	out << "<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">" << std::endl;
	out << "<asset>" << std::endl;
	out << "  <contributor>" << std::endl;
	out << "    <authoring_tool>libgeometry</authoring_tool>" << std::endl;
	out << "  </contributor>" << std::endl;
	out << "  <unit name=\"inches\" meter=\"0.0254\"/>" << std::endl;
	out << "  <up_axis>Y_UP</up_axis>" << std::endl;
	out << "</asset>" << std::endl;


	// make geometry buffers
	std::vector<float> vertBuf, normBuf;
	rms::VertexMap vertMap;
	vertMap.Resize( pWriteMesh->GetMaxVertexID(), pWriteMesh->GetMaxVertexID() );
	unsigned int nCounter = 0;
	VFTriangleMesh::vertex_iterator curv(pWriteMesh->BeginVertices()), endv(pWriteMesh->EndVertices());
	while ( curv != endv ) {
		IMesh::VertexID vid = *curv;  ++curv;
		vertMap.SetMap( vid, nCounter++ );
		Wml::Vector3f v, n;
		pWriteMesh->GetVertex(vid, v, &n);
		vertBuf.push_back(v[0]);	vertBuf.push_back(v[1]);	vertBuf.push_back(v[2]);
		normBuf.push_back(n[0]);	normBuf.push_back(n[1]);	normBuf.push_back(n[2]);
	}

	std::vector<int> triBuf;
	VFTriangleMesh::triangle_iterator curt(pWriteMesh->BeginTriangles()), endt(pWriteMesh->EndTriangles());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt; ++curt;
		IMesh::VertexID vTri[3];
		pWriteMesh->GetTriangle(tID, vTri);
		for ( int j = 0; j < 3; ++j )
			triBuf.push_back( vertMap.GetNew(vTri[j]) );
	}

	out << "<library_geometries>" << std::endl;
	out << "  <geometry id=\"mesh1-geometry\" name=\"mesh1-geometry\">" << std::endl;
	out << "    <mesh>" << std::endl;

	out << "        <source id=\"mesh1-geometry-position\">" << std::endl;
	out << "        <float_array id=\"mesh1-geometry-position-array\" count=\""
		<< vertBuf.size() << "\">";
		for ( unsigned int i = 0; i < vertBuf.size(); ++i )
			out << vertBuf[i] << " ";
		out << "</float_array>" << std::endl;
	out << "          <technique_common>" << std::endl;
	out << "            <accessor source=\"#mesh1-geometry-position-array\" count=\"" << (vertBuf.size() / 3) << "\" stride=\"3\">" << std::endl;
	out << "              <param name=\"X\" type=\"float\"/>" << std::endl;
	out << "              <param name=\"Y\" type=\"float\"/>" << std::endl;
	out << "              <param name=\"Z\" type=\"float\"/>" << std::endl;
	out << "            </accessor>" << std::endl;
	out << "          </technique_common>" << std::endl;
	out << "        </source>" << std::endl;

	out << "        <source id=\"mesh1-geometry-normal\">" << std::endl;
	out << "        <float_array id=\"mesh1-geometry-normal-array\" count=\""
		<< normBuf.size() << "\">";
		for ( unsigned int i = 0; i < normBuf.size(); ++i )
			out << normBuf[i] << " ";
		out << "</float_array>" << std::endl;
	out << "          <technique_common>" << std::endl;
	out << "            <accessor source=\"#mesh1-geometry-normal-array\" count=\"" << (normBuf.size() / 3) << "\" stride=\"3\">" << std::endl;
	out << "              <param name=\"X\" type=\"float\"/>" << std::endl;
	out << "              <param name=\"Y\" type=\"float\"/>" << std::endl;
	out << "              <param name=\"Z\" type=\"float\"/>" << std::endl;
	out << "            </accessor>" << std::endl;
	out << "          </technique_common>" << std::endl;
	out << "        </source>" << std::endl;

	out << "      <vertices id=\"mesh1-geometry-vertex\">" << std::endl;
	out << "        <input semantic=\"POSITION\" source=\"#mesh1-geometry-position\"/>" << std::endl;
	out << "      </vertices>" << std::endl;

	out << "      <triangles count=\"" << (triBuf.size() / 3) << "\">" << std::endl;
	out << "        <input semantic=\"VERTEX\" source=\"#mesh1-geometry-vertex\" offset=\"0\"/>" << std::endl;
	out << "        <input semantic=\"NORMAL\" source=\"#mesh1-geometry-normal\" offset=\"1\"/>" << std::endl;
	out << "          <p>";
		for ( unsigned int i = 0; i < triBuf.size(); ++i )
			out << triBuf[i] << " " << triBuf[i] << " ";
		out << "</p>" << std::endl;
	out << "      </triangles>" << std::endl;
	out << "    </mesh>" << std::endl;
	out << "  </geometry>" << std::endl;
	out << "</library_geometries>" << std::endl;


	out << "<library_visual_scenes>" << std::endl;
	out << "  <visual_scene id=\"libgeometryScene\" name=\"libgeometryScene\">" << std::endl;
	out << "    <node id=\"Model\" name=\"Model\">" << std::endl;
	out << "      <node id=\"mesh1\" name=\"mesh1\">" << std::endl;
	out << "        <instance_geometry url=\"#mesh1-geometry\">" << std::endl;
	out << "        </instance_geometry>" << std::endl;
	out << "      </node>" << std::endl;
	out << "    </node>" << std::endl;
	out << "  </visual_scene>" << std::endl;
	out << "</library_visual_scenes>" << std::endl;
	out << "<scene>" << std::endl;
	out << "  <instance_visual_scene url=\"#libgeometryScene\"/>" << std::endl;
	out << "</scene>" << std::endl;
	out << "</COLLADA>" << std::endl;

	out.close();

	m_errstring = std::string("no error");
	return true;
}