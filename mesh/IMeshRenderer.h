// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef __RMS_IMESH_RENDERER_H__
#define __RMS_IMESH_RENDERER_H__

#include "config.h"
#include "IMesh.h"
#include "MeshPolygons.h"

namespace rms {

class IMeshRenderer
{
public:
	IMeshRenderer( IMesh * pMesh, MeshPolygons * pPolygons = NULL );
	~IMeshRenderer(void);

	void Render();
	void Render_Wire();

	enum NormalMode {
		NoNormals,
		VertexNormals,
		FaceNormals
	};
	NormalMode GetNormalMode() { return m_eNormalMode; }
	void SetNormalMode( NormalMode eMode ) { m_eNormalMode = eMode; }

	enum ColorMode {
		NoColors,
		VertexColors
	};
	ColorMode GetColorMode() { return m_eColorMode; }
	void SetColorMode( ColorMode eMode ) { m_eColorMode = eMode; }

	enum Texture2DMode {
		NoTexture2D,
		VertexTexture2D
	};
	Texture2DMode GetTexture2DMode() { return m_eTexture2DMode; }
	void SetTexture2DMode( Texture2DMode eMode ) { m_eTexture2DMode = eMode; }

	enum EdgeMode {
		NoEdges,
		AllEdges,
		PolygonEdges
	};
	EdgeMode GetEdgeMode() { return m_eEdgeMode; }
	void SetEdgeMode( EdgeMode eMode ) { m_eEdgeMode = eMode; }


	void SetModes( NormalMode eNormalMode, Texture2DMode eTex2DMode ) 
		{ m_eNormalMode = eNormalMode; m_eTexture2DMode = eTex2DMode; }


	void SetDrawNormals( bool bDrawNormals ) { m_bDrawNormals = bDrawNormals; }
	bool GetDrawNormals() { return m_bDrawNormals; }

	void SetDrawBoundaryEdges( bool bDrawBoundary ) { m_bDrawBoundaryEdges = bDrawBoundary; }
	bool GetDrawBoundaryEdges() { return m_bDrawBoundaryEdges; }

	void SetForcePolygonOffset( bool bEnable ) { m_bForcePolygonOffset = bEnable; }
	bool GetForcePolygonOffset() { return m_bForcePolygonOffset; }

protected:
	IMesh * m_pMesh;
	MeshPolygons * m_pPolygons;

	NormalMode m_eNormalMode;
	ColorMode m_eColorMode;
	Texture2DMode m_eTexture2DMode;
	EdgeMode m_eEdgeMode;

	bool m_bForcePolygonOffset;

	bool m_bDrawNormals;
	void RenderNormals();

	bool m_bDrawBoundaryEdges;
	void DrawBoundaryEdges();
};

} // end namespace rms


#endif // __RMS_IMESH_RENDERER_H__