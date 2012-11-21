// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef __RMS_GSURFACE_RENDERER_H__
#define __RMS_GSURFACE_RENDERER_H__
#include "config.h"
#include "GSurface.h"

namespace rms {

class GSurfaceRenderer
{
public:
	GSurfaceRenderer( GSurface * pSurface );
	~GSurfaceRenderer(void);

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
		VertexTexture2D,
		FaceTexture2D,
		VertexTexture2D_Required
	};
	Texture2DMode GetTexture2DMode() { return m_eTexture2DMode; }
	void SetTexture2DMode( Texture2DMode eMode ) { m_eTexture2DMode = eMode; }

	enum EdgeMode {
		NoEdges,
		AllEdges
	};
	EdgeMode GetEdgeMode() { return m_eEdgeMode; }
	void SetEdgeMode( EdgeMode eMode ) { m_eEdgeMode = eMode; }

	void SetModes( NormalMode eNormalMode, Texture2DMode eTex2DMode ) 
		{ m_eNormalMode = eNormalMode; m_eTexture2DMode = eTex2DMode; }

	void SetDrawNormals( bool bDrawNormals ) { m_bDrawNormals = bDrawNormals; }
	bool GetDrawNormals() { return m_bDrawNormals; }

	void SetEnableScalarColor( bool bEnable ) { m_bEnableScalarColor = bEnable; }
	bool GetEnableScalarColor() { return m_bEnableScalarColor; }

	void SetScalarColorSet( int nSet ) { m_nScalarColorSet = nSet; }
	int GetScalarColorSet() { return m_nScalarColorSet; }

	enum ColorTransferMode {
		Unscaled,
		ScaledToUnit,
		SinModulate
	};
	ColorTransferMode GetColorTransferMode() { return m_eColorTransferMode; }
	void SetColorTransferMode( ColorTransferMode eMode ) { m_eColorTransferMode = eMode; }


protected:
	GSurface * m_pSurface;

	NormalMode m_eNormalMode;
	ColorMode m_eColorMode;
	ColorTransferMode m_eColorTransferMode;
	Texture2DMode m_eTexture2DMode;
	EdgeMode m_eEdgeMode;

	bool m_bDrawNormals;
	void RenderNormals();

	bool m_bEnableScalarColor;
	int m_nScalarColorSet;

	void DrawBoundaryEdges();
};




} // end namespace rmsmesh


#endif // __RMS_GSURFACE_RENDERER_H__