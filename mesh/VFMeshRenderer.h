// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_VFMESH_RENDERER_H__
#define __RMS_VFMESH_RENDERER_H__
#include "config.h"
#include "VFTriangleMesh.h"
#include "MeshPolygons.h"
#include <BitSet.h>

namespace rms {

class VFMeshRenderer
{
public:
	VFMeshRenderer( const VFTriangleMesh * pMesh = NULL, const MeshPolygons * pPolygons = NULL );
	~VFMeshRenderer(void);

	void SetMesh( const VFTriangleMesh * pMesh, const MeshPolygons * pPolygons = NULL );

	void SetFaceFilterSet( const BitSet * pFaces, bool bInvert = false );

	void Render();

	void Render_Mesh();
	void Render_Wire( const Wml::ColorRGBA & cColor = Wml::ColorRGBA::BLACK );
	void Render_UV();

	enum NormalMode {
		NoNormals,
		VertexNormals,
		FaceNormals
	};
	NormalMode GetNormalMode() { return m_eNormalMode; }
	void SetNormalMode( NormalMode eMode ) { 
		if(m_eNormalMode != eMode) {
			m_bGLDisplayListValid = false; m_eNormalMode = eMode; }
	}

	enum ColorMode {
		NoColors,
		VertexColors
	};
	ColorMode GetColorMode() { return m_eColorMode; }
	void SetColorMode( ColorMode eMode ) { 
		if( m_eColorMode != eMode) { 
			m_bGLDisplayListValid = false; m_eColorMode = eMode; } 
	}

	enum Texture2DMode {
		NoTexture2D,
		VertexTexture2D,
		FaceTexture2D,
		VertexTexture2D_Required
	};
	Texture2DMode GetTexture2DMode() { return m_eTexture2DMode; }
	void SetTexture2DMode( Texture2DMode eMode ) { 
		if( m_eTexture2DMode != eMode ) {
			m_bGLDisplayListValid = false; m_eTexture2DMode = eMode; }
	}
		

	enum EdgeMode {
		NoEdges,
		AllEdges,
		PolygonEdges
	};
	EdgeMode GetEdgeMode() { return m_eEdgeMode; }
	void SetEdgeMode( EdgeMode eMode ) { 
		if( m_eEdgeMode != eMode) {
			m_bGLDisplayListValid = false; m_eEdgeMode = eMode; }
	}

	void SetModes( NormalMode eNormalMode, Texture2DMode eTex2DMode ) {
		if ( m_eNormalMode != eNormalMode || m_eTexture2DMode != eTex2DMode ) {
			m_bGLDisplayListValid = false; m_eNormalMode = eNormalMode; m_eTexture2DMode = eTex2DMode; }
	}


	bool GetDrawNormals() { return m_bDrawNormals; }
	void SetDrawNormals( bool bDrawNormals ) { 
		if( m_bDrawNormals != bDrawNormals ) {
			m_bGLDisplayListValid = false; m_bDrawNormals = bDrawNormals; }
	}

	bool GetDrawBoundaryEdges() { return m_bDrawBoundary; }
	void SetDrawBoundaryEdges( bool bDrawBoundary ) { 
		if(m_bDrawBoundary != bDrawBoundary) {
			m_bGLDisplayListValid = false; m_bDrawBoundary = bDrawBoundary; }
	}

	bool GetDrawNonManifoldEdges() { return m_bDrawNonManifoldEdges; }
	void SetDrawNonManifoldEdges( bool bEnable ) { 
		if( m_bDrawNonManifoldEdges != bEnable ) {
			m_bGLDisplayListValid = false; m_bDrawNonManifoldEdges = bEnable; }
	}

	bool GetForcePolygonOffset() { return m_bForcePolygonOffset; }
	void SetForcePolygonOffset( bool bEnable ) { 
		if(m_bForcePolygonOffset != bEnable) {
			m_bGLDisplayListValid = false; m_bForcePolygonOffset = bEnable; }
	}

	bool GetEnableScalarColor() { return m_bEnableScalarColor; }
	void SetEnableScalarColor( bool bEnable ) { 
		if( m_bEnableScalarColor != bEnable ) {
			m_bGLDisplayListValid = false; m_bEnableScalarColor = bEnable; }
	}

	int GetScalarColorSet() { return m_nScalarColorSet; }
	void SetScalarColorSet( int nSet ) { if 
		( m_nScalarColorSet != nSet ) {
			m_bGLDisplayListValid = false; m_nScalarColorSet = nSet; }
	}

	enum ColorTransferMode {
		Unscaled,
		ScaledToUnit,
		ScaledToUnit_Custom,
		SinModulate
	};
	ColorTransferMode GetColorTransferMode() { return m_eColorTransferMode; }
	void SetColorTransferMode( ColorTransferMode eMode ) { 
		if (m_eColorTransferMode != eMode) {
			m_bGLDisplayListValid = false; m_eColorTransferMode = eMode; }
	}

	void GetScalarColorMap( Wml::ColorRGBA & minColor,  Wml::ColorRGBA & maxColor ) { minColor = m_cScalarColorMin; maxColor = m_cScalarColorMax; }
	void SetScalarColorMap( const Wml::ColorRGBA & minColor,  const Wml::ColorRGBA & maxColor ) {
		if ( m_cScalarColorMin != minColor || m_cScalarColorMax != maxColor ) {
			m_bGLDisplayListValid = false;
			m_cScalarColorMin = minColor; m_cScalarColorMax = maxColor; }
	}

	void GetScalarColorRange( float & fMin, float & fMax ) { fMin = m_fScalarColorRange[0];  fMax = m_fScalarColorRange[1]; }
	void SetScalarColorRange( float fMin, float fMax ) { 
		if ( m_fScalarColorRange[0] != fMin ||  m_fScalarColorRange[1] != fMax ) {
			m_bGLDisplayListValid = false;
			m_fScalarColorRange[0] = fMin;  m_fScalarColorRange[1] = fMax; }
	}

	void EnableGLDisplayList( bool bEnable ) { m_bUseDisplayList = bEnable; m_bGLDisplayListValid = false; }
	void InvalidateGLDisplayList() { m_bGLDisplayListValid = false; }

protected:
	const VFTriangleMesh * m_pMesh;
	const MeshPolygons * m_pPolygons;

	bool m_bUseDisplayList;
	int m_nGLDisplayList;
	bool m_bGLDisplayListValid;

	const BitSet * m_pDrawFaces;
	bool m_bInvertDrawFacesBits;

	NormalMode m_eNormalMode;
	ColorMode m_eColorMode;
	ColorTransferMode m_eColorTransferMode;
	Texture2DMode m_eTexture2DMode;
	EdgeMode m_eEdgeMode;

	bool m_bForcePolygonOffset;

	Wml::ColorRGBA m_cScalarColorMin, m_cScalarColorMax;
	float m_fScalarColorRange[2];

	bool m_bDrawNormals;
	void RenderNormals();

	bool m_bEnableScalarColor;
	int m_nScalarColorSet;

	bool m_bDrawBoundary;
	void DrawBoundaryEdges();

	bool m_bDrawNonManifoldEdges;
	void DrawNonManifoldEdges();
};




} // end namespace rmsmesh


#endif // __RMS_VFMESH_RENDERER_H__