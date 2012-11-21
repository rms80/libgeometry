// Ryan Schmidt - rms@unknownroad.com
// Copyright (c) 2006-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt


#ifndef RMS_EXTENDED_WML_CAMERA_H
#define RMS_EXTENDED_WML_CAMERA_H

#include <map>
#include <Wm4Vector3.h>
#include <Wm4Matrix3.h>

namespace Wml
{

class ExtendedWmlCamera
{

public:
	ExtendedWmlCamera( );
	ExtendedWmlCamera( float fWidth, float fHeight );

/*
 * 
 */

	void Update();

	//! returns a pointer to a 4-element buffer [left,bottom,width,height]
	const unsigned int * GetViewport() const;
	const Vector3f & GetTarget() const;
	float GetHorzFovDegrees() const;
	float GetAspectRatio() const;

	//! get frame (orientation basis matrix)
	void GetFrameMatrix( Wml::Matrix3f & matrix ) const;


	// RMS TODO
	//  - auto-compute proper z depth to contain scene
	//  - ??



    // Set orthogonal view frustum (L = -R, B = -T) using field of view in
    // the "up" direction and an aspect ratio "width/height".  This call is
    // the equivalent of gluPerspective in OpenGL.  As such, the field of
    // view in this function must be specified in degrees and be in the
    // interval (0,180).
    void SetFrustum (float fUpFovDegrees, float fAspectRatio, float fNear, float fFar);

   // view frustum
    void SetFrustum (float fNear, float fFar, float fLeft, float fRight,
        float fTop, float fBottom);
    void SetFrustumNear (float fNear);
    void SetFrustumFar (float fFar);
    void GetFrustum (float& rfNear, float& rfFar, float& rfLeft,
        float& rfRight, float& rfTop, float& rfBottom) const;
    float GetFrustumNear () const;
    float GetFrustumFar () const;
    float GetFrustumLeft () const;
    float GetFrustumRight () const;
    float GetFrustumTop () const;
    float GetFrustumBottom () const;
    float GetMaxCosSqrFrustumAngle () const;


	void SetViewPort (int nLeft, int nBottom, int nWidth, int nHeight);

	void SetTargetFrame( const Vector3f & ptEye, const Vector3f & vecLeft,
		const Vector3f & vecUp, const Vector3f & ptTarget );
 
	void SetLocation (const Vector3f& rkLocation);
    void SetAxes (const Vector3f& rkLeft, const Vector3f& rkUp,
        const Vector3f& rkDirection);
    void SetAxes (const Matrix3f& rkAxes);
    const Vector3f& GetLocation () const;
    const Vector3f& GetLeft () const;
    const Vector3f& GetUp () const;

	//! this direction is normalized.
    const Vector3f& GetDirection () const;

    // Mouse picking support.  The (x,y) input point is in left-handed screen
    // coordinates (what you usually read from the windowing system).  The
    // function returns 'true' if and only if the input point is located in
    // the current viewport.  When 'true', the origin and direction values
    // are valid and are in world coordinates.  The direction vector is unit
    // length.
    bool GetPickRay (int iX, int iY, int iWidth, int iHeight,
        Vector3f& rkOrigin, Vector3f& rkDirection) const;
    bool GetPickRayPerspective (int iX, int iY, int iWidth, int iHeight,
        Vector3f& rkOrigin, Vector3f& rkDirection) const;
    bool GetPickRayOrtho (int iX, int iY, int iWidth, int iHeight,
        Vector3f& rkOrigin, Vector3f& rkDirection) const;
        
    // Perspective (default) or orthogonal projection.
    void SetUsePerspective (bool bPerspective);
    bool GetUsePerspective () const;



/*
 * camera movement
 */

	void PanLateral( float fDistance );
	void PanVertical( float fDistance );

	void DollyZoom( float fDistance );

	void RotateLateral( float fAngle );
	void RotateVertical( float fAngle );


	void OrbitLateral( float fAngle );
	void OrbitVertical( float fAngle );


protected:

	// update ortho frustum for dolly zoom
	void OrthoDollyZoom( float fDistance );

	// dolly is hidden because it isn't really useful...
	void Dolly( float fDistance );

    // camera frame (world coordinates)
	// hidden because this camera has a target - use SetTargetFrame !!
    void SetFrame (const Vector3f& rkLocation, const Vector3f& rkLeft,
        const Vector3f& rkUp, const Vector3f& rkDirection);
    void SetFrame (const Vector3f& rkLocation, const Matrix3f& rkAxes);

	//! target point that camera is looking at ( and will orbit around )
	Vector3f m_ptTarget;

	// fov/aspect ratio perspective settings
	float m_fHorzFovDegrees;
	float m_fAspectRatio;
	float m_fNearZ;
	float m_fFarZ;

	//! stored OpenGL viewport
	unsigned int m_nViewport[4];


	// Wml OpenGLCamera implementation

    virtual void OnFrustumChange ();
    virtual void OnViewPortChange ();
    virtual void OnFrameChange ();

    float m_fWidth, m_fHeight;

	// Wml Camera implementation

	// view frustum
    float m_fFrustumN, m_fFrustumF;
    float m_fFrustumL, m_fFrustumR, m_fFrustumT, m_fFrustumB;

    float m_fOrthoFrustumL, m_fOrthoFrustumR, m_fOrthoFrustumT, m_fOrthoFrustumB;

    // camera frame (world location and coordinate axes)
    Vector3f m_kLocation, m_kLeft, m_kUp, m_kDirection;
	
	// perspective flag
    bool m_bUsePerspective;

    // Temporary values computed in OnFrustumChange that are needed if a
    // call is made to OnFrameChange.
    float m_afCoeffL[2], m_afCoeffR[2], m_afCoeffB[2], m_afCoeffT[2];
};


// inline functions

inline void ExtendedWmlCamera::Update()
{
    OnFrustumChange();
    OnViewPortChange();
    OnFrameChange();
}


inline const unsigned int * ExtendedWmlCamera::GetViewport() const
{
	return m_nViewport;
}


inline const Vector3f & ExtendedWmlCamera::GetTarget() const
{
	return m_ptTarget;
}

inline float ExtendedWmlCamera::GetHorzFovDegrees() const
{
	return m_fHorzFovDegrees;
}

inline float ExtendedWmlCamera::GetAspectRatio() const
{
	return m_fAspectRatio;
}

inline void ExtendedWmlCamera::GetFrameMatrix( Wml::Matrix3f & matrix ) const
{
	matrix.SetRow(0, GetLeft() * -1.0 );
	matrix.SetRow(1, GetUp());
	matrix.SetRow(2, GetDirection() * -1.0);
}


inline float ExtendedWmlCamera::GetFrustumNear () const
{
    return m_fFrustumN;
}

inline float ExtendedWmlCamera::GetFrustumFar () const
{
    return m_fFrustumF;
}

inline float ExtendedWmlCamera::GetFrustumLeft () const
{
    return m_fFrustumL;
}

inline float ExtendedWmlCamera::GetFrustumRight () const
{
    return m_fFrustumR;
}

inline float ExtendedWmlCamera::GetFrustumTop () const
{
    return m_fFrustumT;
}

inline float ExtendedWmlCamera::GetFrustumBottom () const
{
    return m_fFrustumB;
}

inline const Vector3f& ExtendedWmlCamera::GetLocation () const
{
    return m_kLocation;
}

inline const Vector3f& ExtendedWmlCamera::GetLeft () const
{
    return m_kLeft;
}

inline const Vector3f& ExtendedWmlCamera::GetUp () const
{
    return m_kUp;
}

inline const Vector3f& ExtendedWmlCamera::GetDirection () const
{
    return m_kDirection;
}


inline bool ExtendedWmlCamera::GetUsePerspective () const
{
    return m_bUsePerspective;
}



}  // end namespace Wml



#endif