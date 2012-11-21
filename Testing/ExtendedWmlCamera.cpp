// Ryan Schmidt - rms@unknownroad.com
// Copyright (c) 2006-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt


#include "ExtendedWmlCamera.h"


#ifdef WIN_32
  #include "winstuff.h"
  #include "glut.h"
  #include <gl/GL.h>
  #include <gl/GLU.h>
#else
  #include <GL/gl.h>
  #include <GL/glu.h>
  #include <GL/glut.h>
#endif


using namespace Wml;
using namespace std;


ExtendedWmlCamera::ExtendedWmlCamera( )
	: m_kLocation(Vector3f::ZERO),
	  m_kLeft(Vector3f::UNIT_X),
	  m_kUp(Vector3f::UNIT_Y),
	  m_kDirection(Vector3f::UNIT_Z),
	  m_ptTarget(Vector3f::UNIT_Z)
{
	// [begin] from Wml::Camera
    m_fFrustumN = 1.0f;
    m_fFrustumF = 2.0f;
    m_fFrustumL = -0.5f;
    m_fFrustumR = 0.5f;
    m_fFrustumT = 0.5f;
    m_fFrustumB = -0.5f;
	m_bUsePerspective = true;
	// [end] from Wml::Camera

	m_fWidth = 0.0f;
	m_fHeight = 0.0f;

	m_fHorzFovDegrees = m_fAspectRatio = m_fNearZ = m_fFarZ = 0.0;
	memset(m_nViewport, 0, sizeof(unsigned int)*4);

	OnFrustumChange();
	OnViewPortChange();
	OnFrameChange();
}


ExtendedWmlCamera::ExtendedWmlCamera( float fWidth, float fHeight )
	: m_kLocation(Vector3f::ZERO),
	  m_kLeft(Vector3f::UNIT_X),
	  m_kUp(Vector3f::UNIT_Y),
	  m_kDirection(Vector3f::UNIT_Z),
	  m_ptTarget(Vector3f::UNIT_Z)
{
	// [begin] from Wml::Camera
    m_fFrustumN = 1.0f;
    m_fFrustumF = 2.0f;

    m_fFrustumL = -0.5f;
    m_fFrustumR = 0.5f;
    m_fFrustumT = 0.5f;
    m_fFrustumB = -0.5f;

    m_fOrthoFrustumL = -0.5f;
    m_fOrthoFrustumR = 0.5f;
    m_fOrthoFrustumT = 0.5f;
    m_fOrthoFrustumB = -0.5f;

	m_bUsePerspective = true;
	// [end] from Wml::Camera

	// [begin] from Wml::OpenGLCamera
	m_fWidth = fWidth;
	m_fHeight = fHeight;

	OnFrustumChange();
	OnViewPortChange();
	OnFrameChange();
	// [end] from Wml::OpenGLCamera

	m_fHorzFovDegrees = m_fAspectRatio = m_fNearZ = m_fFarZ = 0.0;
	memset(m_nViewport, 0, sizeof(unsigned int)*4);
}

void ExtendedWmlCamera::SetTargetFrame( const Vector3f & ptEye, const Vector3f & vecLeft,
									    const Vector3f & vecUp, const Vector3f & ptTarget )
{
	m_ptTarget = ptTarget;
	Vector3f vDirection( m_ptTarget - ptEye );
	vDirection.Normalize();
	SetFrame( ptEye, vecLeft, vecUp, vDirection );
}

void ExtendedWmlCamera::SetFrustum( float fUpFovDegrees, float fAspectRatio, float fNearZ, float fFarZ )
{
    float fHalfAngleRadians = 0.5f*fUpFovDegrees*Mathf::DEG_TO_RAD;
    m_fFrustumT = fNearZ*Mathf::Tan(fHalfAngleRadians);
    m_fFrustumB = -m_fFrustumT;
    m_fFrustumR = fAspectRatio*m_fFrustumT;
    m_fFrustumL = -m_fFrustumR;

	// set ortho frustum to same...
    m_fOrthoFrustumT = m_fFrustumT;
    m_fOrthoFrustumB = m_fFrustumB;
    m_fOrthoFrustumR = m_fFrustumR;
    m_fOrthoFrustumL = m_fFrustumL;

    m_fFrustumN = fNearZ;
    m_fFrustumF = fFarZ;
    OnFrustumChange();

	m_fHorzFovDegrees = fUpFovDegrees;
	m_fAspectRatio = fAspectRatio;
	m_fNearZ = fNearZ;
	m_fFarZ = fFarZ;
}



void ExtendedWmlCamera::PanLateral( float fDistance )
{
	Vector3f left(GetLeft());
	left.Normalize();
	Vector3f translate( left * fDistance );

	SetTargetFrame( GetLocation() + translate, GetLeft(), GetUp(), m_ptTarget + translate );
}


void ExtendedWmlCamera::PanVertical( float fDistance )
{
	Vector3f up(GetUp());
	up.Normalize();
	Vector3f translate( up * fDistance );

	SetTargetFrame( GetLocation() + translate, GetLeft(), GetUp(), m_ptTarget + translate );
}


void ExtendedWmlCamera::Dolly( float fDistance )
{
	Vector3f in(GetDirection());
	in.Normalize();
	Vector3f translate( in * fDistance );

	SetTargetFrame( GetLocation() + translate, GetLeft(), GetUp(), m_ptTarget + translate );
}


void ExtendedWmlCamera::DollyZoom( float fDistance )
{
	Vector3f in(GetDirection());
	in.Normalize();
	Vector3f translate( in * fDistance );

	// if new camera point is past target point, ignore this dolly request
	Vector3f newEye = GetLocation() + translate;
	Vector3f normDir = GetDirection();
	normDir.Normalize();
	if ( newEye.Dot(normDir) < m_ptTarget.Dot(normDir) ) {

		if (!m_bUsePerspective)
			OrthoDollyZoom( fDistance );

		SetTargetFrame( GetLocation() + translate, GetLeft(), GetUp(), m_ptTarget );
	}
}

void ExtendedWmlCamera::OrthoDollyZoom( float fDistance )
{
	Vector3f in(GetDirection());
	in.Normalize();
	Vector3f translate( in * fDistance );
	Vector3f vNewEye = GetLocation() + translate;

    float fHalfAngleRadians = 0.5f*m_fHorzFovDegrees*Mathf::DEG_TO_RAD;
	float fHorzDelta = -fDistance * Mathf::Tan(fHalfAngleRadians);

    m_fOrthoFrustumT += fHorzDelta;
    m_fOrthoFrustumB = -m_fOrthoFrustumT;
    m_fOrthoFrustumR += m_fAspectRatio*fHorzDelta;
    m_fOrthoFrustumL = -m_fOrthoFrustumR;

}

void ExtendedWmlCamera::RotateLateral( float fAngle )
{
	Matrix3f rotate;
	rotate.FromAxisAngle( GetUp(), fAngle );

	Vector3f direction( m_ptTarget - GetLocation () );

	Vector3f newLeft( rotate * GetLeft() );
	newLeft.Normalize();

	SetTargetFrame( GetLocation(), newLeft, GetUp(), 
		GetLocation() + ( rotate * direction ) );
}


void ExtendedWmlCamera::RotateVertical( float fAngle )
{
	Matrix3f rotate;
	rotate.FromAxisAngle( GetLeft(), fAngle );

	Vector3f direction( m_ptTarget - GetLocation() );
	Vector3f newUp( rotate * GetUp() );
	newUp.Normalize();

	SetTargetFrame( GetLocation(), GetLeft(), newUp, 
		GetLocation() + ( rotate * direction ) );
}


void ExtendedWmlCamera::OrbitLateral( float fAngle )
{
	Matrix3f rotate;
	rotate.FromAxisAngle( GetUp(), fAngle );
	
	Vector3f direction( GetLocation() - m_ptTarget );
	Vector3f newLeft( rotate * GetLeft() );
	newLeft.Normalize();

	SetTargetFrame( m_ptTarget + (rotate * direction), newLeft, 
		GetUp(), m_ptTarget );
}


void ExtendedWmlCamera::OrbitVertical( float fAngle )
{
	Matrix3f rotate;
	rotate.FromAxisAngle( GetLeft(), fAngle );
	
	Vector3f direction( GetLocation() - m_ptTarget );
	Vector3f newUp( rotate * GetUp() );
	newUp.Normalize();

	SetTargetFrame( m_ptTarget + (rotate * direction), GetLeft(), 
		newUp, m_ptTarget );
}




void ExtendedWmlCamera::SetUsePerspective (bool bPerspective)
{
	if ( bPerspective == false && m_bUsePerspective == true ) {
		// update ortho frustum...
		m_fOrthoFrustumT = m_fFrustumT;
		m_fOrthoFrustumB = m_fFrustumB;
		m_fOrthoFrustumR = m_fFrustumR;
		m_fOrthoFrustumL = m_fFrustumL;

		float fDist =(m_ptTarget - GetLocation()).Length();
		OrthoDollyZoom(-fDist);
	}

    m_bUsePerspective = bPerspective;
}


// [begin] from Wml::OpenGLCamera



void ExtendedWmlCamera::OnFrustumChange ()
{
	float fFrustumR, fFrustumL, fFrustumB, fFrustumT;
	if ( m_bUsePerspective ) {
		fFrustumR = m_fFrustumR;
		fFrustumL = m_fFrustumL;
		fFrustumB = m_fFrustumB;
		fFrustumT = m_fFrustumT;
	} else {
		fFrustumR = m_fOrthoFrustumR;
		fFrustumL = m_fOrthoFrustumL;
		fFrustumB = m_fOrthoFrustumB;
		fFrustumT = m_fOrthoFrustumT;
	}

	// recalculate frustum
	float fNSqr = m_fFrustumN*m_fFrustumN;
    float fLSqr = fFrustumL*fFrustumL;
    float fRSqr = fFrustumR*fFrustumR;
    float fBSqr = fFrustumB*fFrustumB;
    float fTSqr = fFrustumT*fFrustumT;

    float fInvLength = 1.0f/Mathf::Sqrt(fNSqr + fLSqr);
    m_afCoeffL[0] = m_fFrustumN*fInvLength;
    m_afCoeffL[1] = -fFrustumL*fInvLength;

    fInvLength = 1.0f/Mathf::Sqrt(fNSqr + fRSqr);
    m_afCoeffR[0] = -m_fFrustumN*fInvLength;
    m_afCoeffR[1] = fFrustumR*fInvLength;

    fInvLength = 1.0f/Mathf::Sqrt(fNSqr + fBSqr);
    m_afCoeffB[0] = m_fFrustumN*fInvLength;
    m_afCoeffB[1] = -fFrustumB*fInvLength;

    fInvLength = 1.0f/Mathf::Sqrt(fNSqr + fTSqr);
    m_afCoeffT[0] = -m_fFrustumN*fInvLength;
    m_afCoeffT[1] = fFrustumT*fInvLength;

    // set projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if ( GetUsePerspective() ) {
        glFrustum(fFrustumL,fFrustumR,fFrustumB,fFrustumT,m_fFrustumN,m_fFrustumF);
    } else {
        glOrtho(fFrustumL,fFrustumR,fFrustumB,fFrustumT,m_fFrustumN,m_fFrustumF);
    }
}


void ExtendedWmlCamera::OnViewPortChange ()
{
    // set view port
    glViewport(m_nViewport[0], m_nViewport[1], m_nViewport[2], m_nViewport[3]);
}


void ExtendedWmlCamera::OnFrameChange ()
{
    // set view matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    Vector3f kLookAt = m_kLocation + m_kDirection;
    gluLookAt(m_kLocation.X(),m_kLocation.Y(),m_kLocation.Z(),kLookAt.X(),
        kLookAt.Y(),kLookAt.Z(),m_kUp.X(),m_kUp.Y(),m_kUp.Z());

    // The Wild Magic matrix that corresponds to this operation is as follows.
    // Given location E = (Ex,Ey,Ez), left L = (Lx,Ly,Lz), up U = (Ux,Uy,Uz),
    // and direction D = (Dx,Dy,Dz).
    //
    // +-                         -+
    // | -Lx  -Ly  -Lz  -Dot(-L,E) |
    // | +Ux  +Uy  +Uz  -Dot(+U,E) |
    // | -Dx  -Dy  -Dz  -Dot(-D,E) |
    // |  0    0    0        1     |
    // +-                         -+
    //
    // The gluLookAt call can be replaced by
    // float M[16];
    // M[0 ] = -Lx;       M[ 1] = +Ux;        M[ 2] = -Dx;       M[ 3] = 0.0f;
    // M[4 ] = -Ly;       M[ 5] = +Uy;        M[ 6] = -Dy;       M[ 7] = 0.0f;
    // M[8 ] = -Lz;       M[ 9] = +Uz;        M[10] = -Dz;       M[11] = 0.0f;
    // M[12] = Dot(L,E);  M[13] = -Dot(U,E);  M[14] = Dot(D,E);  M[15] = 1.0f;
    // glMatrixMultf(M);
}


// [end] from Wml::OpenGLCamera


// [begin] from Wml::Camera

void ExtendedWmlCamera::SetFrustum (float fNear, float fFar, float fLeft, float fRight, float fTop, float fBottom)
{
    m_fFrustumN = fNear;
    m_fFrustumF = fFar;

    m_fFrustumL = fLeft;
    m_fFrustumR = fRight;
    m_fFrustumT = fTop;
    m_fFrustumB = fBottom;

    m_fOrthoFrustumL = fLeft;
    m_fOrthoFrustumR = fRight;
    m_fOrthoFrustumT = fTop;
    m_fOrthoFrustumB = fBottom;

    OnFrustumChange();
}


void ExtendedWmlCamera::SetFrustumNear (float fNear)
{
    m_fFrustumN = fNear;
    OnFrustumChange();
}


void ExtendedWmlCamera::SetFrustumFar (float fFar)
{
    m_fFrustumF = fFar;
    OnFrustumChange();
}


void ExtendedWmlCamera::GetFrustum (float& rfNear, float& rfFar, float& rfLeft,
    float& rfRight, float& rfTop, float& rfBottom) const
{
    rfNear = m_fFrustumN;
    rfFar = m_fFrustumF;
    rfLeft = m_fFrustumL;
    rfRight = m_fFrustumR;
    rfTop = m_fFrustumT;
    rfBottom = m_fFrustumB;
}


float ExtendedWmlCamera::GetMaxCosSqrFrustumAngle () const
{
    // Compute (cos(A))^2 where A is the largest angle between the frustum
    // axis direction D and the four edges of the frustum that lie along the
    // rays from the frustum origin.

    float fNSqr = m_fFrustumN*m_fFrustumN;

    float fDenom = fNSqr;
    if ( Mathf::FAbs(m_fFrustumL) >= Mathf::FAbs(m_fFrustumR) )
    {
        fDenom += m_fFrustumL*m_fFrustumL;
        if ( Mathf::FAbs(m_fFrustumB) >= Mathf::FAbs(m_fFrustumT) )
            fDenom += m_fFrustumB*m_fFrustumB;
        else
            fDenom += m_fFrustumT*m_fFrustumT;
    }
    else
    {
        fDenom += m_fFrustumR*m_fFrustumR;
        if ( Mathf::FAbs(m_fFrustumB) >= Mathf::FAbs(m_fFrustumT) )
            fDenom += m_fFrustumB*m_fFrustumB;
        else
            fDenom += m_fFrustumT*m_fFrustumT;
    }

    return fNSqr/fDenom;
}


void ExtendedWmlCamera::SetViewPort (int nLeft, int nBottom, int nWidth, int nHeight)
{
	m_nViewport[0] = nLeft;
	m_nViewport[1] = nBottom;
	m_nViewport[2] = nWidth;
	m_nViewport[3] = nHeight;

    OnViewPortChange();
}

void ExtendedWmlCamera::SetFrame (const Vector3f& rkLocation, const Vector3f& rkLeft,
    const Vector3f& rkUp, const Vector3f& rkDirection)
{
    m_kLocation = rkLocation;
    m_kLeft = rkLeft;
    m_kUp = rkUp;
    m_kDirection = rkDirection;
    OnFrameChange();
}


void ExtendedWmlCamera::SetFrame (const Vector3f& rkLocation, const Matrix3f& rkAxes)
{
    m_kLocation = rkLocation;
    m_kLeft = rkAxes.GetColumn(0);
    m_kUp = rkAxes.GetColumn(1);
    m_kDirection = rkAxes.GetColumn(2);
    OnFrameChange();
}


void ExtendedWmlCamera::SetLocation (const Vector3f& rkLocation)
{
    m_kLocation = rkLocation;
	m_kDirection = m_ptTarget - m_kLocation;
	m_kDirection.Normalize();
    OnFrameChange();
}


void ExtendedWmlCamera::SetAxes (const Vector3f& rkLeft, const Vector3f& rkUp, const Vector3f& rkDirection)
{
    m_kLeft = rkLeft;
    m_kUp = rkUp;
    m_kDirection = rkDirection;
    OnFrameChange();
}


void ExtendedWmlCamera::SetAxes (const Matrix3f& rkAxes)
{
    m_kLeft = rkAxes.GetColumn(0);
    m_kUp = rkAxes.GetColumn(1);
    m_kDirection = rkAxes.GetColumn(2);
    OnFrameChange();
}

bool ExtendedWmlCamera::GetPickRay (int iX, int iY, int iWidth, int iHeight,
    Vector3f& rkOrigin, Vector3f& rkDirection) const
{
	if ( m_bUsePerspective )
		return GetPickRayPerspective(iX, iY, iWidth, iHeight, rkOrigin, rkDirection );
	else
		return GetPickRayOrtho(iX, iY, iWidth, iHeight, rkOrigin, rkDirection );
}

bool ExtendedWmlCamera::GetPickRayPerspective (int iX, int iY, int iWidth, int iHeight,
    Vector3f& rkOrigin, Vector3f& rkDirection) const
{
	if (!m_bUsePerspective)
		return GetPickRayOrtho( iX, iY, iWidth, iHeight, rkOrigin, rkDirection );

    float fPortX = ((float)(iWidth-1-iX))/(float)iWidth;
    if ( fPortX < 0.0 || fPortX > 1.0 )
        return false;

    float fPortY = ((float)(iHeight-1-iY))/(float)iHeight;
    if ( fPortY < 0.0 || fPortY > 1.0 )
        return false;

    float fViewX = (1.0f-fPortX)*m_fFrustumL + fPortX*m_fFrustumR;
    float fViewY = (1.0f-fPortY)*m_fFrustumB + fPortY*m_fFrustumT;

    rkOrigin = m_kLocation;
    rkDirection = m_fFrustumN*m_kDirection+fViewX*m_kLeft+fViewY*m_kUp;
    rkDirection.Normalize();

    return true;
}


bool ExtendedWmlCamera::GetPickRayOrtho (int iX, int iY, int iWidth, int iHeight,
    Vector3f& rkOrigin, Vector3f& rkDirection) const
{
    float fPortX = ((float)(iWidth-1-iX))/(float)iWidth;
    if ( fPortX < 0.0 || fPortX > 1.0 )
        return false;
    float fPortY = ((float)(iHeight-1-iY))/(float)iHeight;
    if ( fPortY < 0.0 || fPortY > 1.0 )
        return false;
    float fViewX = (1.0f-fPortX)*m_fOrthoFrustumL + fPortX*m_fOrthoFrustumR;
    float fViewY = (1.0f-fPortY)*m_fOrthoFrustumB + fPortY*m_fOrthoFrustumT;
    rkOrigin = m_kLocation + m_fFrustumN*m_kDirection +
        fViewX*m_kLeft + fViewY*m_kUp;
    rkDirection = m_kDirection;
    return true;
}

