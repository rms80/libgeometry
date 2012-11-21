// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_VECTOR_UTIL_H__
#define __RMS_VECTOR_UTIL_H__
#include "config.h"
#include <Wm4Vector2.h>
#include <Wm4Vector3.h>
#include <Wm4Matrix3.h>
#include <Wm4AxisAlignedBox2.h>
#include <Wm4AxisAlignedBox3.h>

namespace rms
{

	/*
	 * if bToZAxis is false, compute matrix that rotates Z axis into vAlignWith
	 * if bToZAxis is true, compute matrix that rotates vAlignWith into Z axis
	 */
	template <class Real>
	void ComputeAlignZAxisMatrix( const Wml::Vector3<Real> & vAlignWith,
											  Wml::Matrix3<Real> & matrix, bool bToZAxis = false );

	template <class Real>
	void ComputeAlignAxisMatrix( const Wml::Vector3<Real> & vInitial,
											 const Wml::Vector3<Real> & vAlignWith, Wml::Matrix3<Real> & matrix );

	//! compute vectors in a plane perpendicular to vIn
	template <class Real>
	void ComputePerpVectors( const Wml::Vector3<Real> & vIn,
										 Wml::Vector3<Real> & vOut1, Wml::Vector3<Real> & vOut2,
										 bool bInIsNormalized = false);

	//! compute tangent vectors in plane perp to vNormal, using non-orthogonal vEstX as estimate of vOut1
	template <class Real>
	void ComputePerpVectors( const Wml::Vector3<Real> & vNormal,  const Wml::Vector3<Real> & vEstX,
							 Wml::Vector3<Real> & vOut1, Wml::Vector3<Real> & vOut2,
							 bool bInputIsNormalized = false);


	template <class Real>
	void ToGLMatrix( const Wml::Matrix3<Real> & matrix, Real glMatrix[16] );

	template <class Real>
	void Union( Wml::AxisAlignedBox3<Real> & dest, const Wml::AxisAlignedBox3<Real> & with );
	template <class Real>
	void Union( Wml::AxisAlignedBox3<Real> & dest, const Wml::Vector3<Real> & point );

	template <class Real>
	void Union( Wml::AxisAlignedBox2<Real> & dest, const Wml::AxisAlignedBox2<Real> & with );
	template <class Real>
	void Union( Wml::AxisAlignedBox2<Real> & dest, const Wml::Vector2<Real> & with );

	float Volume( const Wml::AxisAlignedBox3f & box );

	//! create box that contains sphere
	template <class Real>
	void FitAABox( const Wml::Vector3<Real> & vCenter, Real fRadius, Wml::AxisAlignedBox3<Real> & aaBox );

	bool Contained( const Wml::AxisAlignedBox3f & box, float fX, float fY, float fZ );
	bool Contained( const Wml::AxisAlignedBox2f & box, float fX, float fY );

	template <class Real>
	Wml::Vector3<Real> Center( const Wml::AxisAlignedBox3<Real> & box );
	template <class Real>
	Wml::Vector2<Real> Center( const Wml::AxisAlignedBox2<Real> & box );

	template <class Real>
	Real GetDimension( const Wml::AxisAlignedBox3<Real> & box, int nDimension );
	template <class Real>
	Real GetDimension( const Wml::AxisAlignedBox2<Real> & box, int nDimension );

	template <class Real>
	Real MaxDimension( const Wml::AxisAlignedBox3<Real> & box );
	template <class Real>
	Real MinDimension( const Wml::AxisAlignedBox3<Real> & box );

	template<class Real>
	void Translate( Wml::AxisAlignedBox2<Real> & box, Real fX, Real fY, bool bRelative );

	// casting operators for vector
	Wml::Vector3f VectorCastdf( const Wml::Vector3d & vec );
	Wml::Vector3d VectorCastfd( const Wml::Vector3f & vec );

	template<class Real>
	Wml::Vector2<Real> ToUV( const Wml::Vector3<Real> & vec, int nUIndex, int nVIndex );
	template<class Real>
	Wml::Vector3<Real> To3D( const Wml::Vector2<Real> & vec, int nUIndex, int nVIndex );

	template<class Real>
	Real Clamp( const Real & fValue, const Real & fMin, const Real & fMax );

	template<class Real>
	Wml::Vector2<Real> Normalize( const Wml::Vector2<Real> & v );
	template<class Real>
	Wml::Vector3<Real> Normalize( const Wml::Vector3<Real> & v );

	template<class Real>
	Wml::Vector3<Real> NCross( const Wml::Vector3<Real> & v1, const Wml::Vector3<Real> & v2 );


	template <class Real>
	Real VectorAngle( const Wml::Vector2<Real> & v1, const Wml::Vector2<Real> & v2 );
	template <class Real>
	Real VectorAngle( const Wml::Vector3<Real> & v1, const Wml::Vector3<Real> & v2 );

	template <class Real>
	Real VectorCot( const Wml::Vector3<Real> & v1, const Wml::Vector3<Real> & v2 );

	template <class Real>
	void BarycentricCoords( const Wml::Vector3<Real> & vTriVtx1, 
										const Wml::Vector3<Real> & vTriVtx2,
										const Wml::Vector3<Real> & vTriVtx3,
										const Wml::Vector3<Real> & vVertex,
										Real & fBary1, Real & fBary2, Real & fBary3 );

	template <class Real>
	Real Area( const Wml::Vector3<Real> & vTriVtx1, 
						   const Wml::Vector3<Real> & vTriVtx2,
						   const Wml::Vector3<Real> & vTriVtx3 );

	template <class Real>
	void BarycentricCoords( const Wml::Vector2<Real> & vTriVtx1, 
										const Wml::Vector2<Real> & vTriVtx2,
										const Wml::Vector2<Real> & vTriVtx3,
										const Wml::Vector2<Real> & vVertex,
										Real & fBary1, Real & fBary2, Real & fBary3 );

	template <class Real>
	Real Area( const Wml::Vector2<Real> & vTriVtx1, 
						   const Wml::Vector2<Real> & vTriVtx2,
						   const Wml::Vector2<Real> & vTriVtx3 );


	template <class Real>
	Wml::Vector3<Real> Normal( const Wml::Vector3<Real> & vTriVtx1, 
								const Wml::Vector3<Real> & vTriVtx2,
								const Wml::Vector3<Real> & vTriVtx3, Real * pArea = NULL );


	template <class Real>
	Wml::Vector3<Real> InterpNormal( const Wml::Vector3<Real> & vTriVtx1, 
									 const Wml::Vector3<Real> & vTriVtx2,
									 const Wml::Vector3<Real> & vTriVtx3, 
									 const Wml::Vector3<Real> & vTriNorm1, 
									 const Wml::Vector3<Real> & vTriNorm2,
									 const Wml::Vector3<Real> & vTriNorm3,
									 const Wml::Vector3<Real> & vPointInTri );
	


	//! This metric is from Texture Mapping Progressive Meshes, Sander et al, Siggraph 2001
	template <class Real>
	void StretchMetric1( const Wml::Vector3<Real> & vTriVtx1, 
									 const Wml::Vector3<Real> & vTriVtx2,
									 const Wml::Vector3<Real> & vTriVtx3,
									 const Wml::Vector2<Real> & vVtxParam1,
									 const Wml::Vector2<Real> & vVtxParam2,
									 const Wml::Vector2<Real> & vVtxParam3,
									 Real & MaxSV, Real & MinSV, Real & L2Norm, Real & LInfNorm );

	template <class Real>
	void StretchMetric3( const Wml::Vector3<Real> & vTriVtx1, 
									 const Wml::Vector3<Real> & vTriVtx2,
									 const Wml::Vector3<Real> & vTriVtx3,
									 const Wml::Vector3<Real> & vVtxParam1,
									 const Wml::Vector3<Real> & vVtxParam2,
									 const Wml::Vector3<Real> & vVtxParam3,
									 Real & MaxSV, Real & MinSV, Real & L2Norm, Real & LInfNorm );


	template <class Real>
	bool IsObtuse( const Wml::Vector2<Real> & v1, const Wml::Vector2<Real> & v2, const Wml::Vector2<Real> & v3 );
	template <class Real>
	bool IsObtuse( const Wml::Vector3<Real> & v1, const Wml::Vector3<Real> & v2, const Wml::Vector3<Real> & v3 );


}  // namespace Wml



#endif // __RMS_VECTOR_UTIL_H__