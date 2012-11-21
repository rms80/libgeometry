// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef _RMS_FRAME_H
#define _RMS_FRAME_H
#include "config.h"
#include <Wm4Vector3.h>
#include <Wm4Matrix3.h>
#include <Wm4Vector2.h>
#include <Wm4Matrix2.h>
#include "VectorUtil.h"


namespace rms
{

template<class Real>
class Frame3
{
public:
	//! @enum reference frame axes
	enum FrameAxis { AxisX = 0, AxisY, AxisZ };

	//! create a reference frame at a specific origin
	Frame3( const Wml::Vector3<Real> & Origin = Wml::Vector3<Real>::ZERO )
		: m_ptOrigin(Origin),  m_vScale((Real)1,(Real)1,(Real)1),
		  m_matFrame(false),  m_matFrameInverse(false) {}

	//! create an orthogonal frame at an origin with a given z axis vector
	Frame3( const Wml::Vector3<Real> & vOrigin, const Wml::Vector3<Real> & vZAxis );

	//! create frame at an origin with a given x/y/z vectors
	Frame3( const Wml::Vector3<Real> & vOrigin, const Wml::Vector3<Real> & vXAxis, const Wml::Vector3<Real> & vYAxis, const Wml::Vector3<Real> & vZAxis );

	//! copy a reference frame
	Frame3( const Frame3 & copy )
		: m_ptOrigin(copy.m_ptOrigin), m_vScale(copy.m_vScale), 
		  m_matFrame(copy.m_matFrame), m_matFrameInverse(copy.m_matFrameInverse) {}

	//! create frame from matrices and origin
	Frame3( const Wml::Matrix3<Real> & matFrame, const Wml::Matrix3<Real> & matFrameInverse, 
			const Wml::Vector3<Real> & ptOrigin )
			: m_ptOrigin( ptOrigin ), m_matFrame( matFrame ), m_matFrameInverse( matFrameInverse ) {}

	~Frame3();

	//! get the reference frame origin point
	Wml::Vector3<Real> & Origin()
		{ return m_ptOrigin; }

	//! get the reference frame origin point
	const Wml::Vector3<Real> & Origin() const
		{ return m_ptOrigin; }

	//! get the reference frame scaling values
	Wml::Vector3<Real> & Scale()
		{ return m_vScale; }

	//! get the reference frame scaling values
	const Wml::Vector3<Real> & Scale() const
		{ return m_vScale; }

	//! get the matrix that takes reference frame points into XYZ space
	const Wml::Matrix3<Real> & FrameMatrix() const
		{ return m_matFrame; }

	//! get the matrix that takes reference frame points into XYZ space
	const Wml::Matrix3<Real> &  FromFrameMatrix() const
		{ return m_matFrame; }

	//! get the inverse matrix that takes XYZ points into the reference frame
	const Wml::Matrix3<Real> & FrameInverse() const
		{ return m_matFrameInverse; }

	//! get the inverse matrix that takes XYZ points into the reference frame
	const Wml::Matrix3<Real> &  ToFrameMatrix() const
		{ return m_matFrameInverse; }

	//! get an axis of the reference frame
	Wml::Vector3<Real> Axis( FrameAxis eAxis ) const
		{ return m_matFrameInverse.GetRow( (unsigned int)eAxis ); }

	//! access the different axis
	Wml::Vector3<Real> X() const
		{ return m_matFrameInverse.GetRow( 0 ); }
	Wml::Vector3<Real> Y() const
		{ return m_matFrameInverse.GetRow( 1 ); }
	Wml::Vector3<Real> Z() const
		{ return m_matFrameInverse.GetRow( 2 ); }

	//! put a point in XYZ space into the frame orientation
	void ToFrameOrientation( Wml::Vector3<Real> & vec ) const
		{ vec = m_matFrameInverse * vec; }

	//! take a point in the frame and put it into standard XYZ space
	void FromFrameOrientation( Wml::Vector3<Real> & vec ) const
		{ vec = m_matFrame * vec; }


	//! tranform a world-space vector *into* frame-local coords
	void ToFrameLocal( Wml::Vector3<Real> & vec ) const
		{ vec = m_matFrameInverse * vec; }

	//! transform a frame-local vector into world-space coords
	void ToWorld( Wml::Vector3<Real> & vec ) const
		{ vec = m_matFrame * vec; }


	//! tranform a world-space vector *into* frame-local coords
	Wml::Vector3<Real> GetFrameLocal( const Wml::Vector3<Real> & vec ) const
		{ return m_matFrameInverse * vec; }

	//! transform a frame-local vector into world-space coords
	Wml::Vector3<Real> GetWorld( const Wml::Vector3<Real> & vec ) const
		{ return m_matFrame * vec; }


	//! tranform a world-space frame *into* frame-local coords
	Frame3<Real> GetFrameLocal( const Frame3<Real> & f ) const
	{	
		Wml::Vector3<Real> x = m_matFrameInverse * f.X(); 
		Wml::Vector3<Real> y = m_matFrameInverse * f.Y();
		Wml::Vector3<Real> z = m_matFrameInverse * f.Z();
		Frame3<Real> l(f.m_ptOrigin);
		l.SetFrame(x,y,z);
		return l;
	}

	//! transform a frame-local vector into world-space coords
	Frame3<Real> GetWorld( const Frame3<Real> & f ) const
	{	
		Wml::Vector3<Real> x = m_matFrame * f.X(); 
		Wml::Vector3<Real> y = m_matFrame * f.Y();
		Wml::Vector3<Real> z = m_matFrame * f.Z();
		Frame3<Real> w(f.m_ptOrigin);
		w.SetFrame(x,y,z);
		return w;
	}




	//! set the reference frame axes by calculating the axis that rotates the unit Z axis
	//! into vAxisZ (apply this matrix to unit x and y)
	void SetFrameZ( const Wml::Vector3<Real> & vAxisZ )
	{
		ComputeAlignZAxisMatrix( vAxisZ, m_matFrame );
		m_matFrameInverse = m_matFrame.Transpose();
	}

	//! same as above, but set the reverse matrix. So this doesn't actually align the current frame Z to vAxisZ. 
	//! (I can't figure out why this function is here...but it is called in a few places and I don't want to debug right now...)
	void SetFrameZ_Inverted( const Wml::Vector3<Real> & vAxisZ )
	{
		ComputeAlignZAxisMatrix( vAxisZ, m_matFrameInverse );
		m_matFrame = m_matFrameInverse.Transpose();
	}

	//! set the reference frame axes
	void SetFrame( const Wml::Vector3<Real> & vAxisX, const Wml::Vector3<Real> & vAxisY, const Wml::Vector3<Real> & vAxisZ )
	{
		Wml::Vector3<Real> row0(vAxisX);
		row0.Normalize();
		m_matFrameInverse.SetRow(0, row0);
		Wml::Vector3<Real> row1(vAxisY);
		row1.Normalize();
		m_matFrameInverse.SetRow(1, row1);
		Wml::Vector3<Real> row2(vAxisZ);
		row2.Normalize();
		m_matFrameInverse.SetRow(2, row2);
		m_matFrame = m_matFrameInverse.Transpose();
	}
	
	//! translate the reference frame origin
	void Translate( const Wml::Vector3<Real> & vTranslate, bool bRelative = true)
	{
		if (bRelative) 
			m_ptOrigin += vTranslate;
		else
			m_ptOrigin = vTranslate;
	}

	//! scale the reference frame
	void Scale( const Wml::Vector3<Real> & vScale, bool bRelative = true)
	{
		if (bRelative) {
			m_vScale.X() *= vScale.X();
			m_vScale.Y() *= vScale.Y();
			m_vScale.Z() *= vScale.Z();
		} else
			m_vScale = vScale;
	}

	//! rotate the reference frame
	void Rotate( const Wml::Matrix3<Real> & rotation, bool bReNormalize = true )
	{
		// [RMS] does not seem to work as expected...??

		Wml::Vector3<Real> rowX(rotation * m_matFrameInverse.GetRow(0));
		Wml::Vector3<Real> rowY(rotation * m_matFrameInverse.GetRow(1));
		Wml::Vector3<Real> rowZ(rotation * m_matFrameInverse.GetRow(2));

		if ( bReNormalize ) {
			Wml::Vector3<Real> vCrossY = rowZ.Cross(rowX);
			vCrossY.Normalize();
			rowY = ( vCrossY.Dot(rowY) < 0 ) ? -vCrossY : vCrossY;
			Wml::Vector3<Real> vCrossX = rowZ.Cross(rowY);
			vCrossX.Normalize();
			rowX = ( vCrossX.Dot(rowX) < 0 ) ? -vCrossX : vCrossX;
			Wml::Vector3<Real> vCrossZ = rowX.Cross(rowY);
			vCrossZ.Normalize();
			rowZ = ( vCrossZ.Dot(rowZ) < 0 ) ? -vCrossZ : vCrossZ;
		}

		m_matFrameInverse.SetRow(0, rowX);
		m_matFrameInverse.SetRow(1, rowY);
		m_matFrameInverse.SetRow(2, rowZ);

		m_matFrame = m_matFrameInverse.Transpose();
	}

	//! allow external setting of matrix...
	void SetRotation( const Wml::Matrix3<Real> & matRotate ) 
	{
		m_matFrameInverse = matRotate;
		m_matFrame = m_matFrameInverse.Transpose();
	}

	//! rotate this frame's Z vector into toFrame's Z vector
	void AlignZAxis( const Frame3<Real> & toFrame );

	//! rotate this frame's Z vector to new Z vector
	void AlignZAxis( const Wml::Vector3<Real> & toZ );

	//! compute matrix that rotates this frame into toFrame
	void ComputeAlignmentMatrix( const Frame3<Real> & toFrame, Wml::Matrix3<Real> & matRotate );

	//! make axes perpendicular. can "preserve" one axis by setting nPreserveAxis (0=X,1=Y,2=Z)
	void ReNormalize(int nPreserveAxis = -1);

protected:
	//! origin point of reference frame
	Wml::Vector3<Real> m_ptOrigin;

	//! scaling values along referenceframe axes
	Wml::Vector3<Real> m_vScale;

	//! reference frame matrix (columns are axes)
	Wml::Matrix3<Real> m_matFrame;

	//! inverse (transpose) reference frame matrix (rows are axes)
	Wml::Matrix3<Real> m_matFrameInverse;
};

typedef Frame3<float> Frame;
typedef Frame3<float> Frame3f;
typedef Frame3<double> Frame3d;









template<class Real>
class Frame2
{
public:
	//! @enum reference frame axes
	enum FrameAxis { AxisX = 0, AxisY };

	//! create a reference frame at a specific origin
	Frame2( const Wml::Vector2<Real> & Origin = Wml::Vector2<Real>::ZERO )
		: m_ptOrigin(Origin), m_matFrame(false),  m_matFrameInverse(false) {}

	//! copy a reference frame
	Frame2( const Frame2 & copy )
		: m_ptOrigin(copy.m_ptOrigin),
		  m_matFrame(copy.m_matFrame), m_matFrameInverse(copy.m_matFrameInverse) {}

	~Frame2();

	//! get the reference frame origin point
	Wml::Vector2<Real> & Origin()
		{ return m_ptOrigin; }

	//! get the reference frame origin point
	const Wml::Vector2<Real> & Origin() const
		{ return m_ptOrigin; }

	//! get an axis of the reference frame
	Wml::Vector2<Real> GetAxis( FrameAxis eAxis ) const
		{ return m_matFrameInverse.GetRow( (unsigned int)eAxis ); }

	//! access the different axis
	Wml::Vector2<Real> X() const
		{ return m_matFrameInverse.GetRow( 0 ); }
	Wml::Vector2<Real> Y() const
		{ return m_matFrameInverse.GetRow( 1 ); }

	//! get the matrix that takes reference frame points into XYZ space
	const Wml::Matrix2<Real> & FrameMatrix() const
		{ return m_matFrame; }

	//! get the inverse matrix that takes XYZ points into the reference frame
	const Wml::Matrix2<Real> & FrameInverse() const
		{ return m_matFrameInverse; }


	//! tranform a world-space vector *into* frame-local coords
	void ToFrameLocal( Wml::Vector2<Real> & vec ) const
		{ vec = m_matFrameInverse * vec; }

	//! transform a frame-local vector into world-space coords
	void ToWorld( Wml::Vector2<Real> & vec ) const
		{ vec = m_matFrame * vec; }

	//! set the reference frame axes
	void SetFrame( const Wml::Vector2<Real> & vAxisX, const Wml::Vector2<Real> & vAxisY )
	{
		Wml::Vector2<Real> row0(vAxisX);
		row0.Normalize();
		m_matFrameInverse.SetRow(0, row0);
		Wml::Vector2<Real> row1(vAxisY);
		row1.Normalize();
		m_matFrameInverse.SetRow(1, row1);
		m_matFrame = m_matFrameInverse.Transpose();
	}

	//! set the frame axes in the frame determined by rotating the Z axis of the 3D
	//  frame into the unit Z axis
	void SetFrame( const Frame3<Real> & v3DFrame );
	
	//! translate the reference frame origin
	void Translate( const Wml::Vector2<Real> & vTranslate, bool bRelative = true)
	{
		if (bRelative) 
			m_ptOrigin += vTranslate;
		else
			m_ptOrigin = vTranslate;
	}

	//! rotate the reference frame
	void Rotate( const Wml::Matrix2<Real> & rotation, bool bReNormalize = true )
	{
		// [RMS] does not seem to work as expected...??

		Wml::Vector2<Real> row0(rotation * m_matFrameInverse.GetRow(0));
		Wml::Vector2<Real> row1(rotation * m_matFrameInverse.GetRow(1));
		if (bReNormalize) {
			row0.Normalize();
			row1.Normalize();
		}
		m_matFrameInverse.SetRow(0, row0);
		m_matFrameInverse.SetRow(1, row1);

		m_matFrame = m_matFrameInverse.Transpose();
	}

	//! find the angle of rotation around Z that rotates this frame into toFrame
	Real ComputeFrameRotationAngle( const Frame2<Real> & toFrame );


protected:
	//! origin point of reference frame
	Wml::Vector2<Real> m_ptOrigin;

	//! reference frame matrix (columns are axes)
	Wml::Matrix2<Real> m_matFrame;

	//! inverse (transpose) reference frame matrix (rows are axes)
	Wml::Matrix2<Real> m_matFrameInverse;
};

typedef Frame2<float> Frame2f;
typedef Frame2<double> Frame2d;




}	// namespace rms

#endif // _RMS_FRAME_H