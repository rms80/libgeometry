// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "WmlExtPlane3.h"

#include <limits>

using namespace rms;

template<class Real>
ExtPlane3<Real>::ExtPlane3() 
	: Wml::Plane3<Real>(), m_vOrigin(Wml::Vector3<Real>::ZERO) 
{
}

template<class Real>
ExtPlane3<Real>::ExtPlane3( const Wml::Vector3<Real> & vNormal, const Wml::Vector3<Real> & vOrigin ) 
	: Wml::Plane3<Real>( vNormal, vOrigin ), m_vOrigin(vOrigin) 
{
}

template<class Real>
ExtPlane3<Real>::ExtPlane3( const Wml::Vector3<Real> & vPoint1, 
						    const Wml::Vector3<Real> & vPoint2, 
							const Wml::Vector3<Real> & vPoint3 )
							: Wml::Plane3<Real>( vPoint1, vPoint2, vPoint3 )
{
	this->Normal.Normalize();
	m_vOrigin = vPoint1;
}


template<class Real>
Wml::Vector3<Real> ExtPlane3<Real>::ProjectPointToPlane( const Wml::Vector3<Real> & vPoint ) const 
{
	return vPoint - DistanceTo(vPoint) * this->Normal;
}

template<class Real>
Wml::Vector3<Real> ExtPlane3<Real>::RotatePointIntoPlane( const Wml::Vector3<Real> & vPoint ) const
{
	// find projected point in plane
	Wml::Vector3<Real> vProjected = ProjectPointToPlane( vPoint );

	// find angle between projected point and original point
	Wml::Vector3<Real> vOrig( vPoint - Origin() );
	Real fOrigLen = vOrig.Length();
	if ( fOrigLen < Wml::Math<Real>::EPSILON )
		return Origin();

	Wml::Vector3<Real> vProj( vProjected - Origin() );
	Real fProjLen = vProj.Length();
	if ( fProjLen < Wml::Math<Real>::EPSILON )
		return Origin();

	Real fScale = fOrigLen / vProj.Length();
	return Origin() + vProj * fScale;

	// [RMS] following code uses rotation to do the same thing, just less efficiently
	//   (but conceivably a bit more robustly...)

	//Real CosTheta = vOrig.Dot(vProj) / (vOrig.Length() * vProj.Length());
	//Real Theta = std::acos( CosTheta );

	//// compute rotation axis
	//Wml::Vector3<Real> vAxis( vOrig.Cross(vProj) );
	//vAxis.Normalize();

	//// rotate point
	//Wml::Matrix3<Real> matRotate( vAxis, Theta );
	//return (matRotate * vOrig) + plane.Origin();
}


template<class Real>
Real ExtPlane3<Real>::IntersectRay( const Wml::Vector3<Real> & vOrigin,
								    const Wml::Vector3<Real> & vDirection ) const
{
	Wml::Vector3<Real> vN(this->Normal);
	Real fDenom = vDirection.Dot(vN);
	if ( fDenom == 0 )
		return std::numeric_limits<Real>::max();
	return (this->Constant - vOrigin.Dot(vN)) / fDenom; 
}



//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
namespace Wml
{
template class ExtPlane3<float>;
template class ExtPlane3<double>;
}
//----------------------------------------------------------------------------
