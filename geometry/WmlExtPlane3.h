// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __WMLEXT_EXT_PLANE3_H__
#define __WMLEXT_EXT_PLANE3_H__
#include "config.h"
#include <Wm4Plane3.h>

namespace rms
{

template<class Real>
class  ExtPlane3 : public Wml::Plane3<Real>
{
public:
	ExtPlane3();
	ExtPlane3( const Wml::Vector3<Real> & vNormal, const Wml::Vector3<Real> & vOrigin );
	ExtPlane3( const Wml::Vector3<Real> & vPoint1, const Wml::Vector3<Real> & vPoint2, const Wml::Vector3<Real> & vPoint3 );

	Wml::Vector3<Real> & Origin() { return m_vOrigin; }
	const Wml::Vector3<Real> & Origin() const { return m_vOrigin; }

	Wml::Vector3<Real> ProjectPointToPlane( const Wml::Vector3<Real> & vPoint ) const;

	//! preserves distance from vPoint to plane origin - assumes plane normal is normalized!
	Wml::Vector3<Real> RotatePointIntoPlane( const Wml::Vector3<Real> & vPoint ) const;	

	Real IntersectRay( const Wml::Vector3<Real> & vOrigin,
					   const Wml::Vector3<Real> & vDirection ) const;

protected:
	Wml::Vector3<Real> m_vOrigin;
};

typedef ExtPlane3<float> ExtPlane3f;
typedef ExtPlane3<double> ExtPlane3d;


}	// namespace Wml



#endif	// __WMLEXT_EXT_PLANE3_H__