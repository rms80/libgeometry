// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_MESH_PROJECTION_H__
#define __RMS_MESH_PROJECTION_H__

#include <GSurface.h>
#include <set>

namespace rms {

class MeshProjection
{
public:
	MeshProjection(GSurface * pSurface);
	~MeshProjection(void);

	bool GetUVFromNearest( Wml::Vector3f vPoint, Wml::Vector2f & vUV );

protected:
	GSurface * m_pSurface;
};


} // end namespace rmsmesh


#endif  // __RMS_MESH_PROJECTION_H__