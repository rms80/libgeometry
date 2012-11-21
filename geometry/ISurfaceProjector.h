// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#pragma once

#include "config.h"


namespace rms {


class ISurfaceProjector
{
public:
	virtual Wml::Vector2f ProjectToUV( const Wml::Vector3f & v3D, bool * bStatus = NULL ) = 0;
	virtual Wml::Vector3f ProjectTo3D( const Wml::Vector2f & vUV, Wml::Vector3f * pNormal = NULL, bool * bStatus = NULL ) = 0;
};

}		// end namespace rms