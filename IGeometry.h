// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"

#include <Wm4Vector3.h>
#include <vector>

namespace rms {

/*
 * currently assumes T is something we can use to index into a vector (hence MaxID() function)
 */
template <class T>
class INeighbourSource
{
public:
	virtual void GetNeighbours( T nID, std::vector<T> & vNbrIDs ) = 0;
	virtual T MaxID() = 0;

	virtual bool IsOrdered() { return false; }
};


/*
 * currently assumes T is something we can use to index into a vector (hence MaxID() function)
 */
template<class T>
class IPositionSource {
public:
	virtual void GetPosition( T nID, Wml::Vector3f & vPosition, Wml::Vector3f * pNormal = NULL ) = 0;
	virtual T MaxID() = 0;
};



} // end namespace rms