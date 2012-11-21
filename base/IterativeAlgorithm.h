// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_ITERATIVE_ALGORITHM_H__
#define __RMS_ITERATIVE_ALGORITHM_H__

#include "config.h"

namespace rms {

class IterationCallback
{
public:
	enum State {
		Continue,
		Pause,
		Terminate
	};

	virtual State CheckIteration() = 0;
};



class DefaultIterationCallback : public IterationCallback
{
public:
	DefaultIterationCallback() {}
	virtual State CheckIteration() { return IterationCallback::Continue; }
};

} // namespace rms



#endif // __RMS_ITERATIVE_MESH_OPTIMIZER_H__