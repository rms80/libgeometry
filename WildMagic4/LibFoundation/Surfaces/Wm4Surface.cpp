// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4Surface.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
Surface<Real>::Surface ()
{
}
//----------------------------------------------------------------------------
template <class Real>
Surface<Real>::~Surface ()
{
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class Surface<float>;

template WM4_FOUNDATION_ITEM
class Surface<double>;
//----------------------------------------------------------------------------
}
