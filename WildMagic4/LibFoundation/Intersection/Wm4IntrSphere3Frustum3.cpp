// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrSphere3Frustum3.h"
#include "Wm4DistVector3Frustum3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrSphere3Frustum3<Real>::IntrSphere3Frustum3 (const Sphere3<Real>& rkSphere,
    const Frustum3<Real>& rkFrustum)
    :
    m_pkSphere(&rkSphere),
    m_pkFrustum(&rkFrustum)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Sphere3<Real>& IntrSphere3Frustum3<Real>::GetSphere () const
{
    return *m_pkSphere;
}
//----------------------------------------------------------------------------
template <class Real>
const Frustum3<Real>& IntrSphere3Frustum3<Real>::GetFrustum () const
{
    return *m_pkFrustum;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSphere3Frustum3<Real>::Test ()
{
    Real fDist = DistVector3Frustum3<Real>(m_pkSphere->Center,
        *m_pkFrustum).Get();
    return fDist <= m_pkSphere->Radius;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrSphere3Frustum3<float>;

template WM4_FOUNDATION_ITEM
class IntrSphere3Frustum3<double>;
//----------------------------------------------------------------------------
}
