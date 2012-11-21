// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrRay3Lozenge3.h"
#include "Wm4DistRay3Rectangle3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrRay3Lozenge3<Real>::IntrRay3Lozenge3 (const Ray3<Real>& rkRay,
    const Lozenge3<Real>& rkLozenge)
    :
    m_pkRay(&rkRay),
    m_pkLozenge(&rkLozenge)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Ray3<Real>& IntrRay3Lozenge3<Real>::GetRay () const
{
    return *m_pkRay;
}
//----------------------------------------------------------------------------
template <class Real>
const Lozenge3<Real>& IntrRay3Lozenge3<Real>::GetLozenge () const
{
    return *m_pkLozenge;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrRay3Lozenge3<Real>::Test ()
{
    Real fDist = DistRay3Rectangle3<Real>(*m_pkRay,
        m_pkLozenge->Rectangle).Get();

    return fDist <= m_pkLozenge->Radius;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrRay3Lozenge3<float>;

template WM4_FOUNDATION_ITEM
class IntrRay3Lozenge3<double>;
//----------------------------------------------------------------------------
}
