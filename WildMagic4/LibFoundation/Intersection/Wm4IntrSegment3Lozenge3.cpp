// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrSegment3Lozenge3.h"
#include "Wm4DistSegment3Rectangle3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrSegment3Lozenge3<Real>::IntrSegment3Lozenge3 (
    const Segment3<Real>& rkSegment, const Lozenge3<Real>& rkLozenge)
    :
    m_pkSegment(&rkSegment),
    m_pkLozenge(&rkLozenge)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Segment3<Real>& IntrSegment3Lozenge3<Real>::GetSegment () const
{
    return *m_pkSegment;
}
//----------------------------------------------------------------------------
template <class Real>
const Lozenge3<Real>& IntrSegment3Lozenge3<Real>::GetLozenge () const
{
    return *m_pkLozenge;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSegment3Lozenge3<Real>::Test ()
{
    Real fDist = DistSegment3Rectangle3<Real>(*m_pkSegment,
        m_pkLozenge->Rectangle).Get();

    return fDist <= m_pkLozenge->Radius;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrSegment3Lozenge3<float>;

template WM4_FOUNDATION_ITEM
class IntrSegment3Lozenge3<double>;
//----------------------------------------------------------------------------
}
