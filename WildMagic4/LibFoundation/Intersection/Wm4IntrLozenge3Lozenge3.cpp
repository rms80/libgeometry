// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrLozenge3Lozenge3.h"
#include "Wm4DistRectangle3Rectangle3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrLozenge3Lozenge3<Real>::IntrLozenge3Lozenge3 (
    const Lozenge3<Real>& rkLozenge0, const Lozenge3<Real>& rkLozenge1)
    :
    m_pkLozenge0(&rkLozenge0),
    m_pkLozenge1(&rkLozenge1)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Lozenge3<Real>& IntrLozenge3Lozenge3<Real>::GetLozenge0 () const
{
    return *m_pkLozenge0;
}
//----------------------------------------------------------------------------
template <class Real>
const Lozenge3<Real>& IntrLozenge3Lozenge3<Real>::GetLozenge1 () const
{
    return *m_pkLozenge1;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLozenge3Lozenge3<Real>::Test ()
{
    Real fDist = DistRectangle3Rectangle3<Real>(m_pkLozenge0->Rectangle,
        m_pkLozenge1->Rectangle).Get();
    Real fRSum = m_pkLozenge0->Radius + m_pkLozenge1->Radius;
    return fDist <= fRSum;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrLozenge3Lozenge3<float>;

template WM4_FOUNDATION_ITEM
class IntrLozenge3Lozenge3<double>;
//----------------------------------------------------------------------------
}
