// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrLine3Lozenge3.h"
#include "Wm4DistLine3Rectangle3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrLine3Lozenge3<Real>::IntrLine3Lozenge3 (const Line3<Real>& rkLine,
    const Lozenge3<Real>& rkLozenge)
    :
    m_pkLine(&rkLine),
    m_pkLozenge(&rkLozenge)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Line3<Real>& IntrLine3Lozenge3<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
const Lozenge3<Real>& IntrLine3Lozenge3<Real>::GetLozenge () const
{
    return *m_pkLozenge;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Lozenge3<Real>::Test ()
{
    Real fDist = DistLine3Rectangle3<Real>(*m_pkLine,
        m_pkLozenge->Rectangle).Get();

    return fDist <= m_pkLozenge->Radius;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrLine3Lozenge3<float>;

template WM4_FOUNDATION_ITEM
class IntrLine3Lozenge3<double>;
//----------------------------------------------------------------------------
}
