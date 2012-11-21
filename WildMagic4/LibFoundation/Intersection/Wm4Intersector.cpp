// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4Intersector.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real, class TVector>
Intersector<Real,TVector>::Intersector ()
{
    m_fContactTime = (Real)0.0;
    m_iIntersectionType = IT_EMPTY;
}
//----------------------------------------------------------------------------
template <class Real, class TVector>
Intersector<Real,TVector>::~Intersector ()
{
}
//----------------------------------------------------------------------------
template <class Real, class TVector>
Real Intersector<Real,TVector>::GetContactTime () const
{
    return m_fContactTime;
}
//----------------------------------------------------------------------------
template <class Real, class TVector>
int Intersector<Real,TVector>::GetIntersectionType () const
{
    return m_iIntersectionType;
}
//----------------------------------------------------------------------------
template <class Real, class TVector>
bool Intersector<Real,TVector>::Test ()
{
    // stub for derived class
    assert(false);
    return false;
}
//----------------------------------------------------------------------------
template <class Real, class TVector>
bool Intersector<Real,TVector>::Find ()
{
    // stub for derived class
    assert(false);
    return false;
}
//----------------------------------------------------------------------------
template <class Real, class TVector>
bool Intersector<Real,TVector>::Test (Real, const TVector&, const TVector&)
{
    // stub for derived class
    assert(false);
    return false;
}
//----------------------------------------------------------------------------
template <class Real, class TVector>
bool Intersector<Real,TVector>::Find (Real, const TVector&, const TVector&)
{
    // stub for derived class
    assert(false);
    return false;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class Intersector<float,Vector2f>;

template WM4_FOUNDATION_ITEM
class Intersector<float,Vector3f>;

template WM4_FOUNDATION_ITEM
class Intersector<double,Vector2d>;

template WM4_FOUNDATION_ITEM
class Intersector<double,Vector3d>;
//----------------------------------------------------------------------------
}
