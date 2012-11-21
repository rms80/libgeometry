// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Segment3<Real>::Segment3 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Segment3<Real>::Segment3 (const Vector3<Real>& rkOrigin,
    const Vector3<Real>& rkDirection, Real fExtent)
    :
    Origin(rkOrigin),
    Direction(rkDirection),
    Extent(fExtent)
{
}
//----------------------------------------------------------------------------
template <class Real>
Segment3<Real>::Segment3 (const Vector3<Real>& rkEnd0,
    const Vector3<Real>& rkEnd1)
{
    Origin = ((Real)0.5)*(rkEnd0 + rkEnd1);
    Direction = rkEnd1 - rkEnd0;
    Extent = ((Real)0.5)*Direction.Normalize();
}
//----------------------------------------------------------------------------
template <class Real>
Vector3<Real> Segment3<Real>::GetPosEnd () const
{
    return Origin + Extent*Direction;
}
//----------------------------------------------------------------------------
template <class Real>
Vector3<Real> Segment3<Real>::GetNegEnd () const
{
    return Origin - Extent*Direction;
}
//----------------------------------------------------------------------------
