// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Segment2<Real>::Segment2 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Segment2<Real>::Segment2 (const Vector2<Real>& rkOrigin,
    const Vector2<Real>& rkDirection, Real fExtent)
    :
    Origin(rkOrigin),
    Direction(rkDirection),
    Extent(fExtent)
{
}
//----------------------------------------------------------------------------
template <class Real>
Vector2<Real> Segment2<Real>::GetPosEnd () const
{
    return Origin + Direction*Extent;
}
//----------------------------------------------------------------------------
template <class Real>
Vector2<Real> Segment2<Real>::GetNegEnd () const
{
    return Origin - Direction*Extent;
}
//----------------------------------------------------------------------------
