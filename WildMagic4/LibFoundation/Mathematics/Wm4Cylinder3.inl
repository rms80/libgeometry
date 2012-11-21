// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Cylinder3<Real>::Cylinder3 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Cylinder3<Real>::Cylinder3 (const Segment3<Real>& rkSegment, Real fHeight,
    Real fRadius)
    :
    Segment(rkSegment)
{
    Height = fHeight;
    Radius = fRadius;
}
//----------------------------------------------------------------------------
