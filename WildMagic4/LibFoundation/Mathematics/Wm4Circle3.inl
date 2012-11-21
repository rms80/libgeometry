// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Circle3<Real>::Circle3 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Circle3<Real>::Circle3 (const Vector3<Real>& rkCenter,
    const Vector3<Real>& rkU, const Vector3<Real>& rkV,
    const Vector3<Real>& rkN, Real fRadius)
    :
    Center(rkCenter),
    U(rkU),
    V(rkV),
    N(rkN)
{
    Radius = fRadius;
}
//----------------------------------------------------------------------------
