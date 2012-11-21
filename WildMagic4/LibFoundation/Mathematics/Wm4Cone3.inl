// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Cone3<Real>::Cone3 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Cone3<Real>::Cone3 (const Vector3<Real>& rkVertex,
    const Vector3<Real>& rkAxis, Real fAngle)
    :
    Vertex(rkVertex),
    Axis(rkAxis)
{
    CosAngle = Math<Real>::Cos(fAngle);
    SinAngle = Math<Real>::Sin(fAngle);
}
//----------------------------------------------------------------------------
template <class Real>
Cone3<Real>::Cone3 (const Vector3<Real>& rkVertex,
    const Vector3<Real>& rkAxis, Real fCosAngle, Real fSinAngle)
    :
    Vertex(rkVertex),
    Axis(rkAxis)
{
    CosAngle = fCosAngle;
    SinAngle = fSinAngle;
}
//----------------------------------------------------------------------------
