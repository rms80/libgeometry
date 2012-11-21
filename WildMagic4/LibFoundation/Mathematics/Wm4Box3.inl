// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Box3<Real>::Box3 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Box3<Real>::Box3 (const Vector3<Real>& rkCenter, const Vector3<Real>* akAxis,
    const Real* afExtent)
    :
    Center(rkCenter)
{
    for (int i = 0; i < 3; i++)
    {
        Axis[i] = akAxis[i];
        Extent[i] = afExtent[i];
    }
}
//----------------------------------------------------------------------------
template <class Real>
Box3<Real>::Box3 (const Vector3<Real>& rkCenter, const Vector3<Real>& rkAxis0,
    const Vector3<Real>& rkAxis1, const Vector3<Real>& rkAxis2, Real fExtent0,
    Real fExtent1, Real fExtent2)
    :
    Center(rkCenter)
{
    Axis[0] = rkAxis0;
    Axis[1] = rkAxis1;
    Axis[2] = rkAxis2;
    Extent[0] = fExtent0;
    Extent[1] = fExtent1;
    Extent[2] = fExtent2;
}
//----------------------------------------------------------------------------
template <class Real>
void Box3<Real>::ComputeVertices (Vector3<Real> akVertex[8]) const
{
    Vector3<Real> akEAxis[3] =
    {
        Extent[0]*Axis[0],
        Extent[1]*Axis[1],
        Extent[2]*Axis[2]
    };

    akVertex[0] = Center - akEAxis[0] - akEAxis[1] - akEAxis[2];
    akVertex[1] = Center + akEAxis[0] - akEAxis[1] - akEAxis[2];
    akVertex[2] = Center + akEAxis[0] + akEAxis[1] - akEAxis[2];
    akVertex[3] = Center - akEAxis[0] + akEAxis[1] - akEAxis[2];
    akVertex[4] = Center - akEAxis[0] - akEAxis[1] + akEAxis[2];
    akVertex[5] = Center + akEAxis[0] - akEAxis[1] + akEAxis[2];
    akVertex[6] = Center + akEAxis[0] + akEAxis[1] + akEAxis[2];
    akVertex[7] = Center - akEAxis[0] + akEAxis[1] + akEAxis[2];
}
//----------------------------------------------------------------------------
