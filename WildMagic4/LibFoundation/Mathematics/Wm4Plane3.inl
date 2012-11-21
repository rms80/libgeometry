// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Plane3<Real>::Plane3 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Plane3<Real>::Plane3 (const Plane3& rkPlane)
    :
    Normal(rkPlane.Normal)
{
    Constant = rkPlane.Constant;
}
//----------------------------------------------------------------------------
template <class Real>
Plane3<Real>::Plane3 (const Vector3<Real>& rkNormal, Real fConstant)
    :
    Normal(rkNormal)
{
    Constant = fConstant;
}
//----------------------------------------------------------------------------
template <class Real>
Plane3<Real>::Plane3 (const Vector3<Real>& rkNormal, const Vector3<Real>& rkP)
    :
    Normal(rkNormal)
{
    Constant = rkNormal.Dot(rkP);
}
//----------------------------------------------------------------------------
template <class Real>
Plane3<Real>::Plane3 (const Vector3<Real>& rkP0, const Vector3<Real>& rkP1,
    const Vector3<Real>& rkP2)
{
    Vector3<Real> kEdge1 = rkP1 - rkP0;
    Vector3<Real> kEdge2 = rkP2 - rkP0;
    Normal = kEdge1.UnitCross(kEdge2);
    Constant = Normal.Dot(rkP0);
}
//----------------------------------------------------------------------------
template <class Real>
Plane3<Real>& Plane3<Real>::operator= (const Plane3& rkPlane)
{
    Normal = rkPlane.Normal;
    Constant = rkPlane.Constant;
    return *this;
}
//----------------------------------------------------------------------------
template <class Real>
Real Plane3<Real>::DistanceTo (const Vector3<Real>& rkP) const
{
    return Normal.Dot(rkP) - Constant;
}
//----------------------------------------------------------------------------
template <class Real>
int Plane3<Real>::WhichSide (const Vector3<Real>& rkQ) const
{
    Real fDistance = DistanceTo(rkQ);

    if (fDistance < (Real)0.0)
    {
        return -1;
    }

    if (fDistance > (Real)0.0)
    {
        return +1;
    }

    return 0;
}
//----------------------------------------------------------------------------
