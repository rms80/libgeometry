// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrSphere3Cone3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrSphere3Cone3<Real>::IntrSphere3Cone3 (const Sphere3<Real>& rkSphere,
    const Cone3<Real>& rkCone)
    :
    m_pkSphere(&rkSphere),
    m_pkCone(&rkCone)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Sphere3<Real>& IntrSphere3Cone3<Real>::GetSphere () const
{
    return *m_pkSphere;
}
//----------------------------------------------------------------------------
template <class Real>
const Cone3<Real>& IntrSphere3Cone3<Real>::GetCone () const
{
    return *m_pkCone;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSphere3Cone3<Real>::Test ()
{
    Real fInvSin = ((Real)1.0)/m_pkCone->SinAngle;
    Real fCosSqr = m_pkCone->CosAngle*m_pkCone->CosAngle;

    Vector3<Real> kCmV = m_pkSphere->Center - m_pkCone->Vertex;
    Vector3<Real> kD = kCmV + (m_pkSphere->Radius*fInvSin)*m_pkCone->Axis;
    Real fDSqrLen = kD.SquaredLength();
    Real fE = kD.Dot(m_pkCone->Axis);
    if (fE > (Real)0.0 && fE*fE >= fDSqrLen*fCosSqr)
    {
        Real fSinSqr = m_pkCone->SinAngle*m_pkCone->SinAngle;
        fDSqrLen = kCmV.SquaredLength();
        fE = -kCmV.Dot(m_pkCone->Axis);
        if (fE > (Real)0.0 && fE*fE >= fDSqrLen*fSinSqr)
        {
            Real fRSqr = m_pkSphere->Radius*m_pkSphere->Radius;
            return fDSqrLen <= fRSqr;
        }
        return true;
    }
    return false;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrSphere3Cone3<Real>::Find ()
{
    // test if cone vertex is in sphere
    Vector3<Real> kDiff = m_pkSphere->Center - m_pkCone->Vertex;
    Real fRSqr = m_pkSphere->Radius*m_pkSphere->Radius;
    Real fLSqr = kDiff.SquaredLength();
    if (fLSqr <= fRSqr)
    {
        return true;
    }

    // test if sphere center is in cone
    Real fDot = kDiff.Dot(m_pkCone->Axis);
    Real fDotSqr = fDot*fDot;
    Real fCosSqr = m_pkCone->CosAngle*m_pkCone->CosAngle;
    if (fDotSqr >= fLSqr*fCosSqr && fDot > (Real)0.0)
    {
        // sphere center is inside cone, so sphere and cone intersect
        return true;
    }

    // Sphere center is outside cone.  Problem now reduces to looking for
    // an intersection between circle and ray in the plane containing
    // cone vertex and spanned by cone axis and vector from vertex to
    // sphere center.

    // Ray is t*D+V (t >= 0) where |D| = 1 and dot(A,D) = cos(angle).
    // Also, D = e*A+f*(C-V).  Plugging ray equation into sphere equation
    // yields R^2 = |t*D+V-C|^2, so the quadratic for intersections is
    // t^2 - 2*dot(D,C-V)*t + |C-V|^2 - R^2 = 0.  An intersection occurs
    // if and only if the discriminant is nonnegative.  This test becomes
    //
    //     dot(D,C-V)^2 >= dot(C-V,C-V) - R^2
    //
    // Note that if the right-hand side is nonpositive, then the inequality
    // is true (the sphere contains V).  I have already ruled this out in
    // the first block of code in this function.

    Real fULen = Math<Real>::Sqrt(Math<Real>::FAbs(fLSqr-fDotSqr));
    Real fTest = m_pkCone->CosAngle*fDot + m_pkCone->SinAngle*fULen;
    Real fDiscr = fTest*fTest - fLSqr + fRSqr;

    // compute point of intersection closest to vertex V
    Real fT = fTest - Math<Real>::Sqrt(fDiscr);
    Vector3<Real> kB = kDiff - fDot*m_pkCone->Axis;
    Real fTmp = m_pkCone->SinAngle/fULen;
    m_kPoint = fT*(m_pkCone->CosAngle*m_pkCone->Axis + fTmp*kB);

    return fDiscr >= (Real)0.0 && fTest >= (Real)0.0;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& IntrSphere3Cone3<Real>::GetPoint () const
{
    return m_kPoint;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrSphere3Cone3<float>;

template WM4_FOUNDATION_ITEM
class IntrSphere3Cone3<double>;
//----------------------------------------------------------------------------
}
