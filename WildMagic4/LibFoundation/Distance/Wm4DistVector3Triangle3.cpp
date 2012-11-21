// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4DistVector3Triangle3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
DistVector3Triangle3<Real>::DistVector3Triangle3 (
    const Vector3<Real>& rkVector, const Triangle3<Real>& rkTriangle)
    :
    m_pkVector(&rkVector),
    m_pkTriangle(&rkTriangle)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Vector3<Real>& DistVector3Triangle3<Real>::GetVector () const
{
    return *m_pkVector;
}
//----------------------------------------------------------------------------
template <class Real>
const Triangle3<Real>& DistVector3Triangle3<Real>::GetTriangle () const
{
    return *m_pkTriangle;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Triangle3<Real>::Get ()
{
    Real fSqrDist = GetSquared();
    return Math<Real>::Sqrt(fSqrDist);
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Triangle3<Real>::GetSquared ()
{
    Vector3<Real> kDiff = m_pkTriangle->V[0] - *m_pkVector;
    Vector3<Real> kEdge0 = m_pkTriangle->V[1] - m_pkTriangle->V[0];
    Vector3<Real> kEdge1 = m_pkTriangle->V[2] - m_pkTriangle->V[0];
    Real fA00 = kEdge0.SquaredLength();
    Real fA01 = kEdge0.Dot(kEdge1);
    Real fA11 = kEdge1.SquaredLength();
    Real fB0 = kDiff.Dot(kEdge0);
    Real fB1 = kDiff.Dot(kEdge1);
    Real fC = kDiff.SquaredLength();
    Real fDet = Math<Real>::FAbs(fA00*fA11-fA01*fA01);
    Real fS = fA01*fB1-fA11*fB0;
    Real fT = fA01*fB0-fA00*fB1;
    Real fSqrDistance;

    if (fS + fT <= fDet)
    {
        if (fS < (Real)0.0)
        {
            if (fT < (Real)0.0)  // region 4
            {
                if (fB0 < (Real)0.0)
                {
                    fT = (Real)0.0;
                    if (-fB0 >= fA00)
                    {
                        fS = (Real)1.0;
                        fSqrDistance = fA00+((Real)2.0)*fB0+fC;
                    }
                    else
                    {
                        fS = -fB0/fA00;
                        fSqrDistance = fB0*fS+fC;
                    }
                }
                else
                {
                    fS = (Real)0.0;
                    if (fB1 >= (Real)0.0)
                    {
                        fT = (Real)0.0;
                        fSqrDistance = fC;
                    }
                    else if (-fB1 >= fA11)
                    {
                        fT = (Real)1.0;
                        fSqrDistance = fA11+((Real)2.0)*fB1+fC;
                    }
                    else
                    {
                        fT = -fB1/fA11;
                        fSqrDistance = fB1*fT+fC;
                    }
                }
            }
            else  // region 3
            {
                fS = (Real)0.0;
                if (fB1 >= (Real)0.0)
                {
                    fT = (Real)0.0;
                    fSqrDistance = fC;
                }
                else if (-fB1 >= fA11)
                {
                    fT = (Real)1.0;
                    fSqrDistance = fA11+((Real)2.0)*fB1+fC;
                }
                else
                {
                    fT = -fB1/fA11;
                    fSqrDistance = fB1*fT+fC;
                }
            }
        }
        else if (fT < (Real)0.0)  // region 5
        {
            fT = (Real)0.0;
            if (fB0 >= (Real)0.0)
            {
                fS = (Real)0.0;
                fSqrDistance = fC;
            }
            else if (-fB0 >= fA00)
            {
                fS = (Real)1.0;
                fSqrDistance = fA00+((Real)2.0)*fB0+fC;
            }
            else
            {
                fS = -fB0/fA00;
                fSqrDistance = fB0*fS+fC;
            }
        }
        else  // region 0
        {
            // minimum at interior point
            Real fInvDet = ((Real)1.0)/fDet;
            fS *= fInvDet;
            fT *= fInvDet;
            fSqrDistance = fS*(fA00*fS+fA01*fT+((Real)2.0)*fB0) +
                fT*(fA01*fS+fA11*fT+((Real)2.0)*fB1)+fC;
        }
    }
    else
    {
        Real fTmp0, fTmp1, fNumer, fDenom;

        if (fS < (Real)0.0)  // region 2
        {
            fTmp0 = fA01 + fB0;
            fTmp1 = fA11 + fB1;
            if (fTmp1 > fTmp0)
            {
                fNumer = fTmp1 - fTmp0;
                fDenom = fA00-2.0f*fA01+fA11;
                if (fNumer >= fDenom)
                {
                    fS = (Real)1.0;
                    fT = (Real)0.0;
                    fSqrDistance = fA00+((Real)2.0)*fB0+fC;
                }
                else
                {
                    fS = fNumer/fDenom;
                    fT = (Real)1.0 - fS;
                    fSqrDistance = fS*(fA00*fS+fA01*fT+2.0f*fB0) +
                        fT*(fA01*fS+fA11*fT+((Real)2.0)*fB1)+fC;
                }
            }
            else
            {
                fS = (Real)0.0;
                if (fTmp1 <= (Real)0.0)
                {
                    fT = (Real)1.0;
                    fSqrDistance = fA11+((Real)2.0)*fB1+fC;
                }
                else if (fB1 >= (Real)0.0)
                {
                    fT = (Real)0.0;
                    fSqrDistance = fC;
                }
                else
                {
                    fT = -fB1/fA11;
                    fSqrDistance = fB1*fT+fC;
                }
            }
        }
        else if (fT < (Real)0.0)  // region 6
        {
            fTmp0 = fA01 + fB1;
            fTmp1 = fA00 + fB0;
            if (fTmp1 > fTmp0)
            {
                fNumer = fTmp1 - fTmp0;
                fDenom = fA00-((Real)2.0)*fA01+fA11;
                if (fNumer >= fDenom)
                {
                    fT = (Real)1.0;
                    fS = (Real)0.0;
                    fSqrDistance = fA11+((Real)2.0)*fB1+fC;
                }
                else
                {
                    fT = fNumer/fDenom;
                    fS = (Real)1.0 - fT;
                    fSqrDistance = fS*(fA00*fS+fA01*fT+((Real)2.0)*fB0) +
                        fT*(fA01*fS+fA11*fT+((Real)2.0)*fB1)+fC;
                }
            }
            else
            {
                fT = (Real)0.0;
                if (fTmp1 <= (Real)0.0)
                {
                    fS = (Real)1.0;
                    fSqrDistance = fA00+((Real)2.0)*fB0+fC;
                }
                else if (fB0 >= (Real)0.0)
                {
                    fS = (Real)0.0;
                    fSqrDistance = fC;
                }
                else
                {
                    fS = -fB0/fA00;
                    fSqrDistance = fB0*fS+fC;
                }
            }
        }
        else  // region 1
        {
            fNumer = fA11 + fB1 - fA01 - fB0;
            if (fNumer <= (Real)0.0)
            {
                fS = (Real)0.0;
                fT = (Real)1.0;
                fSqrDistance = fA11+((Real)2.0)*fB1+fC;
            }
            else
            {
                fDenom = fA00-2.0f*fA01+fA11;
                if (fNumer >= fDenom)
                {
                    fS = (Real)1.0;
                    fT = (Real)0.0;
                    fSqrDistance = fA00+((Real)2.0)*fB0+fC;
                }
                else
                {
                    fS = fNumer/fDenom;
                    fT = (Real)1.0 - fS;
                    fSqrDistance = fS*(fA00*fS+fA01*fT+((Real)2.0)*fB0) +
                        fT*(fA01*fS+fA11*fT+((Real)2.0)*fB1)+fC;
                }
            }
        }
    }

    // account for numerical round-off error
    if (fSqrDistance < (Real)0.0)
    {
        fSqrDistance = (Real)0.0;
    }

    m_kClosestPoint0 = *m_pkVector;
    m_kClosestPoint1 = m_pkTriangle->V[0] + fS*kEdge0 + fT*kEdge1;
    m_afTriangleBary[1] = fS;
    m_afTriangleBary[2] = fT;
    m_afTriangleBary[0] = (Real)1.0 - fS - fT;
    return fSqrDistance;
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Triangle3<Real>::Get (Real fT,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector3<Real> kMV0 = m_pkTriangle->V[0] + fT*rkVelocity1;
    Vector3<Real> kMV1 = m_pkTriangle->V[1] + fT*rkVelocity1;
    Vector3<Real> kMV2 = m_pkTriangle->V[2] + fT*rkVelocity1;
    Triangle3<Real> kMTriangle(kMV0,kMV1,kMV2);
    return DistVector3Triangle3<Real>(kMVector,kMTriangle).Get();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Triangle3<Real>::GetSquared (Real fT,
    const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
{
    Vector3<Real> kMVector = *m_pkVector + fT*rkVelocity0;
    Vector3<Real> kMV0 = m_pkTriangle->V[0] + fT*rkVelocity1;
    Vector3<Real> kMV1 = m_pkTriangle->V[1] + fT*rkVelocity1;
    Vector3<Real> kMV2 = m_pkTriangle->V[2] + fT*rkVelocity1;
    Triangle3<Real> kMTriangle(kMV0,kMV1,kMV2);
    return DistVector3Triangle3<Real>(kMVector,kMTriangle).GetSquared();
}
//----------------------------------------------------------------------------
template <class Real>
Real DistVector3Triangle3<Real>::GetTriangleBary (int i) const
{
    assert(0 <= i && i < 3);
    return m_afTriangleBary[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class DistVector3Triangle3<float>;

template WM4_FOUNDATION_ITEM
class DistVector3Triangle3<double>;
//----------------------------------------------------------------------------
}
