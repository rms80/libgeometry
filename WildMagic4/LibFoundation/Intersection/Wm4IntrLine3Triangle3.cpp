// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntrLine3Triangle3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntrLine3Triangle3<Real>::IntrLine3Triangle3 (const Line3<Real>& rkLine,
    const Triangle3<Real>& rkTriangle)
    :
    m_pkLine(&rkLine),
    m_pkTriangle(&rkTriangle)
{
}
//----------------------------------------------------------------------------
template <class Real>
const Line3<Real>& IntrLine3Triangle3<Real>::GetLine () const
{
    return *m_pkLine;
}
//----------------------------------------------------------------------------
template <class Real>
const Triangle3<Real>& IntrLine3Triangle3<Real>::GetTriangle () const
{
    return *m_pkTriangle;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Triangle3<Real>::Test ()
{
    // compute the offset origin, edges, and normal
    Vector3<Real> kDiff = m_pkLine->Origin - m_pkTriangle->V[0];
    Vector3<Real> kEdge1 = m_pkTriangle->V[1] - m_pkTriangle->V[0];
    Vector3<Real> kEdge2 = m_pkTriangle->V[2] - m_pkTriangle->V[0];
    Vector3<Real> kNormal = kEdge1.Cross(kEdge2);

    // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = line direction,
    // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
    //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
    //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
    //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
    Real fDdN = m_pkLine->Direction.Dot(kNormal);
    Real fSign;
    if (fDdN > Math<Real>::ZERO_TOLERANCE)
    {
        fSign = (Real)1.0;
    }
    else if (fDdN < -Math<Real>::ZERO_TOLERANCE)
    {
        fSign = (Real)-1.0;
        fDdN = -fDdN;
    }
    else
    {
        // Line and triangle are parallel, call it a "no intersection"
        // even if the line does intersect.
        m_iIntersectionType = IT_EMPTY;
        return false;
    }

    Real fDdQxE2 = fSign*m_pkLine->Direction.Dot(kDiff.Cross(kEdge2));
    if (fDdQxE2 >= (Real)0.0)
    {
        Real fDdE1xQ = fSign*m_pkLine->Direction.Dot(kEdge1.Cross(kDiff));
        if (fDdE1xQ >= (Real)0.0)
        {
            if (fDdQxE2 + fDdE1xQ <= fDdN)
            {
                // line intersects triangle
                m_iIntersectionType = IT_POINT;
                return true;
            }
            // else: b1+b2 > 1, no intersection
        }
        // else: b2 < 0, no intersection
    }
    // else: b1 < 0, no intersection

    m_iIntersectionType = IT_EMPTY;
    return false;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntrLine3Triangle3<Real>::Find ()
{
    // compute the offset origin, edges, and normal
    Vector3<Real> kDiff = m_pkLine->Origin - m_pkTriangle->V[0];
    Vector3<Real> kEdge1 = m_pkTriangle->V[1] - m_pkTriangle->V[0];
    Vector3<Real> kEdge2 = m_pkTriangle->V[2] - m_pkTriangle->V[0];
    Vector3<Real> kNormal = kEdge1.Cross(kEdge2);

    // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = line direction,
    // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
    //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
    //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
    //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
    Real fDdN = m_pkLine->Direction.Dot(kNormal);
    Real fSign;
    if (fDdN > Math<Real>::ZERO_TOLERANCE)
    {
        fSign = (Real)1.0;
    }
    else if (fDdN < -Math<Real>::ZERO_TOLERANCE)
    {
        fSign = (Real)-1.0;
        fDdN = -fDdN;
    }
    else
    {
        // Line and triangle are parallel, call it a "no intersection"
        // even if the line does intersect.
        m_iIntersectionType = IT_EMPTY;
        return false;
    }

    Real fDdQxE2 = fSign*m_pkLine->Direction.Dot(kDiff.Cross(kEdge2));
    if (fDdQxE2 >= (Real)0.0)
    {
        Real fDdE1xQ = fSign*m_pkLine->Direction.Dot(kEdge1.Cross(kDiff));
        if (fDdE1xQ >= (Real)0.0)
        {
            if (fDdQxE2 + fDdE1xQ <= fDdN)
            {
                // line intersects triangle
                Real fQdN = -fSign*kDiff.Dot(kNormal);
                Real fInv = ((Real)1.0)/fDdN;
                m_fLineT = fQdN*fInv;
                m_fTriB1 = fDdQxE2*fInv;
                m_fTriB2 = fDdE1xQ*fInv;
                m_fTriB0 = (Real)1.0 - m_fTriB1 - m_fTriB2;
                m_iIntersectionType = IT_POINT;
                return true;
            }
            // else: b1+b2 > 1, no intersection
        }
        // else: b2 < 0, no intersection
    }
    // else: b1 < 0, no intersection

    m_iIntersectionType = IT_EMPTY;
    return false;
}
//----------------------------------------------------------------------------
template <class Real>
Real IntrLine3Triangle3<Real>::GetLineT () const
{
    return m_fLineT;
}
//----------------------------------------------------------------------------
template <class Real>
Real IntrLine3Triangle3<Real>::GetTriB0 () const
{
    return m_fTriB0;
}
//----------------------------------------------------------------------------
template <class Real>
Real IntrLine3Triangle3<Real>::GetTriB1 () const
{
    return m_fTriB1;
}
//----------------------------------------------------------------------------
template <class Real>
Real IntrLine3Triangle3<Real>::GetTriB2 () const
{
    return m_fTriB2;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntrLine3Triangle3<float>;

template WM4_FOUNDATION_ITEM
class IntrLine3Triangle3<double>;
//----------------------------------------------------------------------------
}
