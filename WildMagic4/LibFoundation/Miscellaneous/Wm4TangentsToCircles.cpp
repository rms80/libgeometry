// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4TangentsToCircles.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
static void GetDirections (const Vector2<Real>& rkW, Real fA,
    Vector2<Real>& rkDir0, Vector2<Real>& rkDir1)
{
    Real fA2 = fA*fA;
    Real fWx2 = rkW.X()*rkW.X();
    Real fWy2 = rkW.Y()*rkW.Y();
    Real fC2 = fWx2 + fWy2;
    Real fMHalfInvC2 = ((Real)-0.5)/fC2;
    Real fC0, fC1, fDiscr, fRoot, fInv;

    if (Math<Real>::FAbs(rkW.X()) >= Math<Real>::FAbs(rkW.Y()))
    {
        fC0 = fA2 - fWx2;
        fC1 = -((Real)2.0)*fA*rkW.Y();
        fDiscr = fC1*fC1 - ((Real)4.0)*fC0*fC2;
        fRoot = Math<Real>::Sqrt(Math<Real>::FAbs(fDiscr));
        fInv = ((Real)1.0)/rkW.X();
        rkDir0.Y() = (fC1 + fRoot)*fMHalfInvC2;
        rkDir0.X() = (fA - rkW.Y()*rkDir0.Y())*fInv;
        rkDir1.Y() = (fC1 - fRoot)*fMHalfInvC2;
        rkDir1.X() = (fA - rkW.Y()*rkDir1.Y())*fInv;
    }
    else
    {
        fC0 = fA2 - fWy2;
        fC1 = -((Real)2.0)*fA*rkW.X();
        fDiscr = fC1*fC1 - ((Real)4.0)*fC0*fC2;
        fRoot = Math<Real>::Sqrt(Math<Real>::FAbs(fDiscr));
        fInv = ((Real)1.0)/rkW.Y();
        rkDir0.X() = (fC1 + fRoot)*fMHalfInvC2;
        rkDir0.Y() = (fA - rkW.X()*rkDir0.X())*fInv;
        rkDir1.X() = (fC1 - fRoot)*fMHalfInvC2;
        rkDir1.Y() = (fA - rkW.X()*rkDir1.X())*fInv;
    }
}
//----------------------------------------------------------------------------
template <class Real>
bool GetTangentsToCircles (const Circle2<Real>& rkCircle0,
    const Circle2<Real>& rkCircle1, Line2<Real> akLine[4])
{
    Vector2<Real> kW = rkCircle1.Center - rkCircle0.Center;
    Real fWLenSqr = kW.SquaredLength();
    Real fRSum = rkCircle0.Radius + rkCircle1.Radius;
    if (fWLenSqr <= fRSum*fRSum)
    {
        // circles are either intersecting or nested
        return false;
    }

    Real fR0Sqr = rkCircle0.Radius*rkCircle0.Radius;
    Real fTmp, fA;

    Real fRDiff = rkCircle1.Radius - rkCircle0.Radius;
    if (Math<Real>::FAbs(fRDiff) >= Math<Real>::ZERO_TOLERANCE)
    {
        // solve (R1^2-R0^2)*s^2 + 2*R0^2*s - R0^2 = 0.
        Real fR1Sqr = rkCircle1.Radius*rkCircle1.Radius;
        Real fC0 = -fR0Sqr;
        Real fC1 = ((Real)2.0)*fR0Sqr;
        Real fC2 = rkCircle1.Radius*rkCircle1.Radius - fR0Sqr;
        Real fMHalfInvC2 = ((Real)-0.5)/fC2;
        Real fDiscr = Math<Real>::FAbs(fC1*fC1 - ((Real)4.0)*fC0*fC2);
        Real fRoot = Math<Real>::Sqrt(fDiscr);
        Real fS, fOmS;

        // first root
        fS = (fC1 + fRoot)*fMHalfInvC2;
        akLine[0].Origin = rkCircle0.Center + fS*kW;
        akLine[1].Origin = akLine[0].Origin;
        if (fS >= (Real)0.5)
        {
            fTmp = Math<Real>::FAbs(fWLenSqr - fR0Sqr/(fS*fS));
            fA = Math<Real>::Sqrt(fTmp);
        }
        else
        {
            fOmS = (Real)1.0 - fS;
            fTmp = Math<Real>::FAbs(fWLenSqr - fR1Sqr/(fOmS*fOmS));
            fA = Math<Real>::Sqrt(fTmp);
        }
        GetDirections(kW,fA,akLine[0].Direction,akLine[1].Direction);

        // second root
        fS = (fC1 - fRoot)*fMHalfInvC2;
        akLine[2].Origin = rkCircle0.Center + fS*kW;
        akLine[3].Origin = akLine[2].Origin;
        if (fS >= (Real)0.5)
        {
            fTmp = Math<Real>::FAbs(fWLenSqr - fR0Sqr/(fS*fS));
            fA = Math<Real>::Sqrt(fTmp);
        }
        else
        {
            fOmS = (Real)1.0 - fS;
            fTmp = Math<Real>::FAbs(fWLenSqr - fR1Sqr/(fOmS*fOmS));
            fA = Math<Real>::Sqrt(fTmp);
        }
        GetDirections(kW,fA,akLine[2].Direction,akLine[3].Direction);
    }
    else
    {
        // circles effectively have same radius

        // midpoint of circle centers
        Vector2<Real> kMid = ((Real)0.5)*(rkCircle0.Center+rkCircle1.Center);

        // tangent lines passing through midpoint
        fTmp = Math<Real>::FAbs(fWLenSqr - ((Real)4.0)*fR0Sqr);
        fA = Math<Real>::Sqrt(fTmp);
        GetDirections(kW,fA,akLine[0].Direction,akLine[1].Direction);
        akLine[0].Origin = kMid;
        akLine[1].Origin = kMid;

        // normalize W
        kW /= Math<Real>::Sqrt(fWLenSqr);

        // tangent lines parallel to normalized W
        //   1.  D = W
        //   2.  a. P = mid+R0*perp(W), perp(a,b) = (b,-a)
        //       b. P = mid-R0*perp(W)
        akLine[2].Origin.X() = kMid.X() + rkCircle0.Radius*kW.Y();
        akLine[2].Origin.Y() = kMid.Y() - rkCircle0.Radius*kW.X();
        akLine[2].Direction = kW;
        akLine[3].Origin.X() = kMid.X() - rkCircle0.Radius*kW.Y();
        akLine[3].Origin.Y() = kMid.Y() + rkCircle0.Radius*kW.X();
        akLine[3].Direction = kW;
    }

    return true;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
bool GetTangentsToCircles<float> (const Circle2<float>&,
    const Circle2<float>&, Line2<float>[4]);

template WM4_FOUNDATION_ITEM
bool GetTangentsToCircles<double> (const Circle2<double>&,
    const Circle2<double>&, Line2<double>[4]);
//----------------------------------------------------------------------------
}
