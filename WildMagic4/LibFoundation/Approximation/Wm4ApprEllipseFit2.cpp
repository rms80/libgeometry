// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4ApprEllipseFit2.h"
#include "Wm4ContBox2.h"
#include "Wm4DistVector2Ellipse2.h"
#include "Wm4MinimizeN.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
EllipseFit2<Real>::EllipseFit2 (int iQuantity, const Vector2<Real>* akPoint,
    Vector2<Real>& rkU, Matrix2<Real>& rkR, Real afD[2], Real& rfError)
{
    // Energy function is E : R^5 -> R where
    // V = (V0,V1,V2,V3,V4)
    //   = (D[0],D[1],U.x,U,y,atan2(R[1][0],R[1][1])).

    m_iQuantity = iQuantity;
    m_akPoint = akPoint;
    m_akTemp = WM4_NEW Vector2<Real>[iQuantity];

    MinimizeN<Real> kMinimizer(5,Energy,8,8,32,this);

    InitialGuess(iQuantity,akPoint,rkU,rkR,afD);
    Real fAngle = Math<Real>::ACos(rkR[0][0]);
    Real fE0 = afD[0]*Math<Real>::FAbs(rkR[0][0]) + 
        afD[1]*Math<Real>::FAbs(rkR[0][1]);
    Real fE1 = afD[0]*Math<Real>::FAbs(rkR[1][0]) +
        afD[1]*Math<Real>::FAbs(rkR[1][1]);

    Real afV0[5] =
    {
        ((Real)0.5)*afD[0],
        ((Real)0.5)*afD[1],
        rkU.X() - fE0,
        rkU.Y() - fE1,
        (Real)0.0
    };

    Real afV1[5] =
    {
        ((Real)2.0)*afD[0],
        ((Real)2.0)*afD[1],
        rkU.X() + fE0,
        rkU.Y() + fE1,
        Math<Real>::PI
    };

    Real afVInitial[5] =
    {
        afD[0],
        afD[1],
        rkU.X(),
        rkU.Y(),
        fAngle
    };

    Real afVMin[5];
    kMinimizer.GetMinimum(afV0,afV1,afVInitial,afVMin,rfError);

    afD[0] = afVMin[0];
    afD[1] = afVMin[1];
    rkU.X() = afVMin[2];
    rkU.Y() = afVMin[3];
    rkR.FromAngle(afVMin[4]);

    WM4_DELETE[] m_akTemp;
}
//----------------------------------------------------------------------------
template <class Real>
void EllipseFit2<Real>::InitialGuess (int iQuantity,
    const Vector2<Real>* akPoint, Vector2<Real>& rkU, Matrix2<Real>& rkR,
    Real afD[2])
{
    Box2<Real> kBox = ContOrientedBox(iQuantity,akPoint);

    rkU = kBox.Center;
    rkR[0][0] = kBox.Axis[0].X();
    rkR[0][1] = kBox.Axis[0].Y();
    rkR[1][0] = kBox.Axis[1].X();
    rkR[1][1] = kBox.Axis[1].Y();
    afD[0] = kBox.Extent[0];
    afD[1] = kBox.Extent[1];
}
//----------------------------------------------------------------------------
template <class Real>
Real EllipseFit2<Real>::Energy (const Real* afV, void* pvData)
{
    EllipseFit2& rkSelf = *(EllipseFit2*)pvData;

    // build rotation matrix
    Matrix2<Real> kRot(afV[4]);

    Ellipse2<Real> kEllipse(Vector2<Real>::ZERO,Vector2<Real>::UNIT_X,
        Vector2<Real>::UNIT_Y,afV[0],afV[1]);

    // transform the points to the coordinate system of U and R
    Real fEnergy = (Real)0.0;
    for (int i = 0; i < rkSelf.m_iQuantity; i++)
    {
        Vector2<Real> kDiff(
            rkSelf.m_akPoint[i].X() - afV[2],
            rkSelf.m_akPoint[i].Y() - afV[3]);

        rkSelf.m_akTemp[i] = kDiff*kRot;
        Real fDist =
            DistVector2Ellipse2<Real>(rkSelf.m_akTemp[i],kEllipse).Get();
        fEnergy += fDist;
    }

    return fEnergy;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class EllipseFit2<float>;

template WM4_FOUNDATION_ITEM
class EllipseFit2<double>;
//----------------------------------------------------------------------------
}
