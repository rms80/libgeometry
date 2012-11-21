// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4ImplicitSurface.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
ImplicitSurface<Real>::ImplicitSurface ()
{
}
//----------------------------------------------------------------------------
template <class Real>
ImplicitSurface<Real>::~ImplicitSurface ()
{
}
//----------------------------------------------------------------------------
template <class Real>
bool ImplicitSurface<Real>::IsOnSurface (const Vector3<Real>& rkP,
    Real fEpsilon) const
{
    return Math<Real>::FAbs(F(rkP)) <= fEpsilon;
}
//----------------------------------------------------------------------------
template <class Real>
Vector3<Real> ImplicitSurface<Real>::GetGradient (const Vector3<Real>& rkP)
    const
{
    Real fFX = FX(rkP);
    Real fFY = FY(rkP);
    Real fFZ = FZ(rkP);
    return Vector3<Real>(fFX,fFY,fFZ);
}
//----------------------------------------------------------------------------
template <class Real>
Matrix3<Real> ImplicitSurface<Real>::GetHessian (const Vector3<Real>& rkP)
    const
{
    Real fFXX = FXX(rkP);
    Real fFXY = FXY(rkP);
    Real fFXZ = FXZ(rkP);
    Real fFYY = FYY(rkP);
    Real fFYZ = FYZ(rkP);
    Real fFZZ = FZZ(rkP);
    return Matrix3<Real>(fFXX,fFXY,fFXZ,fFXY,fFYY,fFYZ,fFXZ,fFYZ,fFZZ);
}
//----------------------------------------------------------------------------
template <class Real>
void ImplicitSurface<Real>::GetFrame (const Vector3<Real>& rkP,
    Vector3<Real>& rkTangent0, Vector3<Real>& rkTangent1,
    Vector3<Real>& rkNormal) const
{
    rkNormal = GetGradient(rkP);
    Vector3<Real>::GenerateOrthonormalBasis(rkTangent0,rkTangent1,rkNormal);
}
//----------------------------------------------------------------------------
template <class Real>
bool ImplicitSurface<Real>::ComputePrincipalCurvatureInfo (
    const Vector3<Real>& rkP, Real& rfCurv0, Real& rfCurv1,
    Vector3<Real>& rkDir0, Vector3<Real>& rkDir1)
{
    // Principal curvatures and directions for implicitly defined surfaces
    // F(x,y,z) = 0.
    //
    // DF = (Fx,Fy,Fz), L = Length(DF)
    //
    // D^2 F = +-           -+
    //         | Fxx Fxy Fxz |
    //         | Fxy Fyy Fyz |
    //         | Fxz Fyz Fzz |
    //         +-           -+
    //
    // adj(D^2 F) = +-                                                 -+
    //              | Fyy*Fzz-Fyz*Fyz  Fyz*Fxz-Fxy*Fzz  Fxy*Fyz-Fxz*Fyy |
    //              | Fyz*Fxz-Fxy*Fzz  Fxx*Fzz-Fxz*Fxz  Fxy*Fxz-Fxx*Fyz |
    //              | Fxy*Fyz-Fxz*Fyy  Fxy*Fxz-Fxx*Fyz  Fxx*Fyy-Fxy*Fxy |
    //              +-                                                 -+
    //
    // Gaussian curvature = [DF^t adj(D^2 F) DF]/L^4
    // 
    // Mean curvature = 0.5*[trace(D^2 F)/L - (DF^t D^2 F DF)/L^3]

    // first derivatives
    Real fFX = FX(rkP);
    Real fFY = FY(rkP);
    Real fFZ = FZ(rkP);
    Real fL = Math<Real>::Sqrt(fFX*fFX + fFY*fFY + fFZ*fFZ);
    if (fL <= Math<Real>::ZERO_TOLERANCE)
    {
        return false;
    }

    Real fFXFX = fFX*fFX;
    Real fFXFY = fFX*fFY;
    Real fFXFZ = fFX*fFZ;
    Real fFYFY = fFY*fFY;
    Real fFYFZ = fFY*fFZ;
    Real fFZFZ = fFZ*fFZ;

    Real fInvL = ((Real)1.0)/fL;
    Real fInvL2 = fInvL*fInvL;
    Real fInvL3 = fInvL*fInvL2;
    Real fInvL4 = fInvL2*fInvL2;

    // second derivatives
    Real fFXX = FXX(rkP);
    Real fFXY = FXY(rkP);
    Real fFXZ = FXZ(rkP);
    Real fFYY = FYY(rkP);
    Real fFYZ = FYZ(rkP);
    Real fFZZ = FZZ(rkP);

    // mean curvature
    Real fMCurv = ((Real)0.5)*fInvL3*(fFXX*(fFYFY+fFZFZ) + fFYY*(fFXFX+fFZFZ)
        + fFZZ*(fFXFX+fFYFY)
        - ((Real)2.0)*(fFXY*fFXFY+fFXZ*fFXFZ+fFYZ*fFYFZ));

    // Gaussian curvature
    Real fGCurv = fInvL4*(fFXFX*(fFYY*fFZZ-fFYZ*fFYZ)
        + fFYFY*(fFXX*fFZZ-fFXZ*fFXZ) + fFZFZ*(fFXX*fFYY-fFXY*fFXY)
        + ((Real)2.0)*(fFXFY*(fFXZ*fFYZ-fFXY*fFZZ)
        + fFXFZ*(fFXY*fFYZ-fFXZ*fFYY)
        + fFYFZ*(fFXY*fFXZ-fFXX*fFYZ)));

    // solve for principal curvatures
    Real fDiscr = Math<Real>::Sqrt(Math<Real>::FAbs(fMCurv*fMCurv-fGCurv));
    rfCurv0 = fMCurv - fDiscr;
    rfCurv1 = fMCurv + fDiscr;

    Real fM00 = ((-(Real)1.0 + fFXFX*fInvL2)*fFXX)*fInvL + (fFXFY*fFXY)*fInvL3
        + (fFXFZ*fFXZ)*fInvL3;
    Real fM01 = ((-(Real)1.0 + fFXFX*fInvL2)*fFXY)*fInvL + (fFXFY*fFYY)*fInvL3
        + (fFXFZ*fFYZ)*fInvL3;
    Real fM02 = ((-(Real)1.0 + fFXFX*fInvL2)*fFXZ)*fInvL + (fFXFY*fFYZ)*fInvL3
        + (fFXFZ*fFZZ)*fInvL3;
    Real fM10 = (fFXFY*fFXX)*fInvL3 + ((-(Real)1.0 + fFYFY*fInvL2)*fFXY)*fInvL
        + (fFYFZ*fFXZ)*fInvL3;
    Real fM11 = (fFXFY*fFXY)*fInvL3 + ((-(Real)1.0 + fFYFY*fInvL2)*fFYY)*fInvL
        + (fFYFZ*fFYZ)*fInvL3;
    Real fM12 = (fFXFY*fFXZ)*fInvL3 + ((-(Real)1.0 + fFYFY*fInvL2)*fFYZ)*fInvL
        + (fFYFZ*fFZZ)*fInvL3;
    Real fM20 = (fFXFZ*fFXX)*fInvL3 + (fFYFZ*fFXY)*fInvL3 + ((-(Real)1.0
        + fFZFZ*fInvL2)*fFXZ)*fInvL;
    Real fM21 = (fFXFZ*fFXY)*fInvL3 + (fFYFZ*fFYY)*fInvL3 + ((-(Real)1.0
        + fFZFZ*fInvL2)*fFYZ)*fInvL;
    Real fM22 = (fFXFZ*fFXZ)*fInvL3 + (fFYFZ*fFYZ)*fInvL3 + ((-(Real)1.0
        + fFZFZ*fInvL2)*fFZZ)*fInvL;

    // solve for principal directions
    Real fTmp1 = fM00 + rfCurv0;
    Real fTmp2 = fM11 + rfCurv0;
    Real fTmp3 = fM22 + rfCurv0;

    Vector3<Real> akU[3];
    Real afLength[3];

    akU[0].X() = fM01*fM12-fM02*fTmp2;
    akU[0].Y() = fM02*fM10-fM12*fTmp1;
    akU[0].Z() = fTmp1*fTmp2-fM01*fM10;
    afLength[0] = akU[0].Length();

    akU[1].X() = fM01*fTmp3-fM02*fM21;
    akU[1].Y() = fM02*fM20-fTmp1*fTmp3;
    akU[1].Z() = fTmp1*fM21-fM01*fM20;
    afLength[1] = akU[1].Length();

    akU[2].X() = fTmp2*fTmp3-fM12*fM21;
    akU[2].Y() = fM12*fM20-fM10*fTmp3;
    akU[2].Z() = fM10*fM21-fM20*fTmp2;
    afLength[2] = akU[2].Length();

    int iMaxIndex = 0;
    Real fMax = afLength[0];
    if (afLength[1] > fMax)
    {
        iMaxIndex = 1;
        fMax = afLength[1];
    }
    if (afLength[2] > fMax)
    {
        iMaxIndex = 2;
    }

    Real fInvLength = ((Real)1.0)/afLength[iMaxIndex];
    akU[iMaxIndex] *= fInvLength;

    rkDir1 = akU[iMaxIndex];
    rkDir0 = rkDir1.UnitCross(Vector3<Real>(fFX,fFY,fFZ));

    return true;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class ImplicitSurface<float>;

template WM4_FOUNDATION_ITEM
class ImplicitSurface<double>;
//----------------------------------------------------------------------------
}
