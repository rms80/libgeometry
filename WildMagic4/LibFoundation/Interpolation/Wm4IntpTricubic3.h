// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPTRICUBIC3_H
#define WM4INTPTRICUBIC3_H

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpTricubic3
{
public:
    // Construction and destruction.  IntpTricubic3 does not accept
    // responsibility for deleting the input array.  The application must do
    // so.  The interpolator is for uniformly spaced (x,y,z)-values.  The
    // function values are assumed to be organized as f(x,y,z) = F[z][y][x].
    // Exact interpolation is achieved by setting bCatmullRom to 'true',
    // giving you the Catmull-Rom blending matrix.  If a smooth interpolation
    // is desired, set bCatmullRom to 'false' to obtain B-spline blending.

    IntpTricubic3 (int iXBound, int iYBound, int iZBound, Real fXMin,
        Real fXSpacing, Real fYMin, Real fYSpacing, Real fZMin,
        Real fZSpacing, Real*** aaafF, bool bCatmullRom);

    int GetXBound () const;
    int GetYBound () const;
    int GetZBound () const;
    int GetQuantity () const;
    Real*** GetF () const;

    Real GetXMin () const;
    Real GetXMax () const;
    Real GetXSpacing () const;
    Real GetYMin () const;
    Real GetYMax () const;
    Real GetYSpacing () const;
    Real GetZMin () const;
    Real GetZMax () const;
    Real GetZSpacing () const;

    // Evaluate the function and its derivatives.  The application is
    // responsible for ensuring that xmin <= x <= xmax, ymin <= y <= ymax,
    // and zmin <= z <= zmax.  If (x,y,z) is outside the extremes, the
    // function returns MAXREAL.  The first operator is for function
    // evaluation.  The second operator is for function or derivative
    // evaluations.  The uiXOrder argument is the order of the x-derivative,
    // the uiYOrder argument is the order of the y-derivative, and the
    // uiZOrder argument is the order of the z-derivative.  All orders are
    // zero to get the function value itself.
    Real operator() (Real fX, Real fY, Real fZ) const;
    Real operator() (int iXOrder, int iYOrder, int iZOrder, Real fX,
        Real fY, Real fZ) const;

private:
    int m_iXBound, m_iYBound, m_iZBound, m_iQuantity;
    Real m_fXMin, m_fXMax, m_fXSpacing, m_fInvXSpacing;
    Real m_fYMin, m_fYMax, m_fYSpacing, m_fInvYSpacing;
    Real m_fZMin, m_fZMax, m_fZSpacing, m_fInvZSpacing;
    Real*** m_aaafF;
    const Real (*m_aafBlend)[4];

    static const Real ms_aafCRBlend[4][4];
    static const Real ms_aafBSBlend[4][4];
};

typedef IntpTricubic3<float> IntpTricubic3f;
typedef IntpTricubic3<double> IntpTricubic3d;

}

#endif
