// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4BISECT2_H
#define WM4BISECT2_H

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM Bisect2
{
public:
    typedef Real (*Function)(Real,Real);

    Bisect2 (Function oF, Function oG, int iMaxLevel, Real fTolerance);

    bool Bisect (Real fX0, Real fY0, Real fX1, Real fY1, Real& rfXRoot,
        Real& rfYRoot);

private:
    class WM4_FOUNDATION_ITEM BisectNode
    {
    public:
        Real X, Y, F, G;
        BisectNode* XNext;
        BisectNode* YNext;
    };

    bool ZeroTest (Real fX, Real fY, Real& rfF, Real& rfG, Real& rfXRoot,
        Real& rfYRoot);

    static BisectNode* AddNode (Real fX, Real fY, Real fF, Real fG);

    // input data and functions
    Function m_oF, m_oG;
    int m_iLevel, m_iMaxLevel;
    Real m_fTolerance;
    Real m_fXRoot, m_fYRoot;

    // fixed storage to avoid stack depletion during recursion
    Real m_fX0, m_fXm, m_fX1, m_fY0, m_fYm, m_fY1;
    Real m_fF00, m_fF10, m_fF01, m_fF11, m_fF0m, m_fF1m, m_fFm0, m_fFm1;
    Real m_fFmm, m_fG00, m_fG10, m_fG01, m_fG11, m_fG0m, m_fG1m, m_fGm0;
    Real m_fGm1, m_fGmm;
    int m_iNetSign;

    // the graph and recursion routine for building it
    BisectNode* m_pkGraph;
    bool BisectRecurse (BisectNode* pkN00);
};

typedef Bisect2<float> Bisect2f;
typedef Bisect2<double> Bisect2d;

}

#endif
