// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4MINIMIZE1_H
#define WM4MINIMIZE1_H

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM Minimize1
{
public:
    typedef Real (*Function)(Real,void*);

    Minimize1 (Function oFunction, int iMaxLevel, int iMaxBracket,
        void* pvData = 0);

    int& MaxLevel ();
    int& MaxBracket ();
    void*& UserData ();

    void GetMinimum (Real fT0, Real fT1, Real fTInitial, Real& rfTMin,
        Real& rfFMin);

private:
    Function m_oFunction;
    int m_iMaxLevel, m_iMaxBracket;
    Real m_fTMin, m_fFMin;
    void* m_pvData;

    void GetMinimum (Real fT0, Real fF0, Real fT1, Real fF1, int iLevel);

    void GetMinimum (Real fT0, Real fF0, Real fTm, Real fFm, Real fT1,
        Real fF1, int iLevel);

    void GetBracketedMinimum (Real fT0, Real fF0, Real fTm,
        Real fFm, Real fT1, Real fF1, int iLevel);
};

typedef Minimize1<float> Minimize1f;
typedef Minimize1<double> Minimize1d;

}

#endif
