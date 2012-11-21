// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPAKIMANONUNIFORM1_H
#define WM4INTPAKIMANONUNIFORM1_H

#include "Wm4FoundationLIB.h"
#include "Wm4IntpAkima1.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpAkimaNonuniform1 : public IntpAkima1<Real>
{
public:
    // Construction and destruction.  IntpAkimaNonuniform1 does not
    // accept responsibility for deleting the input arrays.  The application
    // must do so.  The interpolator is for arbitrarily spaced x-values.
    IntpAkimaNonuniform1 (int iQuantity, Real* afX, Real* afF);
    virtual ~IntpAkimaNonuniform1 ();

    const Real* GetX () const;
    virtual Real GetXMin () const;
    virtual Real GetXMax () const;

protected:
    using IntpAkima1<Real>::m_iQuantity;
    using IntpAkima1<Real>::m_akPoly;

    virtual bool Lookup (Real fX, int& riIndex, Real& rfDX) const;

    Real* m_afX;
};

typedef IntpAkimaNonuniform1<float> IntpAkimaNonuniform1f;
typedef IntpAkimaNonuniform1<double> IntpAkimaNonuniform1d;

}

#endif
