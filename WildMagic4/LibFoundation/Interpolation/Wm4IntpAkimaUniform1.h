// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPAKIMAUNIFORM1_H
#define WM4INTPAKIMAUNIFORM1_H

#include "Wm4FoundationLIB.h"
#include "Wm4IntpAkima1.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpAkimaUniform1 : public IntpAkima1<Real>
{
public:
    // Construction and destruction.  IntpAkimaUniform1 accepts
    // responsibility for deleting the input array.  The interpolator is for
    // uniformly spaced x-values.
    IntpAkimaUniform1 (int iQuantity, Real fXMin, Real fXSpacing, Real* afF);
    virtual ~IntpAkimaUniform1 ();

    virtual Real GetXMin () const;
    virtual Real GetXMax () const;
    Real GetXSpacing () const;

protected:
    using IntpAkima1<Real>::m_iQuantity;
    using IntpAkima1<Real>::m_akPoly;

    virtual bool Lookup (Real fX, int& riIndex, Real& rfDX) const;

    Real m_fXMin, m_fXMax, m_fXSpacing;
};

typedef IntpAkimaUniform1<float> IntpAkimaUniform1f;
typedef IntpAkimaUniform1<double> IntpAkimaUniform1d;

}

#endif
