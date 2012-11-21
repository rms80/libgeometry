// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4IntpVectorField2.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
IntpVectorField2<Real>::IntpVectorField2 (int iQuantity,
    Vector2<Real>* akInput, Vector2<Real>* akOutput, bool bOwner,
    Query::Type eQueryType)
{
    // Repackage the output vectors into individual components.  This is
    // required because of the format that the quadratic interpolator expects
    // for its input data.
    Real* afXOutput = WM4_NEW Real[iQuantity];
    Real* afYOutput = WM4_NEW Real[iQuantity];
    for (int i = 0; i < iQuantity; i++)
    {
        afXOutput[i] = akOutput[i].X();
        afYOutput[i] = akOutput[i].Y();
    }

    if (bOwner)
    {
        WM4_DELETE[] akOutput;
    }

    // common triangulator for the interpolators
    m_pkDel = WM4_NEW Delaunay2<Real>(iQuantity,akInput,(Real)0.001,bOwner,
        eQueryType);

    // Create interpolator for x-coordinate of vector field.
    m_pkXInterp = WM4_NEW IntpQdrNonuniform2<Real>(*m_pkDel,afXOutput,true);

    // Create interpolator for y-coordinate of vector field, but share the
    // already created triangulation for the x-interpolator.
    m_pkYInterp = WM4_NEW IntpQdrNonuniform2<Real>(*m_pkDel,afYOutput,true);
}
//----------------------------------------------------------------------------
template <class Real>
IntpVectorField2<Real>::~IntpVectorField2 ()
{
    WM4_DELETE m_pkDel;
    WM4_DELETE m_pkXInterp;
    WM4_DELETE m_pkYInterp;
}
//----------------------------------------------------------------------------
template <class Real>
bool IntpVectorField2<Real>::Evaluate (const Vector2<Real>& rkInput,
    Vector2<Real>& rkOutput)
{
    Real fXDeriv, fYDeriv;
    return m_pkXInterp->Evaluate(rkInput,rkOutput.X(),fXDeriv,fYDeriv)
        && m_pkYInterp->Evaluate(rkInput,rkOutput.Y(),fXDeriv,fYDeriv);
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
class IntpVectorField2<float>;

template WM4_FOUNDATION_ITEM
class IntpVectorField2<double>;
//----------------------------------------------------------------------------
}
