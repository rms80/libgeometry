// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4IVECTOR2_H
#define WM4IVECTOR2_H

#include "Wm4FoundationLIB.h"
#include "Wm4TIVector.h"

namespace Wm4
{

class IVector2 : public TIVector<2>
{
public:
    // construction
    IVector2 ();
    IVector2 (const IVector2& rkV);
    IVector2 (const TIVector<2>& rkV);
    IVector2 (const Integer64& riX, const Integer64& riY);

    // member access
    Integer64 X () const;
    Integer64& X ();
    Integer64 Y () const;
    Integer64& Y ();

    // assignment
    IVector2& operator= (const IVector2& rkV);
    IVector2& operator= (const TIVector<2>& rkV);

    // returns Dot(this,V)
    Integer64 Dot (const IVector2& rkV) const;

    // returns (y,-x)
    IVector2 Perp () const;

    // returns Cross((x,y,0),(V.x,V.y,0)) = x*V.y - y*V.x
    Integer64 DotPerp (const IVector2& rkV) const;

protected:
    using TIVector<2>::m_aiTuple;
};

#include "Wm4IVector2.inl"

}

#endif
