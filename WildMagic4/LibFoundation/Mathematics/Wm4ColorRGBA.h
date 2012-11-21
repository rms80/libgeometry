// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4COLORRGBA_H
#define WM4COLORRGBA_H

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

namespace Wm4
{

class WM4_FOUNDATION_ITEM ColorRGBA
{
public:
    // Construction.  The components are intended to be in the interval [0,1].
    // For accessing of colors by array index, the map is 0 = red, 1 = green,
    // 2 = blue, and 3 = alpha.
    ColorRGBA ();  // initial values (0,0,0,0)
    ColorRGBA (float fR, float fG, float fB, float fA);
    ColorRGBA (const float* afTuple);
    ColorRGBA (const ColorRGBA& rkC);

    // member access
    operator const float* () const;
    operator float* ();
    float operator[] (int i) const;
    float& operator[] (int i);
    float R () const;
    float& R ();
    float G () const;
    float& G ();
    float B () const;
    float& B ();
    float A () const;
    float& A ();

    // assignment
    ColorRGBA& operator= (const ColorRGBA& rkC);

    // comparison
    bool operator== (const ColorRGBA& rkC) const;
    bool operator!= (const ColorRGBA& rkC) const;
    bool operator<  (const ColorRGBA& rkC) const;
    bool operator<= (const ColorRGBA& rkC) const;
    bool operator>  (const ColorRGBA& rkC) const;
    bool operator>= (const ColorRGBA& rkC) const;

    // arithmetic operations
    ColorRGBA operator+ (const ColorRGBA& rkC) const;
    ColorRGBA operator- (const ColorRGBA& rkC) const;
    ColorRGBA operator* (const ColorRGBA& rkC) const;
    ColorRGBA operator* (float fScalar) const;

    // arithmetic updates
    ColorRGBA& operator+= (const ColorRGBA& rkC);
    ColorRGBA& operator-= (const ColorRGBA& rkC);
    ColorRGBA& operator*= (const ColorRGBA& rkC);
    ColorRGBA& operator*= (float fScalar);

    // Transform the color channels to [0,1].  Clamp sets negative values to
    // zero and values larger than one to one.  ScaleByMax assumes the color
    // channels are nonnegative, finds the maximum RGB channel, and divides
    // all RGB channels by that value.  The alpha channel is clamped to [0,1].
    void Clamp ();
    void ScaleByMax ();

    static const ColorRGBA BLACK;   // = (0,0,0,1) 
    static const ColorRGBA WHITE;   // = (1,1,1,1)
    static const ColorRGBA INVALID; // = (-1,-1,-1,-1)

private:
    // support for comparisons
    int CompareArrays (const ColorRGBA& rkC) const;

    float m_afTuple[4];
};

WM4_FOUNDATION_ITEM ColorRGBA operator* (float fScalar, const ColorRGBA& rkC);

#include "Wm4ColorRGBA.inl"

}

#endif
