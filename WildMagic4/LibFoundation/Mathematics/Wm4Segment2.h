// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4SEGMENT2_H
#define WM4SEGMENT2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"

namespace Wm4
{

template <class Real>
class Segment2
{
public:
    // The segment is represented as P+t*D, where P is the segment origin,
    // D is a unit-length direction vector and |t| <= e.  The value e is
    // referred to as the extent of the segment.  The end points of the
    // segment are P-e*D and P+e*D.  The user must ensure that the direction
    // vector is unit-length.  The representation for a segment is analogous
    // to that for an oriented bounding box.  P is the center, D is the
    // axis direction, and e is the extent.

    // construction
    Segment2 ();  // uninitialized
    Segment2 (const Vector2<Real>& rkOrigin, const Vector2<Real>& rkDirection,
        Real fExtent);

    // end points
    Vector2<Real> GetPosEnd () const;  // P+e*D
    Vector2<Real> GetNegEnd () const;  // P-e*D

    Vector2<Real> Origin, Direction;
    Real Extent;
};

#include "Wm4Segment2.inl"

typedef Segment2<float> Segment2f;
typedef Segment2<double> Segment2d;

}

#endif
