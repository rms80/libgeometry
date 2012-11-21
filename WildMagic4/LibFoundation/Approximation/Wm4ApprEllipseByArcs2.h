// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4APPRELLIPSEBYARCS2_H
#define WM4APPRELLIPSEBYARCS2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"

namespace Wm4
{

// The ellipse is (x/a)^2 + (y/b)^2 = 1, but only the portion in the first
// quadrant (x >= 0 and y >= 0) is approximated.  Generate iNumArcs >= 2 arcs
// by constructing points corresponding to the weighted averages of the
// curvatures at the ellipse points (a,0) and (0,b).  The returned input point
// array has iNumArcs+1 elements and the returned input center and radius
// arrays each have iNumArc elements.  The arc associated with rakPoint[i] and
// rakPoint[i+1] has center rakCenter[i] and radius rafRadius[i].

template <class Real> WM4_FOUNDATION_ITEM
void ApproximateEllipseByArcs (Real fA, Real fB, int iNumArcs,
    Vector2<Real>*& rakPoint, Vector2<Real>*& rakCenter, Real*& rafRadius);

}

#endif
