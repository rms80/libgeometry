// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4MAPPER2_H
#define WM4MAPPER2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"

namespace Wm4
{

template <class Real>
class Mapper2
{
public:
    // Construction and destruction.  The value of epsilon is used as a
    // relative error when computing the dimension of the point set.
    Mapper2 (int iVQuantity, const Vector2<Real>* akVertex, Real fEpsilon);
    ~Mapper2 ();

    // Axis-aligned bounding box of the input points.
    const Vector2<Real>& GetMin () const;
    const Vector2<Real>& GetMax () const;
    Real GetMaxRange () const;

    // Dimension d of the set (0, 1, or 2).
    int GetDimension () const;

    // Coordinate system.  The origin is valid for any dimension d.  The
    // unit-length direction vector is valid only for 0 <= i < d.  The extreme
    // index is relative to the array of input points, and is also valid only
    // for 0 <= i < d.  If d = 0, all points are effectively the same, but the
    // use of an epsilon may lead to an extreme index that is not zero.  If
    // d = 1, all points effectively lie on a line segment.  The extreme
    // indices correspond to input points that are the end points of the
    // segment.  If d = 2, the first two extreme indices correspond to a
    // line segment.  The next extreme index corresponds to the input point
    // that is farthest from this line segment in the direction perpendicular
    // to the segment.
    const Vector2<Real>& GetOrigin () const;
    const Vector2<Real>& GetDirection (int i) const;
    int GetExtremeIndex (int i) const;

    // If d = 2, the direction vectors {U0,U1} form a right-handed set.  The
    // three extreme points form a triangle.  This function indicates if that
    // triangle is counterclockwise ordered.
    bool GetExtremeCCW () const;

private:
    // Axis-aligned bounding box of input points.  The maximum range is the
    // larger of max[0]-min[0] and max[1]-min[1].
    Vector2<Real> m_kMin, m_kMax;
    Real m_fMaxRange;

    // The intrinsic dimension of the input set.  The parameter fEpsilon to
    // the constructor is used to provide a tolerance when determining the
    // dimension.
    int m_iDimension;

    // The indices that define the maximum dimensional extents.  The values
    // m_aiExtreme[0] and m_aiExtreme[1] are the indices for the vertices
    // which define the largest extent in one of the coordinate axis
    // directions.  If the intrinsic dimensionality is 2, then m_aiExtreme[2]
    // is the index for the vertex which causes the largest extent in the
    // direction perpendicular to the line through the vertices corresponding
    // to m_aiExtreme[0] and m_aiExtreme[1].  The triangle
    // <V[extreme0],V[extreme1],V[extreme2]> can be clockwise or
    // counterclockwise, the condition stored in m_bExtremeCCW.
    int m_aiExtreme[3];
    bool m_bExtremeCCW;

    // See the comments describing the member functions which return these
    // values.
    Vector2<Real> m_kOrigin;
    Vector2<Real> m_akDirection[2];
};

#include "Wm4Mapper2.inl"

typedef Mapper2<float> Mapper2f;
typedef Mapper2<double> Mapper2d;

}

#endif
