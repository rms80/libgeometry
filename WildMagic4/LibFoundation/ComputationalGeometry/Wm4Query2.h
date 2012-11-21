// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4QUERY2_H
#define WM4QUERY2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Query.h"
#include "Wm4Vector2.h"

namespace Wm4
{

template <class Real>
class Query2 : public Query
{
public:
    // The base class handles floating-point queries.
    Query2 (int iVQuantity, const Vector2<Real>* akVertex);
    virtual ~Query2 ();

    // run-time type information
    virtual Query::Type GetType () const;

    // member access
    int GetQuantity () const;
    const Vector2<Real>* GetVertices () const;

    // Queries about the relation of a point to various geometric objects.

    // returns
    //   +1, on right of line
    //   -1, on left of line
    //    0, on the line
    virtual int ToLine (int i, int iV0, int iV1) const;
    virtual int ToLine (const Vector2<Real>& rkP, int iV0, int iV1) const;

    // returns
    //   +1, outside triangle
    //   -1, inside triangle
    //    0, on triangle
    virtual int ToTriangle (int i, int iV0, int iV1, int iV2) const;
    virtual int ToTriangle (const Vector2<Real>& rkP, int iV0, int iV1,
        int iV2) const;

    // returns
    //   +1, outside circumcircle of triangle
    //   -1, inside circumcircle of triangle
    //    0, on circumcircle of triangle
    virtual int ToCircumcircle (int i, int iV0, int iV1, int iV2) const;
    virtual int ToCircumcircle (const Vector2<Real>& rkP, int iV0, int iV1,
        int iV2) const;

protected:
    // input points
    int m_iVQuantity;
    const Vector2<Real>* m_akVertex;

    static Real Dot (Real fX0, Real fY0, Real fX1, Real fY1);

    static Real Det2 (Real fX0, Real fY0, Real fX1, Real fY1);

    static Real Det3 (Real iX0, Real iY0, Real iZ0, Real iX1, Real iY1,
        Real iZ1, Real iX2, Real iY2, Real iZ2);
};

#include "Wm4Query2.inl"

typedef Query2<float> Query2f;
typedef Query2<double> Query2d;

}

#endif
