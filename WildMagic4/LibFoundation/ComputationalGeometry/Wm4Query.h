// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4QUERY_H
#define WM4QUERY_H

#include "Wm4FoundationLIB.h"

namespace Wm4
{

class Query
{
public:
    // abstract base class
    virtual ~Query ();

    // run-time type information
    enum Type
    {
        QT_INT64,
        QT_INTEGER,
        QT_RATIONAL,
        QT_REAL,
        QT_FILTERED
    };

    virtual Type GetType () const = 0;

    // Support for ordering a set of unique indices into the vertex pool.  On
    // output it is guaranteed that:  v0 < v1 < v2.  This is used to guarantee
    // consistent queries when the vertex ordering of a primitive is permuted,
    // a necessity when using floating-point arithmetic that suffers from
    // numerical round-off errors.  The input indices are considered the
    // positive ordering.  The output indices are either positively ordered
    // (an even number of transpositions occurs during sorting) or negatively
    // ordered (an odd number of transpositions occurs during sorting).  The
    // functions return 'true' for a positive ordering and 'false' for a
    // negative ordering.
    static bool Sort (int& iV0, int& iV1);
    static bool Sort (int& iV0, int& iV1, int& iV2);
    static bool Sort (int& iV0, int& iV1, int& iV2, int& iV3);

protected:
    Query ();
};

}

#endif
