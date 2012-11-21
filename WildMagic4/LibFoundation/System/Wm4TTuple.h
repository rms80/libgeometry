// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4TTUPLE_H
#define WM4TTUPLE_H

#include "Wm4FoundationLIB.h"
#include "Wm4System.h"

// The class TYPE is either native data or is class data that has the
// following member functions:
//   TYPE::TYPE ()
//   TYPE::TYPE (const TYPE&);
//   TYPE& TYPE::operator= (const TYPE&)

namespace Wm4
{

template <int DIMENSION, class TYPE>
class TTuple
{
public:
    // Construction and destruction.  The default constructor does not
    // initialize the tuple elements for native elements.  The tuple elements
    // are initialized for class data whenever TYPE initializes during its
    // default construction.
    TTuple ();
    TTuple (const TTuple& rkT);
    ~TTuple ();

    // coordinate access
    operator const TYPE* () const;
    operator TYPE* ();
    TYPE operator[] (int i) const;
    TYPE& operator[] (int i);

    // assignment
    TTuple& operator= (const TTuple& rkT);

    // Comparison.  The inequalities make the comparisons using memcmp, thus
    // treating the tuple as an array of unsigned bytes.
    bool operator== (const TTuple& rkT) const;
    bool operator!= (const TTuple& rkT) const;
    bool operator<  (const TTuple& rkT) const;
    bool operator<= (const TTuple& rkT) const;
    bool operator>  (const TTuple& rkT) const;
    bool operator>= (const TTuple& rkT) const;

private:
    TYPE m_atTuple[DIMENSION];
};

#include "Wm4TTuple.inl"

}

#endif
