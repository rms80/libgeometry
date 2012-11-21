// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4THASHSET_H
#define WM4THASHSET_H

#include "Wm4FoundationLIB.h"

// The class TKEY is either native data or is class data that has the
// following member functions:
//   TKEY::TKEY ()
//   TKEY& TKEY::operator= (const TKEY&)
//   bool TKEY::operator== (const TKEY&) const
//   bool TKEY::operator!= (const TKEY&) const
//   TKEY::operator unsigned int () const
// The implicit conversion to unsigned int is used to select a hash table
// index for the T object.  The return value need not be within the range of
// hash table indices.  THashSet will use modular arithmetic to make this
// happen.

#include "Wm4System.h"

namespace Wm4
{

template <class TKEY>
class THashSet
{
public:
    // construction and destruction
    THashSet (int iTableSize);
    ~THashSet ();

    // element access
    int GetQuantity () const;

    // A pointer to the actual storage is returned so that the caller has
    // direct access to it.  This allows a subset of TKEY members to be used
    // in key comparison.
    TKEY* Insert (const TKEY& rtKey);

    // If the input key exists, a pointer to the actual storage is returned.
    // This allows a subset of TKEY members to be used in key comparison,
    // but gives the caller a chance to modify other TKEY members.
    TKEY* Get (const TKEY& rtKey) const;

    bool Remove (const TKEY& rtKey);
    void RemoveAll ();

    // linear traversal of map
    TKEY* GetFirst () const;
    TKEY* GetNext () const;

    // user-specified key-to-index construction
    int (*UserHashFunction)(const TKEY&);

private:
    class HashItem
    {
    public:
        TKEY m_tKey;
        HashItem* m_pkNext;
    };

    // Default key-to-index construction (override by user-specified when
    // requested).
    int HashFunction (const TKEY& rtKey) const;

    // hash table
    int m_iTableSize;
    int m_iQuantity;
    HashItem** m_apkTable;

    // iterator for traversal
    mutable int m_iIndex;
    mutable HashItem* m_pkItem;
};

#include "Wm4THashSet.inl"

}

#endif
