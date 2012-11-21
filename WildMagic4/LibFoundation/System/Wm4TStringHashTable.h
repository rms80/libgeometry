// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4TSTRINGHASHTABLE_H
#define WM4TSTRINGHASHTABLE_H

#include "Wm4FoundationLIB.h"

// The class TVALUE is either native data or is class data that has the
// following member functions:
//   TVALUE::TVALUE ()
//   TVALUE& TVALUE::operator= (const TVALUE&)

#include "Wm4System.h"

namespace Wm4
{

template <class TVALUE>
class TStringHashTable
{
public:
    // construction and destruction
    TStringHashTable (int iTableSize);
    ~TStringHashTable ();

    // element access
    int GetQuantity () const;

    // insert a key-value pair into the hash table
    bool Insert (const std::string& rkKey, const TVALUE& rtValue);

    // search for a key and returns it value (null, if key does not exist)
    TVALUE* Find (const std::string& rkKey) const;

    // remove key-value pairs from the hash table
    bool Remove (const std::string& rkKey);
    void RemoveAll ();

    // linear traversal of table
    TVALUE* GetFirst (std::string* pkKey) const;
    TVALUE* GetNext (std::string* pkKey) const;

private:
    class HashItem
    {
    public:
        HashItem () : m_kKey("") { /**/ }

        std::string m_kKey;
        TVALUE m_tValue;
        HashItem* m_pkNext;
    };

    // key-to-index construction
    int HashFunction (const std::string& rkKey) const;

    // hash table
    int m_iTableSize;
    int m_iQuantity;
    HashItem** m_apkTable;

    // iterator for traversal
    mutable int m_iIndex;
    mutable HashItem* m_pkItem;
};

#include "Wm4TStringHashTable.inl"

}

#endif
