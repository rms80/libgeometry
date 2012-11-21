// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4TMINHEAP_H
#define WM4TMINHEAP_H

#include "Wm4System.h"

namespace Wm4
{

template <typename Generator, typename Real> class TMinHeap;

template <typename Generator, typename Real>
class TMinHeapRecord
{
public:
    TMinHeapRecord ();
    ~TMinHeapRecord ();

    Generator GetGenerator () const;
    Real GetValue () const;

private:
    friend class TMinHeap<Generator,Real>;

    Generator m_tGenerator;
    Real m_fValue;
    int m_iIndex;
};

template <typename Generator, typename Real>
class TMinHeap
{
public:
    TMinHeap (int iMaxQuantity, int iGrowBy);
    ~TMinHeap ();

    // Member access.
    int GetMaxQuantity () const;
    int GetGrowBy () const;
    int GetQuantity () const;
    const TMinHeapRecord<Generator,Real>* GetRecord (int i) const;

    // Insert into the heap the number fValue that corresponds to the object
    // identified by iGenerator.  The return value is a pointer to the heap
    // record storing the information.
    const TMinHeapRecord<Generator,Real>* Insert (Generator tGenerator,
        Real fValue);

    // Remove the root of the heap.  The root contains the minimum value of
    // all heap elements.  The root information is returned by the function's
    // output parameters.
    void Remove (Generator& rtGenerator, Real& rfValue);

    // The value of a heap record must be modified through this function call.
    // The side effect is that the heap must be updated accordingly to
    // accommodate the new value.
    void Update (const TMinHeapRecord<Generator,Real>* pkConstRecord,
        Real fValue);

    // Support for debugging.  The first two functions check if the array of
    // records really do form a heap.  The last function prints the heap
    // to a file.
    bool IsValid (int iStart, int iFinal);
    bool IsValid ();
    void Print (const char* acFilename);

private:
    // The actual record storage, allocated in one large chunk.
    int m_iMaxQuantity, m_iGrowBy, m_iQuantity;
    TMinHeapRecord<Generator,Real>* m_akRecords;

    // Pointers to the records in storage.  The two-level system avoids the
    // large number of allocations and deallocations that would occur if each
    // element of m_apkRecord were to be allocated/deallocated individually.
    TMinHeapRecord<Generator,Real>** m_apkRecords;
};

#include "Wm4TMinHeap.inl"

}

#endif
