// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4MESHSMOOTHER_H
#define WM4MESHSMOOTHER_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM MeshSmoother
{
public:
    // The caller is responsible for deleting the input arrays.
    MeshSmoother (int iVQuantity, Vector3<Real>* akVertex, int iTQuantity,
        const int* aiIndex);

    virtual ~MeshSmoother ();

    // For deferred construction and destruction.  The caller is responsible
    // for deleting the input arrays.
    MeshSmoother ();
    void Create (int iVQuantity, Vector3<Real>* akVertex, int iTQuantity,
        const int* aiIndex);
    void Destroy ();

    // input values from the constructor
    int GetVQuantity () const;
    const Vector3<Real>* GetVertices () const;
    int GetTQuantity () const;
    const int* GetIndices () const;

    // derived quantites from the input mesh
    const Vector3<Real>* GetNormals () const;
    const Vector3<Real>* GetMeans () const;

    // Apply one iteration of the smoother.  The input time is supported for
    // applications where the surface evolution is time-dependent.
    void Update (Real fTime = (Real)0.0);

protected:
    virtual bool VertexInfluenced (int i, Real fTime);
    virtual Real GetTangentWeight (int i, Real fTime);
    virtual Real GetNormalWeight (int i, Real fTime);

    int m_iVQuantity;
    Vector3<Real>* m_akVertex;
    int m_iTQuantity;
    const int* m_aiIndex;

    Vector3<Real>* m_akNormal;
    Vector3<Real>* m_akMean;
    int* m_aiNeighborCount;
};

typedef MeshSmoother<float> MeshSmootherf;
typedef MeshSmoother<double> MeshSmootherd;

}

#endif
