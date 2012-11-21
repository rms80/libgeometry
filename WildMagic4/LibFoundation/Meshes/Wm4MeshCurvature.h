// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4MESHCURVATURE_H
#define WM4MESHCURVATURE_H

#include "Wm4FoundationLIB.h"
#include "Wm4Matrix3.h"
#include "Wm4Matrix2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM MeshCurvature
{
public:
    // The caller is responsible for deleting the input arrays.
    MeshCurvature (int iVQuantity, const Vector3<Real>* akVertex,
        int iTQuantity, const int* aiIndex);

    virtual ~MeshCurvature ();

    // input values from the constructor
    int GetVQuantity () const;
    const Vector3<Real>* GetVertices () const;
    int GetTQuantity () const;
    const int* GetIndices () const;

    // derived quantites from the input mesh
    const Vector3<Real>* GetNormals () const;
    const Real* GetMinCurvatures () const;
    const Real* GetMaxCurvatures () const;
    const Vector3<Real>* GetMinDirections () const;
    const Vector3<Real>* GetMaxDirections () const;

protected:
    int m_iVQuantity;
    const Vector3<Real>* m_akVertex;
    int m_iTQuantity;
    const int* m_aiIndex;

    Vector3<Real>* m_akNormal;
    Real* m_afMinCurvature;
    Real* m_afMaxCurvature;
    Vector3<Real>* m_akMinDirection;
    Vector3<Real>* m_akMaxDirection;
};

typedef MeshCurvature<float> MeshCurvaturef;
typedef MeshCurvature<double> MeshCurvatured;

}

#endif
