// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4CONFORMALMAP_H
#define WM4CONFORMALMAP_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"
#include "Wm4Vector3.h"
#include "Wm4BasicMesh.h"
#include "Wm4LinearSystem.h"
#include "Wm4PolynomialRoots.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM ConformalMap
{
public:
    // The input mesh should be a closed, manifold surface.  That is, it must
    // have the topology of a sphere.
    //
    // The number of vertices in the input is iVQuantity.  The vertex array
    // is usually passed as akPoint, but this input can be any data type you
    // prefer (points+attributes).  The number of triangles is iTQuantity.
    // The triangles are represented as triples of indices into the vertex
    // array.  These triples are stored in aiIndex.  The caller is responsible
    // for deleting the input arrays.
    ConformalMap (int iVQuantity, const Vector3<Real>* akPoint,
        int iTQuantity, const int* aiIndex, int iPunctureTriangle = 0);

    ~ConformalMap ();

    // Conformal mapping of mesh to plane.  The array of coordinates has a
    // one-to-one correspondence with the input vertex array.
    const Vector2<Real>* GetPlaneCoordinates () const;
    const Vector2<Real>& GetPlaneMin () const;
    const Vector2<Real>& GetPlaneMax () const;

    // Conformal mapping of mesh to sphere (centered at origin).  The array
    // of coordinates has a one-to-one correspondence with the input vertex
    // array.
    const Vector3<Real>* GetSphereCoordinates () const;
    Real GetSphereRadius () const;

private:
    Real ComputeRadius (const Vector2<Real>& rkV0, const Vector2<Real>& rkV1,
        const Vector2<Real>& rkV2, Real fAreaFraction) const;

    // Conformal mapping to a plane.  The plane's (px,py) points correspond to
    // the mesh's (mx,my,mz) points.
    Vector2<Real>* m_akPlane;
    Vector2<Real> m_kPlaneMin, m_kPlaneMax;

    // Conformal mapping to a sphere.  The sphere's (sx,sy,sz) points
    // correspond to the mesh's (mx,my,mz) points.
    Vector3<Real>* m_akSphere;
    Real m_fRadius;
};

typedef ConformalMap<float> ConformalMapf;
typedef ConformalMap<double> ConformalMapd;

}

#endif
