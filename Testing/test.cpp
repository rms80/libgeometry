

#include <cstddef>
#include "VFTriangleMesh.h"

//#include "pointset/ParticleGrid.h"
#include "MeshObject.h"
#include "parameterization/ExpMapGenerator.h"

using namespace std;

enum EXPMAP_NBR_MODE {
  MaxGeoDist,
  MaxK,
  Hybrid
};



int main(int argc, char ** args){
  // rms::ParticleGrid<int> p_grid;

  // string name("/usr/gast/engelhan/ros/master_thesis/libgeometry/build/test.obj");

  rms::VFTriangleMesh mesh;
  // rms::ExpMapGenerator generator();
  //MeshObject g_mesh;
  // g_mesh.ReadMeshOBJ("/usr/gast/engelhan/ros/master_thesis/libgeometry/build/sphere.obj");
  
  std::string err;
  mesh.ReadOBJ("sphere.obj",err);

  // std::cout << "Object" << endl;

  // return 23;

  rms::ExpMapGenerator expmapgen;
  //expmapgen.SetUseNeighbourNormalSmoothing(false);
  //expmapgen.SetUseUpwindAveraging(false);
  rms::IMeshBVTree bvTree;

  bvTree.SetMesh(&mesh);
  expmapgen.SetSurface(&mesh, &bvTree);



  Wml::Vector3f vVertex, vNormal;
  mesh.GetVertex(1, vVertex, &vNormal);
  rms::Frame3f vInitFrame(vVertex, vNormal);
  expmapgen.SetSurfaceDistances( vInitFrame, 0.0f, 1 );


  // _RMSTUNE_end(1);
  // std::cerr << "[expmapCL] neighbour precomputation: " << _RMSTUNE_time(1) << "s" << std::endl;
  // std::cerr << "           (threshold is : " << expmapgen.GetNeighbourThreshold() << ")" << std::endl;

  // std::ostream & output = std::cout;

  std::vector<unsigned int> vIdx;
  std::vector<float> vU, vV;
  rms::VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
  unsigned int nExpmaps = mesh.GetVertexCount();
  float fAvgNbrs = 0;

  int eMode = MaxK;
  int nMaxK = 10;

  while ( curv != endv ) {
    // _RMSTUNE_start(2);
    rms::IMesh::VertexID vID = *curv;  curv++;
    mesh.GetVertex(vID, vVertex, &vNormal);
    rms::Frame3f vSeedFrame(vVertex, vNormal);

    switch ( eMode ) {
    case MaxK:
      expmapgen.SetSurfaceDistances( vSeedFrame, 0.0f, nMaxK);
      break;
      //		  case MaxGeoDist:
      //			  expmapgen.SetSurfaceDistances( vVertex, 0.0f, fMaxGeoDist, &vSeedFrame);
      //			  break;
      //		  case Hybrid:
      //			  expmapgen.SetSurfaceDistances( vSeedFrame, 0.0f, fMaxGeoDist, nMaxK);
      //			  break;
    }

    vIdx.resize(0); vU.resize(0); vV.resize(0);
    expmapgen.GetVertexUVs(vIdx, vU, vV);
    // _RMSTUNE_end(2);
    // _RMSTUNE_accum(2);
    size_t nCount = vIdx.size();

    //			output << (vID+1) << " -999 -999" << std::endl;
    for ( unsigned int k = 0; k < nCount; ++k ) {
      //				output << (vIdx[k]+1) << " " << vU[k] << " " << vV[k] << " " << sqrt(vU[k]*vU[k]+vV[k]*vV[k]) << std::endl;
      //				output << (vIdx[k]+1) << " " << vU[k] << " " << vV[k] << std::endl;
      cout << (vID+1) << " " << (vIdx[k]+1) << " " << vU[k] << " " << vV[k] << std::endl;
    }

    fAvgNbrs += (float)nCount / (float)nExpmaps;
  }
  //float fTotalTime = _RMSTUNE_accum_time(2);
  //float EPS = nExpmaps / fTotalTime;
  //std::cerr << "[expmapCL] computed " << nExpmaps << " expmaps in "  << fTotalTime << "s" << std::endl;
  //std::cerr << "    average nbr count: " << fAvgNbrs << std::endl;
  //std::cerr << "    expmaps per second " << EPS << std::endl;

  return 42;
}
