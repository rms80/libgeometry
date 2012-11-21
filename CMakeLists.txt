cmake_minimum_required (VERSION 2.6) 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated   -Wno-extra-tokens -Wno-parentheses  -Wno-c++0x-compat -Wc++0x-extensions -ferror-limit=2 -Wno-enum-compare -lstdc++ -Wno-address-of-temporary -Wno-switch-enum")
set(CMAKE_CXX_COMPILER "/usr/bin/clang")

#set(CMAKE_CXX_COMPILER "/usr/bin/gcc-4.4")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -fpermissive   ")


set(GSI_FOLDER "/work/home/engelhan/code/GSI")
set(GEO_FOLDER "/work/home/engelhan/code/libgeometry")
#include_directories(/usr/include/eigen3/) # replace by find_package
include_directories(external/eigen)



add_definitions(-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)


include_directories(parameterization )
include_directories(geometry) # Frames.h
include_directories(WildMagic4/SDK/Include) # Wm4Vector3.h
include_directories(pointset) # ParticleGrid.h

include_directories(base) #MemoryPool.h
include_directories(mesh) #VFTriangleMesh.h
include_directories(spatial) # IMeshBVTree.h
include_directories(mesh_processing) # MeshUtils.h

include_directories(${GSI_FOLDER}/SparseMatrixSolver)
include_directories(${GEO_FOLDER})
include_directories(${GSI_FOLDER}/Base)

include_directories(mesh_processing) #MeshUtils.h
include_directories(curve) #curve/PolyLoop3.h


include_directories(${GSI_FOLDER}/packages/taucs/build/linux64)


file(GLOB mesh_src
    "*/*.cpp"
    MeshObject.cpp
    Texture.cpp
    config.cpp
    ExtendedWmlCamera.cpp
)




include_directories(${GSI_FOLDER}/SparseMatrixSolver)
include_directories(${GSI_FOLDER}/packages/taucs_full/src)


MESSAGE(STATUS "Compiler: " ${CMAKE_CXX_COMPILER})
Message(STATUS "CXX_Flags: " ${CMAKE_CXX_FLAGS} )
MESSAGE(STATUS "Mesh: " ${mesh_src} )

Message(STATUS "CMAKE_TARGET_DEFINITIONS: ${CMAKE_TARGET_DEFINITIONS}")
Message(STATUS "OpenGl-lib: ${OPENGL_LIBRARIES}")
Message(STATUS "Glut-lib: ${GLUT_LIBRARIES}")


# add_executable (foo test.cpp ${mesh_src})
add_executable (foo expmapCL.cpp ${mesh_src})

TARGET_LINK_LIBRARIES(foo ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${GEO_FOLDER}/WildMagic4/SDK/Library/Release/libWm4Foundation.a)
 
TARGET_LINK_LIBRARIES(foo ${GSI_FOLDER}/build/libgsi.a)
TARGET_LINK_LIBRARIES(foo ${GSI_FOLDER}/packages/taucs_full/lib/linux64/libtaucs.a)
TARGET_LINK_LIBRARIES(foo ${GSI_FOLDER}/packages/UMFPACK/build/libUMFPACK.a)
TARGET_LINK_LIBRARIES(foo ${GSI_FOLDER}/packages/AMD/Lib/libamd.a)
TARGET_LINK_LIBRARIES(foo ${GSI_FOLDER}/packages/taucs_full/external/lib/linux64/libmetis.a)
TARGET_LINK_LIBRARIES(foo ${GSI_FOLDER}/packages/LAPACK3.1.1/liblapackLinux.a)
TARGET_LINK_LIBRARIES(foo ${GSI_FOLDER}/packages/LAPACK3.1.1/libblasLinux.a)
TARGET_LINK_LIBRARIES(foo /usr/lib/gcc/x86_64-linux-gnu/4.6/libgfortran.a)

