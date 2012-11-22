#! /bin/bash

# wildMagic:
cd WildMagic4/LibFoundation
make -f makefile.wm4 -j3 CFG=Release SYS=linux GRF=OpenGL


echo 'Please edit CmakeLists.txt (GSI_FOLDER and GEO_FOLDER)'

exit

cd ../../Testing
mkdir build
cmake .. && make -j3
./foo sphere.obj
