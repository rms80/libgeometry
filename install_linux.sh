#! /bin/bash

# wildMagic:
cd WildMagic4/LibFoundation
make -f makefile.wm4 -j3 CFG=Release SYS=linux GRF=OpenGL


echo 'Please edit CMakeLists.txt (GSI_FOLDER and GEO_FOLDER)'

exit

cd ../../Testing
cmake .. && make -j3
./foo sphere.obj
