# wildMagic:
cd LibFoundation
make -f makefile.wm4 CFG=Release SYS=linux GRF=OpenGL

# to compile a demo program, go into Testing and change the GSI_FOLDER/GEO_FOLDER-variables
to fit to your installation. 
then simple
cmake .. && make && ./foo sphere.obj
will compile and start the demo


TODO

- in MeshSubdivider.cpp (l 113.) there is still a line not converted to linux (triggers a 1==0-assertion)

- the Timing/Debug-Functions in rmsdebug.* are not yet implemented on Linux. 
