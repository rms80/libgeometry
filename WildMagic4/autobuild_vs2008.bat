call vsvars32_vs2008.bat

devenv /build ReleaseMD LibFoundation\LibFoundation_VC90.vcproj   2>&1  > ReleaseMD_log.txt

devenv /build ReleaseMDD LibFoundation\LibFoundation_VC90.vcproj  2>&1  > ReleaseMDD_log.txt

devenv /build Debug LibFoundation\LibFoundation_VC90.vcproj       2>&1  > Debug_log.txt
