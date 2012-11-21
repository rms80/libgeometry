call vsvars32_vs2008_x64.bat

devenv /Rebuild ReleaseMD LibFoundation\LibFoundation_VC90.vcproj   2>&1  > ReleaseMD_log.txt

devenv /Rebuild ReleaseMDD LibFoundation\LibFoundation_VC90.vcproj  2>&1  > ReleaseMDD_log.txt

devenv /Rebuild Debug LibFoundation\LibFoundation_VC90.vcproj       2>&1  > Debug_log.txt
