# Building and modifying patch_brand tool

## Where patch_brand normally exists:
```
$ which patch_brand
/usr/local/ics/bin/patch_brand
```

## Usage for patch_brand:
```
$ patch_brand -h
patch_brand: invalid option -- 'h'
Usage:     patch_brand [-m marker] brand [file ...]
```

## How embedded build runs patch_brand:
This line in Makerules/Maketargets.bsp is run:
`$(PATCH_BRAND) "$(BUILD_BRAND)" $(EXECUTABLE)`
It will appear as follows in the capture build output for embedded:
`$ patch_brand "" build.STL1.q7.release/STL1.q7`
Note: The empty brand string tells patch_brand, aka, patch_version.c, to use DEFAULT_BRAND.

## How to tell if the file has the default brand of Intel:
```
grep Intel  <ifs-all-dir>/STL1_Firmware/Firmware/build.STL1.q7.release/STL1.q7
Binary file ../STL1_Firmware/Firmware/build.STL1.q7.release/STL1.q7 matches
```

## How to build patch_version by itself  (make your changes in opa-dev branch only):
```
cd MakeTools/patch_version/
target linux
setrel
setver  ##choose 1 (which is probably the default Linux that was booted)
export PROJ_FILE_DIR=OpenIb_Host
make
```
You will find patch_brand executable in your current directory.

## How to run patch_version to test your change:
`./patch_brand “” <ifs-all-dir>/STL1_Firmware/Firmware/build.release*/STL1.q7`
For example:
```
./patch_brand "" ~/HEAD-10_8/ifs-all/STL1_Firmware/Firmware/build.STL1.q7.release/STL1.q7
Patching /home/usrinivasan/HEAD-10_8/ifs-all/STL1_Firmware/Firmware/build.STL1.q7.release/STL1.q7 with brand 'Cornelis'
```

## How to tell if the file has the default brand of Cornelis:
`grep Cornelis <ifs-all-dir>/STL1_Firmware/Firmware/build.release*/STL1.q7`
For example:
```
grep "THIS_IS_THE_ICS_BRAND:Cornelis" /home/usrinivasan/HEAD-10_8/ifs-all/STL1_Firmware/Firmware/build.STL1.q7.release/STL1.q7
Binary file /home/usrinivasan/HEAD-10_8/ifs-all/STL1_Firmware/Firmware/build.STL1.q7.release/STL1.q7 matches
```
## Finally, verify that a grep for Intel yields nothing.
`grep "THIS_IS_THE_ICS_BRAND:Intel"  <ifs-all-dir>/STL1_Firmware/Firmware/build.STL1.q7.release/STL1.q7




