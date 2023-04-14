# How to bulid

This document describes the Windows/Linux build method of DRFL (Doosan Robotics Framework Library).
The manual can be found at the link below.

[Manual](https://manual.doosanrobotics.com/help/api/latest/publish/en_us/api-manual-36471062.html)

## Environment

Please refer to below link.

### [Composition of Library](https://manual.doosanrobotics.com/help/api/latest/publish/en_us/composition-of-library-36471066.html)
### [Recommended Operational Specification](https://manual.doosanrobotics.com/help/api/latest/publish/en_us/recommended-operational-specification-50890483.html)

> DRFL only works on the x86 architecture.

## Build Instruction
The library files (.a, .dll, etc.) included in this repository are library files for examples and may not be the latest version. 
The latest version of DRFL can be downloaded from the link below.
[Download Site](https://robotlab.doosanrobotics.com/en/board/Resources/Software/b1093461-88c9-eb11-bacc-000d3aa2bc06?pageId=864ff7c8-5545-e911-a824-000d3a07f6fe&searchKeyword=)

### Windows

The link below is a link to the solution file for using the windows example.

[Visual Studio 2010 Solution](https://github.com/doosan-robotics/API-DRFL/blob/main/example/Windows/windows_example/windows_example.sln)


### Linux(64bits)

#### Ubuntu 20.04

1. Build with g++ in the example directory
`$ g++ -c main.cpp`
2.	If main.o is created normally, enter the command below to create an example file

`$ g++ -o drfl_tets main.o ../../library/Linux/64bits/20.04/libDRFL.a /usr/lib/libPocoFoundation.so /usr/lib/libPocoNet.so`
3.	Check the build. When the build is complete, test the actual controller and connection.

> The Poco library can be downloaded with the command below.
> **$ sudo apt-get install libpoco-dev**
