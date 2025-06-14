# FAQ for build and install

Source build a project sometimes can be not easy. This document helps you out of
the trouble.

## Question 1
```
CMake Error at eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp/download-eigen-populate.cmake:162 (message):
  Each download failed!

    error: downloading 'https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz' failed
          status_code: 35
          status_string: "SSL connect error"
          log:
          --- LOG BEGIN ---
            Trying 172.65.251.78:443...
```
`eigen-3.3.7.tar.gz` is not accessable in your computer. You can check by open this link in the browser.
If so, please fix network issue and continue building.

Another solution is to download `eigen-3.3.7.tar.gz` to local path, and set path in [cmake/eigen.cmake](../cmake/eigen.cmake#L4).

## Question 2
```
Compiling the C compiler identification source file "CMakeCCompilerId.c" failed.
Compiler: /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc 
Build flags: 
Id flags:  

The output was:
1
ld: library 'System' not found
cc: error: linker command failed with exit code 1 (use -v to see invocation)


Compiling the CXX compiler identification source file "CMakeCXXCompilerId.cpp" failed.
Compiler: /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ 
Build flags: 
Id flags:  

The output was:
1
ld: library 'c++' not found
c++: error: linker command failed with exit code 1 (use -v to see invocation)
```

Make sure you have installed xcode and Command Line Tools. Try below commands:
```bash
xcode-select --install
sudo xcode-select --reset
sudo xcode-select -s /Applications/Xcode.app/Contents/Developer

# Then rebuild
rm -rf build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu)
```
