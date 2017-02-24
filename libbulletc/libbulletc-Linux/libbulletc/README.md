# Linux libbulletc README

## Build instructions

### Requirements

- Cmake greater than or equal to version 3.8
- A C compiler that supports link-time-optimizations (GCC with gold linker or clang)
- A 64-bit or 32-bit linux computer.

### 64-bit Build

Create a new folder for building and then cd to that folder
```shell
mkdir build_64 ; cd build_64
```
then run cmake
```shell
cmake ../
```
then run make
```shell
make
```
It's that easy! You now have a 64-bit linux libbulletc.so! Now copy it to your Unity plugins folder.

### 32-bit Build

Create a new folder for building and then cd to that folder
```shell
mkdir build_32 ; cd build_32
```
then run cmake
```shell
cmake -D32BIT=ON ../
```
then run make
```shell
make
```
It's that easy! You now have a 32-bit linux libbulletc.so! Now copy it to your Unity plugins folder.