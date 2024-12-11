# Installation Instructions

This document assumes you have downloaded the repository and installed the library dependencies.
If not, please read the instructions at [cornellev.github.io/icp/](https://cornellev.github.io/icp/).

## System Installation (Static)

> **Warning**: these instructions must be followed from start whenever an update to the installation is desired.

1. The project uses CMake, but there is a Makefile provided for convenience.
2. Run `sudo make install` to install the library.
    You can specify the installation locations by passing the `LIB_INSTALL` and `HEADER_INSTALL` variables.
    The default behavior is
    ```
    sudo make install LIB_INSTALL=/usr/local/lib HEADER_INSTALL=/usr/local/include
    ```
    The library `libcevicp.a` will be located at `LIB_INSTALL`.
    The header files will be accessible from `HEADER_INSTALL/cev_icp`, e.g., `HEADER_INSTALL/cev_icp/icp/icp.h`.
2.  Run `sudo make uninstall` to remove the library.
    You can similarly pass the `INSTALL_PREFIX` variable, but it must be the same as where you installed.
    Note that uninstalling may leave some remanant directory structure. 

## Local Installation (Static)

Follow the same instructions as above, but supply `INSTALL_PREFIX` to be the desired local path.

## Git Submodule

If you choose to include this as a git submodule, you must take care to only include the `lib` folder in your build path and only include the `include` folder in your include path. It's much easier if your project uses CMake as well.
