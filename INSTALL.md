# Installation Instructions

This document assumes you have downloaded the repository and installed the library dependencies.
If not, please read the instructions at [cornellev.github.io/icp/](https://cornellev.github.io/icp/).

## System Installation (Static)

> **Warning**: these instructions must be followed from start whenever an update to the installation is desired.

1. The project uses CMake, but there is a Makefile provided for convenience.
    Run `sudo make install` to install the library.
    You can specify the installation locations by passing the `INSTALL_PREFIX` variable.
    The default behavior is
    ```sh
    sudo make install INSTALL_PREFIX=/usr/local
    ```
    which will install the library to `/usr/local/lib/libcevicp.a` and the headers in `/usr/local/include/cev_icp`.
2.  Run `sudo make uninstall` to remove the library.
    You can similarly pass the `INSTALL_PREFIX` variable, but it must be the same as where you installed.
    Note that uninstalling may leave some remanant directory structure. 

## Local Installation (Static)

Follow the same instructions as above, but supply `INSTALL_PREFIX` to be the desired local path.

## Git Submodule

If you choose to include this as a git submodule, you must take care to only include the `lib` folder in your build path and only include the `include` folder in your include path. It's much easier if your project uses CMake as well.
