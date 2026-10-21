# IM FIXING TS JUST GIVE ME A LITTLE BIT OF TIME -jaiden

# Installation Instructions

## Library and tests with Eigen only

Use a C++17 compiler, CMake, and Eigen3. The library and unit tests do not require
the viewer's SDL/PCL dependencies or a separate test library:

```sh
cmake -S . -B build/library -DCEV_ICP_BUILD_TOOLS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/library --parallel
ctest --test-dir build/library --output-on-failure
```

`CEV_ICP_BUILD_TESTS` defaults to `ON`; set it to `OFF` for a library-only build.
`CEV_ICP_BUILD_ZENOH` defaults to `OFF`, so ordinary builds do not require Zenoh
or JSON dependencies.

## Optional C++ Zenoh node

Install **zenoh-c 1.10.1**, **zenoh-cpp 1.10.1**, and **nlohmann_json 3.11+** into
your normal CMake search path or a local prefix. The node uses the official
`zenoh.hxx` C++ API, linked to its required zenoh-c runtime. Official standalone
packages are available from the [zenoh-c release](https://github.com/eclipse-zenoh/zenoh-c/releases/tag/1.10.1)
and [zenoh-cpp release](https://github.com/eclipse-zenoh/zenoh-cpp/releases/tag/1.10.1).
Choose the zenoh-c archive for your platform and extract both packages into the
same prefix. The JSON library's [CMake installation guide](https://json.nlohmann.me/integration/cmake/)
describes its package setup.

On macOS, the 1.10.1 standalone zenoh-c dylib can contain its release runner's
absolute install name. If that package reports `Library not loaded` pointing at
`/Users/runner/`, fix the downloaded library before rebuilding your executable
(adjust the prefix if needed):

```sh
install_name_tool -id @rpath/libzenohc.dylib "$HOME/.local/lib/libzenohc.dylib"
codesign --force --sign - "$HOME/.local/lib/libzenohc.dylib"
```

```sh
# Replace the prefix with the directory containing include/ and lib/cmake/.
cmake -S . -B build/zenoh -DCEV_ICP_BUILD_TOOLS=OFF \
  -DCEV_ICP_BUILD_ZENOH=ON -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HOME/.local"
cmake --build build/zenoh --parallel
./build/zenoh/icp_zenoh --help
ctest --test-dir build/zenoh --output-on-failure
```

The loopback and CLI tests bind localhost TCP sockets. Python 3 is optional and
enables the executable's CLI/signal test. The C++ decoder/registration and
two-session transport tests do not require Python or LiDAR hardware. Continue
with the [Zenoh usage guide](book/zenoh.md).

## Legacy viewer and system installation

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
    You can similarly pass the `LIB_INSTALL` and `HEADER_INSTALL`, but they must be the same as where you installed.
    Note that uninstalling may leave some remanant directory structure. 

## Local Installation (Static)

Follow the same instructions as above, but supply `LIB_INSTALL` and `HEADER_INSTALL` to be the desired local paths.

## Git Submodule

If you choose to include this as a git submodule, you must take care to only include the `lib` folder in your build path and only include the `include` folder in your include path. It's much easier if your project uses CMake as well.
