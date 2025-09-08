# Developing

This document explains how to effectively develop `icp`. It covers development environment setup, the general repository structure, how to add new ICP methods, and how to add benchmarks and tests.

## Development environment

### Dependencies
`libicp` itself can be built and installed with only Eigen as a dependency---this is by design. However, to effectively develop the project, you'll need a few extra things.

1. [libcmdapp2](https://github.com/cornellev/libcmdapp2), a command-line parsing library (built by Ethan) which we use for the 2D visualization.
2. [libconfig](https://github.com/cornellev/config.git), a library for loading configuration from files
3. [sdl-wrapper](https://github.com/cornellev/sdl-wrapper), a C++ wrapper for SDL to make things easier for the visualization.
4. [simple-test](https://github.com/cornellev/simple-test), a header-only testing library.
5. [PCL](https://pointclouds.org/) 1.12.1 (ideally), a point cloud processing library. We actually only use this to parse 3D point clouds for testing.

Each of the links above should have installation instructions. If you have everything installed correctly, you should be able to [build](#building) the project and not have any errors.

### Tooling setup
I'll mainly cover VSCode here as it's relatively popular, but other IDEs shouldn't be too difficult.

You'll need the [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd) and [clang-format](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format) extensions. You may also need to install `clang-format` itself. Please install at least version 19 as some of the rules depend on recent versions.

The build system, by default, will generate the `compile_commands.json` file that `cland` relies on. However, it will be in `build/`. To configure VSCode to pick up on this file, you can add the following to your `.vscode/settings.json`

```json
{
    "clangd.arguments": [
        "-compile-commands-dir=build"
    ],
}
```

## Repository structure
```
.
├── bench - code for benchmarking
├── book - documentation and files for the website
├── common - common code across benches, tests, library, and vis
├── ex_data
│   ├── ply - .ply example data (3D)
│   └── scan{n} - .csv example data (2D)
├── include - library header files
├── lib - library source files
├── script - various utility scripts
├── tests - unit test code
└── vis - 2D visualization app
```

A few things to note:
1. `tests` also contains `test_ply.cpp`. This isn't really a test, but a program you can run to look at the output from aligning two `.ply` scans. We might want to mvoe this in the future.
2. A lot of the library code is templated. So you may need to look in `include` for some of the core functionality.
3. You'll probably be doing most of your work in `include` and `lib`. These are the core folders.

## Building
`CMakeLists.txt` is the main build file. However, for convenience, there is a `Makefile` that shortens the `cmake` commands. Just typing `make` will build everything: the library, visualization, benchmarks, and tests.

There are some extra options you should be aware of. Setting `OPT=[Release | Debug]` (i.e. `make OPT=Release`) changes the optimization level. `USE_SANTIZERS=ON` will enable the address and undefined behavior sanitizers, which can be useful when debugging UB. `CI=ON` will run a CI build, which compiles with warnings as errors and enables the sanitizers.

Make sure to check the `Makefile` for other utilities you might find useful. You may be interested in `make tidy` and `make view`.

## Testing
You can run `make test` to run the 2D tests and `make test_3d` to run the 3D tests. Ideally, in the future, we combine these into one executable, but we haven't done this yet.

### Adding Tests
To add tests, you can follow the examples already in `test.cpp` and `test3d.cpp`. If you want to create a new test file, it's probably best that you refactor and bring all the tests under one executable.

## Benchmarking
You can use `make bench` to run the benchmarks. They'll run 2D ICP on the example data. There are currently no 3D ICP benchmarks.

## Continuous Integration
The CI system is defined in `.github/workflows/ci.yaml`. It's pretty standard: it just runs the tests on MacOS and Linux.
