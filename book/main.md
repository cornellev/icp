<!--  
Copyright (C) 2025 Ethan Uppal.
SPDX-License-Identifier: MIT
--->
\mainpage Introduction
\tableofcontents

\section overview_sec Overview

Scan-matching is a method for localization that uses high-frequency LiDAR data.
This project is [Ethan](https://ethanuppal.com)'s implementation of algorithms for scan-matching.
Please read [this document](icp.pdf) to learn more about the math.

<table>
    <tr>
        <th style="text-align:left; vertical-align:top">Item</th>
        <th style="text-align:left; vertical-align:top">Location</th>
    </tr>
    <tr>
        <td style="text-align:left; vertical-align:top"><b>Documentation</b></td>
        <td style="text-align:left; vertical-align:top">icp::ICP</td>
    </tr>
    <tr>
        <td style="text-align:left; vertical-align:top"><b>Repository</b></td>
        <td style="text-align:left; vertical-align:top">
            <a href="https://github.com/cornellev/icp">https://github.com/cornellev/icp</a>
        </td>
    </tr>
    <tr>
        <td style="text-align:left; vertical-align:top"><b>Contents</b></td>
        <td style="text-align:left; vertical-align:top">
            \ref overview_sec <br>
            \ref feature_sec <br>
            \ref install_sec <br>
            \ref usage_sec <br>
        </td>
    </tr>
</table>

\section feature_sec Features

- ICP library for integration with robotics systems and other use cases
- Interactive (graphical) visualization of point clouds
  - You can supply custom point clouds in a config file using fields from [`sensor_msgs::LaserScan`](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html).

\section install_sec Install

\warning This section applies only to Unix-like operating systems, such as MacOS and Linux.

First, download and install the dependencies.
**Only eigen3 is necessary for the library. If you only wish to build the library, install eigen3 only. The remaining dependencies are only for the visualization tool.**

| Dependency                                                     | Library Location (at which)      | Header Location (under which)    |
| -------------------------------------------------------------- | -------------------------------- | -------------------------------- |
| [eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) | N/A                              | `/usr/include/eigen3/`           |
| [SDL2](https://www.libsdl.org)                                 | `$(sdl2-config --libs)`          | `$(sdl2-config --cflags)`        |
| My [SDL2 wrapper](https://github.com/cornellev/sdl-wrapper)    | `/usr/local/lib/libsdlwrapper.a` | `/usr/local/include/sdlwrapper/` |
| [libcmdapp2](https://ethanuppal.com/libcmdapp2/)               | `/usr/local/lib/libcmdapp.a`     | `/usr/local/include/cmdapp`      |
| [libconfig](https://github.com/ethanuppal/config)              | `/usr/local/lib/libconfig.a`     | `/usr/local/include/config`      |

There is also a dependency on [simple-test](https://github.com/ethanuppal/simple-test) if you want to run the tests (`make test`).
Please follow the installation instructions there.

It is likely that you already have SDL installed.
If not, follow the instructions at the link provided (which goes to the SDL website).
Do the same for eigen3.
Then, to download and install the remaining dependencies, run

```shell
# install libcmdapp
git clone https://github.com/ethanuppal/libcmdapp2.git
cd libcmdapp2
sudo make install
cd ..

# install libconfig
git clone https://github.com/ethanuppal/config.git
cd config
sudo make install
cd ..

# install sdl-wrapper
git clone https://github.com/cornellev/sdl-wrapper.git
cd config
sudo make install
cd ..
```

If you encounter any errors in this process, please open an issue on the corresponding repositories, and I will try to resolve it as soon as possible.

Finally, you can clone the icp repository.

```shell
git clone https://github.com/cornellev/icp.git
cd icp
```

You should run `make test` to make sure everything has been setup correctly for the library.

\section usage_sec Usage

\subsection library_sec Library

Follow the instructions in the [INSTALL.md](https://github.com/cornellev/icp/blob/main/INSTALL.md) document to install locally.
The library contains tested implementations of different ICP algorithms.
Common to all of them is the icp::ICP interface, the documentation for which describes how to construct and use these implementations.
Read the documentation of the specific implementations for more information
(sorted alphabetically):

<!-- ICP_DOCS_BUILDER EDIT MARKER START -->
- \ref feature_aware_icp (Builds on top of \ref trimmed_icp. In addition to matching points based on a point-to-point
distance criteria, matches them based on a local "feature vector.")
- \ref trimmed_icp (Trimmed ICP is identical to \ref vanilla_icp with the addition of an
overlap rate parameter, which specifies the percentage of points between the two
point sets that have correspondences. When the overlap rate is `1`, the algorithm
reduces to vanilla.)
- \ref vanilla_icp (The vanilla algorithm for ICP will match the point-cloud centers
exactly and then iterate until an optimal rotation has been found.)
<!-- ICP_DOCS_BUILDER EDIT MARKER END -->

Optionally, read \ref write_icp_instance to use your own ICP implementations.

\subsection vis_tool_sec Visualization & Benchmarking Tool

The following command visualizes the two LiDAR scans at the given files.
Instructions are printed to standard output.

```shell
make
./main \
    -S ex_data/scan1/first.conf \
    -D ex_data/scan1/second.conf \
    --method vanilla \
    --gui
```

You can benchmark with `make bench`; by default, this will pass `-mvanilla`.

The program itself can be built with

```shell
make
```

which will create an executable named `main` in the working directory.
You can query `./main -h` and `./main -v` for usage and versioning information.
