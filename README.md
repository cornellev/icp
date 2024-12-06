<!-- THIS FILE IS GENERATED AUTOMATICALLY. -->
<!-- DO NOT EDIT THIS FILE. -->
<!-- EDIT README.md.build INSTEAD. -->
# icp

> Last updated 2024-12-06 13:39:07.091841.  
> Version v1.3.0.
> Made by [Ethan Uppal](https://www.ethanuppal.com) and [Utku Melemetci](https://utku.sh).

## Introduction

This repository hosts Ethan's implementation of Iterative Closest Points (ICP) as applied to scan matching.
It is a first step in my project to implement Simultaneous Localization and Mapping (SLAM).

## Install

You can view installation instructions at [INSTALL.md](INSTALL.md).

## Usage and Documentation

I host the usage information and documentation at [cornellev.github.io/icp/](https://cornellev.github.io/icp/).
Please see there for information on how to download and how to use the library.

You can build the documentation yourself locally with `make docs`.
The main page will be located at `docs/index.html` relative to the project root.

## Versions

### v1.3.0
- Various bug fixes, including finding and fixing undefined behavior
- Rewrite of vanilla and trimmed ICP
- Feature-aware ICP method to improve convergence in structured scenarios
- New `ICPDriver` to make specifying convergence criteria easier
- Improved tests
- Improved build pipeline and CI

### v1.1.2

Started work on a new implementation (read [this paper](book/icp.pdf)).
It passes all `make test` tests.
Here is the performance currently:
```
ICP ALGORITHM BENCHMARKING
=======================================
* Method name: test1
* Number of trials: 50
* Burn-in period: 0
* Ideal convergence threshold: 20
* Min cost: 19.4709
* Max cost: 19.4709
* Median cost: 19.4709
* Mean cost: 19.4709
* Min iterations: 7 (real: 7)
* Max iterations: 7 (real: 7)
* Median iterations: 7 (real: 7)
* Mean iterations: 7 (real: 7)
* Average time per invocation: 0.00413281s
```

### v1.1.1

The algorithm runs extremely fast now.
We only need it to run at 6hz with our current LiDAR.

![](book/asset/img/v1.1.1bench.png)

It also matches quite well.
Below is the result of running this ICP implementation on two point clouds obtained within the workspace.

![](book/asset/img/v1.1.1result.png)

However, there is still remove for improvement with regard to outlier rejection and other parts of the algorithm (for instance, adopting a point-to-line metric).

