# icp

![CI Badge](https://github.com/cornellev/icp/actions/workflows/ci.yaml/badge.svg)
![Code Style Badge](https://github.com/cornellev/icp/actions/workflows/lint.yaml/badge.svg)
![Docs Badge](https://github.com/cornellev/icp/actions/workflows/docs.yaml/badge.svg)

## Introduction
This repository hosts CEV's implementation of Iterative Closest Points (ICP) as applied to scan matching.

## Authors
- [Ethan Uppal](https://www.ethanuppal.com)
- [Jess Wang](https://github.com/jesswang7)
- [Utku Melemetci](https://utku.sh)

## Licensing
Please see [LICENSE](LICENSE). Note that all code is licensed under the MIT license, with the exception of some additions to `doxygen-style.css`. Files with copyright headers denoting specific copyright holders belong to said holders. Files under `book/asset` are from [doxygen_theme_flat_design](https://github.com/kcwongjoe/doxygen_theme_flat_design) under its MIT license (again with the exception of some additions to `doxygen-style.css`). Files without copyright headers are under the MIT license, Copyright (c) 2024-2025 Cornell Electric Vehicles.

## Install
You can view installation instructions at [INSTALL.md](INSTALL.md).

## Usage
```cpp
#include "icp/icp.h"
#include "icp/geo.h"
#include "icp/driver.h"
#include <iostream>

void align_clouds(const icp::PointCloud2& a, const icp::PointCloud2& b) {
    std::unique_ptr<icp::ICP2> icp = icp::ICP2::from_method("vanilla", icp::Config()).value();
    icp::ICPDriver driver(std::move(icp));

    driver.set_max_iterations(100);
    driver.set_transform_tolerance(0.1 * M_PI / 180, 0.1);
    auto result = driver.converge(a, b, icp::RBTransform2::Identity());

    std::cout << "Rotation: " << result.transform.rotation() << "\n";
    std::cout << "Translation: " << result.transform.translation() << "\n";
}
```

## Documentation
Some documentation is available in the `book` folder. There are also Doxygen docs and other generated documentation hosted at 
[cornellev.github.io/icp/](https://cornellev.github.io/icp/), but we haven't update this system in a bit and it might be out of date.

## Versions
Version information can be found in the [releases](https://github.com/cornellev/icp/releases) page.
