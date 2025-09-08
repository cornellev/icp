# Contributing: Adding a New ICP Instance

This section explains how to implement and document a new **ICP instance**.

---

## 1. Overview

An ICP instance is a specific implementation of the Iterative Closest Point algorithm. Each instance should be:

- A `final` C++ class that inherits from `icp::ICP`
- Properly registered so users can instantiate it

---

## 2. File and Class Requirements

Create a new `method_xd.cpp` file in `lib/icp/impl/`, and implement your instance class.

### Required

- Class must be `final` and inherit from `public icp::ICP`
- Must implement `void iterate() override`
- Must provide a constructor that calls the `icp::ICP` constructor
- A destructor (can be empty)

Example:

```cpp
class MyICP final : public icp::ICP {
public:
    MyICP();
    MyICP(const Config& config);
    ~MyICP();

    void setup() override;    // Optional
    void iterate() override;
};
````

---

## 3. ICP Lifecycle

### Provided Instance Variables

| Variable    | Type               | Description                        |
| ----------- | ------------------ | ---------------------------------- |
| `a`         | `PointCloud`       | Source point cloud                 |
| `b`         | `PointCloud`       | Target point cloud                 |
| `match`     | `std::vector<int>` | Matches from `a` to `b`            |
| `transform` | `RBTransform`      | Current accumulated transformation |

### Method Responsibilities

| Method      | When it is Called            | What to Do                                     |
| ----------- | --------------------------- | ---------------------------------------------- |
| `setup()`   | Once before first iteration | Initialize resources (e.g. KD-tree)            |
| `iterate()` | Once per iteration          | Compute correspondences and update `transform` |

---

## 4. Registration

To allow users to select your ICP instance by name, add it to the static methods map in `ICP2` or `ICP3` in `icp.cpp`

```cpp
#include "icp/impl/my_icp.h"

namespace icp {
    template<>
    std::unordered_map<std::string, ICPX::MethodConstructor> ICPX::methods{
        {"my_icp", CONSTRUCT_CONFIG(MyICP)},  // ← Register your method here
    };
}
```

---

## 5. Examples

See the following reference implementations:

| File                                                     | Type     |
| -------------------------------------------------------- | ---------|
| [`vanilla.cpp`](/lib/icp/impl/vanilla.cpp)                | 2d       |
| [`trimmed.cpp`](/lib/icp/impl/trimmed.cpp)                | 2d       |
| [`feature_aware.cpp`](/lib/icp/impl/feature_aware.cpp)    | 2d       |
| [`vanilla_3d.cpp`](/lib/icp/impl/vanilla_3d.cpp)          | 3d       |
| [`trimmed_3d.cpp`](/lib/icp/impl/trimmed_3d.cpp)          | 3d.      |
