<!--  
Copyright (C) 2025 Ethan Uppal.
SPDX-License-Identifier: MIT
--->
\page write_icp_instance Writing an ICP Instance

\tableofcontents

To write an ICP instance, create a C++ source file with at least the following:

1. A `final` class that inherits from `public icp::ICP`
2. A static initialization variable (described below)

It is highly recommended you also provide documentation for your ICP instance in the format described here.

\section core_func_sec Core Functionality

The class must define the following behavior:

- A constructor that calls the `icp::ICP` constructor
- An overridden destructor (which may do nothing)
- `void iterate() override`

In `iterate`, the point clouds are given by the instance variables `a` and `b`.
There is also the `match` instance variable, allocated to have size `a.size()`, which cannot be assumed to contain any definite values.
At the end of `iterate`, the `transform` instance variable should have been updated (although the update may be zero).

Optionally, the class can override:

- `void setup() override`

`setup` is invoked upon the user call to `ICP::begin` after the internals of ICP have been readied.

\section static_init_sec Static Initialization

TODO: update -- we do this in icp.cpp now and the documentation builder will
automatically search there.

The static initialization is required so that users can instantiate your ICP instance.
Define

```cpp
static bool static_initialization = []() {
    assert(ICP::register_method("name_of_instance", []() -> std::unique_ptr<ICP> {
        return std::make_unique<NameOfClass>();
    }));
    return true;
}();
```

where `"name_of_instance"` is the name of your ICP implementation and `NameOfClass` is the name of the class.

\section icp_dpc_sec Documentation

The script icp_doc_builder.py will automatically generate documentation for your ICP instances as markdown files and place them in a desired directory. The invocation format is:

```shell
python3 icp_doc_builder.py dir/where/your/icps/are/ dir/where/markdown/should/go/ where/main/file/is.md
```

Notably, `where/main/file/is.md` should contain two lines of the form

```md
<!-- ICP_DOCS_BUILDER EDIT MARKER START -->
...
<!-- ICP_DOCS_BUILDER EDIT MARKER END -->
```

the contents between which markers will be automatically updated by the
documentation builder.

If the file name is `foo_bar.cpp`, then the Doxygen page reference (from which you can refer to from other pages) will be be `foo_bar_icp Foo_bar`. Information about the file should be encoded in special block comments of the following format.

```cpp
/*
#command values...
*/
```

Supported commands are described below.

- The name of the instance should be given by `/* #name Name of Instance */`.
- An overview of the algorithm should be provided by `/* #desc Overview of algorithm. */`. This description will be rendered as Doxygen source, so you can use markdown and Doxygen commands such as `\ref`.
- If your instance uses icp::ICP::Config parameters, document them as `/* #conf "name_of_param" This is a sentence. This is another sentence describing the parameter. */`. This description will be rendered as Doxygen source.
- Every major step your instance takes should be documented by

    ```cpp
    /*
        #step My Step: brief description

        Detailed explanation.

        Sources:
        https://www.example.com
        https://www.example.com
        https://www.example.com
        https://www.example.com
    */
    ```

    The `: brief description` section is optional, as are the detailed explanation and sources sections.
    These descriptions will be rendered as Doxygen source.

See the source code of vanilla.cpp or trimmed.cpp as examples.
