\page feature-aware_icp Feature-Aware ICP
\par Usage
You can construct a new instance of Feature-Aware ICP with `icp::ICP::from_method("feature_aware", config)`. Supply the following parameters to `config` (via icp::ICP::Config::set):

Key | Description
--- | ---
`"overlap_rate"` | A `double` between `0.0` and `1.0` for      * the overlap rate. The default is `1.0`. 
`"feature_weight"` | A `double` with default value `0.7`. 
`"symmetric_neighbors"` | An `int` with default value `10`. 

\par Description
TODO

1.     **TODO**: write smth Matching Step:

2.     **Trimming Step**: see \ref trimmed for details.

3.     **SVD**: see \ref vanilla_icp for details.

4.     **Reflection Handling**: see \ref vanilla_icp for details.

5.     **Transformation Step**: see \ref vanilla_icp for details.


Read \ref icp_sources for a list of all resources used in this project.
This page was automatically generated from feature_aware.cpp with icp_doc_builder.py.