\page trimmed_icp Trimmed ICP
\par Usage
You can construct a new instance of Trimmed ICP with `icp::ICP::from_method("trimmed", config)`. Supply the following parameters to `config` (via icp::ICP::Config::set):

Key | Description
--- | ---
`"overlap_rate"` | A `double` between `0.0` and `1.0` for      * the overlap rate. The default is `1.0`. 

\par Description
Trimmed ICP is identical to \ref vanilla_icp with the addition of an
overlap rate parameter, which specifies the percentage of points between the two
point sets that have correspondences. When the overlap rate is 1, the algorithm
reduces to vanilla.

1.     **Matching Step**: see \ref vanilla_icp for details.

2.     **Trimming Step**
    
    Matches are considered in increasing order of distance.
    
    Sources:  
    - https://ieeexplore.ieee.org/abstract/document/1047997


3.     **SVD**: see \ref vanilla_icp for details.

4.     **Reflection Handling**: see \ref vanilla_icp for details.

5.     **Transformation Step**: see \ref vanilla_icp for details.


Read \ref icp_sources for a list of all resources used in this project.
This page was automatically generated from trimmed.cpp with icp_doc_builder.py.