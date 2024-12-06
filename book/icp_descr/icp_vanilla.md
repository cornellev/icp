\page vanilla_icp Vanilla ICP
\par Usage
You can construct a new instance of Vanilla ICP with `icp::ICP::from_method("vanilla")`.
\par Description
The vanilla algorithm for ICP will match the point-cloud centers
exactly and then iterate until an optimal rotation has been found.

1.     **Matching Step**: match closest points.
    
    Sources:  
    - https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
    - https://arxiv.org/pdf/2206.06435.pdf
    - https://web.archive.org/web/20220615080318/https://www.cs.technion.ac.il/~cs236329/tutorials/ICP.pdf
    - https://en.wikipedia.org/wiki/Iterative_closest_point
    - https://courses.cs.duke.edu/spring07/cps296.2/scribe_notes/lecture24.pdf


2.     **SVD**
    
    We compute the SVD of this magical matrix. A proof that this yields the optimal
    transform R is in the source below.
    
    Sources:  
    - https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965


3.     **Reflection Handling**
    
    SVD may return a reflection instead of a rotation if it's equally good or better.
    This is exceedingly rare with real data but may happen in very high noise
    environment with sparse point cloud.
    
    In the 2D case, we can always recover a reasonable rotation by negating the last
    column of V. I do not know if this is the optimal rotation among rotations, but
    we can probably get an answer to that question with more effort.
    
    Sources:  
    - https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965


4.     **Transformation Step**: determine optimal transformation.
    
    The translation vector is determined by the displacement between
    the centroids of both point clouds. The rotation matrix is
    calculated via singular value decomposition.
    
    Sources:  
    - https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
    - https://courses.cs.duke.edu/spring07/cps296.2/scribe_notes/lecture24.pdf



Read \ref icp_sources for a list of all resources used in this project.
This page was automatically generated from vanilla.cpp with icp_doc_builder.py.