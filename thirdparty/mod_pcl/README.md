- This directory contains a partially modified version of the library included in PCL
- VoxelGridLarge is based on [this pull request](https://github.com/PointCloudLibrary/pcl/pull/4385) by [MarkHedleyJones](https://github.com/MarkHedleyJones).  
It has been modified to eliminate interference with PCL's VoxelGrid and to obtain the indices of the points used for centroid calculation.
- PCDWriterWithIndices is a PCDWriter modified to output only the point cloud specified by Indices.