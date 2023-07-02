# 1. OpenVDB Level Sets
This is a GUI application trying out diffferent applications for VDB in shape analysis.
Also, in future I hope to make useful visualization of VDB level sets in the UI.

The GUI is based of off Polyscope. This is still very early and WIP.

Currently Implements:
 - Skeleton generation
    - From distance transform divergence

## 1.1. Skeletonization
### 1.1.1. From distance transform divergence
#### 1.1.1.1. Icosahedron skeleton
![Mean flux scheme NEIGHBOR_26](./images/Icosahedron_neighbor_26.png)
                Mean flux scheme NEIGHBOR_26

![Mean flux scheme NEIGHBOR_98](./images/Icosahedron_neighbor_98.png)
                Mean flux scheme NEIGHBOR_98

#### 1.1.1.2. Wall street bull
![Bull_original](./images/Bull_original.png)
             Original

![Bull skeleton](./images/Bull_skeleton.png)
             Skeleton