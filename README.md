# CAD projector

Tool to simulate projections from 3D shapes, e.g., meshes, in arbitrary geometries. Possible applications are simulation of X-ray or optical transmission projections.

This work was inspired by the [ASTRA-toolbox, a MATLAB and Python toolbox of high-performance GPU primitives for 2D and 3D tomography for using volumetric data](https://github.com/astra-toolbox/astra-toolbox). The aim of this project is exclusively to overcome some downsides of simulating projections from volumetric data. There is thus no focus on reconstruction algorithms to generate 3D volumes form the simulated projections. 

The CAD projector project aims to overcome some downsides of using volumetric data for projection simulations by working with geomtric objects, e.g., meshes: 
- First, it is harder to visualize the system geometry and sample movement with volumetric data compared to using geometric objects. 
- Second, if only a shape of the object is available, voxelization of the volume is needed before usage, which requires specifying the voxel size and values.
- Third, volumetric representations requires the expression of the position  of the X-ray source and detector relative to the sample. In this project however, the movement patterns of geomtric object can, in my opinion, be expressed more intuitively.
- Finally, in volumetric representations, the origin of the system is often located in the center of the volume. Here, the origin can be positioned arbitrarilly.

The main drowbacks of this implementation:
- It is much slower (no calculations on the GPU) compared to volumetric representations.
- Limited to applications where the sample have approximatly a homogenoeus density distribution.

## Prerequisites
- MATLAB (>= R2018b) 

## Dependencies
- Geometric computing library for 3D shapes: [geom3d](https://github.com/mattools/matGeom)

## Getting started
Take a look at the example folder and the comments in the cad_projector class.

## Data
Apple fruit mesh downloaded from [here](https://www.myminifactory.com/object/3d-print-apple-3d-scan-59317)

## Contributing
If you want to contribute back, please open an issue or make a pull request.



