# Skelton2: skeleton modeling library

Skelton2 is a library for skeleton-based modeling. 

There are two use cases:

1. Build a scaffold for a skeleton made of line segments.
2. Create an implicit surface with an anisotropic convolution field and generate a quad-dominant mesh of th surface by projecting a scaffold.

The skeletons are made of line segments or arcs of circle. There is no restriction on the topology of the input skeleton.

## Scaffolding

A *scaffold* is a quad-dominant mesh that tightly follows the structure of a skeleton made of line segments. Every quad on the mesh is associated to only one line segment. **Skelton2** can generate scaffolds with the same number of quads around each line segment - *regular scaffolds*. Or, scaffolds that respects a group of symmetries of the underlying skeleton - *symmetric scaffolds*. Regular symmetric scaffolds are also possible.

For details on the *scaffolding algorithm* check out the following paper [here](https://hal.inria.fr/hal-01774909v1/document):

> Alvaro Javier Fuentes Suárez, Evelyne Hubert. Scaffolding skeletons using spherical Voronoi diagrams: feasibility, regularity and symmetry. Computer-Aided Design, Elsevier, 2018, 102, pp.83 - 93. doi:[10.1016/j.cad.2018.04.016](https://doi.org/10.1016/j.cad.2018.04.016). hal:[hal-01774909](https://hal.inria.fr/hal-01774909)

## Anisotropic convolution surfaces

With **skelton2** the user can design a shape (made with line segments or arcs of circle) and then generate a surface around it. The surface is implicitly defined as an anisotropic convolution. An anisotropic convolution surface is defined by a skeleton piece plus radii and rotation information at the extremities. The output of the library is a quad-dominant mesh polygonization of the implicit surface. This mesh is obtained by projecting a scaffold onto the surface. Hence it follows the structure of the skeleton and has some polar-annular regions at the extremities of the model.

For details on *Anisotropic convolution surfaces* check out the following paper [here](https://hal.inria.fr/hal-02137325/document):

> Alvaro Javier Fuentes Suárez, Evelyne Hubert, Cédric Zanni. Anisotropic convolution surfaces. Computers and Graphics, Elsevier, 2019, 82, pp.106-116. doi:[10.1016/j.cag.2019.05.018](https://doi.org/10.1016/j.cag.2019.05.018). hal:[hal-02137325](https://hal.inria.fr/hal-02137325)

## Building:

**TODO:** add description on how to update the paths to the libraries

Clone and make directory called `build` anywhere, then

    cd build
    cmake <path/to/skelton>
    make -j4    

### Dependencies:

* [Qhull](http://www.qhull.org/): Convex hull computations.
* [GSL](https://www.gnu.org/software/gsl/): Mathematical functions, mainly integration and root finding.
* [GLPK](https://www.gnu.org/software/glpk/): Linear programming.
* [Eigen3](http://eigen.tuxfamily.org): Linear algebra.
* [Catch2](https://github.com/catchorg/Catch2): Unit testing (shipped with the library).

Notes:

1. We use the reentrant C++ interface of Qhull so be sure to include/build it.
2. Eigen3 is a header only library thus only the path to the include folder of Eigen3 is needed.
3. Catch2 is header only also, we include a copy of the library in the `test` folder. Update to newer version at your own risk.

