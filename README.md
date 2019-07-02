# Skelton: skeleton modeling library

Skelton is a library for skeleton-based modeling. The user can design a shape (made with line segments or arcs of circle) and then generate a surface around it. The surface is implicitly defined as an anisotropic convolution. The output of the library is a quad-dominant mesh polygonization of the implicit surface. This mesh follows the structure of the skeleton and has some polar-annular regions at the extremities of the model.

There is no restriction in the topology of the input skeleton other than not self-intersection on points not in the extremity of any skeleton piece.

## Building:

Clone and make directory called `build` anywhere, then

    cd build
    ccmake <path/to/skelton>
    make -j4    

### Dependencies:

* boost library: General utilities.
* QHull: Convex hull computations.
* GSL: Mathematical functions, mainly integration and root finding.
* GLPK: Linear programming.
* Eigen3: Linear algebra.
* Catch: Unit testing (shipped with the library).

GLPK is used as an external tool so be sure to have `glpsol` in your path before using the library.
