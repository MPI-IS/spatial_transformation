``spatial_transformation`` is a utility package for working with spatial transformations
in both C++ and Python.

The code is hosted on GitHub_.

.. .. toctree::
..    :caption: General Documentation
..    :maxdepth: 1


The Transformation Class
========================

This package contains native implementations of a ``Transformation`` class in both
Python and C++.  They work very similarly but differ a bit in some details, mostly
regarding data types used for the rotation (scipy's Rotation class in Python vs Eigen's
Quaternion in C++).

- Python: :py:class:`spatial_transformation.Transformation`
- C++: :cpp:class:`spatial_transformation::Transformation`

There are also Python bindings for the C++ version
(:class:`spatial_transformation.cpp.Transformation`), so it can be used in other
bindings (e.g. a function that returns a transformation).  However, when working in
Python, the native implementation provides more flexibility, so you may want to convert
it.  The Python class implements methods
:meth:`~spatial_transformation.Transformation.from_cpp` and
:meth:`~spatial_transformation.Transformation.as_cpp` for this purpose.


Compute Transformation Between Different Systems
================================================

The ``pointcloud`` library provides utility functions for computing the transformation
between two different reference frames, given the positions of a set of points in both
frames.  For more information see :doc:`breathe_apidoc/file/pointcloud_8hpp`.


Use in CMake Project
====================

.. code-block:: cmake

   find_package(spatial_transformation REQUIRED)

   add_library(mylib ...)
   target_link_libraries(mylib
     spatial_transformation::transformation  # for the Transformation class
     spatial_transformation::pointcloud  # for point cloud transformation computation
   )


.. _GitHub: https://github.com/MPI-IS/spatial_transformation
