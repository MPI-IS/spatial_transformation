// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file Python bindings of the relevant C++ classes/functions.
 * @copyright 2022, Max Planck Gesellschaft.  All rights reserved.
 */
#include <sstream>

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <spatial_transformation/transformation.hpp>

PYBIND11_MODULE(cpp, m)
{
    namespace py = pybind11;
    namespace st = spatial_transformation;

    // TODO: add unit tests for bindings of Transformation
    py::class_<st::Transformation>(m, "Transformation")
        .def(py::init<>(), py::call_guard<py::gil_scoped_release>())
        .def(py::init<Eigen::Quaterniond, Eigen::Vector3d>(),
             py::call_guard<py::gil_scoped_release>())
        .def(py::self * py::self)
        .def(py::self * Eigen::Vector3d())
        .def("apply",
             &st::Transformation::apply,
             py::call_guard<py::gil_scoped_release>())
        .def("inverse",
             &st::Transformation::inverse,
             py::call_guard<py::gil_scoped_release>())
        .def("matrix",
             &st::Transformation::matrix,
             py::call_guard<py::gil_scoped_release>())
        .def_readwrite("translation", &st::Transformation::translation)
        // there are no proper bindings for Eigen::Quaterniond, so as a simple
        // workaround provide a getter and setter that provide/expect the
        // quaternion as a list [x, y, z, w].
        .def("get_rotation",
             [](const st::Transformation& self)
             {
                 std::array<double, 4> quat = {self.rotation.x(),
                                               self.rotation.y(),
                                               self.rotation.z(),
                                               self.rotation.w()};
                 return quat;
             })
        .def("set_rotation",
             [](st::Transformation& self, const std::array<double, 4>& quat)
             {
                 self.rotation.x() = quat[0];
                 self.rotation.y() = quat[1];
                 self.rotation.z() = quat[2];
                 self.rotation.w() = quat[3];
             });
}
