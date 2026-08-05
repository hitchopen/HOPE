#include <sstream>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include "libmotioncapture/motioncapture.h"

using namespace libmotioncapture;

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(motioncapture, m) {

  m.attr("__version__") = version();

  m.def("connect", &MotionCapture::connect);

  // Quaternions
  py::class_<Eigen::Quaternionf>(m, "Quaternion")
      .def_property_readonly("x", py::overload_cast<>(&Eigen::Quaternionf::x, py::const_))
      .def_property_readonly("y", py::overload_cast<>(&Eigen::Quaternionf::y, py::const_))
      .def_property_readonly("z", py::overload_cast<>(&Eigen::Quaternionf::z, py::const_))
      .def_property_readonly("w", py::overload_cast<>(&Eigen::Quaternionf::w, py::const_));

  // RigidBody
  py::class_<RigidBody>(m, "RigidBody")
      .def_property_readonly("name", &RigidBody::name)
      .def_property_readonly("position", &RigidBody::position)
      .def_property_readonly("rotation", &RigidBody::rotation);

  //
  py::class_<MotionCapture>(m, "MotionCapture")
      .def("waitForNextFrame", &MotionCapture::waitForNextFrame, py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("rigidBodies", &MotionCapture::rigidBodies);

}
