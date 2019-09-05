#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "VideoCapture.hpp"
namespace py = pybind11;

PYBIND11_MODULE(video, m) {

  py::class_<VideoCapture>(m, "VideoCapture")
      .def(py::init<std::string, int, std::vector<int>>())
      .def("read", &VideoCapture::read)
      .def_readonly("is_stop", &VideoCapture::stop);
  pybind11::class_<cv::Mat>(m, "Image", pybind11::buffer_protocol()).def_buffer([](cv::Mat &im) -> pybind11::buffer_info {
    return pybind11::buffer_info(
        // Pointer to buffer
        im.data,
        // Size of one scalar
        sizeof(unsigned char),
        // Python struct-style format descriptor
        pybind11::format_descriptor<unsigned char>::format(),
        // Number of dimensions
        3,
        // Buffer dimensions
        {im.rows, im.cols, im.channels()},
        // Strides (in bytes) for each index
        {sizeof(unsigned char) * im.channels() * im.cols, sizeof(unsigned char) * im.channels(), sizeof(unsigned char)});
  });
}