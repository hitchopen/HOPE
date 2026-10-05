#include "libmotioncapture/optitrack.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv)
{
  if (argc != 3) return 2;
  try {
    libmotioncapture::MotionCaptureOptitrack client(
      "127.0.0.1", argv[2], std::stoi(argv[1]), false);
    client.waitForNextFrame();
    const auto& points = client.pointCloud();
    if (client.version() != "4.2.0.0" || client.timeStamp() != 1 ||
        points.rows() != 1 || std::abs(points(0, 0) - 1.0f) > 1e-6f ||
        std::abs(points(0, 1) - 2.0f) > 1e-6f ||
        std::abs(points(0, 2) - 3.0f) > 1e-6f) {
      throw std::runtime_error("NatNet frame contents did not match fixture");
    }
    std::cout << "FRAME_DECODED" << std::endl;
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
  return 0;
}
