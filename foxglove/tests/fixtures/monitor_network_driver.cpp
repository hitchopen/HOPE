// Test-only dump of the exact shared projection used by the monitor. No ROS.
#include "network_addresses.hpp"
#include <iostream>
int main() {
  try {
    for (const auto &[index, name, address] : hope_network::ipv4_interface_signature())
      std::cout << index << '\t' << name << '\t' << address << '\n';
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
