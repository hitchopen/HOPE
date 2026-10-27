// Test-only entry point using the EXACT runtime node/timer implementation.
// A file-backed address reader models network changes/failures without touching
// real interfaces. This executable is not installed on robots.
#include "mocap_config_transport.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--dump-addresses") {
    try {
      for (const auto &[interface, ip] : hope_mocap_config::network_addresses())
        std::cout << interface << '\t' << ip << '\n';
      return 0;
    } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
  }
  const auto *file = std::getenv("HOPE_TEST_MOCAP_ADDRESS_FILE");
  if (!file) return 2;
  const std::string name(file);
  return hope_mocap_config::run(argc, argv, [name] {
    std::ifstream input(name);
    if (!input) throw std::runtime_error("Test address reader unavailable");
    hope_mocap_config::Addresses addresses;
    std::string interface, ip;
    while (input >> interface >> ip) {
      if (interface == "ERROR") throw std::runtime_error("Test address probe failure");
      addresses.emplace_back(interface, ip);
    }
    std::sort(addresses.begin(), addresses.end()); return addresses;
  });
}
