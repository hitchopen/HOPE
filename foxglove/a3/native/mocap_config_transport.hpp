// Read-only configuration relay. The injectable address reader lets isolated
// tests exercise the real node/restart path without changing host networking.
#pragma once
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace hope_mocap_config {
using Addresses = std::vector<std::pair<std::string, std::string>>;
using AddressReader = std::function<Addresses()>;
Addresses network_addresses();
int run(int argc, char **argv, AddressReader reader = network_addresses);
}  // namespace hope_mocap_config
