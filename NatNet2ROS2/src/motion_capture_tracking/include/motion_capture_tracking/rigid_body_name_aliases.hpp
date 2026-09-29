#pragma once

#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace motion_capture_tracking::detail {

using RigidBodyNameAliases = std::map<std::string, std::string>;

inline RigidBodyNameAliases parse_rigid_body_name_aliases(
    const std::vector<std::string>& entries) {
  RigidBodyNameAliases aliases;
  std::set<std::string> canonical_names;
  for (const auto& entry : entries) {
    const auto separator = entry.find('=');
    if (separator == std::string::npos || separator == 0 ||
        separator + 1 >= entry.size() ||
        entry.find('=', separator + 1) != std::string::npos) {
      throw std::invalid_argument(
          "rigid_body_name_aliases entries must be SOURCE=CANONICAL: '" +
          entry + "'");
    }

    const std::string source = entry.substr(0, separator);
    const std::string canonical = entry.substr(separator + 1);
    if (source == canonical) {
      throw std::invalid_argument(
          "rigid_body_name_aliases source and canonical name must differ: '" +
          entry + "'");
    }
    if (!aliases.emplace(source, canonical).second) {
      throw std::invalid_argument(
          "duplicate rigid_body_name_aliases source: '" + source + "'");
    }
    if (!canonical_names.insert(canonical).second) {
      throw std::invalid_argument(
          "duplicate rigid_body_name_aliases canonical name: '" + canonical +
          "'");
    }
  }
  return aliases;
}

inline std::string canonical_rigid_body_name(
    const std::string& name, const RigidBodyNameAliases& aliases) {
  const auto match = aliases.find(name);
  return match == aliases.end() ? name : match->second;
}

}  // namespace motion_capture_tracking::detail
