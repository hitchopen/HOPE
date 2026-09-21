#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_capture_tracking/rigid_body_name_aliases.hpp"

using motion_capture_tracking::detail::canonical_rigid_body_name;
using motion_capture_tracking::detail::parse_rigid_body_name_aliases;

int main() {
  const auto aliases = parse_rigid_body_name_aliases(
      std::vector<std::string>{
          "ball=Ball", "SITE_A=CANONICAL_A", "SITE_B=CANONICAL_B"});
  assert(canonical_rigid_body_name("ball", aliases) == "Ball");
  // The exact canonical spelling is unchanged and remains authoritative when
  // a caller sees both source names in one vendor frame.
  assert(canonical_rigid_body_name("Ball", aliases) == "Ball");
  assert(canonical_rigid_body_name("SITE_A", aliases) == "CANONICAL_A");
  assert(canonical_rigid_body_name("SITE_B", aliases) == "CANONICAL_B");
  assert(canonical_rigid_body_name("UCB_P1", aliases) == "UCB_P1");

  for (const auto& invalid : std::vector<std::vector<std::string>>{
           {"SITE_A"},
           {"=CANONICAL_A"},
           {"SITE_A="},
           {"CANONICAL_A=CANONICAL_A"},
           {"SITE_A=CANONICAL_A", "SITE_A=CANONICAL_B"},
           {"SITE_A=CANONICAL_A", "SITE_B=CANONICAL_A"},
       }) {
    bool threw = false;
    try {
      (void)parse_rigid_body_name_aliases(invalid);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
  }
  return 0;
}
