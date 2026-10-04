// Nominal authoring data; not an importable Motive asset or physical calibration.
// Source: documents/marker_transforms_pelvis_link_stickers.csv
// SHA-256: 8b1bf1eba661e2a07ae5591aa5d57b9057b496c3b57489b6d206f4855b7ada9e
// Order: S04, S05, S11, S12, S13, S14, S16, S18, S19, S20, S21, S22. Units: metres. No centroid subtraction.
// Native API basis: X forward, Y up, Z right. NatNet Streaming Up Axis must be Z.
// Motive 3.5 round-trip and live frame verification are still required.
#pragma once
#include <array>
#include <cstddef>

namespace a3_v3 {
constexpr std::size_t kMarkerCount = 12;
constexpr const wchar_t* kAssetName = L"P1";
constexpr std::array<const char*, kMarkerCount> kStationNames = {{
    "S04", "S05", "S11", "S12", "S13", "S14", "S16", "S18", "S19", "S20", "S21", "S22"
}};
constexpr std::array<std::array<double, 3>, kMarkerCount> kPelvisLinkMeters = {{
    {{-0.023680990, 0.072053090, -0.061630517}}, // S04
    {{-0.067166397, 0.047266147, -0.054581659}}, // S05
    {{0.068681054, 0.013752572, -0.093371537}}, // S11
    {{0.055717983, 0.068987617, -0.102705623}}, // S12
    {{0.024198043, 0.082665663, -0.090053739}}, // S13
    {{-0.035525700, 0.082456235, -0.094529074}}, // S14
    {{-0.086615966, -0.018974166, -0.113903749}}, // S16
    {{-0.022709647, -0.083579602, -0.089897553}}, // S18
    {{0.034664172, -0.079647159, -0.092671819}}, // S19
    {{0.066487736, -0.030621789, -0.093037623}}, // S20
    {{0.079131593, 0.034880717, -0.156415262}}, // S21
    {{0.079112904, -0.034871230, -0.156402891}}, // S22
}};

// Returns a writable packed xyz array, as required by CreateRigidBody.
inline std::array<float, 3 * kMarkerCount> nativeYUpMarkerList() {
    std::array<float, 3 * kMarkerCount> result{};
    for (std::size_t i = 0; i < kMarkerCount; ++i) {
        const auto& p = kPelvisLinkMeters[i];
        result[3 * i] = static_cast<float>(p[0]);
        result[3 * i + 1] = static_cast<float>(p[2]);
        result[3 * i + 2] = static_cast<float>(-p[1]);
    }
    return result;
}
} // namespace a3_v3
