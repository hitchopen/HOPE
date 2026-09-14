// Nominal authoring data; not an importable Motive asset or physical calibration.
// Source: documents/marker_transforms_pelvis_link_stickers.csv
// SHA-256: 3b304f521b85ebac8c8595d1666322ef3c64f9dcd40ffa0d339da25b431bdb90
// Order: S01..S24. Units: metres. No centroid subtraction.
// Native API basis: X forward, Y up, Z right. NatNet Streaming Up Axis must be Z.
// Motive 3.5 round-trip and live frame verification are still required.
#pragma once
#include <array>
#include <cstddef>

namespace a3_v3 {
constexpr std::size_t kMarkerCount = 24;
constexpr const wchar_t* kAssetName = L"P1";
constexpr std::array<const char*, kMarkerCount> kStationNames = {{
    "S01", "S02", "S03", "S04", "S05", "S06", "S07", "S08", "S09", "S10", "S11", "S12", "S13", "S14", "S15", "S16", "S17", "S18", "S19", "S20", "S21", "S22", "S23", "S24"
}};
constexpr std::array<std::array<double, 3>, kMarkerCount> kPelvisLinkMeters = {{
    {{0.067929625, 0.005649080, -0.067688434}}, // S01
    {{0.057466257, 0.045682189, -0.055634241}}, // S02
    {{0.020251256, 0.071954032, -0.058451422}}, // S03
    {{-0.023680990, 0.072053090, -0.061630517}}, // S04
    {{-0.067166397, 0.047266147, -0.054581659}}, // S05
    {{-0.079001398, -0.001320496, -0.057603870}}, // S06
    {{-0.063805908, -0.052205169, -0.060498038}}, // S07
    {{-0.019874543, -0.072035522, -0.054669064}}, // S08
    {{0.025486011, -0.070866434, -0.062169045}}, // S09
    {{0.060658280, -0.039912300, -0.055629311}}, // S10
    {{0.068681054, 0.013752572, -0.093371537}}, // S11
    {{0.055717983, 0.068987617, -0.102705623}}, // S12
    {{0.024198043, 0.082665663, -0.090053739}}, // S13
    {{-0.035525700, 0.082456235, -0.094529074}}, // S14
    {{-0.077885198, 0.044044970, -0.097023108}}, // S15
    {{-0.086615966, -0.018974166, -0.113903749}}, // S16
    {{-0.065029671, -0.065130802, -0.099352277}}, // S17
    {{-0.022709647, -0.083579602, -0.089897553}}, // S18
    {{0.034664172, -0.079647159, -0.092671819}}, // S19
    {{0.066487736, -0.030621789, -0.093037623}}, // S20
    {{0.079131593, 0.034880717, -0.156415262}}, // S21
    {{0.079112904, -0.034871230, -0.156402891}}, // S22
    {{-0.086390912, 0.038085312, -0.151171613}}, // S23
    {{-0.085135668, -0.047122895, -0.153207001}}, // S24
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
