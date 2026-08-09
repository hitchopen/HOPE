#pragma once

#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace a3_pingpong {

// Small self-contained SHA-256 implementation used only during serve artifact
// loading.  Keeping the digest check in the runner makes a copied/stale CSV
// fail closed even when an operator bypasses the Python preflight wrapper.
class PpSha256 {
 public:
  static std::string Bytes(const std::vector<std::uint8_t>& input) {
    std::vector<std::uint8_t> message = input;
    const std::uint64_t bit_length =
        static_cast<std::uint64_t>(message.size()) * 8ULL;
    message.push_back(0x80U);
    while ((message.size() % 64U) != 56U) message.push_back(0U);
    for (int shift = 56; shift >= 0; shift -= 8) {
      message.push_back(
          static_cast<std::uint8_t>((bit_length >> shift) & 0xffU));
    }

    std::array<std::uint32_t, 8> h = {
        0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
        0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
    for (std::size_t offset = 0; offset < message.size(); offset += 64U) {
      std::array<std::uint32_t, 64> w{};
      for (int i = 0; i < 16; ++i) {
        const std::size_t j = offset + static_cast<std::size_t>(4 * i);
        w[i] = (static_cast<std::uint32_t>(message[j]) << 24U) |
               (static_cast<std::uint32_t>(message[j + 1]) << 16U) |
               (static_cast<std::uint32_t>(message[j + 2]) << 8U) |
               static_cast<std::uint32_t>(message[j + 3]);
      }
      for (int i = 16; i < 64; ++i) {
        const std::uint32_t s0 =
            RotR_(w[i - 15], 7) ^ RotR_(w[i - 15], 18) ^ (w[i - 15] >> 3U);
        const std::uint32_t s1 =
            RotR_(w[i - 2], 17) ^ RotR_(w[i - 2], 19) ^ (w[i - 2] >> 10U);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
      }

      std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
      std::uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];
      for (int i = 0; i < 64; ++i) {
        const std::uint32_t s1 =
            RotR_(e, 6) ^ RotR_(e, 11) ^ RotR_(e, 25);
        const std::uint32_t ch = (e & f) ^ ((~e) & g);
        const std::uint32_t temp1 =
            hh + s1 + ch + k_[i] + w[i];
        const std::uint32_t s0 =
            RotR_(a, 2) ^ RotR_(a, 13) ^ RotR_(a, 22);
        const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const std::uint32_t temp2 = s0 + maj;
        hh = g;
        g = f;
        f = e;
        e = d + temp1;
        d = c;
        c = b;
        b = a;
        a = temp1 + temp2;
      }
      h[0] += a;
      h[1] += b;
      h[2] += c;
      h[3] += d;
      h[4] += e;
      h[5] += f;
      h[6] += g;
      h[7] += hh;
    }

    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::uint32_t value : h) out << std::setw(8) << value;
    return out.str();
  }

  static std::string String(const std::string& input) {
    return Bytes(std::vector<std::uint8_t>(input.begin(), input.end()));
  }

  static std::string File(const std::string& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("cannot open SHA256 input: " + path);
    std::vector<std::uint8_t> bytes;
    std::array<char, 64 * 1024> block{};
    while (stream) {
      stream.read(block.data(), static_cast<std::streamsize>(block.size()));
      const std::streamsize count = stream.gcount();
      if (count > 0) {
        bytes.insert(bytes.end(), block.begin(), block.begin() + count);
      }
    }
    return Bytes(bytes);
  }

 private:
  static std::uint32_t RotR_(std::uint32_t value, int count) {
    return (value >> count) | (value << (32 - count));
  }

  inline static constexpr std::array<std::uint32_t, 64> k_ = {
      0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
      0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
      0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
      0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
      0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
      0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
      0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
      0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
      0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
      0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
      0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
      0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
      0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
      0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
      0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
      0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};
};

}  // namespace a3_pingpong
