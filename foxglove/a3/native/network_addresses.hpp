// Shared, bounded, read-only Linux IPv4 address dump. No subprocess or ROS state.
#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include <arpa/inet.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace hope_network {
enum class Scope { All, Global };
inline bool accepts_scope(Scope scope, unsigned kernel_scope) {
  return scope == Scope::All || kernel_scope == RT_SCOPE_UNIVERSE;
}
struct IPv4Address { unsigned index; std::string name, address; };
inline std::vector<IPv4Address> ipv4_addresses(Scope scope) {
  const int fd = socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
  if (fd < 0) throw std::runtime_error("Cannot inspect network addresses");
  struct Close { int fd; ~Close() { close(fd); } } closer{fd};
  struct { nlmsghdr header; ifaddrmsg address; } request{};
  request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifaddrmsg));
  request.header.nlmsg_type = RTM_GETADDR;
  request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
  request.header.nlmsg_seq = 1;
  request.address.ifa_family = AF_INET;
  sockaddr_nl kernel{}; kernel.nl_family = AF_NETLINK;
  if (sendto(fd, &request, request.header.nlmsg_len, 0,
      reinterpret_cast<sockaddr *>(&kernel), sizeof(kernel)) < 0)
    throw std::runtime_error("Cannot request network address dump");
  std::vector<IPv4Address> addresses;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    pollfd input{fd, POLLIN, 0};
    if (poll(&input, 1, 25) <= 0) continue;
    alignas(nlmsghdr) std::array<char, 32768> data;
    int length = recv(fd, data.data(), data.size(), MSG_DONTWAIT | MSG_TRUNC);
    if (length <= 0 || length > static_cast<int>(data.size()))
      throw std::runtime_error("Incomplete network address dump");
    for (auto *h = reinterpret_cast<nlmsghdr *>(data.data()); NLMSG_OK(h, length); h = NLMSG_NEXT(h, length)) {
      if (h->nlmsg_flags & NLM_F_DUMP_INTR) throw std::runtime_error("Network address dump interrupted");
      if (h->nlmsg_type == NLMSG_DONE) {
        std::sort(addresses.begin(), addresses.end(), [](const auto &a, const auto &b) {
          return std::tie(a.name, a.index, a.address) < std::tie(b.name, b.index, b.address);
        });
        return addresses;
      }
      if (h->nlmsg_type == NLMSG_ERROR) throw std::runtime_error("Network address dump rejected");
      if (h->nlmsg_type != RTM_NEWADDR || h->nlmsg_len < NLMSG_LENGTH(sizeof(ifaddrmsg))) continue;
      const auto *a = static_cast<ifaddrmsg *>(NLMSG_DATA(h));
      if (a->ifa_family != AF_INET || !accepts_scope(scope, a->ifa_scope)) continue;
      char name[IF_NAMESIZE];
      // Never turn an incomplete dump into an apparent address removal. The
      // monitor warns/retries a failed poll; the config relay retains its clean
      // nonzero-exit/restart policy on probe failure.
      if (!if_indextoname(a->ifa_index, name)) throw std::runtime_error("Network interface disappeared during dump");
      if (std::string(name) == "lo") continue;
      int remaining = IFA_PAYLOAD(h);
      for (auto *attribute = IFA_RTA(a); RTA_OK(attribute, remaining); attribute = RTA_NEXT(attribute, remaining)) {
        if (attribute->rta_type != IFA_LOCAL || RTA_PAYLOAD(attribute) != sizeof(in_addr)) continue;
        char ip[INET_ADDRSTRLEN];
        if (!inet_ntop(AF_INET, RTA_DATA(attribute), ip, sizeof(ip)))
          throw std::runtime_error("Invalid network address");
        addresses.push_back({a->ifa_index, name, ip});
      }
    }
  }
  throw std::runtime_error("Network address dump timed out");
}
// Matches Python ipv4_interface_signature: global scope, index/name/address.
using InterfaceSignature = std::set<std::tuple<unsigned, std::string, std::string>>;
inline InterfaceSignature ipv4_interface_signature() {
  InterfaceSignature result;
  for (const auto &a : ipv4_addresses(Scope::Global)) result.emplace(a.index, a.name, a.address);
  return result;
}
}  // namespace hope_network
