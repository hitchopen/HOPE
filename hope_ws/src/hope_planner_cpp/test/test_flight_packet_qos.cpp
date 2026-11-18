#include "hope_planner_cpp/flight_packet_qos.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <condition_variable>
#include <future>
#include <mutex>
#include <vector>

namespace hope_planner_cpp {
namespace {
using namespace std::chrono_literals;

TEST(FlightPacketTransport, ReliableBoundedVolatileAndExpiring) {
  const auto qos = flight_packet_qos().get_rmw_qos_profile();
  EXPECT_EQ(qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(qos.depth, 8U);
  EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_EQ(qos.lifespan.sec, 0U);
  EXPECT_EQ(qos.lifespan.nsec, 100'000'000U);
}

TEST(FlightPacketTransport, RetryCannotRefreshOldFreezeOrSource) {
  constexpr std::int64_t now = 1'000'000'000;
  EXPECT_TRUE(flight_packet_transport_fresh(now - 24'000'000, now - 24'000'000, now));
  EXPECT_TRUE(flight_packet_transport_fresh(now - kFlightPacketMaxAgeNs,
      now - kFlightPacketMaxAgeNs, now));
  EXPECT_FALSE(flight_packet_transport_fresh(now - kFlightPacketMaxAgeNs - 1, now, now));
  EXPECT_FALSE(flight_packet_transport_fresh(now, now - kFlightPacketMaxAgeNs - 1, now));
  EXPECT_TRUE(flight_packet_transport_fresh(now + kFlightPacketFutureToleranceNs, now, now));
  EXPECT_FALSE(flight_packet_transport_fresh(now + kFlightPacketFutureToleranceNs + 1, now, now));
  EXPECT_FALSE(flight_packet_transport_fresh(now, now + kFlightPacketFutureToleranceNs + 1, now));
  EXPECT_FALSE(flight_packet_transport_fresh(0, now, now));
  EXPECT_FALSE(flight_packet_transport_fresh(now, 0, now));
}

struct BlockedWriter {
  std::mutex mutex;
  std::condition_variable changed;
  bool released = false;
  std::vector<std::uint64_t> sent;
  std::atomic<std::int64_t> now{1'000'000'000};
  void send(BoundedFlightPacketSender::Packet packet) {
    std::unique_lock<std::mutex> lock(mutex);
    sent.push_back(packet.flight_sequence);
    changed.notify_all();
    if (sent.size() == 1) changed.wait(lock, [this] { return released; });
  }
  bool wait_for(std::size_t size) {
    std::unique_lock<std::mutex> lock(mutex);
    return changed.wait_for(lock, 1s, [this, size] { return sent.size() >= size; });
  }
  void release() {
    { std::lock_guard<std::mutex> lock(mutex); released = true; }
    changed.notify_all();
  }
  BoundedFlightPacketSender::Packet packet(std::uint64_t sequence) const {
    BoundedFlightPacketSender::Packet result;
    result.flight_sequence = sequence;
    result.freeze_wall_unix_ns = now.load();
    result.samples.resize(1);
    result.samples.back().exposure_unix_stamp_ns = now.load();
    return result;
  }
};

TEST(FlightPacketTransport, BlockedDdsNeverBlocksProducerAndQueueIsBounded) {
  BlockedWriter writer;
  BoundedFlightPacketSender sender(
      [&](auto p) { writer.send(std::move(p)); }, [&] { return writer.now.load(); });
  sender.set_gate(1, true);
  sender.enqueue(writer.packet(1), 1);
  EXPECT_TRUE(writer.wait_for(1));
  auto producer = std::async(std::launch::async, [&] {
    for (int i = 2; i <= 30; ++i) sender.enqueue(writer.packet(i), 1);
    return sender.pending_size();
  });
  const auto completed = producer.wait_for(50ms);
  EXPECT_EQ(completed, std::future_status::ready);
  if (completed == std::future_status::ready) {
    EXPECT_EQ(producer.get(), 8U);
  }
  writer.release();
  EXPECT_TRUE(writer.wait_for(9));
  std::lock_guard<std::mutex> lock(writer.mutex);
  ASSERT_EQ(writer.sent.size(), 9U);
  EXPECT_EQ(writer.sent[1], 23U);
  EXPECT_EQ(writer.sent.back(), 30U);
}

TEST(FlightPacketTransport, NewGenerationAndClosedGateCancelQueuedWrites) {
  BlockedWriter writer;
  BoundedFlightPacketSender sender(
      [&](auto p) { writer.send(std::move(p)); }, [&] { return writer.now.load(); });
  sender.set_gate(1, true);
  sender.enqueue(writer.packet(1), 1);
  EXPECT_TRUE(writer.wait_for(1));
  sender.enqueue(writer.packet(2), 1);
  sender.set_gate(1, false);
  EXPECT_EQ(sender.pending_size(), 0U);
  EXPECT_FALSE(sender.enqueue(writer.packet(3), 1));
  sender.set_gate(1, true);
  sender.enqueue(writer.packet(4), 1);
  sender.set_gate(2, true);
  EXPECT_EQ(sender.pending_size(), 0U);
  EXPECT_FALSE(sender.enqueue(writer.packet(5), 1));
  EXPECT_TRUE(sender.enqueue(writer.packet(6), 2));
  writer.release();
  EXPECT_TRUE(writer.wait_for(2));
  std::lock_guard<std::mutex> lock(writer.mutex);
  EXPECT_EQ(writer.sent, (std::vector<std::uint64_t>{1, 6}));
}

TEST(FlightPacketTransport, BackendBacklogCannotSendExpiredFlight) {
  BlockedWriter writer;
  BoundedFlightPacketSender sender(
      [&](auto p) { writer.send(std::move(p)); }, [&] { return writer.now.load(); });
  sender.set_gate(1, true);
  sender.enqueue(writer.packet(1), 1);
  EXPECT_TRUE(writer.wait_for(1));
  sender.enqueue(writer.packet(2), 1);
  writer.now += kFlightPacketMaxAgeNs + 1;
  sender.enqueue(writer.packet(3), 1);
  writer.release();
  EXPECT_TRUE(writer.wait_for(2));
  std::lock_guard<std::mutex> lock(writer.mutex);
  EXPECT_EQ(writer.sent, (std::vector<std::uint64_t>{1, 3}));
}
}  // namespace
}  // namespace hope_planner_cpp
