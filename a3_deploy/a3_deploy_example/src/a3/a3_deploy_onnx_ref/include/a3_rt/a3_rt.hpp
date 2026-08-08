// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// PR 4/10 of the A3 backend adaptation (see notes/a3_backend_plan.md §PR 4).
//
// Minimal, self-contained RT primitives for periodic real-time threads.
// Ported / simplified from motion_control_a3/src/base/rt/sched_rt.hpp +
// memory_lock.hpp (no AimRT / DataBoard / ActionGraph dependencies).
#pragma once

#include <cstddef>
#include <cstdint>

namespace a3_rt {

// Describes how an A3BasedTask thread should be scheduled.
struct RtSched {
  int priority = 0;  // 0 = no SCHED_FIFO; >0 = SCHED_FIFO with this priority
  int cpu = -1;      // -1 = no affinity; >=0 = pin to this core
};

// Set SCHED_FIFO + priority on the current thread.
// Returns true on success. Logs errno on failure (does not throw).
// Typical failure modes: EPERM on non-privileged processes
// (requires CAP_SYS_NICE or root). Callers should tolerate failure.
bool SetRtSchedFifo(int priority);

// Pin the current thread to CPU core `cpu`. Returns true on success.
bool PinCurrentThreadToCpu(int cpu);

// mlockall(MCL_CURRENT | MCL_FUTURE) and touch `stack_bytes` of stack to
// pre-fault pages. Typically called once at process start from main().
// Returns true on success; logs errno on failure (does not throw).
bool LockMemory(std::size_t stack_bytes = 512 * 1024);

}  // namespace a3_rt
