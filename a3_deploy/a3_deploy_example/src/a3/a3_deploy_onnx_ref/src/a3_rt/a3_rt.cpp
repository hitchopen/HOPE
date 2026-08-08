// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// PR 4/10 of the A3 backend adaptation (see notes/a3_backend_plan.md §PR 4).
#include "a3_rt/a3_rt.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <vector>

namespace a3_rt {

bool SetRtSchedFifo(int priority) {
  if (priority <= 0) return true;  // no-op
  sched_param param{};
  param.sched_priority = priority;
  const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (rc != 0) {
    std::fprintf(stderr,
                 "[a3_rt] SetRtSchedFifo(%d) failed: %s "
                 "(need CAP_SYS_NICE or root)\n",
                 priority, std::strerror(rc));
    return false;
  }
  return true;
}

bool PinCurrentThreadToCpu(int cpu) {
  if (cpu < 0) return true;  // no-op
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
  if (rc != 0) {
    std::fprintf(stderr, "[a3_rt] PinCurrentThreadToCpu(%d) failed: %s\n", cpu,
                 std::strerror(rc));
    return false;
  }
  return true;
}

bool LockMemory(std::size_t stack_bytes) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::fprintf(stderr, "[a3_rt] mlockall failed: %s\n", std::strerror(errno));
    return false;
  }
  // Pre-fault `stack_bytes` of stack pages.
  if (stack_bytes > 0) {
    std::vector<char> buf(stack_bytes, 0);
    // Touch every page (typical page size 4K).
    for (std::size_t i = 0; i < buf.size(); i += 4096) {
      buf[i] = 1;
    }
    // Prevent optimization: read something.
    volatile char sink = buf[0];
    (void)sink;
  }
  return true;
}

}  // namespace a3_rt
