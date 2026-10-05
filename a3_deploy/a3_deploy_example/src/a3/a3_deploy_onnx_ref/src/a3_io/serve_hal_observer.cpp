// ABI-pinned, passive observer for f83690f1 libhal_elink_pkg.so (A3_T3D0).
// The original serializer runs exactly once; its arguments/result are unchanged.
// No command publication, new thread, sleep, or hand-loop scheduling change.
#include "a3_io/serve_hal_clock.hpp"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <dlfcn.h>
#include <link.h>
#include <vector>

namespace aimrt_hal::elink_module {
struct HandCmdData;
class UpperNode {
 public:
  void WriteHandOuput();
  std::vector<unsigned char> update_hand_cmd_tlv(HandCmdData&);
};
}
namespace {
using Serializer = std::vector<unsigned char>(*)(void*, void*);
using Output = void(*)(void*);
constexpr char kSymbol[] = "_ZN9aimrt_hal12elink_module9UpperNode19update_hand_cmd_tlvERNS0_11HandCmdDataE";
struct Observer {
  Serializer original{};
  Output original_output{};
  a3_io::ServeHalClock* clock{};
  std::uint64_t previous_left = ~0ULL, node = 0;
  Observer() {
    // AimRT may load packages RTLD_LOCAL, outside RTLD_NEXT's lookup scope.
    void* handle = dlopen("libhal_elink_pkg.so", RTLD_NOW | RTLD_NOLOAD);
    original = reinterpret_cast<Serializer>(handle ? dlsym(handle, kSymbol) : nullptr);
    original_output = reinterpret_cast<Output>(handle ? dlsym(handle,
        "_ZN9aimrt_hal12elink_module9UpperNode14WriteHandOuputEv") : nullptr);
    if (!original || !original_output) { std::fputs("[serve_hal_clock] HAL ABI unavailable\n", stderr); std::abort(); }
    const char* path = std::getenv("HOPE_SERVE_HAL_CLOCK");
    if (!path || !*path) return;
    const int fd = open(path, O_RDWR | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (fd < 0) return;  // Missing clock fails closed in Runner, never alters HAL.
    if (ftruncate(fd, sizeof(a3_io::ServeHalClock)) == 0) {
      void* p = mmap(nullptr, sizeof(a3_io::ServeHalClock), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
      if (p != MAP_FAILED) clock = static_cast<a3_io::ServeHalClock*>(p);
    }
    close(fd);
    if (clock) std::fprintf(stderr, "[serve_hal_clock] passive selection observer pid=%d path=%s\n", getpid(), path);
  }
  void Heartbeat(std::uint64_t stamp) {
    if (!clock) return;
    __atomic_add_fetch(&clock->sequence, 1ULL, __ATOMIC_SEQ_CST);
    __atomic_store_n(&clock->magic, a3_io::ServeHalClock::kMagic, __ATOMIC_SEQ_CST);
    __atomic_store_n(&clock->pid, static_cast<std::uint64_t>(getpid()), __ATOMIC_SEQ_CST);
    __atomic_store_n(&clock->heartbeat_ns, stamp, __ATOMIC_SEQ_CST);
    __atomic_add_fetch(&clock->sequence, 1ULL, __ATOMIC_SEQ_CST);
  }
  void Record(void* self, const void* cmd, std::uint64_t stamp) {
    if (!clock) return;
    const auto current_node = reinterpret_cast<std::uintptr_t>(self);
    if (node && node != current_node) return;  // Exactly one audited UpperNode.
    node = current_node;
    std::uint16_t left;
    std::memcpy(&left, cmd, sizeof(left));
    __atomic_add_fetch(&clock->sequence, 1ULL, __ATOMIC_SEQ_CST);
    __atomic_store_n(&clock->magic, a3_io::ServeHalClock::kMagic, __ATOMIC_SEQ_CST);
    __atomic_store_n(&clock->pid, static_cast<std::uint64_t>(getpid()), __ATOMIC_SEQ_CST);
    __atomic_store_n(&clock->selected_ns, stamp, __ATOMIC_SEQ_CST);
    // Do not call the initial cached OPEN, or repeated RELEASE, a new edge.
    if (left == 2000 && previous_left != 2000 && previous_left != ~0ULL)
      __atomic_store_n(&clock->release_edge_ns, stamp, __ATOMIC_SEQ_CST);
    __atomic_store_n(&clock->left, static_cast<std::uint64_t>(left), __ATOMIC_SEQ_CST);
    __atomic_store_n(&clock->node, node, __ATOMIC_SEQ_CST);
    __atomic_add_fetch(&clock->sequence, 1ULL, __ATOMIC_SEQ_CST);
    previous_left = left;
  }
};
Observer& GetObserver() { static Observer observer; return observer; }
std::uint64_t NowNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return std::uint64_t(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}
}
void aimrt_hal::elink_module::UpperNode::WriteHandOuput() {
  auto& observer = GetObserver();
  // Runs before any command is received. Liveness is separate from selection;
  // never manufacture a release receipt to satisfy Runner's startup check.
  observer.Heartbeat(NowNs());
  observer.original_output(this);
}
std::vector<unsigned char> aimrt_hal::elink_module::UpperNode::update_hand_cmd_tlv(HandCmdData& command) {
  auto& observer = GetObserver();
  const auto stamp = NowNs();
  auto packet = observer.original(this, &command);
  observer.Record(this, &command, stamp);
  return packet;
}

namespace {
void ObserveOutput(void* self) {
  auto& observer = GetObserver();
  observer.Heartbeat(NowNs());
  observer.original_output(self);
}
std::vector<unsigned char> ObserveSerialize(void* self, void* command) {
  auto& observer = GetObserver();
  const auto stamp = NowNs();
  auto packet = observer.original(self, command);
  observer.Record(self, command, stamp);
  return packet;
}

// AimRT uses RTLD_NOW | RTLD_DEEPBIND. Preserve those flags and every other
// binding (notably protobuf/libstdc++); redirect only these two audited PLT
// slots after relocation, before AimRT initializes the HAL module. No code
// bytes or on-disk vendor files are changed. RELRO is restored immediately.
bool BindObserverSlots(void* handle) {
  link_map* map = nullptr;
  if (dlinfo(handle, RTLD_DI_LINKMAP, &map) != 0 || !map) return false;
  const ElfW(Sym)* symbols = nullptr;
  const char* strings = nullptr;
  const ElfW(Rela)* relocations = nullptr;
  std::size_t bytes = 0;
  for (auto* d = map->l_ld; d->d_tag != DT_NULL; ++d) {
    switch (d->d_tag) {
      case DT_SYMTAB: symbols = reinterpret_cast<const ElfW(Sym)*>(d->d_un.d_ptr); break;
      case DT_STRTAB: strings = reinterpret_cast<const char*>(d->d_un.d_ptr); break;
      case DT_JMPREL: relocations = reinterpret_cast<const ElfW(Rela)*>(d->d_un.d_ptr); break;
      case DT_PLTRELSZ: bytes = d->d_un.d_val; break;
      case DT_PLTREL: if (d->d_un.d_val != DT_RELA) return false; break;
    }
  }
  if (!symbols || !strings || !relocations) return false;
  const auto page_size = static_cast<std::uintptr_t>(sysconf(_SC_PAGESIZE));
  const auto* elf = reinterpret_cast<const ElfW(Ehdr)*>(map->l_addr);
  const auto* headers = reinterpret_cast<const ElfW(Phdr)*>(map->l_addr + elf->e_phoff);
  std::uintptr_t relro_begin = 0, relro_end = 0;
  for (unsigned i = 0; i < elf->e_phnum; ++i) if (headers[i].p_type == PT_GNU_RELRO) {
    relro_begin = (map->l_addr + headers[i].p_vaddr) & ~(page_size - 1);
    relro_end = (map->l_addr + headers[i].p_vaddr + headers[i].p_memsz) & ~(page_size - 1);
  }
  struct Binding { const char* name; void* target; void** slot{}; } bindings[] = {
    {"_ZN9aimrt_hal12elink_module9UpperNode14WriteHandOuputEv", reinterpret_cast<void*>(&ObserveOutput)},
    {kSymbol, reinterpret_cast<void*>(&ObserveSerialize)}};
  for (std::size_t i = 0; i < bytes / sizeof(ElfW(Rela)); ++i) {
    const auto& r = relocations[i];
    for (auto& b : bindings) if (std::strcmp(strings + symbols[ELF64_R_SYM(r.r_info)].st_name, b.name) == 0) {
#if defined(__aarch64__)
      if (ELF64_R_TYPE(r.r_info) != R_AARCH64_JUMP_SLOT) return false;
#elif defined(__x86_64__)
      if (ELF64_R_TYPE(r.r_info) != R_X86_64_JUMP_SLOT) return false;
#endif
      b.slot = reinterpret_cast<void**>(map->l_addr + r.r_offset);
      const auto page = reinterpret_cast<std::uintptr_t>(b.slot) & ~(page_size - 1);
      if (page < relro_begin || page >= relro_end) return false;
      if (*b.slot != dlsym(handle, b.name) && *b.slot != b.target) return false;
    }
  }
  for (const auto& b : bindings) if (!b.slot) return false;
  for (const auto& b : bindings) {
    void* page = reinterpret_cast<void*>(reinterpret_cast<std::uintptr_t>(b.slot) & ~(page_size - 1));
    if (mprotect(page, page_size, PROT_READ | PROT_WRITE) != 0) return false;
    __atomic_store_n(b.slot, b.target, __ATOMIC_RELEASE);
    if (mprotect(page, page_size, PROT_READ) != 0) return false;
  }
  std::fputs("[serve_hal_clock] HAL deepbind preserved; two observer PLT slots bound; RELRO restored\n", stderr);
  return true;
}
}

extern "C" void* dlopen(const char* filename, int flags) noexcept {
  using Open = void*(*)(const char*, int);
  static auto original_open = reinterpret_cast<Open>(dlsym(RTLD_NEXT, "dlopen"));
  if (!original_open) return nullptr;
  void* handle = original_open(filename, flags);
  const char* base = filename ? std::strrchr(filename, '/') : nullptr;
  base = base ? base + 1 : filename;
  if (handle && base && std::strcmp(base, "libhal_elink_pkg.so") == 0 &&
      (flags & RTLD_DEEPBIND) && !(flags & RTLD_NOLOAD) &&
      std::getenv("HOPE_SERVE_HAL_CLOCK") && !BindObserverSlots(handle)) {
    std::fputs("[serve_hal_clock] HAL observer binding failed before module initialization\n", stderr);
    return nullptr;
  }
  return handle;
}
