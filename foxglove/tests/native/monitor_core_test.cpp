#include "monitor_core.hpp"
#include "network_addresses.hpp"
#include <iostream>
namespace hm = hope_monitor;
#define CHECK(x) do { if (!(x)) throw std::runtime_error("failed: " #x); } while (0)
template<class F> void rejects(F function) {
  bool threw = false; try { function(); } catch (const std::exception &) { threw = true; }
  CHECK(threw);
}
int main() {
  try {
    CHECK(hope_network::accepts_scope(hope_network::Scope::Global, RT_SCOPE_UNIVERSE));
    CHECK(!hope_network::accepts_scope(hope_network::Scope::Global, RT_SCOPE_LINK));
    CHECK(!hope_network::accepts_scope(hope_network::Scope::Global, RT_SCOPE_HOST));
    CHECK(hope_network::accepts_scope(hope_network::Scope::All, RT_SCOPE_LINK));
    CHECK(hope_network::accepts_scope(hope_network::Scope::All, RT_SCOPE_HOST));
    const std::string track = "506E6C47,192.0.2.10,2,1234567890,0.002,0,0,1,0,13,0.001,0.004,64,Normal";
    auto ntp = hm::parse_ntp(track,"^* 192.0.2.10\n",10,0);
    CHECK(ntp.pass && ntp.qualified && ntp.offset == 2 && ntp.dispersion == 4);
    CHECK(!hm::parse_ntp(track,"^? 192.0.2.10\n",10,0).qualified);
    CHECK(!hm::parse_ntp(track,"^* x\n",10,5).pass);
    CHECK(!hm::parse_ntp(track,"^* x\n",1,0).pass);
    rejects([&] { hm::parse_ntp("bad","",10,0); });
    rejects([&] { hm::parse_ntp(track,"",NAN,0); });
    auto a = hm::parse_cpu("cpu 100 0 100 700 50 10 20 20 0 0\n");
    auto b = hm::parse_cpu("cpu 120 0 130 750 60 10 30 20 0 0\n");
    CHECK(std::abs(hm::cpu_load(a,b)-50) < 1e-9);
    rejects([&] { hm::cpu_load(b,a); });
    rejects([] { hm::parse_cpu("cpu -1 0 0 0 0 0 0 0\n"); });
    rejects([] { hm::exact(NAN); }); rejects([] { hm::exact(-1); }); rejects([] { hm::exact(1.1); });
    CHECK(hm::exact(42) == 42);
    std::string stat = "123 (name with ) parens) R";
    for (int i = 1; i <= 19; ++i) stat += " " + std::to_string(i);
    auto p = hm::parse_process(stat); CHECK(p.pid == 123 && p.name == "name with ) parens" && p.ticks == 23 && p.start == 19);
    const auto request = hm::estop_request(1700000000123456789ULL,"hope-test-trace");
    auto fields = hm::fields(request); CHECK(fields.size() == 2 && fields[1].number == 2);
    CHECK(fields[1].data == hm::Bytes({0x10,0x01}));
    // Golden bytes from Python build_software_estop_request, same timestamp/trace.
    const hm::Bytes expected{10,52,10,18,8,128,226,207,170,6,16,149,154,239,58,24,251,208,149,255,188,49,34,15,104,111,112,101,45,116,101,115,116,45,116,114,97,99,101,42,13,104,111,112,101,45,102,111,120,103,108,111,118,101,18,2,16,1};
    CHECK(request == expected);
    CHECK(hm::estop_response({10,0}).first == 0);
    CHECK(hm::estop_response({10,11,8,17,18,7,'b','l','o','c','k','e','d'}).first == 17);
    rejects([] { hm::estop_response({}); }); rejects([] { hm::estop_response({10,5,8}); });
    rejects([] { hm::estop_response({10,0,10,0}); }); rejects([] { hm::estop_response({10,4,8,0,8,1}); });
    rejects([] { hm::estop_response({10,4,18,2,0xc0,0x80}); });
    CHECK(hm::json_string("a\"\n") == "\"a\\\"\\u000a\"");
    rejects([] { hm::fields({0}); }); rejects([] { hm::fields({15}); });
    CHECK(hm::estop_status(true,true,false,true).full);
    CHECK(hm::estop_status(false,true,false,true).ready);
    CHECK(!hm::estop_status(false,true,false,true).full);
    CHECK(!hm::estop_status(true,true,true,true).ready);
    for (bool vendor : {false,true}) for (bool runner : {false,true}) {
      const auto result = hm::combine_estop(vendor,"v",runner,"r");
      CHECK(result.first == (vendor || runner)); CHECK(result.second.find("physical E-stop") != std::string::npos);
    }
    char temporary[] = "/tmp/hope-monitor-core.XXXXXX";
    auto path = mkdtemp(temporary); CHECK(path);
    std::filesystem::path dir(path), latch = dir / "latch";
    hm::persist_latch(latch,42); CHECK(hm::read_file(latch) == "asserted_utc_ns=42\n");
    hm::persist_latch(latch,43); CHECK(hm::read_file(latch).find("43") != std::string::npos);
    hm::reset_latch(latch); CHECK(!std::filesystem::exists(latch)); hm::reset_latch(latch);
    std::filesystem::create_directory(latch); rejects([&] { hm::reset_latch(latch); });
    std::filesystem::remove(latch); std::filesystem::remove(dir);
    CHECK(hm::command({"/usr/bin/printf","probe"}).output == "probe");
    const auto start = hm::mono(); rejects([] { hm::command({"/usr/bin/sleep","5"},.05); }); CHECK(hm::mono()-start < 1);
    hm::ProbeWorker<std::thread::id> worker;
    const auto worker_id = worker.submit([] { return std::this_thread::get_id(); }).get();
    for (unsigned i = 0; i < 50; ++i)
      CHECK(worker.submit([] { return std::this_thread::get_id(); }).get() == worker_id);
    rejects([&] { worker.submit([]() -> std::thread::id { throw std::runtime_error("probe failed"); }).get(); });
    CHECK(worker.submit([] { return std::this_thread::get_id(); }).get() == worker_id);
    std::promise<void> release; auto released = release.get_future().share();
    auto busy = worker.submit([released] { released.wait(); return std::this_thread::get_id(); });
    rejects([&] { worker.submit([] { return std::this_thread::get_id(); }).get(); });
    release.set_value(); CHECK(busy.get() == worker_id);
    worker.stop(); rejects([&] { worker.submit([] { return std::this_thread::get_id(); }).get(); });
    std::cout << "monitor core contracts passed\n"; return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
