// Differential replay CLI, never installed or compiled into production.
#include "operations_observer_core.hpp"
#include <iostream>
using namespace hope_operations;
int main() {
  std::string line;
  while(std::getline(std::cin,line)) {
    try {
      J request=J::parse(line);Observer o;Players players;
      for(const auto &event:request.at("events")) {
        const std::string kind=event.at("kind");auto now=event.at("now").get<std::int64_t>();
        if(kind=="ball"){o.ball(now);continue;}
        std::vector<double> data;for(const auto &v:event.at("data")) data.push_back(numeric(v));
        if(kind=="base")o.base.update(data,now,"BASE PACKET",decode_base);
        else if(kind=="runner")o.runner.update(data,now,"RUNNER STATE",decode_runner);
        else if(kind=="command")o.command.update(data,now,"RACKET PACKET",decode_racket,false);
      }
      for(const auto &p:request.value("players",J::array())) {
        std::vector<double> pos,q;for(auto &v:p.at("p"))pos.push_back(numeric(v));for(auto &v:p.at("q"))q.push_back(numeric(v));
        players.update(p.at("name"),pos,q,p.at("stamp"),p.at("frame"),numeric(p.at("now")));
      }
      J result={{"base",o.base.snapshot()},{"runner",o.runner.snapshot()},{"command",o.command.snapshot()},
        {"view",o.view(request.at("now"),request.at("ros"),request.at("runtime"))},
        {"players",players.status(request.at("now").get<std::int64_t>()*1e-9)}};
      std::cout<<result.dump()<<std::endl;
    }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
  }
}
