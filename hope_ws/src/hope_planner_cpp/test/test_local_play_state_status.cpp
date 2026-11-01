#include "hope_planner_cpp/local_play_state_status.hpp"
#include "local_status_client.hpp"
#include "hope_planner_cpp/local_control_key.hpp"
#include <gtest/gtest.h>

namespace hope_planner_cpp {
TEST(LocalPlayStateStatus, ReadOnlyLocalFeedExpiresStaleProducerAndIsNotRos) {
  LocalPlayStateStatus server("127.0.0.1",0);
  EXPECT_NE(local_status_get(server.port()).find("503 Service Unavailable"),std::string::npos);
  server.update("{\"state\":\"OPPONENT_RALLY\"}");
  auto response=local_status_get(server.port());
  EXPECT_NE(response.find("200 OK"),std::string::npos);
  EXPECT_NE(response.find("Cache-Control: no-store"),std::string::npos);
  EXPECT_NE(response.find("OPPONENT_RALLY"),std::string::npos);
  EXPECT_NE(local_status_get(server.port(),"OPTIONS","/ball-state","Origin: https://app.foxglove.dev\r\n").find("204 No Content"),std::string::npos);
  EXPECT_NE(local_status_get(server.port(),"POST","/ball-state","Origin: https://app.foxglove.dev\r\n").find("404 Not Found"),std::string::npos);
  std::this_thread::sleep_for(std::chrono::milliseconds(1050));
  response=local_status_get(server.port());
  EXPECT_NE(response.find("503 Service Unavailable"),std::string::npos);
  EXPECT_EQ(response.find("OPPONENT_RALLY"),std::string::npos);
}
TEST(LocalPlayStateStatus, SlowReaderDoesNotBlockProducerUpdates) {
  LocalPlayStateStatus server("127.0.0.1",0);
  int fd=::socket(AF_INET,SOCK_STREAM,0);
  sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(server.port());a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  ASSERT_EQ(::connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a)),0);
  const auto start=std::chrono::steady_clock::now();
  for(int i=0;i<1000;++i)server.update(std::to_string(i));
  EXPECT_LT(std::chrono::steady_clock::now()-start,std::chrono::milliseconds(50));
  ::close(fd);
  EXPECT_NE(local_status_get(server.port()).find("999"),std::string::npos);
}
TEST(LocalPlayStateStatus, IntentsAreLocalProducerScopedOrderedAndBounded) {
  LocalPlayStateStatus server("127.0.0.1",0,"producer-new");
  server.update("{}");
  auto post=[&](const std::string& action, const std::string& revision,
                const std::string& producer="producer-new",const std::string& extra="") {
    return local_status_get(server.port(),"POST","/ball-state/intent/"+action,
        "Origin: https://app.foxglove.dev\r\nX-Hope-Producer: "+producer+"\r\nX-Hope-Intent-Revision: "+revision+"\r\n"+extra);
  };
  EXPECT_NE(post("READY","0","producer-old").find("409 Conflict"),std::string::npos);
  EXPECT_NE(post("READY","18446744073709551616").find("409 Conflict"),std::string::npos);
  EXPECT_NE(post("READY","0","producer-new","Content-Length: 1\r\n").find("400 Bad Request"),std::string::npos);
  EXPECT_NE(post("READY","0","producer-new","X-Hope-Producer: producer-new\r\n").find("409 Conflict"),std::string::npos);
  EXPECT_NE(post("MOTION","0").find("404 Not Found"),std::string::npos);
  EXPECT_TRUE(server.take_intents().empty());
  EXPECT_NE(post("READY","0").find("202 Accepted"),std::string::npos);
  EXPECT_NE(post("STOP","0").find("409 Conflict"),std::string::npos);
  EXPECT_NE(post("STOP","1").find("202 Accepted"),std::string::npos);
  const auto intents=server.take_intents();ASSERT_EQ(intents.size(),2U);
  EXPECT_EQ(intents[0].action,"READY");EXPECT_EQ(intents[0].revision,1U);
  EXPECT_EQ(intents[1].action,"STOP");EXPECT_EQ(intents[1].revision,2U);
  // Draining never permits a delayed READY to overwrite a newer STOP.
  EXPECT_NE(post("READY","0").find("409 Conflict"),std::string::npos);
  for(int n=2;n<10;++n)EXPECT_NE(post("SERVE",std::to_string(n)).find("202 Accepted"),std::string::npos);
  EXPECT_NE(post("READY","10").find("429 Too Many Requests"),std::string::npos);
  EXPECT_EQ(server.take_intents().size(),8U);
  std::this_thread::sleep_for(std::chrono::milliseconds(1050));
  EXPECT_NE(post("READY","10").find("503 Service Unavailable"),std::string::npos);
  EXPECT_TRUE(server.take_intents().empty());
}

TEST(LocalPlayStateStatus, RejectsForeignOriginsBeforeMutationAndNeverExposesDesktopKey) {
  const std::string key(64,'a');LocalPlayStateStatus server("127.0.0.1",0,"producer",key);
  server.update("{\"producer_instance_id\":\"producer\",\"intent_revision\":\"0\"}");
  const std::string identity="X-Hope-Producer: producer\r\nX-Hope-Intent-Revision: 0\r\n";
  for(const std::string origin:{"https://evil.example","https://app.foxglove.dev.evil.example",
      "http://app.foxglove.dev","https://app.foxglove.dev:444","https://app.foxglove.dev/",
      "https://app.foxglove.dev null","null",""}) {
    const auto headers=origin.empty()?"":"Origin: "+origin+"\r\n";
    for(const auto* action:{"READY","SERVE","STOP"}) {
      auto result=local_status_get(server.port(),"POST",std::string("/ball-state/intent/")+action,headers+identity);
      EXPECT_NE(result.find("403 Forbidden"),std::string::npos)<<origin;
      EXPECT_EQ(result.find("Access-Control-Allow-Origin:"),std::string::npos);
    }
    EXPECT_TRUE(server.take_intents().empty());
    auto get=local_status_get(server.port(),"GET","/ball-state",headers);
    EXPECT_EQ(get.find(key),std::string::npos);
    if(!origin.empty())EXPECT_NE(get.find("403 Forbidden"),std::string::npos);
  }
  for(const auto* method:{"GET","POST","OPTIONS"}) {
    auto r=local_status_get(server.port(),method,"/ball-state/intent/STOP",
        "Origin: https://evil.example\r\n"+identity+"X-Hope-Local-Key: "+key+"\r\n");
    EXPECT_NE(r.find("403 Forbidden"),std::string::npos); // Key never grants a foreign web origin access.
  }
  EXPECT_NE(local_status_get(server.port(),"POST","/ball-state/intent/STOP",
      "Origin: https://app.foxglove.dev\r\nOrigin: https://app.foxglove.dev\r\n"+identity).find("403 Forbidden"),std::string::npos);
  auto preflight=local_status_get(server.port(),"OPTIONS","/ball-state/intent/STOP",
      "Origin: null\r\nAccess-Control-Request-Method: POST\r\nAccess-Control-Request-Headers: x-hope-local-key\r\n");
  EXPECT_NE(preflight.find("204 No Content"),std::string::npos);
  EXPECT_TRUE(server.take_intents().empty()); // An opaque preflight cannot authenticate a POST.
  for(const auto bad:{std::string(64,'b'),std::string()})
    EXPECT_NE(local_status_get(server.port(),"POST","/ball-state/intent/STOP",
      "Origin: null\r\n"+identity+"X-Hope-Local-Key: "+bad+"\r\n").find("403 Forbidden"),std::string::npos);
  auto accepted=local_status_get(server.port(),"POST","/ball-state/intent/STOP",
      "Origin: null\r\n"+identity+"X-Hope-Local-Key: "+key+"\r\n");
  EXPECT_NE(accepted.find("202 Accepted"),std::string::npos);
  EXPECT_NE(accepted.find("Access-Control-Allow-Origin: null\r\n"),std::string::npos);
  EXPECT_NE(accepted.find("Vary: Origin"),std::string::npos);
  EXPECT_EQ(accepted.find(key),std::string::npos);
  auto intents=server.take_intents();ASSERT_EQ(intents.size(),1U);EXPECT_EQ(intents[0].action,"STOP");
  auto web=local_status_get(server.port(),"GET","/ball-state","Origin: https://app.foxglove.dev\r\n");
  EXPECT_NE(web.find("200 OK"),std::string::npos);
  EXPECT_NE(web.find("Access-Control-Allow-Origin: https://app.foxglove.dev\r\n"),std::string::npos);
  EXPECT_EQ(web.find("Access-Control-Allow-Origin: *"),std::string::npos);
}

TEST(LocalPlayStateStatus, DesktopKeyPersistsPrivatelyAndRejectsUnsafeFiles) {
  char directory[]="/tmp/hope-control-key-XXXXXX";ASSERT_NE(::mkdtemp(directory),nullptr);
  const auto path=std::string(directory)+"/key";
  const auto key=load_local_control_key(path);EXPECT_EQ(key.size(),64U);
  EXPECT_EQ(load_local_control_key(path),key);
  struct stat st{};ASSERT_EQ(::stat(path.c_str(),&st),0);EXPECT_EQ(st.st_mode&0777,0600);
  ::chmod(path.c_str(),0644);EXPECT_THROW(load_local_control_key(path),std::runtime_error);
  ::chmod(path.c_str(),0600);
  const auto link=std::string(directory)+"/link";ASSERT_EQ(::symlink(path.c_str(),link.c_str()),0);
  EXPECT_THROW(load_local_control_key(link),std::runtime_error);
  EXPECT_THROW(load_local_control_key("relative/key"),std::runtime_error);
  std::filesystem::remove_all(directory);
}

}
