#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
namespace hope_planner_cpp {
inline std::string local_status_get(int port, const std::string& method = "GET",
    const std::string& path = "/ball-state", const std::string& headers = "") {
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{}; address.sin_family=AF_INET;
  address.sin_port=htons(port); address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  timeval timeout{1,0}; ::setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
  if (::connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address)) != 0) {
    ::close(fd); return {};
  }
  const auto request=method+" "+path+" HTTP/1.1\r\nHost: localhost\r\n"+headers+"\r\n";
  ::send(fd,request.data(),request.size(),0);
  std::string response; char bytes[4096]; ssize_t n;
  while ((n=::recv(fd,bytes,sizeof(bytes),0))>0) response.append(bytes,n);
  ::close(fd);return response;
}
inline std::string local_status_field(const std::string& text,const std::string& key) {
  const auto prefix="\""+key+"\":\"";auto start=text.find(prefix);
  if(start==std::string::npos)return {};
  start+=prefix.size();
  return text.substr(start,text.find('"',start)-start);
}
inline std::string local_intent(int port,const std::string& action) {
  const auto status=local_status_get(port);
  return local_status_get(port,"POST","/ball-state/intent/"+action,
      "Origin: https://app.foxglove.dev\r\nX-Hope-Producer: "+local_status_field(status,"producer_instance_id")+"\r\n"
      "X-Hope-Intent-Revision: "+local_status_field(status,"intent_revision")+"\r\n");
}
}
