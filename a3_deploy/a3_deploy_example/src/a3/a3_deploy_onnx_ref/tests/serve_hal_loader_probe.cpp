#include "a3_io/serve_hal_clock.hpp"
#include <dlfcn.h>
#include <link.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
static constexpr const char* names[] = {
 "_ZN9aimrt_hal12elink_module9UpperNode14WriteHandOuputEv",
 "_ZN9aimrt_hal12elink_module9UpperNode19update_hand_cmd_tlvERNS0_11HandCmdDataE"};
int main(int argc,char** argv) {
 if(argc!=2)return 1;
 void* h=dlopen(argv[1],RTLD_NOW|RTLD_DEEPBIND);
 if(!h){std::fprintf(stderr,"dlopen: %s\n",dlerror());return 2;}
 link_map* m=nullptr; if(dlinfo(h,RTLD_DI_LINKMAP,&m))return 3;
 const ElfW(Sym)* syms=nullptr; const char* strings=nullptr;
 const ElfW(Rela)* rela=nullptr; size_t bytes=0;
 for(auto* d=m->l_ld;d->d_tag!=DT_NULL;++d) switch(d->d_tag){
 case DT_SYMTAB:syms=(const ElfW(Sym)*)d->d_un.d_ptr;break;
 case DT_STRTAB:strings=(const char*)d->d_un.d_ptr;break;
 case DT_JMPREL:rela=(const ElfW(Rela)*)d->d_un.d_ptr;break;
 case DT_PLTRELSZ:bytes=d->d_un.d_val;break; }
 void* f[2]{}; int observer_slots=0;
 for(size_t i=0;i<bytes/sizeof(*rela);++i){
  void* target=*(void**)(m->l_addr+rela[i].r_offset);
  Dl_info info{}; if(dladdr(target,&info) && info.dli_fname && std::strstr(info.dli_fname,"libserve_hal_observer.so"))++observer_slots;
  for(int j=0;j<2;++j)if(!std::strcmp(strings+syms[ELF64_R_SYM(rela[i].r_info)].st_name,names[j])) {
   f[j]=target;
   // Assert the relocation page is read-only again after binding.
   FILE* maps=std::fopen("/proc/self/maps","r");char line[512],perms[8];unsigned long begin,end;
   while(std::fgets(line,sizeof(line),maps))if(std::sscanf(line,"%lx-%lx %7s",&begin,&end,perms)==3){
    auto addr=m->l_addr+rela[i].r_offset;
    if(begin<=addr && addr<end && perms[1]=='w')return 7;
   }
   std::fclose(maps);
  }
 }
 if(!f[0]||!f[1])return 4;
 alignas(64) unsigned char node[512]{}; // Actual vendor output-enable byte 424 remains zero.
 reinterpret_cast<void(*)(void*)>(f[0])(node);
 const char* path=std::getenv("HOPE_SERVE_HAL_CLOCK");
 if(path){
  a3_io::ServeHalClockReader r;a3_io::ServeHalSnapshot s;
  if(!r.Open(path)||!r.Read(s)||!s.Ready(s.heartbeat_ns)||s.selected_ns||s.release_edge_ns)return 10;
  if(observer_slots!=2)return 11;
  std::fprintf(stderr,"DEEPBIND_COLD_BOOT_PASS observer_slots=%d RELRO_READ_ONLY\n",observer_slots);
 }
 using Serialize=std::vector<unsigned char>(*)(void*,void*);
 std::uint64_t edge=0;int index=0;
 for(std::uint16_t pos:{2000,1000,2000,2000,1000,2000}){
  unsigned char cmd[80]{};std::memcpy(cmd,&pos,2);
  auto bytes=reinterpret_cast<Serialize>(f[1])(node,cmd);
  for(auto byte:bytes)std::printf("%02x",byte);std::puts("");
  if(path){
   a3_io::ServeHalClockReader r;a3_io::ServeHalSnapshot s;
   if(!r.Open(path)||!r.Read(s)||s.left!=pos)return 12;
   if(index<=1&&s.release_edge_ns!=0)return 13;
   if((index==2||index==5)&&s.release_edge_ns<=edge)return 14;
   if((index==3||index==4)&&s.release_edge_ns!=edge)return 15;
   edge=s.release_edge_ns;
  }++index;
 }
 std::fprintf(stderr,"DEEPBIND_SERIALIZER_AND_EDGE_PASS\n");
}
