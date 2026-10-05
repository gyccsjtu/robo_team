// Read-only native Gazebo contact observer. No world or controller changes.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t stopped = 0;
std::mutex mutex;
std::set<std::string> virtualNames;
std::set<std::string> models;
std::map<std::pair<std::string,std::string>, double> emitted;
unsigned long frames=0, virtualPairs=0, bodyPairs=0;
double first=-1., latest=-1., maximumGap=0.;
void stop(int) { stopped=1; }
std::string quoted(const std::string &value) {
  std::string result="\"";
  for (char c:value) {
    if (c=='"' || c=='\\') result+='\\';
    if (c=='\n') result+="\\n"; else result+=c;
  }
  return result+'"';
}
bool aircraft(const std::string &name) {
  for(const auto &model:models) {
    const auto prefix=model+"::";
    if(name.compare(0,prefix.size(),prefix)==0) return true;
  }
  return false;
}
void contacts(ConstContactsPtr &message) {
  std::lock_guard<std::mutex> lock(mutex);
  const double sample=message->time().sec()+message->time().nsec()*1e-9;
  if(first<0) first=sample;
  if(latest>=0 && sample>latest) maximumGap=std::max(maximumGap,sample-latest);
  latest=sample; ++frames;
  for(int i=0;i<message->contact_size();++i) {
    const auto &contact=message->contact(i);
    auto a=contact.collision1(), b=contact.collision2();
    if(virtualNames.count(a) || virtualNames.count(b)) { ++virtualPairs; continue; }
    if(!aircraft(a) && !aircraft(b)) continue;
    ++bodyPairs;
    if(b<a) std::swap(a,b);
    const auto key=std::make_pair(a,b);
    const auto previous=emitted.find(key);
    if(previous!=emitted.end() && sample-previous->second<.5) continue;
    emitted[key]=sample;
    std::cout << "{\"kind\":\"UAV_BODY_CONTACT\",\"sample_s\":" << sample
      << ",\"collision1\":" << quoted(a) << ",\"collision2\":" << quoted(b)
      << "}" << std::endl;
  }
}
void summary(const char *kind) {
  std::lock_guard<std::mutex> lock(mutex);
  std::cout << "{\"kind\":" << quoted(kind) << ",\"frames\":" << frames
    << ",\"first_sample_s\":" << first << ",\"last_sample_s\":" << latest
    << ",\"maximum_sample_gap_s\":" << maximumGap
    << ",\"sonar_virtual_pairs\":" << virtualPairs
    << ",\"body_pairs\":" << bodyPairs << "}" << std::endl;
}
}
int main(int argc,char **argv) {
  if(argc==2 && std::string(argv[1])=="--self-test") {
    models.insert("typhoon_h480_0");
    virtualNames.insert("typhoon_h480_0::sonar::link::default::typhoon_h480_0::sonar::link::sonarsensor_collision");
    auto *raw=new gazebo::msgs::Contacts();
    raw->mutable_time()->set_sec(5); raw->mutable_time()->set_nsec(0);
    auto *body=raw->add_contact();
    body->set_collision1("typhoon_h480_0::base_link::collision");
    body->set_collision2("house_1::link::collision");
    auto *sensor=raw->add_contact();
    sensor->set_collision1(*virtualNames.begin()); sensor->set_collision2("house_1::link::collision");
    auto *unrelated=raw->add_contact();
    unrelated->set_collision1("actor_0::leg::collision"); unrelated->set_collision2("ground::link::collision");
    ConstContactsPtr message(raw); contacts(message);
    return bodyPairs==1 && virtualPairs==1 && frames==1 && emitted.size()==1
      && !aircraft("typhoon_h480_01::base_link::collision") ? 0 : 1;
  }
  if(argc!=2) { std::cerr << "Expected verified sonar whitelist path\n"; return 2; }
  std::ifstream input(argv[1]); std::string line;
  while(std::getline(input,line)) if(!line.empty()) {
    virtualNames.insert(line); models.insert(line.substr(0,line.find("::")));
  }
  if(virtualNames.size()!=6 || models.size()!=6) { std::cerr << "Six verified sonar names required\n"; return 2; }
  std::cout << std::setprecision(12);
  std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
  gazebo::client::setup();
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init("default");
  auto subscriber=node->Subscribe("~/physics/contacts", contacts);
  while(!stopped) { summary("CONTACT_OBSERVER_HEARTBEAT"); std::this_thread::sleep_for(std::chrono::seconds(1)); }
  subscriber.reset(); summary("CONTACT_OBSERVER_STOPPED");
  gazebo::client::shutdown(); return 0;
}
