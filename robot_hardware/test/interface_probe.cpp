#include <fstream>
#include <sstream>
#include <iostream>
#include <set>
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"
int main(int argc,char **argv) {
 if(argc!=2)return 2;
 try {
  std::ifstream file(argv[1]);std::stringstream text;text<<file.rdbuf();
  auto infos=hardware_interface::parse_control_resources_from_urdf(text.str());
  if(infos.size()!=2)throw std::runtime_error("expected two hardware components");
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader("hardware_interface","hardware_interface::SystemInterface");
  std::set<std::string> all;
  for(auto &info:infos) {
   auto plugin=loader.createSharedInstance(info.hardware_class_type);
   if(plugin->on_init(info)!=hardware_interface::CallbackReturn::SUCCESS)throw std::runtime_error(info.name+" init failed");
   auto commands=plugin->export_command_interfaces();auto states=plugin->export_state_interfaces();
   std::set<std::string> actual,expected;
   for(auto &c:commands)actual.insert(c.get_name());
   for(auto &j:info.joints) {
    if(!all.insert(j.name).second)throw std::runtime_error("duplicate joint ownership");
    for(auto &c:j.command_interfaces)expected.insert(j.name+"/"+c.name);
   }
   if(actual!=expected)throw std::runtime_error(info.name+" mismatched command interfaces");
   std::cout<<info.name<<": "<<info.joints.size()<<" joints, "<<commands.size()<<" commands, "<<states.size()<<" states\n";
  }
  if(all.size()!=13)throw std::runtime_error("expected 13 unique physical joints");
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
