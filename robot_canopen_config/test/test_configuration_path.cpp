#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include "canopen_core/configuration_manager.hpp"
TEST(ConfigurationPath, FollowsBusYamlWhenInstalledDirectoryMoves)
{
 char pattern[]="/tmp/canopen-path-XXXXXX";auto created=mkdtemp(pattern);ASSERT_NE(created,nullptr);
 const auto root=std::filesystem::path(created);
 struct Clean {std::filesystem::path root;~Clean(){std::filesystem::remove_all(root);}} clean{root};
 for(auto folder:{"install_a","install_b"}) {
  auto directory=root/folder;std::filesystem::create_directory(directory);
  auto file=directory/"bus.yml";
  std::ofstream(file)<<"options:\n  dcf_path: .\naxis:\n  node_id: 1\n";
  auto name=file.string();ros2_canopen::ConfigurationManager config(name);config.init_config();
  EXPECT_EQ(config.get_entry<std::string>("axis","dcf_path").value(),directory.string());
 }
}
