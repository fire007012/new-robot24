#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "canopen_hw/dcf_path_utils.hpp"

namespace {

std::filesystem::path MakeTempDir() {
  const auto stamp =
      std::to_string(std::filesystem::file_time_type::clock::now()
                         .time_since_epoch()
                         .count());
  const auto dir = std::filesystem::temp_directory_path() /
                   ("canopen_dcf_path_utils_" + stamp);
  std::filesystem::create_directories(dir);
  return dir;
}

void WriteFile(const std::filesystem::path& path, const std::string& text) {
  std::ofstream ofs(path, std::ios::trunc);
  ASSERT_TRUE(ofs.is_open());
  ofs << text;
  ASSERT_TRUE(ofs.good());
}

}  // namespace

TEST(DcfPathUtils, ResolvesRelativeUploadFilesAgainstDcfDirectory) {
  const auto dir = MakeTempDir();
  const auto nested_dir = dir / "subdir";
  std::filesystem::create_directories(nested_dir);

  const auto relative_bin = nested_dir / "joint_1.bin";
  const auto sibling_bin = dir / "joint_2.bin";
  WriteFile(relative_bin, "bin1");
  WriteFile(sibling_bin, "bin2");

  const auto master_dcf = nested_dir / "master.dcf";
  WriteFile(master_dcf,
            "[1F22sub1]\n"
            "UploadFile=./joint_1.bin\n"
            "[1F22sub2]\n"
            "UploadFile=../joint_2.bin\n");

  std::string rewritten;
  std::vector<canopen_hw::DcfUploadFileResolution> upload_files;
  std::string error;
  ASSERT_TRUE(canopen_hw::RewriteMasterDcfUploadPaths(
      master_dcf.string(), &rewritten, &upload_files, &error));
  EXPECT_TRUE(error.empty());

  ASSERT_EQ(upload_files.size(), 2u);
  EXPECT_EQ(upload_files[0].original_path, "./joint_1.bin");
  EXPECT_EQ(upload_files[0].resolved_path, relative_bin.lexically_normal().string());
  EXPECT_TRUE(upload_files[0].exists);
  EXPECT_EQ(upload_files[1].original_path, "../joint_2.bin");
  EXPECT_EQ(upload_files[1].resolved_path, sibling_bin.lexically_normal().string());
  EXPECT_TRUE(upload_files[1].exists);

  EXPECT_NE(rewritten.find("UploadFile=" + relative_bin.lexically_normal().string()),
            std::string::npos);
  EXPECT_NE(rewritten.find("UploadFile=" + sibling_bin.lexically_normal().string()),
            std::string::npos);

  std::filesystem::remove_all(dir);
}

TEST(DcfPathUtils, ReportsMissingSourceDcf) {
  std::string rewritten;
  std::vector<canopen_hw::DcfUploadFileResolution> upload_files;
  std::string error;

  EXPECT_FALSE(canopen_hw::RewriteMasterDcfUploadPaths(
      "/tmp/definitely_missing_master_for_dcf_path_utils_test.dcf", &rewritten,
      &upload_files, &error));
  EXPECT_TRUE(rewritten.empty());
  EXPECT_TRUE(upload_files.empty());
  EXPECT_FALSE(error.empty());
}
