#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

#include "canopen_hw/boot_identity_diag.hpp"

namespace {

TEST(BootIdentityDiag, LoadExpectedIdentityReadsNodeSpecificEntries) {
  const std::string path = "/tmp/test_boot_identity_node_specific.dcf";
  {
    std::ofstream ofs(path);
    ofs << "[1F84Value]\n"
           "NrOfEntries=2\n"
           "5=0x00220811\n"
           "6=0x12345678\n"
           "[1F85Value]\n"
           "NrOfEntries=2\n"
           "5=0x00000668\n"
           "6=0x00000001\n"
           "[1F86Value]\n"
           "NrOfEntries=2\n"
           "5=0x00221701\n"
           "6=0x00000002\n"
           "[1F87Value]\n"
           "NrOfEntries=2\n"
           "5=0x00000010\n"
           "6=0x00000020\n";
  }

  canopen_hw::BootIdentityTuple expected;
  std::string error;
  ASSERT_TRUE(
      canopen_hw::LoadExpectedBootIdentityFromDcf(path, 5, &expected, &error))
      << error;

  EXPECT_TRUE(expected.has_device_type);
  EXPECT_EQ(expected.device_type, 0x00220811u);
  EXPECT_TRUE(expected.has_vendor_id);
  EXPECT_EQ(expected.vendor_id, 0x00000668u);
  EXPECT_TRUE(expected.has_product_code);
  EXPECT_EQ(expected.product_code, 0x00221701u);
  EXPECT_TRUE(expected.has_revision);
  EXPECT_EQ(expected.revision, 0x00000010u);
}

TEST(BootIdentityDiag, LoadExpectedIdentityFailsWhenNodeMissing) {
  const std::string path = "/tmp/test_boot_identity_missing_node.dcf";
  {
    std::ofstream ofs(path);
    ofs << "[1F84Value]\n"
           "NrOfEntries=1\n"
           "6=0x00220811\n";
  }

  canopen_hw::BootIdentityTuple expected;
  std::string error;
  EXPECT_FALSE(
      canopen_hw::LoadExpectedBootIdentityFromDcf(path, 5, &expected, &error));
  EXPECT_NE(error.find("no identity entry"), std::string::npos);
}

TEST(BootIdentityDiag, DiffBootIdentityReportsOnlyComparableMismatches) {
  canopen_hw::BootIdentityTuple expected;
  expected.has_device_type = true;
  expected.device_type = 0x00220811;
  expected.has_vendor_id = true;
  expected.vendor_id = 0x00000668;
  expected.has_product_code = true;
  expected.product_code = 0x00221701;

  canopen_hw::BootIdentityTuple actual;
  actual.has_device_type = true;
  actual.device_type = 0x00220811;  // same
  actual.has_vendor_id = false;     // missing: should not count as mismatch
  actual.has_product_code = true;
  actual.product_code = 0x00221799;  // mismatch

  const std::vector<std::string> diffs =
      canopen_hw::DiffBootIdentity(expected, actual);

  ASSERT_EQ(diffs.size(), 1u);
  EXPECT_EQ(diffs[0], "1018:02(product_code)");
}

}  // namespace
