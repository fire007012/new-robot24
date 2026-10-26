#pragma once

#include <string>
#include <vector>

namespace canopen_hw {

struct DcfUploadFileResolution {
  std::string original_path;
  std::string resolved_path;
  bool exists = false;
};

bool RewriteMasterDcfUploadPaths(
    const std::string& source_dcf_path, std::string* rewritten_text,
    std::vector<DcfUploadFileResolution>* upload_files, std::string* error);

}  // namespace canopen_hw
