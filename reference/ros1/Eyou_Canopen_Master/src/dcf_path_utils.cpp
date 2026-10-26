#include "canopen_hw/dcf_path_utils.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace canopen_hw {
namespace {

bool TryParseUploadFileLine(const std::string& line, std::size_t* value_offset) {
  if (!value_offset) {
    return false;
  }

  std::size_t i = 0;
  while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) {
    ++i;
  }

  constexpr char kPrefix[] = "UploadFile=";
  const std::string prefix(kPrefix);
  if (line.compare(i, prefix.size(), prefix) != 0) {
    return false;
  }

  *value_offset = i + prefix.size();
  return true;
}

}  // namespace

bool RewriteMasterDcfUploadPaths(
    const std::string& source_dcf_path, std::string* rewritten_text,
    std::vector<DcfUploadFileResolution>* upload_files, std::string* error) {
  if (rewritten_text) {
    rewritten_text->clear();
  }
  if (upload_files) {
    upload_files->clear();
  }
  if (error) {
    error->clear();
  }

  if (source_dcf_path.empty()) {
    if (error) {
      *error = "source dcf path is empty";
    }
    return false;
  }

  std::ifstream ifs(source_dcf_path);
  if (!ifs.is_open()) {
    if (error) {
      *error = "open dcf failed: " + source_dcf_path;
    }
    return false;
  }

  const auto base_dir =
      std::filesystem::absolute(std::filesystem::path(source_dcf_path))
          .parent_path();

  std::ostringstream oss;
  std::string line;
  bool first_line = true;
  while (std::getline(ifs, line)) {
    if (!first_line) {
      oss << '\n';
    }
    first_line = false;

    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    std::size_t value_offset = 0;
    if (TryParseUploadFileLine(line, &value_offset)) {
      const auto original = line.substr(value_offset);
      std::filesystem::path resolved =
          std::filesystem::path(original).is_absolute()
              ? std::filesystem::path(original)
              : (base_dir / std::filesystem::path(original)).lexically_normal();

      if (upload_files) {
        upload_files->push_back(
            {original, resolved.string(), std::filesystem::exists(resolved)});
      }

      line.replace(value_offset, std::string::npos, resolved.string());
    }

    oss << line;
  }

  if (ifs.bad()) {
    if (error) {
      *error = "read dcf failed: " + source_dcf_path;
    }
    return false;
  }

  if (rewritten_text) {
    *rewritten_text = oss.str();
  }
  return true;
}

}  // namespace canopen_hw
