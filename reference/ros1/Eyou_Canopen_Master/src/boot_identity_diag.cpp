#include "canopen_hw/boot_identity_diag.hpp"

#include <fstream>
#include <limits>
#include <sstream>

namespace canopen_hw {
namespace {

std::string TrimCopy(const std::string& input) {
  const std::size_t begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return std::string();
  }
  const std::size_t end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

bool ParseUint32(const std::string& input, uint32_t* out) {
  if (!out) {
    return false;
  }
  try {
    std::size_t idx = 0;
    const unsigned long long parsed = std::stoull(input, &idx, 0);
    if (idx != input.size() || parsed > std::numeric_limits<uint32_t>::max()) {
      return false;
    }
    *out = static_cast<uint32_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseNodeValueEntry(const std::string& line, uint8_t node_id,
                         uint32_t* out_value) {
  if (!out_value) {
    return false;
  }

  std::string clean = line;
  const std::size_t semicolon = clean.find(';');
  if (semicolon != std::string::npos) {
    clean.resize(semicolon);
  }
  const std::size_t hash = clean.find('#');
  if (hash != std::string::npos) {
    clean.resize(hash);
  }
  clean = TrimCopy(clean);
  if (clean.empty()) {
    return false;
  }

  const std::size_t eq = clean.find('=');
  if (eq == std::string::npos) {
    return false;
  }

  const std::string key = TrimCopy(clean.substr(0, eq));
  const std::string value = TrimCopy(clean.substr(eq + 1));
  if (key.empty() || value.empty()) {
    return false;
  }

  uint32_t key_num = 0;
  if (!ParseUint32(key, &key_num) ||
      key_num != static_cast<uint32_t>(node_id)) {
    return false;
  }
  return ParseUint32(value, out_value);
}

}  // namespace

bool LoadExpectedBootIdentityFromDcf(const std::string& dcf_path,
                                     uint8_t node_id,
                                     BootIdentityTuple* out,
                                     std::string* error) {
  if (!out) {
    if (error) {
      *error = "null output identity tuple";
    }
    return false;
  }

  std::ifstream ifs(dcf_path);
  if (!ifs.is_open()) {
    if (error) {
      *error = "open dcf failed: " + dcf_path;
    }
    return false;
  }

  BootIdentityTuple parsed;
  std::string section;
  std::string line;
  while (std::getline(ifs, line)) {
    const std::string trimmed = TrimCopy(line);
    if (trimmed.empty()) {
      continue;
    }
    if (trimmed.front() == '[' && trimmed.back() == ']') {
      section = trimmed.substr(1, trimmed.size() - 2);
      continue;
    }

    uint32_t value = 0;
    if (section == "1F84Value" &&
        ParseNodeValueEntry(trimmed, node_id, &value)) {
      parsed.has_device_type = true;
      parsed.device_type = value;
      continue;
    }
    if (section == "1F85Value" &&
        ParseNodeValueEntry(trimmed, node_id, &value)) {
      parsed.has_vendor_id = true;
      parsed.vendor_id = value;
      continue;
    }
    if (section == "1F86Value" &&
        ParseNodeValueEntry(trimmed, node_id, &value)) {
      parsed.has_product_code = true;
      parsed.product_code = value;
      continue;
    }
    if (section == "1F87Value" &&
        ParseNodeValueEntry(trimmed, node_id, &value)) {
      parsed.has_revision = true;
      parsed.revision = value;
      continue;
    }
  }

  if (!parsed.HasAny()) {
    if (error) {
      std::ostringstream oss;
      oss << "no identity entry for node " << static_cast<int>(node_id)
          << " in [1F84/1F85/1F86/1F87]Value";
      *error = oss.str();
    }
    return false;
  }

  *out = parsed;
  return true;
}

std::vector<std::string> DiffBootIdentity(const BootIdentityTuple& expected,
                                          const BootIdentityTuple& actual) {
  std::vector<std::string> mismatch_fields;
  if (expected.has_device_type && actual.has_device_type &&
      expected.device_type != actual.device_type) {
    mismatch_fields.emplace_back("1000:00(device_type)");
  }
  if (expected.has_vendor_id && actual.has_vendor_id &&
      expected.vendor_id != actual.vendor_id) {
    mismatch_fields.emplace_back("1018:01(vendor_id)");
  }
  if (expected.has_product_code && actual.has_product_code &&
      expected.product_code != actual.product_code) {
    mismatch_fields.emplace_back("1018:02(product_code)");
  }
  if (expected.has_revision && actual.has_revision &&
      expected.revision != actual.revision) {
    mismatch_fields.emplace_back("1018:03(revision)");
  }
  return mismatch_fields;
}

}  // namespace canopen_hw
