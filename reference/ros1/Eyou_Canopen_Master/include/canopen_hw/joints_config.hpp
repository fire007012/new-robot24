#pragma once

#include <string>

#include "canopen_hw/canopen_master.hpp"

namespace canopen_hw {

// 读取 joints.yaml 并将配置填入 CanopenMasterConfig。
// 返回 true 表示加载成功; 若失败可从 error 获取原因。
bool LoadJointsYaml(const std::string& path,
                    std::string* error,
                    CanopenMasterConfig* config);

}  // namespace canopen_hw
