//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "parse_livox_lidar_cfg.h"
#include <iostream>

namespace livox_ros {

bool LivoxLidarConfigParser::Parse(std::vector<UserLivoxLidarConfig> &lidar_configs) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    std::cout << "failed to open config file: " << path_ << std::endl;
    return false;
  }

  lidar_configs.clear();
  char read_buffer[kMaxBufferSize];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  rapidjson::Document doc;

  do {
    if (doc.ParseStream(config_file).HasParseError()) {
      std::cout << "failed to parse config jason" << std::endl;
      break;
    }
    if (!doc.HasMember("lidar_configs") ||
        !doc["lidar_configs"].IsArray() ||
        0 == doc["lidar_configs"].Size()) {
      std::cout << "there is no user-defined config" << std::endl;
      break;
    }
    if (!ParseUserConfigs(doc, lidar_configs)) {
      std::cout << "failed to parse basic configs" << std::endl;
      break;
    }
    return true;
  } while (false);

  std::fclose(raw_file);
  return false;
}

bool LivoxLidarConfigParser::ParseUserConfigs(const rapidjson::Document &doc,
                                              std::vector<UserLivoxLidarConfig> &user_configs) {
  const rapidjson::Value &lidar_configs = doc["lidar_configs"];
  for (auto &config : lidar_configs.GetArray()) {
    if (!config.IsObject()) {
      std::cout << "skip an element of lidar_configs, it is not an object" << std::endl;
      continue;
    }
    if (!config.HasMember("ip") || !config["ip"].IsString()) {
      std::cout << "skip a lidar config, \"ip\" is missing or not a string" << std::endl;
      continue;
    }
    UserLivoxLidarConfig user_config;

    // parse user configs
    user_config.handle = IpStringToNum(std::string(config["ip"].GetString()));
    if (!config.HasMember("pcl_data_type") || !config["pcl_data_type"].IsInt()) {
      if (config.HasMember("pcl_data_type")) {
        std::cout << "\"pcl_data_type\" is not an integer, ignored" << std::endl;
      }
      user_config.pcl_data_type = -1;
    } else {
      user_config.pcl_data_type = static_cast<int8_t>(config["pcl_data_type"].GetInt());
    }
    if (!config.HasMember("pattern_mode") || !config["pattern_mode"].IsInt()) {
      if (config.HasMember("pattern_mode")) {
        std::cout << "\"pattern_mode\" is not an integer, ignored" << std::endl;
      }
      user_config.pattern_mode = -1;
    } else {
      user_config.pattern_mode = static_cast<int8_t>(config["pattern_mode"].GetInt());
    }
    if (!config.HasMember("blind_spot_set") || !config["blind_spot_set"].IsInt()) {
      if (config.HasMember("blind_spot_set")) {
        std::cout << "\"blind_spot_set\" is not an integer, ignored" << std::endl;
      }
      user_config.blind_spot_set = -1;
    } else {
      user_config.blind_spot_set = config["blind_spot_set"].GetInt();
    }
    if (!config.HasMember("dual_emit_en") || !config["dual_emit_en"].IsInt()) {
      if (config.HasMember("dual_emit_en")) {
        std::cout << "\"dual_emit_en\" is not an integer, ignored" << std::endl;
      }
      user_config.dual_emit_en = -1;
    } else {
      user_config.dual_emit_en = static_cast<uint8_t>(config["dual_emit_en"].GetInt());
    }
    if (!config.HasMember("frame_id") || !config["frame_id"].IsString()) {
      if (config.HasMember("frame_id")) {
        std::cout << "\"frame_id\" is not a string, ignored" << std::endl;
      }
      user_config.frame_id = "livox_frame";
      std::cout << "No frame id was given, set to default of 'livox_frame'" << std::endl;
    } else {
      user_config.frame_id = static_cast<std::string>(config["frame_id"].GetString());
    }
    if (!config.HasMember("extrinsic_parameter") || !config["extrinsic_parameter"].IsObject()) {
      if (config.HasMember("extrinsic_parameter")) {
        std::cout << "\"extrinsic_parameter\" is not an object, ignored" << std::endl;
      }
      memset(&user_config.extrinsic_param, 0, sizeof(user_config.extrinsic_param));
    } else {
      auto &value = config["extrinsic_parameter"];
      if (!ParseExtrinsics(value, user_config.extrinsic_param)) {
        memset(&user_config.extrinsic_param, 0, sizeof(user_config.extrinsic_param));
        std::cout << "failed to parse extrinsic parameters, ip: "
                  << IpNumToString(user_config.handle) << std::endl;
      }
    }
    user_config.set_bits = 0;
    user_config.get_bits = 0;

    user_configs.push_back(user_config);
  }

  if (0 == user_configs.size()) {
    std::cout << "no valid base configs" << std::endl;
    return false;
  }
  std::cout << "successfully parse base config, counts: "
            << user_configs.size() << std::endl;
  return true;
}

bool LivoxLidarConfigParser::ParseExtrinsics(const rapidjson::Value &value,
                                             ExtParameter &param) {
  if (!value.HasMember("roll")) {
    param.roll = 0.0f;
  } else if (!value["roll"].IsNumber()) {
    return false;
  } else {
    param.roll = static_cast<float>(value["roll"].GetFloat());
  }
  if (!value.HasMember("pitch")) {
    param.pitch = 0.0f;
  } else if (!value["pitch"].IsNumber()) {
    return false;
  } else {
    param.pitch = static_cast<float>(value["pitch"].GetFloat());
  }
  if (!value.HasMember("yaw")) {
    param.yaw = 0.0f;
  } else if (!value["yaw"].IsNumber()) {
    return false;
  } else {
    param.yaw = static_cast<float>(value["yaw"].GetFloat());
  }
  if (!value.HasMember("x")) {
    param.x = 0;
  } else if (!value["x"].IsInt()) {
    return false;
  } else {
    param.x = static_cast<int32_t>(value["x"].GetInt());
  }
  if (!value.HasMember("y")) {
    param.y = 0;
  } else if (!value["y"].IsInt()) {
    return false;
  } else {
    param.y = static_cast<int32_t>(value["y"].GetInt());
  }
  if (!value.HasMember("z")) {
    param.z = 0;
  } else if (!value["z"].IsInt()) {
    return false;
  } else {
    param.z = static_cast<int32_t>(value["z"].GetInt());
  }

  return true;
}

} // namespace livox_ros
