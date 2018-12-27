/*
* Copyright (c) 2015-2018 in2H2 inc.
* System developed for in2H2 inc. by Intermotion Technology, Inc.
*
* Full system RTL, C sources and board design files available at https://github.com/nearist
*
* in2H2 inc. Team Members:
* - Chris McCormick - Algorithm Research and Design
* - Matt McCormick - Board Production, System Q/A
*
* Intermotion Technology Inc. Team Members:
* - Mick Fandrich - Project Lead
* - Dr. Ludovico Minati - Board Architecture and Design, FPGA Technology Advisor
* - Vardan Movsisyan - RTL Team Lead
* - Khachatur Gyozalyan - RTL Design
* - Tigran Papazyan - RTL Design
* - Taron Harutyunyan - RTL Design
* - Hayk Ghaltaghchyan - System Software
*
* Tecno77 S.r.l. Team Members:
* - Stefano Aldrigo, Board Layout Design
*
* We dedicate this project to the memory of Bruce McCormick, an AI pioneer
* and advocate, a good friend and father.
*
* These materials are provided free of charge: you can redistribute them and/or modify
* them under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* These materials are distributed in the hope that they will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
/*
* File description: Application configuration DTO.
*/
#ifndef TRUNK_SW_INC_APPCONFIG_H_
#define TRUNK_SW_INC_APPCONFIG_H_

#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include "Logger.h"
#include "NodeConfig.h"

namespace IFlex {

class AppConfig {
  std::string address;
  uint16_t port;

  std::string log_file;
  spdlog::level::level_enum log_level;

  std::string api_key;

  std::vector<NodeConfig> nodes;

  uint8_t component_size;

  std::string dataset_nearist;
  std::string dataset_user;

  bool temperature_log_enable;
  uint8_t temperature_log_rate;
  uint8_t temperature_log_threshold;
  uint8_t temperature_log_duration;
  std::string temperature_log_file;

 public:
  const std::string &getAddress() const {
    return address;
  }

  void setAddress(const std::string &address) {
    AppConfig::address = address;
  }

  uint16_t getPort() const {
    return port;
  }

  void setPort(uint16_t port) {
    AppConfig::port = port;
  }

  const std::string &getLogFile() const {
    return log_file;
  }

  void setLogFile(const std::string &log_file) {
    AppConfig::log_file = log_file;
  }

  spdlog::level::level_enum getLogLevel() const {
    return log_level;
  }

  void setLogLevel(spdlog::level::level_enum log_level) {
    AppConfig::log_level = log_level;
  }

  const std::string &getApiKey() const {
    return api_key;
  }

  void setApiKey(const std::string &api_key) {
    AppConfig::api_key = api_key;
  }

  const std::string &getDatasetNearist() const {
    return dataset_nearist;
  }

  void setDatasetNearist(const std::string &dataset_nearist) {
    AppConfig::dataset_nearist = dataset_nearist;
  }

  const std::string &getDatasetUser() const {
    return dataset_user;
  }

  void setDatasetUser(const std::string &dataset_user) {
    AppConfig::dataset_user = dataset_user;
  }

  const std::vector<NodeConfig> &getNodes() const {
    return nodes;
  }

  void setNodes(const std::vector<NodeConfig> &nodes) {
    AppConfig::nodes = nodes;
  }

  bool isTemperatureLogEnable() const {
    return temperature_log_enable;
  }

  void setTemperatureLogEnable(bool temperature_log_enable) {
    AppConfig::temperature_log_enable = temperature_log_enable;
  }

  uint8_t getTemperatureLogRate() const {
    return temperature_log_rate;
  }

  void setTemperatureLogRate(uint8_t temperature_log_rate) {
    AppConfig::temperature_log_rate = temperature_log_rate;
  }

  uint8_t getTemperatureLogThreshold() const {
    return temperature_log_threshold;
  }

  void setTemperatureLogThreshold(uint8_t temperature_log_threshold) {
    AppConfig::temperature_log_threshold = temperature_log_threshold;
  }

  uint8_t getTemperatureLogDuration() const {
    return temperature_log_duration;
  }

  void setTemperatureLogDuration(uint8_t temperature_log_duration) {
    AppConfig::temperature_log_duration = temperature_log_duration;
  }

  const std::string &getTemperatureLogFile() const {
    return temperature_log_file;
  }

  void setTemperatureLogFile(const std::string &temperature_log_file) {
    AppConfig::temperature_log_file = temperature_log_file;
  }

  uint8_t getComponentSize() const {
    return component_size;
  }

  void setComponentSize(uint8_t component_size) {
    AppConfig::component_size = component_size;
  }

  void load(const std::string &filename) {
    boost::property_tree::ptree tree;
    boost::property_tree::read_json(filename, tree);

    address = tree.get<std::string>("address", "127.0.0.1");
    port = tree.get<uint16_t>("port", 9857);

    log_file = tree.get<std::string>("log_file", "nearist.log");

    auto level = tree.get<std::string>("log_level", "DEBUG");
    std::transform(level.begin(), level.end(), level.begin(), ::tolower);
    log_level = spdlog::level::from_str(level);

    api_key = tree.get<std::string>("api_key", "");

    dataset_nearist = tree.get<std::string>("dataset_path.nearist", "");
    dataset_user = tree.get<std::string>("dataset_path.user", "");

    component_size = tree.get<uint8_t>("component_size", 1);

    temperature_log_enable = tree.get<bool>("temperature_log.enable", false);
    temperature_log_file = tree.get<std::string>("temperature_log.file", "");
    temperature_log_rate = tree.get<uint8_t>("temperature_log.rate", 1);
    temperature_log_threshold =
        tree.get<uint8_t>("temperature_log.threshold", 255);
    temperature_log_duration =
        tree.get<uint8_t>("temperature_log.duration", 255);

    for (boost::property_tree::ptree::value_type
          &child : tree.get_child("nodes")) {
      NodeConfig nc;

      nc.setType((NodeType) child.second.get<uint8_t>("type"));

      switch (nc.getType()) {
        case NodeType::DEVICE: {
          nc.setRevision(
              (DeviceRevision) child.second.get<uint8_t>("revision"));
          nc.setDeviceFileWrite(child.second.get<std::string>("write"));
          nc.setDeviceFileRead(child.second.get<std::string>("read"));
        }
          break;
        case NodeType::TCP: {
          nc.setAddress(child.second.get<std::string>("address"));
          nc.setPort(child.second.get<uint16_t>("port"));
          nc.setApiKey(child.second.get<std::string>("api_key"));
        }
          break;
        default:
          break;
      }

      nodes.push_back(nc);
    }
  }

  void save(const std::string &filename) {
    boost::property_tree::ptree tree;

    tree.put("address", address);
    tree.put("port", port);
    tree.put("log_file", log_file);
    tree.put("log_level", spdlog::level::to_c_str(log_level));

    boost::property_tree::ptree children;
    for (NodeConfig &node : nodes) {
      boost::property_tree::ptree node_config;

      node_config.put("type", node.getType());

      switch (node.getType()) {
        case NodeType::DEVICE:
          node_config.put("write", node.getDeviceFileWrite());
          node_config.put("read", node.getDeviceFileRead());
          break;
        case NodeType::TCP:
          node_config.put("address", node.getAddress());
          node_config.put("port", node.getPort());
          break;
        default:
          break;
      }

      children.push_back(std::make_pair("", node_config));
    }
    tree.add_child("nodes", children);

    boost::property_tree::write_json(filename, tree);
  }
};
}  // namespace IFlex

#endif  // TRUNK_SW_INC_APPCONFIG_H_
