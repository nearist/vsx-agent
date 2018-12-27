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
* File description: Node configuration DTO.
*/
#ifndef TRUNK_SW_INC_NODECONFIG_H_
#define TRUNK_SW_INC_NODECONFIG_H_

#include <string>
#include "iFlexTypes.h"

namespace IFlex {
class NodeConfig {
  NodeType type;

  DeviceRevision revision;

  std::string device_file_write;
  std::string device_file_read;

  uint8_t component_size;

  std::string address;
  uint16_t port;
  std::string api_key;

 public:
  NodeConfig() : type(NodeType::DEVICE), revision(DeviceRevision::SIMULATOR) {
    device_file_write = "";
    device_file_read = "";
    address = "";
    port = 0;

    component_size = 1;
  }

  NodeType getType() const {
    return type;
  }

  void setType(NodeType type) {
    NodeConfig::type = type;
  }

  DeviceRevision getRevision() const {
    return revision;
  }

  void setRevision(DeviceRevision revision) {
    NodeConfig::revision = revision;
  }

  const std::string &getDeviceFileWrite() const {
    return device_file_write;
  }

  void setDeviceFileWrite(const std::string &device_file_write) {
    NodeConfig::device_file_write = device_file_write;
  }

  const std::string &getDeviceFileRead() const {
    return device_file_read;
  }

  void setDeviceFileRead(const std::string &device_file_read) {
    NodeConfig::device_file_read = device_file_read;
  }

  const std::string &getAddress() const {
    return address;
  }

  void setAddress(const std::string &address) {
    NodeConfig::address = address;
  }

  uint16_t getPort() const {
    return port;
  }

  void setPort(uint16_t port) {
    NodeConfig::port = port;
  }

  const std::string &getApiKey() const {
    return api_key;
  }

  void setApiKey(const std::string &api_key) {
    NodeConfig::api_key = api_key;
  }

  uint8_t getComponentSize() const {
    return component_size;
  }

  void setComponentSize(uint8_t component_size) {
    NodeConfig::component_size = component_size;
  }
};
}  // namespace IFlex

#endif  // TRUNK_SW_INC_NODECONFIG_H_
