/*
* Copyright (c) 2015-2018 in2H2 inc.
* System developed for in2H2 inc. by Intermotion Technology, Inc.
*
* Full system RTL, C sources and board design files available at https://github.com/nearist
*
* in2H2 inc. Team Members:
* - Chris McCormick - Algorithm Research and Design
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
* File description: Abstract class for Node.
*/
#ifndef TRUNK_SW_INC_INODE_H_
#define TRUNK_SW_INC_INODE_H_

#include <string>
#include <vector>
#include <map>
#include <highfive/H5DataSet.hpp>
#include "iFlexTypes.h"
#include "NodeConfig.h"
#include "Logger.h"

namespace IFlex {
class INode {
 protected:
  NodeConfig m_config;

  uint64_t m_offset;
  capacity_t m_capacity;

  DistanceMode m_distance_mode;
  QueryMode m_query_mode;

  uint16_t m_read_count;
  uint32_t m_threshold_lower;
  uint32_t m_threshold_upper;

  uint64_t m_duration;

#ifdef DEBUG_MODE_ENABLED
  packet_stat_t m_packet_stats;
#endif

 public:
  explicit INode(NodeConfig config)
      : m_config(std::move(config)),
        m_offset{},
        m_capacity{},
        m_distance_mode(DistanceMode::NO_DISTANCE_MODE),
        m_query_mode(QueryMode::NO_QUERY_MODE),
        m_read_count{},
        m_threshold_lower{},
        m_threshold_upper{},
        m_duration(0) {
    m_capacity.volume = 0;
    m_capacity.fv_count = 0;
  }

  virtual ~INode() = default;

  capacity_t getCapacity() const {
    return m_capacity;
  }

  virtual void open() = 0;

  virtual void close() = 0;

  virtual void reset() = 0;

  virtual void setDistanceMode(DistanceMode mode) = 0;

  virtual void setQueryMode(QueryMode mode) = 0;

  virtual void setReadCount(uint16_t count) = 0;

  virtual void setThreshold(uint32_t threshold) = 0;

  virtual void setThreshold(uint32_t threshold_lower,
                            uint32_t threshold_upper) = 0;

  virtual void dsLoad(uint64_t offset,
                      const vector8_list_t &vectors) = 0;

  virtual void dsLoadFromFile(const std::string &fileName,
                      const std::string &datasetName,
                      uint64_t offset,
                      uint64_t count) = 0;

  virtual void dsLoadRandom(uint64_t offset,
                            uint64_t vector_count,
                            uint64_t comp_count) = 0;

  virtual void query(const vector8_list_t &vectors,
                     vector_result32_list_t &results) = 0;

  virtual void resetTimer() = 0;

  virtual uint64_t getTimerValue() = 0;

  virtual std::map<uint8_t, uint16_t> getTemperature(bool force) = 0;

  virtual uint32_t getSubNodeCount() = 0;

  NodeConfig &getConfig() {
    return m_config;
  }

#ifdef DEBUG_MODE_ENABLED
  const packet_stat_t &get_packet_stats() const {
    return m_packet_stats;
  }
#endif
};
}  // namespace IFlex

#endif  // TRUNK_SW_INC_INODE_H_
