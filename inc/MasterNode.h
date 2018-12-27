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
* File description: Master node which send commands to iFlex/TCP nodes and collect response/results.
*/
#ifndef TRUNK_SW_INC_MASTERNODE_H_
#define TRUNK_SW_INC_MASTERNODE_H_

#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include <future>
#include <mutex>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/multi_array.hpp>
#include <boost/format.hpp>

#define H5_USE_BOOST
#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>

#include "iFlexTypes.h"
#include "Logger.h"
#include "AppConfig.h"
#include "INode.h"
#include "Request.h"
#include "Response.h"
#include "TimeMeter.h"
#include "RepeatingTimer.h"

namespace IFlex {
class MasterNode {
  const std::string TAG = "MasterNode";

  std::mutex node_mtx;

 protected:
  const uint64_t CP_COUNT = 4096;

  fv_info_t m_fv_info;
  capacity_t m_capacity;

  uint8_t m_component_size;

  DistanceMode m_distance_mode;
  QueryMode m_query_mode;
  uint16_t m_read_count;
  uint32_t m_threshold_lower;
  uint32_t m_threshold_upper;

  std::string m_api_key;
  std::string m_dataset_nearist;
  std::string m_dataset_user;

  std::vector<INode *> m_children;

  uint64_t m_duration_sort;

#ifdef DEBUG_MODE_ENABLED
  boost::format debug_fmt;

  std::thread m_th_debug_status;
  RepeatingTimer rt_debug_status;
#endif

  std::thread m_th_log_temperature;
  RepeatingTimer rt_log_temperature;

  bool m_temperature_log_enable;
  uint8_t m_temperature_log_rate;

 public:
  MasterNode();

  ~MasterNode();

  std::vector<INode *> &getChildren();

  const std::string &getApiKey() const;

  void setApiKey(const std::string &m_api_key);

  void setDatasetNearist(const std::string &dataset_nearist);

  void setDatasetUser(const std::string &dataset_user);

  void setTemperatureLogEnable(bool enable);

  void setTemperatureLogRate(uint8_t rate);

  void setComponentSize(uint8_t m_component_size);

  virtual void open();

  virtual void close();

  virtual void fillCapacity();

  virtual void initTimers();

  virtual void reset();

  virtual void setDistanceMode(DistanceMode mode);

  virtual void setQueryMode(QueryMode mode);

  virtual void setReadCount(uint16_t count);

  virtual void setThreshold(uint32_t threshold);

  virtual void setThreshold(uint32_t threshold_lower, uint32_t threshold_upper);

  virtual void dsLoad(vector8_list_t &vectors);

  virtual void query(vector8_list_t &vectors,
                     vector_result32_list_t &results);

  virtual void dsLoadRandom(uint64_t vector_count,
                            uint64_t comp_count);

  virtual void dsLoadFromFile(const std::string &fileName,
                              const std::string &datasetName);

  virtual void queryFromFile(const std::string &fileName,
                             const std::string &datasetName,
                             const std::string &output,
                             bool overwrite);

  void deserializeVectors(Request *request,
                          vector8_list_t &vectors);

  void serializeResults(Response *response,
                        const vector_result32_list_t &results,
                        bool batch);

  virtual void resetTimer();

  virtual uint64_t getTimerValue();

  virtual std::map<std::string, std::map<uint8_t, uint16_t>> getTemperature();

 private:
  bool isDataSetValid(uint64_t vector_count,
                      uint64_t comp_count,
                      fv_info_t &info);

  bool isQueryValid(const vector8_list_t &vectors);

  void normalizeVectors(vector8_list_t &vectors);

#ifdef DEBUG_MODE_ENABLED
  void debug_status(bool first_loop);
#endif

  void log_temperature(bool first_loop);

  std::vector<std::string> get_dataset_files();

  std::string normalize_file_name(const std::string &file_name);

  bool check_dataset_file_exists(const std::string &file_name);

  bool check_output_file_not_exists(const std::string &file_name);

  void write_results_to_file(const std::string &file_name,
                             const vector_result32_list_t &results);
};
}  // namespace IFlex

#endif  // TRUNK_SW_INC_MASTERNODE_H_
