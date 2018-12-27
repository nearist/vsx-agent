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
* File description: IFlexNode implementation.
*/
#ifndef TRUNK_SW_INC_IFLEXNODE_H_
#define TRUNK_SW_INC_IFLEXNODE_H_

#include <queue>
#include <vector>
#include <string>
#include <map>
#include <numeric>
#include <algorithm>
#include <cmath>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <boost/algorithm/string/replace.hpp>

#define H5_USE_BOOST
#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>

#include "INode.h"
#include "NodeConfig.h"
#include "LinuxWrapper.h"
#include "iFlexNodeReader.h"
#include "TimeMeter.h"

namespace IFlex {
class IFlexNode : public INode {
  std::string TAG = "IFlexNode";

  static constexpr uint32_t PACKET_PREAMBLE[2] = {0xFEDCBA98, 0x76543210};

  static const uint8_t FPGA_COUNT = 7u;
  static const uint8_t FPGA_MASK = 0xFFu >> (8u - FPGA_COUNT);

  static const uint32_t THRESHOLD_LOWER = 0xFFFFFFFF;
  static const uint32_t THRESHOLD_UPPER = 0x00000000;

  static const int16_t dtr_map[64];

 protected:
  enum Command : uint8_t {
    CMD_NOP = 0x00,
    CMD_WRITE_REG = 0x01,
    CMD_READ_REG = 0x02,
    CMD_WRITE_MEM = 0x03,
    CMD_READ_MEM = 0x04,
    CMD_CALC_START = 0x05,
    CMD_CALC_STOP = 0x06,
    CMD_CALC_RESET = 0x07,
    CMD_SEND_QV = 0x08,
    RESULT = 0xAA,
    RESULT_LAST = 0xAB
  };

  enum Register : uint8_t {
    RTL_VERSION_ADDR = 0x0000,
    THRESHOLD_LO_ADDR = 0x0001,
    THRESHOLD_HI_ADDR = 0x0002,
    DSV_COUNT_LO_ADDR = 0x0003,
    DSV_COUNT_HI_ADDR = 0x0004,
    DSV_LENGTH_ADDR = 0x0005,
    KNN_NUM_NUMBER = 0x0006,
    STATUS_ADDR = 0x000C,
    DESIGN_DEF_1_ADDR = 0x000D,
    DESIGN_DEF_2_ADDR = 0x000E
  };

  struct iflex_write_packet_t {
    uint16_t size;
    Command command;
    uint8_t fpga_mask{};
    ulong_t *data;
    uint16_t transaction_id{};
    uint16_t crc{};

    explicit iflex_write_packet_t(uint16_t size)
        : size(size), command(Command::CMD_NOP), data(new ulong_t[size]()) {}

    virtual ~iflex_write_packet_t() {
      if (this->data != nullptr) {
        delete[] this->data;
        this->data = nullptr;
      }
    }

    uint16_t length() { return static_cast<uint16_t>((this->size + 4) * 4); }

    std::shared_ptr<ulong_t[]> value() {
      std::shared_ptr<ulong_t[]> data(new ulong_t[size + 4]);

      data[0].l = PACKET_PREAMBLE[0];
      data[1].l = PACKET_PREAMBLE[1];

      data[2].b[3] = fpga_mask;
      data[2].b[2] = command;
      data[2].w[0] = static_cast<uint16_t>(size + 1);

      std::memcpy(&data[3],
                  &this->data[0],
                  static_cast<size_t>(this->size * 4));

      if (this->data != nullptr) {
        delete[] this->data;
        this->data = nullptr;
      }

      data[size + 3].w[1] = transaction_id;
      data[size + 3].w[0] = crc;

      return data;
    }
  };

  typedef struct {
    uint64_t vector_id;
    vector8_t components;
    vector_result32_t result;
    bool last[FPGA_COUNT];
  } query_request_t;

  typedef struct {
    uint8_t fpga;
    Command command;
    uint16_t length;

    uint16_t pid;
    uint16_t crc;

    ulong_t *data;
  } packet_t;

  std::vector<IFlex::DistanceMode> m_supported_distance_modes;
  std::vector<IFlex::QueryMode> m_supported_query_modes;

  int32_t m_df_read;
  int32_t m_df_write;

  uint16_t m_transaction_id;

  uint64_t m_pending_count;

  bool m_running;

  iFlexNodeReader reader;

  std::queue<iFlexNodeReader::packet_t> m_read_queue;

  std::map<uint16_t, std::queue<query_request_t>> m_request_map;
  std::map<uint64_t, vector_result32_t> m_result_map;

  std::map<Register, std::map<uint8_t, uint16_t>> m_register_map;

  std::map<uint8_t, uint16_t> m_temperature_map;

  std::thread m_th_reader;
  std::thread m_th_parser;

  std::mutex m_mtx_rq;
  std::mutex m_mtx_status;
  std::mutex m_mtx_write;

  std::condition_variable m_cv_rq_not_empty;
  std::condition_variable m_cv_status_ready;


 protected:
  uint16_t GetTransactionId();

  ssize_t __write(int32_t fd, const void *buf, size_t len);

  ssize_t __read(int32_t df, void *buf, size_t len);

  uint16_t calc_crc(const iflex_write_packet_t &packet) {
    uint16_t crc = 0xAAAA;
    return crc;
  }

  void WriteRegister(uint8_t fpga_select, Register address, uint16_t value);

  void ReadRegister(uint8_t fpga_select,
                    Register address,
                    uint16_t mask,
                    std::map<uint8_t, uint16_t> &value);

  void TaskReader();

  void TaskParser();

  void WriteQuery(uint16_t query_id);

  void WriteDSBatch(const vector8_list_t &vectors,
                    uint32_t ram_address[FPGA_COUNT]);

  static void fillVectorList(vector8_list_t &vector_list,
                             uint64_t vector_count, uint64_t comp_count,
                             uint8_t lower, uint8_t upper);

 public:
  explicit IFlexNode(const NodeConfig &config);

  void open() override;

  void close() override;

  void reset() override;

  void StartCalculation();

  void StopCalculation();

  void ResetCalculation();

  void setDistanceMode(DistanceMode mode) override;

  void setQueryMode(QueryMode mode) override;

  void setReadCount(uint16_t count) override;

  void setThreshold(uint32_t threshold) override;

  void setThreshold(uint32_t threshold_lower,
                    uint32_t threshold_upper) override;

  void dsLoad(uint64_t offset,
              const vector8_list_t &vectors) override;

  void dsLoadFromFile(const std::string &fileName,
              const std::string &datasetName,
              uint64_t offset,
              uint64_t count) override;

  void dsLoadRandom(uint64_t offset,
                    uint64_t vector_count,
                    uint64_t comp_count) override;

  void query(const vector8_list_t &vectors,
             vector_result32_list_t &results) override;

  void resetTimer() override;

  uint64_t getTimerValue() override;

  std::map<uint8_t, uint16_t> getTemperature(bool force) override;

  uint32_t getSubNodeCount() override { return FPGA_COUNT; };
};
}  // namespace IFlex
#endif  // TRUNK_SW_INC_IFLEXNODE_H_
