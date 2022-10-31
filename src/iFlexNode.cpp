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
#include "iFlexNode.h"

namespace IFlex {
const int16_t IFlexNode::dtr_map[] = {
    -58, -56, -54, -52, -45, -44, -43, -42, -41, -40, -39, -38, -37,
    -36, -30, -20, -10, -4, 0, 4, 10, 21, 22, 23, 24, 25, 26, 27, 28,
    29, 40, 50, 60, 70, 76, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
    95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108,
    116, 120, 124, 128, 132
};

IFlexNode::IFlexNode(const NodeConfig &config) : INode(config) {
  m_supported_distance_modes.push_back(IFlex::DistanceMode::L1);
  m_supported_distance_modes.push_back(IFlex::DistanceMode::HAMMING);

  m_supported_query_modes.push_back(IFlex::QueryMode::KNN_A);
  m_supported_query_modes.push_back(IFlex::QueryMode::KNN_D);
  m_supported_query_modes.push_back(IFlex::QueryMode::GT);
  m_supported_query_modes.push_back(IFlex::QueryMode::LT);

  m_distance_mode = IFlex::DistanceMode::L1;
  m_query_mode = IFlex::QueryMode::GT;
  m_read_count = 0;
  m_threshold_lower = 0;

  m_read_queue = std::queue<iFlexNodeReader::packet_t>();

  m_request_map = std::map<uint16_t, std::queue<query_request_t>>();
  m_result_map = std::map<uint64_t, vector_result32_t>();
  m_temperature_map = std::map<uint8_t, uint16_t>();

  m_running = false;

  m_pending_count = 0;

  m_transaction_id = 0;
  //TODO: change these to be retrieved from the RTL
  //TODO: understand what kind of a placeholder value these are currently supposed to be. (90194313216 and 45097156608 respectively)
  m_capacity.fv_count = FPGA_COUNT * (1UL << 32UL);
  m_capacity.volume = FPGA_COUNT * 2 * 1024 * 1024 * 1024UL;

#ifdef DEBUG_MODE_ENABLED
  m_packet_stats.source = m_config.getDeviceFileRead();
  m_packet_stats.packet_expected.resize(FPGA_COUNT);
  m_packet_stats.packet_received.resize(FPGA_COUNT);
  m_packet_stats.result_expected.resize(FPGA_COUNT);
  m_packet_stats.result_received.resize(FPGA_COUNT);
#endif
}

uint16_t IFlexNode::GetTransactionId() {
  uint16_t tr_id = m_transaction_id;
  m_transaction_id++;
  return tr_id;
}

ssize_t IFlexNode::__write(std::int32_t fd, const void *buf, size_t len) {
  if (fd == -1) {
    return -1;
  }

  size_t sent = 0;
  ssize_t rc;

  Logger::hexdump(TAG, buf, len);

  while (sent < len) {
    rc = LinuxWrapper::_write(fd, (uintptr_t *) (buf + sent), len - sent);
    LinuxWrapper::_write(fd, nullptr, 0);

    if (rc == 0) {
      Logger::error(TAG, "Reached write EOF (?!)");
      break;
    } else if (rc < 0) {
      if (errno == EINTR) {
        continue;
      } else {
        Logger::error(TAG,
                      "Write error: {}",
                      LinuxWrapper::_strerror(errno));
        break;
      }
    }

    sent += rc;
  }

  return sent;
}

ssize_t IFlexNode::__read(int32_t fd, void *buf, size_t len) {
  if (fd == -1) {
    return -1;
  }

  size_t received = 0;
  ssize_t rc;

  while (received < len) {
    rc =
        LinuxWrapper::_read(fd, (uintptr_t *) (buf + received), len - received);

    if (rc == 0) {
      Logger::error(TAG, "Reached read EOF (?!)");
      break;
    } else if (rc < 0) {
      if (errno == EINTR) {
        continue;
      } else {
        Logger::error(TAG,
                      "Read error: {}",
                      LinuxWrapper::_strerror(errno));
        continue;
      }
    }

    received += rc;
  }

  Logger::hexdump(TAG, buf, received);

  return received;
}

void IFlexNode::open() {
  TAG.append(m_config.getDeviceFileWrite()).append("]");
  boost::replace_all(TAG, "/dev/xillybus_", "[");
  boost::replace_all(TAG, "_write_", "][");

  Logger::hexdump(TAG, dtr_map, sizeof(dtr_map));

  std::string write = m_config.getDeviceFileWrite();
  std::string read = m_config.getDeviceFileRead();

  m_df_write = LinuxWrapper::_open(write.c_str(), LinuxWrapper::_O_WRONLY);
  m_df_read = LinuxWrapper::_open(read.c_str(), LinuxWrapper::_O_RDONLY);

  if (m_df_write == -1) {
    Logger::critical(TAG,
                     "\"{}\" device not opened. {}.",
                     LinuxWrapper::_strerror(errno),
                     write.c_str());
  }

  if (m_df_read == -1) {
    Logger::critical(TAG,
                     "\"{}\" device not opened. {}.",
                     LinuxWrapper::_strerror(errno),
                     read.c_str());
  }

  m_running = true;

  m_th_reader = std::thread(&IFlexNode::TaskReader, this);
  m_th_parser = std::thread(&IFlexNode::TaskParser, this);

  m_th_reader.detach();
  m_th_parser.detach();
}

void IFlexNode::close() {
  m_running = false;

  if (m_df_read != -1) {
    LinuxWrapper::_close(m_df_read);
  }

  if (m_df_write != -1) {
    LinuxWrapper::_close(m_df_write);
  }
}

//this is only called in open but why and is this the async handler call for sending(host) a command? for results only?
void IFlexNode::TaskReader() {
  reader.open(m_df_read);

  reader.set_callbacks(
      [&](std::shared_ptr<iFlexNodeReader::packet_t> packet) {
#ifdef DEBUG_MODE_ENABLED
        if (packet->header.command == iFlexNodeReader::Command::RESULT
            || packet->header.command
                == iFlexNodeReader::Command::RESULT_LAST) {
          m_packet_stats.result_received[__builtin_ffs(packet->header.fpga) - 1]++;
        } else {
          m_packet_stats.packet_received[__builtin_ffs(packet->header.fpga) - 1]++;

          if (packet->header.length == 0) {
            Logger::error(TAG,
                          "Error zero-length packet: {}",
                          packet->header.command);
          }
        }
#endif

        if (packet->header.command == iFlexNodeReader::Command::RESULT
            || packet->header.command
                == iFlexNodeReader::Command::RESULT_LAST
            || packet->header.command
                == iFlexNodeReader::Command::CMD_READ_REG) {
          std::unique_lock<std::mutex> rq_l(m_mtx_rq);
          m_read_queue.push(*packet);
          m_cv_rq_not_empty.notify_one();
        }
      }, [&](boost::system::error_code const &ec) {
        Logger::error(TAG,
                      "Error '{}' during asynchronous operation",
                      ec.message());
      });

  reader.start();
}

// this is only called in open but why andf is this the async handler call for recieving(host) results only?
void IFlexNode::TaskParser() {
  while (m_running) {
    iFlexNodeReader::packet_t packet{};

    {
      std::unique_lock<std::mutex> rq_l(m_mtx_rq);
      m_cv_rq_not_empty.wait(rq_l,
                             [&] { return !IFlexNode::m_read_queue.empty(); });

      packet = m_read_queue.front();
      m_read_queue.pop();
    }

    switch (packet.header.command) {
      case iFlexNodeReader::Command::CMD_READ_REG: {
        auto address = (Register) packet.data.get()[0].w[1];
        auto value = packet.data.get()[0].w[0];
        {
          std::unique_lock<std::mutex> status_l(m_mtx_status);
          m_register_map[address].insert(std::make_pair(
              __builtin_ffs(packet.header.fpga) - 1, value));
          m_cv_status_ready.notify_one();
        }
      }
        break;
      case iFlexNodeReader::Command::RESULT:
      case iFlexNodeReader::Command::RESULT_LAST: {
        uint16_t qv_id = packet.footer.pid;
        if (m_request_map.find(qv_id) != m_request_map.end()) {
          if (!m_request_map[qv_id].empty()) {
            bool packet_last = packet.header.command
                == iFlexNodeReader::Command::RESULT_LAST;

            bool last = false;
            if (packet_last) {
              m_request_map[qv_id].front().last[
                  __builtin_ffs(packet.header.fpga) - 1] = true;

              last = true;

              for (bool i : m_request_map[qv_id].front().last) {
                if (!i) {
                  last = false;
                  break;
                }
              }

              m_temperature_map[__builtin_ffs(packet.header.fpga) - 1] =
                  packet.data.get()[0].b[0] & 0x3F;
            }

            if (last) {
              auto request = m_request_map[qv_id].front();

              m_result_map[request.vector_id] = request.result;
              m_request_map[qv_id].pop();

//              this->WriteQuery(qv_id);

              {
                std::unique_lock<std::mutex> status_l(m_mtx_status);
                m_pending_count = 0;
                if (!m_request_map.empty()) {
                  m_pending_count = std::accumulate(
                      std::begin(m_request_map),
                      std::end(m_request_map),
                      0ULL,
                      [](uint64_t value,
                         const std::map<uint16_t,
                                        std::queue<query_request_t>>::value_type &p) {
                        return value + p.second.size();
                      });
                }
                m_cv_status_ready.notify_one();
              }
            } else if (!packet_last) {
              uint64_t id = packet.data.get()[0].l;
              uint64_t fpga_num = __builtin_ffs(packet.header.fpga) - 1;

              uint64_t ds_id = m_offset +
                  ((id >> 4) * ((uint64_t) FPGA_COUNT << 4) + (fpga_num << 4) +
                      (id & 0xF));
              uint32_t distance = packet.data.get()[1].l;

              m_request_map[qv_id].front().result.emplace_back(result32_t(ds_id,
                                                                          distance));
            }
          }
        }
      }
        break;
      default:
        break;
    }
  }
}

//Command Builder/Handler for CMD_WRITE_REG
void IFlexNode::WriteRegister(uint8_t fpga_select,
                              const Register address,
                              const uint16_t value) {
  iflex_write_packet_t packet(1);
  packet.fpga_mask = fpga_select;
  packet.command = Command::CMD_WRITE_REG;

  packet.data[0].w[1] = address;
  packet.data[0].w[0] = value;

  packet.transaction_id = GetTransactionId();
  packet.crc = calc_crc(packet);

#ifdef DEBUG_MODE_ENABLED
  for (uint8_t i = 0; i < FPGA_COUNT; i++) {
    m_packet_stats.packet_expected[i] += (packet.fpga_mask >> i) & 0x01;
  }
#endif

  this->__write(m_df_write, packet.value().get(), packet.length());
}

//Command Builder/Handler for CMD_READ_REG
void IFlexNode::ReadRegister(uint8_t fpga_select,
                             const Register address,
                             const uint16_t mask,
                             std::map<uint8_t, uint16_t> &values) {
  iflex_write_packet_t packet(1);
  packet.fpga_mask = fpga_select;
  packet.command = Command::CMD_READ_REG;

  packet.data[0].w[1] = address;
  packet.data[0].w[0] = 0x0000;

  packet.transaction_id = GetTransactionId();
  packet.crc = calc_crc(packet);

#ifdef DEBUG_MODE_ENABLED
  for (uint8_t i = 0; i < FPGA_COUNT; i++) {
    m_packet_stats.packet_expected[i] += (packet.fpga_mask >> i) & 0x01;
  }
#endif

  m_register_map[address] = std::map<uint8_t, uint16_t>();

  this->__write(m_df_write, packet.value().get(), packet.length());

  {
    std::unique_lock<std::mutex> status_l(m_mtx_status);
    m_cv_status_ready.wait(status_l, [&] {
      return m_register_map[address].size()
          == __builtin_popcount(packet.fpga_mask);
    });
  }

  values = std::move(m_register_map[address]);

  for (auto &value : values) {
    value.second &= mask;
  }
}

//Command Builder/Handler for CMD_CALC_START
void IFlexNode::StartCalculation() {
  iflex_write_packet_t packet(1);
  packet.fpga_mask = FPGA_MASK;
  packet.command = Command::CMD_CALC_START;

  packet.data[0].l = 0;

  packet.transaction_id = GetTransactionId();
  packet.crc = calc_crc(packet);

#ifdef DEBUG_MODE_ENABLED
  for (uint8_t i = 0; i < FPGA_COUNT; i++) {
    m_packet_stats.packet_expected[i] += (packet.fpga_mask >> i) & 0x01;
  }
#endif

  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  this->__write(m_df_write, packet.value().get(), packet.length());
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

void IFlexNode::StopCalculation() {
  iflex_write_packet_t packet(1);
  packet.fpga_mask = FPGA_MASK;
  packet.command = Command::CMD_CALC_STOP;

  packet.data[0].l = 0;

  packet.transaction_id = GetTransactionId();
  packet.crc = calc_crc(packet);

#ifdef DEBUG_MODE_ENABLED
  for (uint8_t i = 0; i < FPGA_COUNT; i++) {
    m_packet_stats.packet_expected[i] += (packet.fpga_mask >> i) & 0x01;
  }
#endif

  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  this->__write(m_df_write, packet.value().get(), packet.length());
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

void IFlexNode::ResetCalculation() {
  iflex_write_packet_t packet(1);
  packet.fpga_mask = FPGA_MASK;
  packet.command = Command::CMD_CALC_RESET;

  packet.data[0].l = 0;

  packet.transaction_id = GetTransactionId();
  packet.crc = calc_crc(packet);

#ifdef DEBUG_MODE_ENABLED
  for (uint8_t i = 0; i < FPGA_COUNT; i++) {
    m_packet_stats.packet_expected[i] += (packet.fpga_mask >> i) & 0x01;
  }
#endif

  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  this->__write(m_df_write, packet.value().get(), packet.length());
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

void IFlexNode::WriteQuery(const uint16_t query_id) {
  auto tm_logic = TimeMeter();

  std::unique_lock<std::mutex> lock(m_mtx_write);

  if (m_request_map.find(query_id) != m_request_map.end()) {
    if (!m_request_map[query_id].empty()) {
      auto qr = m_request_map[query_id].front();

      iflex_write_packet_t
          packet(static_cast<uint16_t>(qr.components.size() / 4));
      packet.fpga_mask = FPGA_MASK;
      packet.command = Command::CMD_SEND_QV;

      for (uint32_t i = 0; i < qr.components.size(); i++) {
        packet.data[i >> 2].b[3 - (i & 0x03)] = qr.components[i];
      }

      packet.transaction_id = query_id;
      packet.crc = calc_crc(packet);

#ifdef DEBUG_MODE_ENABLED
      for (uint8_t i = 0; i < FPGA_COUNT; i++) {
        m_packet_stats.packet_expected[i] += (packet.fpga_mask >> i) & 0x01;
      }
#endif

      this->__write(m_df_write, packet.value().get(), packet.length());
      Logger::debug(TAG, "Single write time {} us", tm_logic.microseconds());
    }
  }
}

void IFlexNode::setThreshold(const uint32_t threshold) {
  auto tm = TimeMeter();

  m_threshold_lower = threshold;

  ulong_t value = {.l = m_threshold_lower};

  WriteRegister(FPGA_MASK, Register::THRESHOLD_LO_ADDR, value.w[0]);
  WriteRegister(FPGA_MASK, Register::THRESHOLD_HI_ADDR, value.w[1]);

  m_duration += tm.nanoseconds();
}

void IFlexNode::setThreshold(const uint32_t threshold_lower,
                             const uint32_t threshold_upper) {
  IFlexNode::setThreshold(threshold_lower);
}

void IFlexNode::dsLoad(const uint64_t offset, const vector8_list_t &vectors) {
  auto tm = TimeMeter();

  StopCalculation();
  ResetCalculation();

  m_offset = offset;

  uint64_t fv_count = vectors.size();
  uint64_t comp_count = vectors.front().size();

  uint64_t vectors_to_load = fv_count;

  uint32_t vector_per_fpga[FPGA_COUNT] = {};
  uint32_t ram_address[FPGA_COUNT] = {};

  uint8_t fpga_num = 0;
  uint64_t batch_num = 0;

  while (fv_count) {
    for (fpga_num = 0; fpga_num < FPGA_COUNT; fpga_num++) {
      if (fv_count) {
        if (fv_count > 16) {
          vector_per_fpga[fpga_num] += 16;
          fv_count -= 16;
        } else {
          vector_per_fpga[fpga_num] += fv_count;
          fv_count = 0;
        }
      } else {
        break;
      }
    }
  }

  for (fpga_num = 0; fpga_num < FPGA_COUNT; fpga_num++) {
    auto fpga_mask = static_cast<uint8_t>(1u << fpga_num);
    auto c_count = static_cast<uint16_t>(comp_count * m_config.getComponentSize() - 1);
    ulong_t v_count = {.l = (vector_per_fpga[fpga_num] - 1)};

    WriteRegister(fpga_mask, Register::DSV_COUNT_LO_ADDR, v_count.w[0]);
    WriteRegister(fpga_mask, Register::DSV_COUNT_HI_ADDR, v_count.w[1]);
    WriteRegister(fpga_mask, Register::DSV_LENGTH_ADDR, c_count);
  }

  while (vectors_to_load) {
    for (fpga_num = 0; fpga_num < FPGA_COUNT; fpga_num++) {
      if (vectors_to_load) {
        iflex_write_packet_t
            packet(static_cast<uint16_t>(comp_count * 16 / 4 + 1));
        packet.fpga_mask = static_cast<uint8_t>(1u << fpga_num);
        packet.command = Command::CMD_WRITE_MEM;
        packet.data[0].l = ram_address[fpga_num];

        if (vectors_to_load > 16) {
          for (uint16_t i = 0; i < comp_count; i++) {
            for (uint16_t j = 0; j < 16; j++) {
              packet.data[(i << 2) + (j >> 2) + 1].b[3 - (j & 0x3)]
                  = vectors[batch_num * 16 + j][i];
            }
          }

          vectors_to_load -= 16;
        } else {
          for (uint16_t i = 0; i < comp_count; i++) {
            for (uint16_t j = 0; j < 16; j++) {
              if (j < vectors_to_load) {
                packet.data[(i << 2) + (j >> 2) + 1].b[3 - (j & 0x3)] =
                    vectors[batch_num * 16 + j][i];
              }
            }
          }

          vectors_to_load = 0;
        }

        batch_num++;
        ram_address[fpga_num] += comp_count;

        packet.transaction_id = GetTransactionId();
        packet.crc = calc_crc(packet);

#ifdef DEBUG_MODE_ENABLED
        for (uint8_t i = 0; i < FPGA_COUNT; i++) {
          m_packet_stats.packet_expected[i] += (packet.fpga_mask >> i) & 0x01;
        }
#endif

        this->__write(m_df_write, packet.value().get(), packet.length());
      } else {
        break;
      }
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  StartCalculation();

  m_duration += tm.nanoseconds();
}

void IFlexNode::query(const vector8_list_t &vectors,
                      vector_result32_list_t &results) {
  auto tm_all = TimeMeter();

  m_request_map.clear();
  m_result_map.clear();

  uint16_t query_id = GetTransactionId();

  auto tm_prepare = TimeMeter();
  //building the query requests
  for (auto it = vectors.begin(); it != vectors.end(); ++it) {
    auto vector = (*it);

    query_request_t qr;
    qr.vector_id = (uint64_t) (it - vectors.begin());
    qr.components = vector;

    for (bool &i : qr.last) {
      i = false;
    }

    if (m_request_map.find(query_id) == m_request_map.end()) {
      m_request_map[query_id] = std::queue<query_request_t>();
    }

    m_request_map[query_id].push(qr);

    query_id = GetTransactionId();
  }
  Logger::debug(TAG, "Batch Query prepare time {} us", tm_prepare.microseconds());

  m_pending_count = vectors.size();

  auto tm_write = TimeMeter();
  for (auto &it : m_request_map) {
    this->WriteQuery(it.first);
  }
  Logger::debug(TAG, "Batch Query write time {} us", tm_write.microseconds());

  auto tm_read = TimeMeter();
  {
    std::unique_lock<std::mutex> status_l(m_mtx_status);
    m_cv_status_ready.wait(status_l, [&] { return m_pending_count == 0; });
  }
  Logger::debug(TAG, "Batch Result read time {} us", tm_read.microseconds());

  for (auto &it : m_result_map) {
    if (!it.second.empty()) {
      results.emplace_back(vector_result32_t(it.second.begin(), it.second.end()));
    } else {
      results.emplace_back(vector_result32_t());
    }
  }

  if (m_query_mode == QueryMode::KNN_D || m_query_mode == QueryMode::KNN_A) {
    auto tm_sort_node = TimeMeter();
    for (auto &result : results) {
      auto tm_sort_vector = TimeMeter();
      std::sort(result.begin(), result.end(), KNNComparator(m_query_mode));
      Logger::debug(TAG, "Vector Sort time {} us", tm_sort_vector.microseconds());

      if (m_read_count != 0 && result.size() > m_read_count) {
        result.erase(result.begin() + m_read_count, result.end());
      }
    }
    Logger::debug(TAG, "Node Sort time {} us", tm_sort_node.microseconds());
  }

  m_request_map.clear();
  m_result_map.clear();
  m_pending_count = 0;

  m_duration += tm_all.nanoseconds();
  Logger::debug(TAG, "Batch Query time {} us", tm_all.microseconds());
}

void IFlexNode::resetTimer() {
  m_duration = 0;
}

uint64_t IFlexNode::getTimerValue() {
  return m_duration;
}

void IFlexNode::reset() {
  m_read_count = 0;
  m_threshold_lower = THRESHOLD_LOWER;
  m_threshold_upper = THRESHOLD_UPPER;
}

void IFlexNode::setDistanceMode(const IFlex::DistanceMode mode) {
  auto tm = TimeMeter();

  if (std::find(m_supported_distance_modes.begin(),
                m_supported_distance_modes.end(),
                mode) ==
      m_supported_distance_modes.end()) {
    throw DistanceModeNotSupportedException();
  }

  m_distance_mode = mode;

  m_duration += tm.nanoseconds();
}

void IFlexNode::setQueryMode(const IFlex::QueryMode mode) {
  auto tm = TimeMeter();

  if (std::find(m_supported_query_modes.begin(),
                m_supported_query_modes.end(),
                mode) ==
      m_supported_query_modes.end()) {
    throw QueryModeNotSupportedException();
  }

  m_query_mode = mode;

  m_duration += tm.nanoseconds();
}

void IFlexNode::setReadCount(const uint16_t count) {
  auto tm = TimeMeter();
  //the card doesn't support returning a k count larger than 102
  if (count > 0 && count < 102) {
    m_read_count = count;
  } else {
    throw ReadCountNotSupportedException();
  }

  ulong_t value = {.l = m_read_count};
  WriteRegister(FPGA_MASK, Register::KNN_NUM_NUMBER, value.w[0]);

  m_duration += tm.nanoseconds();
}

std::map<uint8_t, uint16_t> IFlexNode::getTemperature(bool force) {
  std::map<uint8_t, uint16_t> value;
  if (force) {
    ReadRegister(FPGA_MASK, Register::STATUS_ADDR, 0x3F, value);
    m_temperature_map.clear();
    m_temperature_map = value;
  } else {
    value = m_temperature_map;
  }

  return value;
}

void IFlexNode::dsLoadRandom(const uint64_t offset, const uint64_t vector_count, const  uint64_t comp_count) {
      auto tm = TimeMeter();

      StopCalculation();
      ResetCalculation();

      m_offset = offset;

      uint64_t fv_count = vector_count;

      uint32_t vector_per_fpga[FPGA_COUNT] = {};

      uint8_t fpga_num = 0;
      //Determine the distribution of DSVs to each vector processing unit
      //this can be drastically optimized so its a single loop forloop over the devices
      while (fv_count) {
        for (fpga_num = 0; fpga_num < FPGA_COUNT; fpga_num++) {
          if (fv_count) {
            if (fv_count > 16) {
              vector_per_fpga[fpga_num] += 16;
              fv_count -= 16;
            } else {
              vector_per_fpga[fpga_num] += fv_count;
              fv_count = 0;
            }
          } else {
            break;
          }
        }
      }

      for (fpga_num = 0; fpga_num < FPGA_COUNT; fpga_num++) {
        auto fpga_mask = static_cast<uint8_t>(1U << fpga_num);
        auto c_count = static_cast<uint16_t>(comp_count * m_config.getComponentSize() - 1);
        ulong_t v_count = {.l = (vector_per_fpga[fpga_num] - 1)};

        WriteRegister(fpga_mask, Register::DSV_COUNT_LO_ADDR, v_count.w[0]);
        WriteRegister(fpga_mask, Register::DSV_COUNT_HI_ADDR, v_count.w[1]);
        WriteRegister(fpga_mask, Register::DSV_LENGTH_ADDR, c_count);
      }

      uint32_t ram_address[FPGA_COUNT] = {};
      uint64_t batch_num = 0;
      uint64_t vectors_to_load = vector_count;


      vector8_list_t vectors(FPGA_COUNT * 16UL);
      fillVectorList(vectors, vectors.size(), comp_count, 0, 255);

      //load vectors onto the VSX cards by batch
      while (vectors_to_load > 0) {
        uint64_t sub_count = std::min(vectors_to_load, FPGA_COUNT * 16UL);

        vectors_to_load -= sub_count;

        WriteDSBatch(vector8_list_t(vectors.begin(), vectors.begin() + sub_count), ram_address);
        batch_num += sub_count;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      StartCalculation();

      m_duration += tm.nanoseconds();
    }

void IFlexNode::dsLoadFromFile(const std::string &fileName,
                       const std::string &datasetName,
                       const uint64_t offset,
                       const uint64_t count) {
  auto tm = TimeMeter();

  StopCalculation();
  ResetCalculation();

  m_offset = offset;

  HighFive::File file(fileName, HighFive::File::ReadOnly);
  auto ds = file.getDataSet(datasetName);

  auto space = ds.getMemSpace().getDimensions();

  uint64_t fv_count = count;
  uint64_t comp_count = space[1];

  uint32_t vector_per_fpga[FPGA_COUNT] = {};

  uint8_t fpga_num = 0;

  while (fv_count) {
    for (fpga_num = 0; fpga_num < FPGA_COUNT; fpga_num++) {
      if (fv_count) {
        if (fv_count > 16) {
          vector_per_fpga[fpga_num] += 16;
          fv_count -= 16;
        } else {
          vector_per_fpga[fpga_num] += fv_count;
          fv_count = 0;
        }
      } else {
        break;
      }
    }
  }

  for (fpga_num = 0; fpga_num < FPGA_COUNT; fpga_num++) {
    auto fpga_mask = static_cast<uint8_t>(1U << fpga_num);
    auto c_count = static_cast<uint16_t>(comp_count * m_config.getComponentSize() - 1);
    ulong_t v_count = {.l = (vector_per_fpga[fpga_num] - 1)};

    WriteRegister(fpga_mask, Register::DSV_COUNT_LO_ADDR, v_count.w[0]);
    WriteRegister(fpga_mask, Register::DSV_COUNT_HI_ADDR, v_count.w[1]);
    WriteRegister(fpga_mask, Register::DSV_LENGTH_ADDR, c_count);
  }

  uint32_t ram_address[FPGA_COUNT] = {};
  uint64_t batch_num = 0;
  uint64_t vectors_to_load = count;

  while (vectors_to_load > 0) {
    uint64_t sub_count = std::min(vectors_to_load, FPGA_COUNT * 16UL);

    vector8_list_t data;

    if(m_config.getComponentSize() == 1) {
      ds.select({offset + batch_num, 0},
                {sub_count, comp_count},
                {}).read(data);
    } else if(m_config.getComponentSize() == 4) {
      vector32_list_t vectors;
      ds.select({offset + batch_num, 0},
                {sub_count, comp_count},
                {}).read(vectors);

      data = vector32_list_2_vector8_list(vectors);
    }

    vectors_to_load -= sub_count;
    WriteDSBatch(data, ram_address);
    batch_num += sub_count;
  }

  H5Fclose(file.getId());

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  StartCalculation();

  m_duration += tm.nanoseconds();
}

void IFlexNode::WriteDSBatch(const vector8_list_t &vectors,
                             uint32_t ram_address[FPGA_COUNT]) {
  uint64_t fv_count = vectors.size();
  uint64_t comp_count = vectors.front().size();

  for (uint8_t fpga_num = 0; fpga_num < FPGA_COUNT; fpga_num++) {
    if (fv_count) {
      iflex_write_packet_t
          packet(static_cast<uint16_t>(comp_count * 16 / 4 + 1));
      packet.fpga_mask = static_cast<uint8_t>(1u << fpga_num);
      packet.command = Command::CMD_WRITE_MEM;
      packet.data[0].l = ram_address[fpga_num];

      if (fv_count > 16) {
        for (uint16_t i = 0; i < comp_count; i++) {
          for (uint16_t j = 0; j < 16; j++) {
            packet.data[(i << 2) + (j >> 2) + 1].b[3 - (j & 0x3)] =
                vectors[fpga_num * 16 + j][i];
          }
        }

        fv_count -= 16;
      } else {
        for (uint16_t i = 0; i < comp_count; i++) {
          for (uint16_t j = 0; j < 16; j++) {
            if (j < fv_count) {
              packet.data[(i << 2) + (j >> 2) + 1].b[3 - (j & 0x3)] =
                  vectors[fpga_num * 16 + j][i];
            }
          }
        }

        fv_count = 0;
      }

      ram_address[fpga_num] += comp_count;

      packet.transaction_id = GetTransactionId();
      packet.crc = calc_crc(packet);

#ifdef DEBUG_MODE_ENABLED
      for (uint8_t i = 0; i < FPGA_COUNT; i++) {
        m_packet_stats.packet_expected[i] += (packet.fpga_mask >> i) & 0x01u;
      }
#endif

      this->__write(m_df_write, packet.value().get(), packet.length());
    } else {
      break;
    }
  }
}

//create a random dataset of vectors
void IFlexNode::fillVectorList(vector8_list_t &vector_list,
                               uint64_t vector_count, uint64_t comp_count,
                               uint8_t lower, uint8_t upper) {
  vector_list.resize(vector_count);
  for (auto &vector : vector_list) {
    vector.resize(comp_count);
    std::generate(vector.begin(), vector.end(), [&]
    {
        return std::rand() % upper;
    });
  }
}

}  // namespace IFlex
