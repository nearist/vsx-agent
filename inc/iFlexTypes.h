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
* File description: General Types and Structures.
*/
#ifndef TRUNK_SW_INC_IFLEXTYPES_H_
#define TRUNK_SW_INC_IFLEXTYPES_H_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <spdlog/fmt/bundled/format.h>

namespace IFlex {
enum DistanceMode {
  NO_DISTANCE_MODE = 0xFFFF,
  L1 = 0x0000,
  LMAX = 0x0001,
  HAMMING = 0x0002,
  BIT_AND = 0x0003,
  BIT_OR = 0x0004,
  JACCARD = 0x0005,
  L2 = 0x0006
};

enum QueryMode {
  NO_QUERY_MODE = 0xFFFF,
  ALL = 0x0000,
  KNN_A = 0x0001,
  KNN_D = 0x0002,
  GT = 0x0003,
  LT = 0x0004,
  EQ = 0x0005,
  RANGE = 0x0006
};

enum Command : uint32_t {
  RESET = 0x00,
  DISTANCE_MODE = 0x01,
  QUERY_MODE = 0x02,
  READ_COUNT = 0x03,
  THRESHOLD = 0x04,
  DS_LOAD = 0x05,
  QUERY = 0x06,
  RESET_TIMER = 0x10,
  GET_TIMER = 0x11,
};

enum Status : uint32_t {
  SUCCESS = 0x00,
  INVALID_SEQUENCE = 0x01,
  INVALID_ARGUMENT = 0x02,
  INVALID_PACKET = 0x03,
  NOT_SUPPORTED = 0x04,
  INVALID_COMMAND = 0x05,
  INVALID_DATA = 0x06,
  TIMEOUT = 0x07,
  INVALID_CHECKSUM = 0x08,
  INVALID_API_KEY = 0x09,
  DATASET_FILE_NOT_FOUND = 0x20,
  DATASET_NOT_FOUND = 0x21,
  DATASET_SIZE_NOT_SUPPORTED = 0x22,
  QUERY_SIZE_NOT_SUPPORTED = 0x23,
  DISTANCE_MODE_NOT_SUPPORTED = 0x24,
  QUERY_MODE_NOT_SUPPORTED = 0x25,
  READ_COUNT_NOT_SUPPORTED = 0x26
};

enum NodeType : uint8_t {
  DEVICE = 0x00,
  TCP = 0x01
};

enum DeviceRevision : uint8_t {
  SIMULATOR = 0x00,
  IFLEX_1_0 = 0x06
};

template<typename T>
struct Result {
  uint64_t ds_id;
  T distance;

 public:
  Result() : ds_id(0), distance(0) {}
  Result(uint64_t ds_id, T distance) : ds_id(ds_id), distance(distance) {}
};

typedef struct {
  uint64_t vector_count;
  uint64_t comp_count;
  uint64_t node_count;
  std::vector<uint64_t> per_node;
} fv_info_t;

typedef struct {
  uint64_t volume;
  uint64_t fv_count;
} capacity_t;

typedef union {
  uint32_t l;
  uint16_t w[2];
  uint8_t b[4];
} ulong_t;

typedef union {
  uint64_t l;
  uint32_t d[2];
  uint16_t w[4];
  uint8_t b[8];
} qword_t;

inline std::string DistanceMode2String(const DistanceMode mode) {
  std::string str;
  switch (mode) {
    case DistanceMode::NO_DISTANCE_MODE:
      str = std::string("NO_DISTANCE_MODE");
      break;
    case DistanceMode::L1:
      str = std::string("L1");
      break;
    case DistanceMode::LMAX:
      str = std::string("LMAX");
      break;
    case DistanceMode::HAMMING:
      str = std::string("HAMMING");
      break;
    case DistanceMode::BIT_AND:
      str = std::string("BIT_AND");
      break;
    case DistanceMode::BIT_OR:
      str = std::string("BIT_OR");
      break;
    case DistanceMode::JACCARD:
      str = std::string("JACCARD");
      break;
  }

  return str;
}

inline std::string QueryMode2String(const QueryMode mode) {
  std::string str;
  switch (mode) {
    case QueryMode::NO_QUERY_MODE:
      str = std::string("NO_QUERY_MODE");
      break;
    case QueryMode::ALL:
      str = std::string("ALL");
      break;
    case QueryMode::KNN_A:
      str = std::string("KNN_A");
      break;
    case QueryMode::KNN_D:
      str = std::string("KNN_D");
      break;
    case QueryMode::GT:
      str = std::string("GT");
      break;
    case QueryMode::LT:
      str = std::string("LT");
      break;
    case QueryMode::EQ:
      str = std::string("EQ");
      break;
    case QueryMode::RANGE:
      str = std::string("RANGE");
      break;
  }

  return str;
}

inline std::string Command2String(Command command) {
  std::string str;

  switch (command) {
    case RESET:
      str = "RESET";
      break;
    case DISTANCE_MODE:
      str = "DISTANCE_MODE";
      break;
    case QUERY_MODE:
      str = "QUERY_MODE";
      break;
    case READ_COUNT:
      str = "READ_COUNT";
      break;
    case THRESHOLD:
      str = "THRESHOLD";
      break;
    case DS_LOAD:
      str = "DS_LOAD";
      break;
    case QUERY:
      str = "QUERY";
      break;
    case RESET_TIMER:
      str = "RESET_TIMER";
      break;
    case GET_TIMER:
      str = "GET_TIMER";
      break;
    default:
      str = fmt::format("UNKNOWN_COMMAND 0x{0:0>2X}", command);
      break;
  }

  return str;
}

inline std::string Status2String(Status status) {
  std::string str;

  switch (status) {
    case SUCCESS:
      str = "Success";
      break;
    case INVALID_SEQUENCE:
      str = "Invalid sequence";
      break;
    case INVALID_ARGUMENT:
      str = "Invalid argument";
      break;
    case INVALID_PACKET:
      str = "Invalid packet";
      break;
    case NOT_SUPPORTED:
      str = "Not supported";
      break;
    case INVALID_COMMAND:
      str = "Invalid command";
      break;
    case INVALID_DATA:
      str = "Invalid data";
      break;
    case TIMEOUT:
      str = "Timeout";
      break;
    case INVALID_CHECKSUM:
      str = "Invalid checksum";
      break;
    case INVALID_API_KEY:
      str = "Invalid key";
      break;
    case DATASET_FILE_NOT_FOUND:
      str = "DATASET_FILE_NOT_FOUND";
      break;
    case DATASET_NOT_FOUND:
      str = "DATASET_NOT_FOUND";
      break;
    case DATASET_SIZE_NOT_SUPPORTED:
      str = "DATASET_SIZE_NOT_SUPPORTED";
      break;
    case QUERY_SIZE_NOT_SUPPORTED:
      str = "QUERY_SIZE_NOT_SUPPORTED";
      break;
    case DISTANCE_MODE_NOT_SUPPORTED:
      str = "DISTANCE_MODE_NOT_SUPPORTED";
      break;
    case QUERY_MODE_NOT_SUPPORTED:
      str = "QUERY_MODE_NOT_SUPPORTED";
      break;
    case READ_COUNT_NOT_SUPPORTED:
      str = "READ_COUNT_NOT_SUPPORTED";
      break;
    default:
      str = fmt::format("UNKNOWN_STATUS 0x{0:0>2X}", status);
      break;
  }

  return str;
}

class DataSetNotFoundException : public std::runtime_error {
 public:
  explicit DataSetNotFoundException(const std::string &datasetName)
      : std::runtime_error("Dataset '" + datasetName + "' not found.") {}
};

class FileNotFoundException : public std::runtime_error {
 public:
  explicit FileNotFoundException(const std::string &fileName)
      : std::runtime_error("Dataset file '" + fileName + "' not found.") {}
};

class OutputFileExistsException : public std::runtime_error {
 public:
  explicit OutputFileExistsException(const std::string &fileName)
      : std::runtime_error("Output file '" + fileName + "' exists.") {}
};

class DistanceModeNotSupportedException : public std::runtime_error {
 public:
  DistanceModeNotSupportedException() : std::runtime_error(
      "Distance mode not supported.") {}
};

class QueryModeNotSupportedException : public std::runtime_error {
 public:
  QueryModeNotSupportedException() : std::runtime_error(
      "Query mode not supported.") {}
};

class ReadCountNotSupportedException : public std::runtime_error {
 public:
  ReadCountNotSupportedException() : std::runtime_error(
      "Read Count must be in range [1, 512].") {}
};

class DataSetSizeNotSupportedException : public std::runtime_error {
 public:
  DataSetSizeNotSupportedException() : std::runtime_error(
      "Dataset size must be in range [1, device_count * 16777216].") {}
};

class QuerySizeNotSupportedException : public std::runtime_error {
 public:
  QuerySizeNotSupportedException() : std::runtime_error(
      "Query vector size must be equal to feature vector size.") {}
};

class TCPException : public std::runtime_error {
 public:
  explicit TCPException(Status status)
      : std::runtime_error(Status2String(status)) {}
};

typedef std::vector<uint8_t> vector8_t;
typedef std::vector<vector8_t> vector8_list_t;

typedef std::vector<uint32_t> vector32_t;
typedef std::vector<vector32_t> vector32_list_t;

typedef Result<uint32_t> result32_t;
typedef std::vector<result32_t> vector_result32_t;
typedef std::vector<vector_result32_t> vector_result32_list_t;

struct KNNComparator {
    explicit KNNComparator(QueryMode mode) { this->mode = mode; }
    bool operator () (const result32_t &left, const result32_t &right) {
      switch (this->mode) {
        case QueryMode::KNN_D:
          return left.distance > right.distance;
        case QueryMode::KNN_A:
          return left.distance < right.distance;
        default:
          return false;
      }
    }

    QueryMode mode;
};

inline vector8_t vector32_2_vector8(const vector32_t &vector32) {
  vector8_t vector8;

  for (const auto &v : vector32) {
    ulong_t data = {.l=v};

    vector8.emplace_back(data.b[0]);
    vector8.emplace_back(data.b[1]);
    vector8.emplace_back(data.b[2]);
    vector8.emplace_back(data.b[3]);
  }

  return vector8;
}

inline vector8_list_t vector32_list_2_vector8_list(const vector32_list_t &vector32_list) {
  vector8_list_t vector8_list;

  for (const auto &v : vector32_list) {
    vector8_list.emplace_back(vector32_2_vector8(v));
  }

  return vector8_list;
}

#ifdef DEBUG_MODE_ENABLED
struct packet_stat_t {
  std::string source;

  std::vector<uint64_t> packet_expected;
  std::vector<uint64_t> packet_received;

  std::vector<uint64_t> result_expected;
  std::vector<uint64_t> result_received;
};
#endif

}  // namespace IFlex

#endif  // TRUNK_SW_INC_IFLEXTYPES_H_
