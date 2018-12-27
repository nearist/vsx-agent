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
#include "MasterNode.h"

namespace IFlex {
MasterNode::MasterNode() :
#ifdef DEBUG_MODE_ENABLED
    debug_fmt("%s [%02d]: P: %10s of %10s\tR: %10s of %10s%s"),
    rt_debug_status(std::bind(&MasterNode::debug_status,
                              this,
                              std::placeholders::_1)),
#endif
    rt_log_temperature(std::bind(&MasterNode::log_temperature,
                                 this,
                                 std::placeholders::_1)) {
  m_children.clear();

  m_distance_mode = DistanceMode::NO_DISTANCE_MODE;
  m_query_mode = QueryMode::NO_QUERY_MODE ;

  m_capacity.volume = 0;
  m_capacity.fv_count = 0;
}

MasterNode::~MasterNode() {
  for (auto child : m_children) {
    delete child;
  }
}

std::vector<INode *> &MasterNode::getChildren() {
  return m_children;
}

const std::string &MasterNode::getApiKey() const {
  return m_api_key;
}

void MasterNode::setApiKey(const std::string &m_api_key) {
  MasterNode::m_api_key = m_api_key;
}

void MasterNode::setDatasetNearist(const std::string &dataset_nearist) {
  MasterNode::m_dataset_nearist = dataset_nearist;
}

void MasterNode::setDatasetUser(const std::string &dataset_user) {
  MasterNode::m_dataset_user = dataset_user;
}

void MasterNode::setTemperatureLogEnable(bool enable) {
  MasterNode::m_temperature_log_enable = enable;
}

void MasterNode::setTemperatureLogRate(uint8_t rate) {
  MasterNode::m_temperature_log_rate = rate;
}

void MasterNode::setComponentSize(uint8_t m_component_size) {
  MasterNode::m_component_size = m_component_size;
}

void MasterNode::open() {
  for (const auto &node : m_children) {
    node->open();
  }
}

void MasterNode::close() {
  for (const auto &node : m_children) {
    node->close();
  }
}

void MasterNode::fillCapacity() {
  for (const auto &node : m_children) {
    m_capacity.volume += node->getCapacity().volume;
    m_capacity.fv_count += node->getCapacity().fv_count;
  }
}

void MasterNode::initTimers() {
#ifdef DEBUG_MODE_ENABLED
  m_th_debug_status = std::thread([&]() {
    rt_debug_status.init(1);
    rt_debug_status.run();
  });
  m_th_debug_status.detach();
#endif

  if (m_temperature_log_enable) {
    m_th_log_temperature = std::thread([&]() {
      rt_log_temperature.init(m_temperature_log_rate);
      rt_log_temperature.run();
    });
    m_th_log_temperature.detach();
  }
}

void MasterNode::reset() {
  std::unique_lock<std::mutex> lock(node_mtx);
  std::vector<std::future<void>> futures;

  Logger::debug(TAG, "reset() begin.");

  m_read_count = 0;

  for (auto node : m_children) {
    futures.emplace_back(std::async(std::launch::async, [node] {
      node->reset();
    }));
  }

  for (auto &e : futures) {
    e.get();
  }

  Logger::debug(TAG, "reset() end.");
}

void MasterNode::setDistanceMode(const DistanceMode mode) {
  std::unique_lock<std::mutex> lock(node_mtx);
  std::vector<std::future<void>> futures;

  Logger::debug(TAG, "setDistanceMode() begin.");

  m_distance_mode = mode;

  for (auto node : m_children) {
    futures.emplace_back(std::async(std::launch::async, [&, node] {
      node->setDistanceMode(MasterNode::m_distance_mode);
    }));
  }

  for (auto &e : futures) {
    e.get();
  }

  Logger::debug(TAG, "setDistanceMode() end.");
}

void MasterNode::setQueryMode(const QueryMode mode) {
  std::unique_lock<std::mutex> lock(node_mtx);
  std::vector<std::future<void>> futures;

  Logger::debug(TAG, "setQueryMode() begin.");

  m_query_mode = mode;

  for (auto node : m_children) {
    futures.emplace_back(std::async(std::launch::async, [&, node] {
      node->setQueryMode(MasterNode::m_query_mode);
    }));
  }

  for (auto &e : futures) {
    e.get();
  }

  Logger::debug(TAG, "setQueryMode() end.");
}

void MasterNode::setReadCount(const uint16_t count) {
  std::unique_lock<std::mutex> lock(node_mtx);
  std::vector<std::future<void>> futures;

  Logger::debug(TAG, "setReadCount() begin.");

  m_read_count = count;

  for (auto node : m_children) {
    futures.emplace_back(std::async(std::launch::async, [&, node] {
      node->setReadCount(MasterNode::m_read_count);
    }));
  }

  for (auto &e : futures) {
    e.get();
  }

  Logger::debug(TAG, "setReadCount() end.");
}

void MasterNode::setThreshold(const uint32_t threshold) {
  std::unique_lock<std::mutex> lock(node_mtx);
  std::vector<std::future<void>> futures;

  Logger::debug(TAG, "setThreshold() begin.");

  m_threshold_lower = threshold;

  for (auto node : m_children) {
    futures.emplace_back(std::async(std::launch::async, [&, node] {
      node->setThreshold(MasterNode::m_threshold_lower);
    }));
  }

  for (auto &e : futures) {
    e.get();
  }

  Logger::debug(TAG, "setThreshold() end.");
}

void MasterNode::setThreshold(const uint32_t threshold_lower,
                              const uint32_t threshold_upper) {
  std::unique_lock<std::mutex> lock(node_mtx);
  std::vector<std::future<void>> futures;

  Logger::debug(TAG, "setThreshold() begin.");

  m_threshold_lower = threshold_lower;
  m_threshold_upper = threshold_upper;

  for (auto node : m_children) {
    futures.emplace_back(std::async(std::launch::async, [&, node] {
      node->setThreshold(MasterNode::m_threshold_lower,
                         MasterNode::m_threshold_upper);
    }));
  }

  for (auto &e : futures) {
    e.get();
  }

  Logger::debug(TAG, "setThreshold() end.");
}

void MasterNode::dsLoadRandom(const uint64_t vector_count, const  uint64_t comp_count) {
  std::unique_lock<std::mutex> lock(node_mtx);
  Logger::debug(TAG, "dsLoadRandom() begin.");

  if (!this->isDataSetValid(vector_count, comp_count, this->m_fv_info)) {
    Logger::error(TAG, "Data Set size {}x{} not supported.", vector_count, comp_count);
    throw DataSetSizeNotSupportedException();
  }

  Logger::info(TAG, "Data Set size: {}x{}", vector_count, comp_count);

  if (!m_children.empty()) {
    std::vector<std::future<void>> futures;

    uint64_t position = 0;
    uint64_t node_id = 0;
    for (auto node : m_children) {
      if (node_id < m_fv_info.node_count) {
        futures.emplace_back(std::async(std::launch::async, [&, node, position, node_id] {
            node->dsLoadRandom(position, m_fv_info.per_node[node_id], comp_count);
        }));
        position += m_fv_info.per_node[node_id];
      } else {
        futures.emplace_back(std::async(std::launch::async, [&, node] {
            node->resetTimer();
            node->reset();
            node->setDistanceMode(m_distance_mode);
            node->setQueryMode(m_query_mode);
            node->setReadCount(m_read_count);
            node->setThreshold(m_threshold_lower, m_threshold_upper);
        }));
      }
      node_id++;
    }

    for (auto &e : futures) {
      e.get();
    }
  } else {
    Logger::critical(TAG,
                     "There are no any nodes connected to Nearist system.");
  }
  Logger::debug(TAG, "dsLoadRandom() end.");

}

void MasterNode::dsLoadFromFile(const std::string &fileName,
                        const std::string &datasetName) {
  std::unique_lock<std::mutex> lock(node_mtx);
  Logger::debug(TAG, "dsLoadFromFile() begin.");

  std::string dataset_file_name = normalize_file_name(fileName);

  if (!check_dataset_file_exists(dataset_file_name)) {
    throw FileNotFoundException(fileName);
  }

  HighFive::File file(dataset_file_name, HighFive::File::ReadOnly);
  if(!file.exist(datasetName)) {
    throw DataSetNotFoundException(datasetName);
  }

  HighFive::DataSet ds = file.getDataSet(datasetName);

  Logger::info(TAG, "Dataset filename: {}", dataset_file_name);
  Logger::info(TAG, "Dataset name: {}", datasetName);

  auto dims = ds.getSpace().getDimensions();

  auto vector_count = dims[0];
  auto comp_count = dims[1];

  if (!this->isDataSetValid(vector_count, comp_count, this->m_fv_info)) {
    throw DataSetSizeNotSupportedException();
  }

  Logger::info(TAG, "Data Set size: {}x{}", vector_count, comp_count);

  H5Fclose(file.getId());

  if (!m_children.empty()) {
    std::vector<std::future<void>> futures;

    uint64_t position = 0;
    uint64_t node_id = 0;
    for (auto node : m_children) {
      if (node_id < m_fv_info.node_count) {
        futures.emplace_back(std::async(std::launch::async, [&, node, position, node_id] {
          node->dsLoadFromFile(dataset_file_name, datasetName,
                       position, m_fv_info.per_node[node_id]);
        }));
        position += m_fv_info.per_node[node_id];
      } else {
        futures.emplace_back(std::async(std::launch::async, [&, node] {
          node->resetTimer();
          node->reset();
          node->setDistanceMode(m_distance_mode);
          node->setQueryMode(m_query_mode);
          node->setReadCount(m_read_count);
          node->setThreshold(m_threshold_lower, m_threshold_upper);
        }));
      }
      node_id++;
    }

    for (auto &e : futures) {
      e.get();
    }
  } else {
    Logger::critical(TAG,
                     "There are no any nodes connected to Nearist system.");
  }
  Logger::debug(TAG, "dsLoadFromFile() end.");
}

void MasterNode::dsLoad(vector8_list_t &vectors) {
  std::unique_lock<std::mutex> lock(node_mtx);
  Logger::debug(TAG, "dsLoad() begin.");

  this->normalizeVectors(vectors);

  auto vector_count = vectors.size();
  auto comp_count = vectors.front().size();

  if (!this->isDataSetValid(vector_count, comp_count, this->m_fv_info)) {
    throw DataSetSizeNotSupportedException();
  }

  Logger::info(TAG, "Data Set size: {}x{}", vector_count, comp_count);

  if (!m_children.empty()) {
    auto position = vectors.begin();

    std::vector<std::future<void>> futures;
    uint64_t node_id = 0;
    for (auto node : m_children) {
      if (node_id < m_fv_info.node_count) {
        auto offset = (uint32_t) std::distance(vectors.begin(), position);
        auto end = std::min(position + this->m_fv_info.per_node[node_id],
                            vectors.end());

        auto arg = vector8_list_t(position, end);

        node->dsLoad(offset, arg);

        position = end;
      } else {
        futures.emplace_back(std::async(std::launch::async, [&, node] {
          node->resetTimer();
          node->reset();
          node->setDistanceMode(m_distance_mode);
          node->setQueryMode(m_query_mode);
          node->setReadCount(m_read_count);
          node->setThreshold(m_threshold_lower, m_threshold_upper);
        }));
      }

      node_id++;
    }
  } else {
    Logger::critical(TAG,
                     "There are no any nodes connected to Nearist system.");
  }
  Logger::debug(TAG, "dsLoad() end.");
}

void
MasterNode::query(vector8_list_t &vectors,
                  vector_result32_list_t &results) {
  std::unique_lock<std::mutex> lock(node_mtx);
  Logger::debug(TAG, "query() begin.");

  this->normalizeVectors(vectors);

  if (!this->isQueryValid(vectors)) {
    throw QuerySizeNotSupportedException();
  }

  Logger::info(TAG, "Query vectors count: {}", vectors.size());
  Logger::info(TAG, "Distance mode: {}", m_distance_mode);
  Logger::info(TAG, "Query mode: {}", m_query_mode);

  switch (m_query_mode) {
    case QueryMode::ALL:
    case QueryMode::KNN_D:
    case QueryMode::KNN_A:
      Logger::info(TAG,
                   "Query read count: {}",
                   m_read_count);
      break;
    case QueryMode::GT:
    case QueryMode::LT:
    case QueryMode::EQ:
      Logger::info(TAG,
                   "Query threshold: {}",
                   m_threshold_lower);
      break;
    case QueryMode::RANGE:
      Logger::info(TAG,
                   "Query threshold lower: {}",
                   m_threshold_lower);
      Logger::info(TAG, "Query threshold upper: {}", m_threshold_upper);
      break;
    case QueryMode::NO_QUERY_MODE:
      break;
  }

  if (!m_children.empty()) {
    std::vector<vector_result32_list_t> node_results;
    std::vector<std::future<vector_result32_list_t>>
        futures;

    uint64_t node_id = 0;
    for (auto node : m_children) {
      futures.emplace_back(std::async(std::launch::async, [node, vectors] {
        vector_result32_list_t dr;
        node->query(vectors, dr);
        return dr;
      }));

      node_id++;

      if (node_id >= this->m_fv_info.node_count) {
        break;
      }
    }

    for (auto &e : futures) {
      auto drs = e.get();
      node_results.emplace_back(vector_result32_list_t(drs.begin(),
                                                       drs.end()));
    }

    Logger::debug(TAG, "Sort & merge...");
    auto tm_sort_all = TimeMeter();
    for (auto node_result : node_results) {
      uint64_t vector_id = 0;

      for (auto &it : node_result) {
        auto tm_merge = TimeMeter();

        if (vector_id >= results.size()) {
          results.emplace_back(vector_result32_t());
        }

        vector_result32_t vector_result;

        std::merge(
            results[vector_id].begin(), results[vector_id].end(),
            it.begin(), it.end(),
            std::back_inserter(vector_result),
            KNNComparator(MasterNode::m_query_mode)
        );

        results[vector_id] = vector_result;

        vector_id++;

        m_duration_sort += tm_merge.nanoseconds();
      }
    }
    Logger::debug(TAG, "Batch Sort time {} us", tm_sort_all.microseconds());

    if ((m_query_mode == QueryMode::KNN_D || m_query_mode == QueryMode::KNN_A) && m_read_count != 0) {
      auto tm_erase = TimeMeter();
      for (auto &result : results) {
        if (result.size() > m_read_count) {
          result.erase(result.begin() + m_read_count, result.end());
        }
      }
      Logger::debug(TAG, "KNN erase time {} us", tm_erase.microseconds());
    }

    Logger::debug(TAG, "Sort & merge end.");
  } else {
    Logger::critical(TAG, "There are no any iFlex device connected to host system.");
  }
  Logger::debug(TAG, "query() end.");
}

void MasterNode::queryFromFile(const std::string &fileName,
                               const std::string &datasetName,
                               const std::string &output,
                               const bool overwrite) {
  Logger::debug(TAG, "queryFromFile() begin.");

  std::string query_file_name = normalize_file_name(fileName);
  std::string output_file_name = normalize_file_name(output);

  if (!overwrite && !check_output_file_not_exists(output_file_name)) {
    throw OutputFileExistsException(fileName);
  }

  if (check_dataset_file_exists(query_file_name)) {
    try {
      HighFive::File file(query_file_name, HighFive::File::ReadOnly);

      vector8_list_t data;
      vector_result32_list_t results;

      if(m_component_size == 1) {
        file.getDataSet(datasetName).read(data);
      } else if(m_component_size == 4) {
        vector32_list_t vectors;
        file.getDataSet(datasetName).read(vectors);

        data = vector32_list_2_vector8_list(vectors);
      }

      this->query(data, results);

      write_results_to_file(output_file_name, results);

    } catch (HighFive::Exception &ex) {
      throw DataSetNotFoundException(datasetName);
    }
  } else {
    throw FileNotFoundException(fileName);
  }
  Logger::debug(TAG, "queryFromFile() end.");
}

void MasterNode::deserializeVectors(Request *request,
                                    vector8_list_t &vectors) {
  uint64_t length = request->getHeader().body_length;
  uint64_t vector_length = request->getHeader().attribute_0;
  uint8_t *body = request->getBody().data.get();

  for (uint64_t i = 0; i < length; i += vector_length) {
    vector8_t vector(body + i, body + i + vector_length);
    vectors.emplace_back(vector);
  }
}

void MasterNode::serializeResults(Response *response,
                                  const vector_result32_list_t &results,
                                  bool batch) {
  size_t v_size = 0;
  for (const auto &vector : results) {
    v_size = std::max(v_size, vector.size());
  }

  size_t size = results.size() * (v_size + 1) * 2;
  auto body = new uint64_t[size]();

  size_t i = 0;
  for (auto vector : results) {
    for (result32_t result : vector) {
      body[i++] = result.ds_id;
      body[i++] = (uint64_t) result.distance;
    }

    body[i++] = 0xFFFFFFFFFFFFFFFF;
    body[i++] = 0xFFFFFFFFFFFFFFFF;
  }

  if (!batch) {
    response->getHeader().attribute_1 = 0;
  } else {
    response->getHeader().attribute_1 = 1;
  }

  response->getHeader().body_length = size * sizeof(uint64_t);
  response->getBody().data.reset(body, std::default_delete<uint64_t[]>());
}

void MasterNode::resetTimer() {
  std::unique_lock<std::mutex> lock(node_mtx);
  std::vector<std::future<void>> futures;

  Logger::debug(TAG, "resetTimer() begin.");

  m_duration_sort = 0;

  for (auto node : m_children) {
    futures.emplace_back(std::async(std::launch::async, [node] {
      node->resetTimer();
    }));
  }

  for (auto &e : futures) {
    e.get();
  }

  Logger::debug(TAG, "resetTimer() end.");
}

uint64_t MasterNode::getTimerValue() {
  std::unique_lock<std::mutex> lock(node_mtx);

  std::vector<std::future<uint64_t>> futures;

  uint64_t duration = 0;

  for (auto node : m_children) {
    futures.emplace_back(std::async(std::launch::async, [node] {
      return node->getTimerValue();
    }));
  }

  for (auto &e : futures) {
    duration += e.get();
  }

  return duration / m_children.size() + m_duration_sort;
}

std::map<std::string, std::map<uint8_t, uint16_t>> MasterNode::getTemperature() {
  std::unique_lock<std::mutex> lock(node_mtx, std::try_to_lock);

  std::map<std::string, std::map<uint8_t, uint16_t>> result;
  std::vector<std::future<std::pair<std::string, std::map<uint8_t, uint16_t>>>> futures;

  bool force = lock.owns_lock();

  for (auto node : m_children) {
    futures.emplace_back(std::async(std::launch::async, [node, force] {
      return std::make_pair(node->getConfig().getDeviceFileRead(), node->getTemperature(force));
    }));
  }

  for (auto &e : futures) {
    result.insert(e.get());
  }

  return result;
}

bool MasterNode::isDataSetValid(const uint64_t vector_count,
                                const uint64_t comp_count,
                                fv_info_t &info) {
  fv_info_t new_info;

  bool valid = false;

  // Store the number of vectors.
  new_info.vector_count = vector_count;

  Logger::debug(TAG, "new_info.vector_count: {}", new_info.vector_count);
  Logger::debug(TAG, "new_info.comp_count: {}", new_info.comp_count);
  Logger::debug(TAG, "new_info.vector_count: {}", new_info.vector_count);
  Logger::debug(TAG, "m_capacity.fv_count: {}", m_capacity.fv_count);
  Logger::debug(TAG, "m_capacity.volume: {}", m_capacity.volume);
  Logger::debug(TAG, "new_info.vector_count * new_info.comp_count: {}", new_info.vector_count * new_info.comp_count);

  // Invalid if vector count is 0.
  if (new_info.vector_count > 0) {
    // Store the vector length.
    new_info.comp_count = comp_count;

    // Vector length must be at least 2, and less than or equal to
    // the maximum supported length (currently 2048).
    if (new_info.comp_count >= 2 && new_info.comp_count <= CP_COUNT) {
      // Store the number of cards in this configuration.
      new_info.node_count = m_children.size();
      new_info.per_node = std::vector<uint64_t>(new_info.node_count);

      // Vector count must be less than the max per card
      // (currently 16M), and less than the total system capacity.
      if (new_info.vector_count <= m_capacity.fv_count &&
          new_info.vector_count * new_info.comp_count <= m_capacity.volume) {
        uint64_t fv_count = new_info.vector_count;

        // Calculate the maximum number of vectors stored per
        // card. (Some cards may have one less than this number).
        uint64_t fv_count_per_node = (uint64_t) std::ceil(
            (double) new_info.vector_count / new_info.node_count);

        // Calculate the actual number of vectors to store per card.
        // Example: 100 vectors would be distributed across 3
        //          cards as [34, 34, 32]
        for (uint64_t i = 0; i < new_info.node_count; i++) {
          new_info.per_node[i] = std::min(fv_count, fv_count_per_node);
          fv_count -= fv_count_per_node;
        }

        info = new_info;
        valid = true;
      }
    }
  }

  return valid;
}

bool MasterNode::isQueryValid(const vector8_list_t &vectors) {
  return true;
//  (this->m_fv_info.comp_count > 0 && this->m_fv_info.vector_count > 0) &&
//  (!vectors.empty() && vectors.front().size() == this->m_fv_info.comp_count);
}

void MasterNode::normalizeVectors(vector8_list_t &vectors) {
  if (!vectors.empty()) {
    uint64_t comp_count = vectors.front().size();
    uint64_t tail = ((comp_count + 4 - 1) / 4) * 4 - comp_count;

    if (tail > 0) {
      const uint8_t zero[CP_COUNT] = {};

      for (auto it = vectors.begin(); it != vectors.end(); ++it) {
        (*it).insert(it->end(), zero, zero + tail);
      }
    }
  }
}

#ifdef DEBUG_MODE_ENABLED
void MasterNode::debug_status(bool first_loop) {
  if (!m_children.empty()) {
    std::string status;

    uint32_t line_count = 0;

    for (auto node : m_children) {
      auto node_sub_count = node->getSubNodeCount();
      auto node_packet_stat = node->get_packet_stats();

      line_count += node_sub_count;

      for (uint32_t i = 0; i < node_sub_count; i++) {
        status.append(boost::str(debug_fmt
                                     % node_packet_stat.source
                                     % (uint32_t) i
                                     % node_packet_stat.packet_received[i]
                                     % node_packet_stat.packet_expected[i]
                                     % node_packet_stat.result_received[i]
                                     % node_packet_stat.result_expected[i]
                                     % (i != node_sub_count - 1 ? "\n" : "")
                      )
        );
      }
      status.append("\n");
    }

    if (first_loop) {
      printf("%s", status.c_str());
    } else {
      printf("\033[%uA%s", line_count, status.c_str());
    }
    fflush(stdout);
  }
}
#endif

void MasterNode::log_temperature(bool first_loop) {
  if (!m_children.empty()) {
    auto temp_list = getTemperature();
    Logger::temperature(TAG, temp_list);
  }
}

std::vector<std::string> MasterNode::get_dataset_files() {
  std::vector<std::string> files;
  try {
    for (auto path : {m_dataset_nearist, m_dataset_user}) {
      boost::filesystem::path p(path);
      if (boost::filesystem::exists(p)) {
        if (boost::filesystem::is_regular_file(p) && p.extension() == ".h5") {
          files.emplace_back(p.c_str());
        } else {
          for (boost::filesystem::directory_entry &x :
              boost::filesystem::recursive_directory_iterator(p)) {
            if (boost::filesystem::is_regular_file(x)
                && x.path().extension() == ".h5") {
              files.emplace_back(x.path().c_str());
            }
          }
        }
      } else {
        Logger::error(TAG, "{} does not exist.", p);
      }
    }
  } catch (const boost::filesystem::filesystem_error &ex) {
    Logger::error(TAG, "{}", ex.what());
  }

  return files;
}

std::string MasterNode::normalize_file_name(const std::string &file_name) {
  std::string final_name = std::string(file_name);

  boost::algorithm::replace_first(final_name, "/user", m_dataset_user);
  boost::algorithm::replace_first(final_name, "/nearist", m_dataset_nearist);
  boost::algorithm::replace_all(final_name, "//", "/");

  return final_name;
}

bool MasterNode::check_dataset_file_exists(const std::string &file_name) {
  auto data_files = this->get_dataset_files();
  if (file_name.empty()) {
    return false;
  } else if (!boost::algorithm::starts_with(file_name, m_dataset_user)
      && !boost::algorithm::starts_with(file_name, m_dataset_nearist)) {
    return false;
  } else {
    return std::find(data_files.begin(), data_files.end(), file_name)
        != data_files.end();
  }
}

bool MasterNode::check_output_file_not_exists(const std::string &file_name) {
  return !(check_dataset_file_exists(file_name)
      && boost::algorithm::starts_with(file_name, m_dataset_user));
}

void MasterNode::write_results_to_file(const std::string &file_name,
                                       const vector_result32_list_t &results) {
  std::vector<std::vector<int32_t>> ids;
  std::vector<std::vector<int32_t>> dists;

  uint64_t max_length = 0;

  for (auto vector : results) {
    std::vector<int32_t> id;
    std::vector<int32_t> dist;
    for (result32_t result : vector) {
      id.emplace_back(result.ds_id);
      dist.emplace_back(result.distance);
    }

    if(max_length < id.size()) {
      max_length = id.size();
    }

    ids.emplace_back(std::move(id));
    dists.emplace_back(std::move(dist));
  }

  for(auto &id : ids) {
    id.resize(max_length, -1);
  }
  for(auto &dist : dists) {
    dist.resize(max_length, -1);
  }

  boost::filesystem::create_directories(boost::filesystem::path(file_name)
                                            .parent_path());

  HighFive::File output_file(file_name,
                             (HighFive::File::ReadWrite
                                 | HighFive::File::Create
                                 | HighFive::File::Truncate));

  HighFive::DataSet dataset_ids = output_file
      .createDataSet<int32_t>("ids", HighFive::DataSpace::From(ids));
  HighFive::DataSet dataset_dists = output_file
      .createDataSet<int32_t>("dists", HighFive::DataSpace::From(dists));

  dataset_ids.write(ids);
  dataset_dists.write(dists);
}
}  // namespace IFlex
