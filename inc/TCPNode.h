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
* File description: TCP node in multi-master scenario.
*/
#ifndef TRUNK_SW_INC_TCPNODE_H_
#define TRUNK_SW_INC_TCPNODE_H_

#include <string>
#include <vector>
#include <map>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include "iFlexTypes.h"
#include "NodeConfig.h"
#include "INode.h"
#include "Request.h"
#include "Response.h"

namespace IFlex {
#ifdef SSL_ENABLED
typedef boost::asio::ssl::stream<boost::asio::ip::tcp::socket> ssl_socket;
#endif

class TCPNode : public INode {
  const std::string TAG = "TCPNode";

#ifdef SSL_ENABLED
  static const uint8_t TLS_CA[];
#endif

 private:
#ifdef SSL_ENABLED
  bool verify_certificate(bool preverified,
                          boost::asio::ssl::verify_context &ctx);
#endif

  void buffer2Results(Response &response, vector_result32_list_t &results);

  uint8_t *data2Buffer(vector8_list_t vectors);

  void write(Request &request);

  Response read();

 public:
  explicit TCPNode(const NodeConfig &config);

  ~TCPNode() final;

  void open() final;

  void close() final;

  void reset() final;

  void setDistanceMode(IFlex::DistanceMode mode) final;

  void setQueryMode(IFlex::QueryMode mode) final;

  void setReadCount(uint16_t count) final;

  void setThreshold(uint32_t threshold) final;

  void setThreshold(uint32_t threshold_lower,
                    uint32_t threshold_upper) final;

  void dsLoad(uint64_t offset,
              const vector8_list_t &vectors) final;

  void dsLoadFromFile(const std::string &fileName,
              const std::string &datasetName,
              const uint64_t offset,
              const uint64_t count) final {};

  void dsLoadRandom(const uint64_t offset,
              const uint64_t vector_count,
              const uint64_t comp_count) final {};

  void query(const vector8_list_t &vectors,
             vector_result32_list_t &results) final;

  void resetTimer() final;

  uint64_t getTimerValue() final;

  std::map<uint8_t, uint16_t> getTemperature(bool force) final {
    return std::map<uint8_t, uint16_t>();
  }

  uint32_t getSubNodeCount() final { return 0; };

 private:
  boost::asio::io_service m_io_service;
#ifdef SSL_ENABLED
  ssl_socket *m_socket;
#else
  boost::asio::ip::tcp::socket *m_socket;
#endif
};
}  // namespace IFlex

#endif  // TRUNK_SW_INC_TCPNODE_H_
