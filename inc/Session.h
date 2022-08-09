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
* File description: Master's TCP Session module. Parse requests dispatch to MaterNode and build response.
*/
#ifndef TRUNK_SW_INC_SESSION_H_
#define TRUNK_SW_INC_SESSION_H_

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/bind.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/make_shared.hpp>
#include "json/json.h"
#include "Logger.h"
#include "Request.h"
#include "Response.h"
#include "MasterNode.h"

namespace IFlex {
#ifdef SSL_ENABLED
typedef boost::asio::ssl::stream<boost::asio::ip::tcp::socket> ssl_socket;
#endif

class Session {
  const std::string TAG = "TCP::Session";
 public:
#ifdef SSL_ENABLED
  Session(boost::asio::io_service &io_service, boost::asio::ssl::context &context, std::shared_ptr<MasterNode> master_node);

  ssl_socket::lowest_layer_type &socket();
#else
  Session(boost::asio::io_service &io_service,
          std::shared_ptr<MasterNode> master_node);

  boost::asio::ip::tcp::socket &socket();
#endif

  void start();

 private:
#ifdef SSL_ENABLED
  void handle_handshake(const boost::system::error_code &error);
#endif

  void handle_write(const boost::system::error_code &error,
                    size_t bytes_transferred,
                    std::shared_ptr<Response> response);

  void handle_header_read(const boost::system::error_code &error,
                          size_t bytes_transferred,
                          std::shared_ptr<Request> request);

  void handle_read(const boost::system::error_code &error,
                   size_t bytes_transferred,
                   std::shared_ptr<Request> request);

  void async_write_response(std::shared_ptr<Response> response);

  void async_write_invalid_checksum_response(std::shared_ptr<Request> request);

  void async_write_invalid_api_key_response(std::shared_ptr<Request> request);

  void async_read_request_header();

  void async_read_request_body(std::shared_ptr<Request> request);

  void dispatch(std::shared_ptr<Request> request,
                std::shared_ptr<Response> response);

 private:
  std::mutex mtx_dispatch;

#ifdef SSL_ENABLED
  ssl_socket m_socket;
#else
  boost::asio::ip::tcp::socket m_socket;
#endif

  std::shared_ptr<MasterNode> m_master_node;
};
}  // namespace IFlex
#endif  // TRUNK_SW_INC_SESSION_H_
