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
* File description: Master TCP acceptor module
*/

#ifndef TRUNK_SW_INC_MASTER_H_
#define TRUNK_SW_INC_MASTER_H_

#include <string>
#include "Logger.h"
#include "Session.h"
#include "iFlexNode.h"
#include "TCPNode.h"

namespace IFlex {
class Master {
  const std::string TAG = "TCP::Master";

#ifdef SSL_ENABLED
  const std::string TLS_CHAIN = "pem/server.pem";
  const std::string TLS_PRIVATE = "pem/server.pem";
  const std::string TLS_DH = "pem/dh2048.pem";
#endif

 public:
  Master();

  ~Master() = default;

  void load_config(const std::string &filename);

 private:
#ifdef SSL_ENABLED
  std::string get_password() const;
#endif

  void start_accept();

  void handle_accept(std::shared_ptr<Session> new_session,
                     const boost::system::error_code &error);

 private:
  boost::asio::io_service m_io_service;
  boost::asio::ip::tcp::acceptor m_acceptor;

#ifdef SSL_ENABLED
  boost::asio::ssl::context m_context;
#endif

  AppConfig m_config;

  std::shared_ptr<MasterNode> m_master_node;
};
}  // namespace IFlex

#endif  // TRUNK_SW_INC_MASTER_H_
