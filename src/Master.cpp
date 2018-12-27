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
#include "Master.h"

namespace IFlex {

Master::Master()
    : m_io_service(),
      m_acceptor(m_io_service),
      m_master_node(new MasterNode())
#ifdef SSL_ENABLED
, m_context(boost::asio::ssl::context::tlsv12)
#endif
{}

void Master::start_accept() {
#ifdef SSL_ENABLED
  std::shared_ptr<Session> new_session(
      new Session(m_io_service, m_context, m_master_node));
#else
  std::shared_ptr<Session> new_session(
      new Session(m_io_service, m_master_node));
#endif

  m_acceptor.async_accept(new_session->socket(),
                          boost::bind(&Master::handle_accept,
                                      this,
                                      new_session,
                                      boost::asio::placeholders::error));

  m_io_service.run();
}

void Master::handle_accept(std::shared_ptr<Session> new_session,
                           const boost::system::error_code &error) {
  if (!error) {
    new_session->start();
  } else {
    new_session.reset();
  }

  start_accept();
}

void Master::load_config(const std::string &filename) {
  m_config.load(filename);

  Logger::getInstance(m_config.getLogFile(), m_config.getTemperatureLogFile());
  Logger::setLevel(m_config.getLogLevel());

  m_master_node->setApiKey(m_config.getApiKey());
  m_master_node->setDatasetNearist(m_config.getDatasetNearist());
  m_master_node->setDatasetUser(m_config.getDatasetUser());
  m_master_node->setTemperatureLogEnable(m_config.isTemperatureLogEnable());
  m_master_node->setTemperatureLogRate(m_config.getTemperatureLogRate());
  m_master_node->setComponentSize(m_config.getComponentSize());

  for (const NodeConfig &node_config : m_config.getNodes()) {
    INode *node = nullptr;

    switch (node_config.getType()) {
      case NodeType::DEVICE:
        switch (node_config.getRevision()) {
          case IFLEX_1_0:
            node = new IFlexNode(node_config);
            node->getConfig().setComponentSize(m_config.getComponentSize());
            break;
          default:
            break;
        }

        break;
      case NodeType::TCP:
        node = new TCPNode(node_config);
        break;
      default:
        break;
    }

    if (node != nullptr) {
      m_master_node->getChildren().push_back(node);
    }
  }

  m_master_node->open();
  m_master_node->fillCapacity();
  m_master_node->initTimers();

  boost::asio::ip::tcp::resolver resolver(m_io_service);
  boost::asio::ip::tcp::endpoint
      endpoint = *resolver.resolve({m_config.getAddress(),
                                    std::to_string(m_config.getPort())});
  m_acceptor.open(endpoint.protocol());
  m_acceptor.set_option(boost::asio::socket_base::keep_alive(true));
  m_acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
  m_acceptor.bind(endpoint);
  m_acceptor.listen();

#ifdef SSL_ENABLED
  m_context.set_options(boost::asio::ssl::context::default_workarounds |
                        boost::asio::ssl::context::no_tlsv1 |
                        boost::asio::ssl::context::single_dh_use);
  m_context.set_password_callback(boost::bind(&Master::get_password, this));
  m_context.use_certificate_chain_file(TLS_CHAIN);
  m_context.use_private_key_file(TLS_PRIVATE, boost::asio::ssl::context::pem);
  m_context.use_tmp_dh_file(TLS_DH);
#endif
  start_accept();
}

}  // namespace IFlex
