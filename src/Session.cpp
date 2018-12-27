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
#include <Request.h>
#include <Response.h>
#include "Session.h"

namespace IFlex {
#ifdef SSL_ENABLED
Session::Session(boost::asio::io_service &io_service,
                             boost::asio::ssl::context &context,
                             MasterNode &master_node)
    : m_socket(io_service, context), m_master_node(master_node) {
}

ssl_socket::lowest_layer_type &Session::socket() {
  return m_socket.lowest_layer();
}

void Session::handle_handshake(const boost::system::error_code &error) {
  if (!error) {
    Logger::info(TAG, "TLS handshake success.");

    async_read_request_header();
  } else {
    delete this;
  }
}
#else

Session::Session(boost::asio::io_service &io_service,
                 std::shared_ptr<MasterNode> master_node)
    : m_socket(io_service), m_master_node(master_node) {
}

boost::asio::ip::tcp::socket &Session::socket() {
  return m_socket;
}

#endif

void Session::async_write_response(std::shared_ptr<Response> response) {
  Logger::debug(TAG, "Write begin()");

  std::vector<boost::asio::const_buffer> buffers;
  buffers.emplace_back(boost::asio::buffer(&response->getHeader(), sizeof(ResponseHeader)));

  if (response->getHeader().status == Status::SUCCESS
      && response->getHeader().body_length > 0) {
    buffers.emplace_back(boost::asio::buffer(response->getBody().data.get(),
                                             response->getHeader().body_length));
    buffers.emplace_back(boost::asio::buffer(&(response->getBody().checksum),
                                             sizeof(ResponseBody::checksum)));
  }

  Logger::debug(TAG, "Write prepared({})", boost::asio::buffer_size(buffers));

  boost::asio::async_write(m_socket,
                           buffers,
                           boost::bind(&Session::handle_write,
                                       this,
                                       boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred,
                                       response));
}

void Session::async_write_invalid_checksum_response(std::shared_ptr<
    Request> request) {
  std::shared_ptr<Response> response(new Response());
  response->getHeader().command = request->getHeader().command;
  response->getHeader().status = Status::INVALID_CHECKSUM;
  response->getHeader().attribute_0 = 0;
  response->getHeader().attribute_1 = 0;
  response->getHeader().body_length = 0;
  response->setCheckSums();

  async_write_response(response);
}

void Session::async_write_invalid_api_key_response(std::shared_ptr<Request> request) {
  std::shared_ptr<Response> response(new Response());
  response->getHeader().command = request->getHeader().command;
  response->getHeader().status = Status::INVALID_API_KEY;
  response->getHeader().attribute_0 = 0;
  response->getHeader().attribute_1 = 0;
  response->getHeader().body_length = 0;
  response->setCheckSums();

  async_write_response(response);
}

void Session::async_read_request_header() {
  std::shared_ptr<Request> request(new Request());
  boost::asio::async_read(m_socket,
                          boost::asio::buffer(&request->getHeader(), sizeof(RequestHeader)),
                          boost::bind(&Session::handle_header_read,
                                      this,
                                      boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred,
                                      request));
}

void Session::async_read_request_body(std::shared_ptr<Request> request) {
  std::vector<boost::asio::mutable_buffer> buffers;

  if (request->getHeader().body_length > 0) {
    Logger::hexdump(TAG,
                    request->getBody().data.get(),
                    request->getHeader().body_length);
  }

  buffers.push_back(boost::asio::buffer(request->getBody().data.get(),
                                        request->getHeader().body_length));
  buffers.push_back(boost::asio::buffer(&(request->getBody().checksum),
                                        sizeof(RequestBody::checksum)));

  boost::asio::async_read(m_socket,
                          buffers,
                          boost::bind(&Session::handle_read,
                                      this,
                                      boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred,
                                      request));
}

void Session::start() {
  Logger::info(TAG,
               "Start session with {}:{}.",
               socket().remote_endpoint().address().to_string().c_str(),
               socket().remote_endpoint().port());

#ifdef SSL_ENABLED
  m_socket.async_handshake(boost::asio::ssl::stream_base::server, boost::bind(&Session::handle_handshake, this, boost::asio::placeholders::error));
#else
  async_read_request_header();
#endif
}

void Session::handle_write(const boost::system::error_code &error,
                           std::size_t bytes_transferred,
                           std::shared_ptr<Response> response) {
  Logger::debug(TAG, "Write end()");
  if (!error) {
    async_read_request_header();
  } else {
    delete this;
  }
}

void Session::handle_header_read(const boost::system::error_code &error,
                                 size_t bytes_transferred,
                                 std::shared_ptr<Request> request) {
  if (!error) {
    if (request->getHeader().checksum == request->header_checksum()) {
      Logger::debug(TAG, "Header: {}", request->getHeader().command);
      if (request->getHeader().body_length > 0) {
        request->getBody().data.reset(new uint8_t[request->getHeader().body_length], std::default_delete<uint8_t[]>());

        async_read_request_body(request);
      } else {
        handle_read(error, bytes_transferred, request);
      }
    } else {
      async_write_invalid_checksum_response(request);
    }
  } else {
    delete this;
  }
}

void
Session::handle_read(const boost::system::error_code &error,
                     size_t bytes_transferred,
                     std::shared_ptr<Request> request) {
  if (!error) {
    if (request->getHeader().body_length > 0
        && request->getBody().checksum != request->body_checksum()) {
      async_write_invalid_checksum_response(request);
    } else if (std::memcmp(m_master_node->getApiKey().c_str(),
                           request->getHeader().api_key,
                           8) != 0) {
      async_write_invalid_api_key_response(request);
    } else {
      std::shared_ptr<Response> response(new Response());
      response->getHeader().command = request->getHeader().command;
      response->getHeader().status = Status::SUCCESS;
      response->getHeader().attribute_0 = 0;
      response->getHeader().attribute_1 = 0;
      response->getHeader().body_length = 0;

      try {
        dispatch(request, response);
      } catch (DistanceModeNotSupportedException &e) {
        response->getHeader().status = Status::DISTANCE_MODE_NOT_SUPPORTED;
      } catch (QueryModeNotSupportedException &e) {
        response->getHeader().status = Status::QUERY_MODE_NOT_SUPPORTED;
      } catch (ReadCountNotSupportedException &e) {
        response->getHeader().status = Status::READ_COUNT_NOT_SUPPORTED;
      } catch (DataSetNotFoundException &e) {
        response->getHeader().status = Status::DATASET_NOT_FOUND;
      } catch (FileNotFoundException &e) {
        response->getHeader().status = Status::DATASET_FILE_NOT_FOUND;
      } catch (DataSetSizeNotSupportedException &e) {
        response->getHeader().status = Status::DATASET_SIZE_NOT_SUPPORTED;
      } catch (QuerySizeNotSupportedException &e) {
        response->getHeader().status = Status::QUERY_SIZE_NOT_SUPPORTED;
      } catch (TCPException &e) {
        response->getHeader().status = Status::TIMEOUT;
      }

      Logger::debug(TAG,
                    "{} finished with {} status.",
                    response->getHeader().command,
                    response->getHeader().status);

      response->setCheckSums();

      async_write_response(response);
    }
  } else {
    delete this;
  }
}

void Session::dispatch(std::shared_ptr<Request> request,
                       std::shared_ptr<Response> response) {
  std::unique_lock<std::mutex> lock(mtx_dispatch);

  vector8_list_t vectors;
  vector_result32_list_t results;

  Command command = request->getHeader().command;
  Logger::debug(TAG, "Received {} command to dispatch.", command);

  switch (command) {
    case Command::RESET:
      m_master_node->reset();
      break;
    case Command::DISTANCE_MODE:
      m_master_node->setDistanceMode((DistanceMode) request->getHeader().attribute_0);
      break;
    case Command::QUERY_MODE:
      m_master_node->setQueryMode((QueryMode) request->getHeader().attribute_0);
      break;
    case Command::THRESHOLD:
      m_master_node->setThreshold((uint32_t) request->getHeader().attribute_0,
                                  (uint32_t) request->getHeader().attribute_1);
      break;
    case Command::READ_COUNT:
      m_master_node->setReadCount((uint16_t) request->getHeader().attribute_0);
      break;
    case Command::DS_LOAD:
      switch (request->getHeader().attribute_1) {
        case 0:
          m_master_node->deserializeVectors(request.get(), vectors);
          m_master_node->dsLoad(vectors);
          break;
        case 1: {
          Json::Value root;
          Json::Reader reader;
          bool isParsed = reader.parse(std::string(request->getBody().data.get(),
                                                   request->getBody().data.get() +
                                                   request->getHeader().body_length),
                                       root);
          if (!isParsed) {
            std::cerr << "Failed to parse"
                      << reader.getFormattedErrorMessages();
            response->getHeader().status = Status::INVALID_PACKET;
          } else {
            m_master_node->dsLoadFromFile(root["fileName"].asString(),
                                          root["datasetName"].asString());
          }
        }
          break;
        case 2: {
          Json::Value root;
          Json::Reader reader;
          bool isParsed = reader.parse(std::string(request->getBody().data.get(),
                                                   request->getBody().data.get() +
                                                   request->getHeader().body_length),
                                       root);
          if (!isParsed) {
            std::cerr << "Failed to parse"
                      << reader.getFormattedErrorMessages();
            response->getHeader().status = Status::INVALID_PACKET;
          } else {
            m_master_node->dsLoadRandom(root["vectorCount"].asUInt64(),
                                          root["compCount"].asUInt64());
          }
        }
          break;
        default:
          break;
      }
      break;
    case Command::QUERY:
      switch (request->getHeader().attribute_1) {
        case 0:
          m_master_node->deserializeVectors(request.get(), vectors);
          m_master_node->query(vectors, results);
          m_master_node->serializeResults(response.get(), results, false);
          break;
        case 1:
          m_master_node->deserializeVectors(request.get(), vectors);
          m_master_node->query(vectors, results);
          m_master_node->serializeResults(response.get(), results, true);
          break;
        case 2: {
          Json::Value root;
          Json::Reader reader;
          bool isParsed = reader.parse(
              std::string(request->getBody().data.get(),
                          request->getBody().data.get()
                              + request->getHeader().body_length),
              root);
          if (!isParsed) {
            std::cerr << "Failed to parse"
                      << reader.getFormattedErrorMessages();
            response->getHeader().status = Status::INVALID_PACKET;
          } else {
            m_master_node->queryFromFile(root["fileName"].asString(),
                                         root["datasetName"].asString(),
                                         root["output"].asString(),
                                         true);
          }
        }
          break;
        default:
          break;
      }
      break;
    case Command::RESET_TIMER:
      m_master_node->resetTimer();
      break;
    case Command::GET_TIMER:
      response->getHeader().attribute_0 = m_master_node->getTimerValue();
      break;
    default:
      response->getHeader().status = Status::INVALID_COMMAND;
      break;
  }
}
}  // namespace IFlex
