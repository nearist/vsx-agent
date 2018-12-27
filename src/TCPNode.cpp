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
#include "TCPNode.h"

namespace IFlex {
#ifdef SSL_ENABLED
const uint8_t TCPNode::TLS_CA[] = "-----BEGIN CERTIFICATE-----\n"
        "MIIDlzCCAn+gAwIBAgIJAMJYU3U6A0IRMA0GCSqGSIb3DQEBBQUAMDsxCzAJBgNV\n"
        "BAYTAkFVMQwwCgYDVQQIEwNOU1cxDzANBgNVBAcTBlN5ZG5leTENMAsGA1UEChME\n"
        "YXNpbzAeFw0xNTExMTgyMjMzNDhaFw0yMDExMTYyMjMzNDhaMDsxCzAJBgNVBAYT\n"
        "AkFVMQwwCgYDVQQIEwNOU1cxDzANBgNVBAcTBlN5ZG5leTENMAsGA1UEChMEYXNp\n"
        "bzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMcRJocHdVMdLUJ/pypY\n"
        "QVSTC0t3IIgjwjazrK3kAaoIMvzPmDFxEXWcDx+nyz8kQ/E38Ir/ef2BCNGci5hu\n"
        "wkfMSuMoW9l2N4hx3QCcF46tTDEZztFxWAH7QbE2wYMlMgKZSxWimNfq0YjxEEXb\n"
        "QM0lGPLFh7Xoko29H0F3LKaaQV9u/vop3Hs0h12HeWlY4PiLp7QQTNGqbWcXycA0\n"
        "NZ/fyismireyEvPAgo6L8iXuAi7g0TVKVNlrticGGjMcMq6IMvxzEpSMkuMQ5rWj\n"
        "pZjWOoBjSYBuXdblcBRvXhOr2Ws8jJLMZfehKq9q1reQfoGV6xMnbwmumSXbWRWT\n"
        "0vkCAwEAAaOBnTCBmjAdBgNVHQ4EFgQUK/Zv/AVtfIeucJw8VEtux1dhI1YwawYD\n"
        "VR0jBGQwYoAUK/Zv/AVtfIeucJw8VEtux1dhI1ahP6Q9MDsxCzAJBgNVBAYTAkFV\n"
        "MQwwCgYDVQQIEwNOU1cxDzANBgNVBAcTBlN5ZG5leTENMAsGA1UEChMEYXNpb4IJ\n"
        "AMJYU3U6A0IRMAwGA1UdEwQFMAMBAf8wDQYJKoZIhvcNAQEFBQADggEBABLYXimq\n"
        "v/HLyIJi7Xn8AJUsICj8LKF/J24nwwiF+ibf7UkoChJURs4nN78bod/lpDVPTEVl\n"
        "gTBdV/vBJs416sCEFfsGjqB9OBYj4gb0VaJDsQd0+NMvXp0faKv2y9wgScxG9/cg\n"
        "aM7eRmyfMn1qjb6tpNxVOPpe/nFi8Vx/1orejBRaZr4zF5TkoPepfwLWQeXDUIdE\n"
        "+QHZ60jZAkR5RXTVU4u3kOKcJs839pmJYyxM4H2VxpR18vy4/YdIVWkREIUM2OgT\n"
        "5iznIQIIgR56QRGP85uef+I6n0BHzrBk6du69bkQFxrFjLVGlal4bIQqSg4KGWgx\n"
        "dEdymMWzmMxpO9s=\n"
        "-----END CERTIFICATE-----\n"
        "-----BEGIN RSA PRIVATE KEY-----\n"
        "MIIEpgIBAAKCAQEAxxEmhwd1Ux0tQn+nKlhBVJMLS3cgiCPCNrOsreQBqggy/M+Y\n"
        "MXERdZwPH6fLPyRD8Tfwiv95/YEI0ZyLmG7CR8xK4yhb2XY3iHHdAJwXjq1MMRnO\n"
        "0XFYAftBsTbBgyUyAplLFaKY1+rRiPEQRdtAzSUY8sWHteiSjb0fQXcspppBX27+\n"
        "+incezSHXYd5aVjg+IuntBBM0aptZxfJwDQ1n9/KKyaKt7IS88CCjovyJe4CLuDR\n"
        "NUpU2Wu2JwYaMxwyrogy/HMSlIyS4xDmtaOlmNY6gGNJgG5d1uVwFG9eE6vZazyM\n"
        "ksxl96Eqr2rWt5B+gZXrEydvCa6ZJdtZFZPS+QIDAQABAoIBAQCOma+SvPoDzvvU\n"
        "DiPOxqgOEMPfjHfGbm86xl0luBalGfiEd6WbjVanfGKtF4MWOUFec+chez+FJMEP\n"
        "fufVC0qrKiJfNVMOpYvEd2SMgkSx1VymM8me6WXVDYsSipn2+1cm228ZEYAR9Emj\n"
        "oqQ4loaGLlP/3RaJbhBF7ruMJvXaZZQ4fZy74Z4tyRaaE1B659ua7Rjne7eNhQE8\n"
        "cR7cQDkxsNNN3LTbfLRwEc/gcDXWgLe5JlR/K4ZrdKc3lyivm+Uew3ubKs+fgkyY\n"
        "kHmuI3RJGIjpnsZW0/So+pHm3b/fo6lmlhTXtNNd+tkkKn2K9ttbXT3Sc13Pc+4w\n"
        "c4MLyUpdAoGBAOxTtGDpeF6U4s+GPuOCzHCwKQyzfOyCL/UTZv1UJX7Kn1FYycJH\n"
        "eOjtBRtS661cGkGd1MPfjdX2VV84AmBGDUmRqJ2KfTI1NjLAEJ115ANTpmSTm3lF\n"
        "UYncgbzl6aflLpjE1mgY+JTJykYeN5jhhO0r2bsdY7S+zaMCSI5NLuznAoGBANej\n"
        "aMtqLg2qKoq+fUkNBHHLXelR5dBXFnKgSrTj++H4yeW9pYbl8bK3gTF3I5+dSjHW\n"
        "DdC4+X09iPqY7p8vm8Gq/vgO8Bu+EnKNVr80PJSj7AzFGd6mk/CVrAzoY2XJWbAp\n"
        "YFwpo1WfHjS5wBfQzBlXY7kWVB7fj32kk14PYmUfAoGBAJXfd7NGHPoOfdCSGGv8\n"
        "VV7ZuQ6+/WiYH4XS6iuaI7VHFsZmAn3dCcbeGbD8Y04r7NLUH0yhB7g7YmTihk87\n"
        "3c1cPIy8eS1QJbEFsQPK8fFSKWH7YkwEM/O0DesX+5hodaaYnkiiHXNujYLuQuAH\n"
        "lV87wfcyajsEDjFkj1L/i9TdAoGBAKYfRUQv8HqmdU+doHb+iEYCHb75UMpHzQtR\n"
        "YTwpxoo3V5Kdnz9lNeYwaF7rIY59ZgMunEYHumw5U6V625nW228/hF0lZOR6cUu+\n"
        "hu2WGHWKMvdDgMJ+IcpeA8WN4cUwcN+9gHZ/vUzg4CxOTSYLvLBpGnIkOXnvUGPC\n"
        "vaTgxTSRAoGBAOHcuZ9hcUrPuVI1HVkjQQLu5mLZ3tz6linEbe/RCdJMK8JrRX4w\n"
        "ubB7gFclMYGbLlDNAJVYkydJaCy/2NAI3rfsOda+VmDqGx6z4BbSGceHhomyU1Oo\n"
        "1H7YaXsuzDkzl23HRsyp0pKJpTdghZdbVsGF8vAB8ygK3ehM233neSln\n"
        "-----END RSA PRIVATE KEY-----";
#endif

TCPNode::TCPNode(const NodeConfig &config) : INode(config), m_io_service() {
#ifdef SSL_ENABLED
  boost::asio::ssl::context ctx(boost::asio::ssl::context::tlsv12);
  ctx.add_certificate_authority(boost::asio::const_buffer(TLS_CA, sizeof(TLS_CA)));

  m_socket = new ssl_socket(m_io_service, ctx);
  m_socket->set_verify_mode(boost::asio::ssl::verify_peer);
  m_socket->set_verify_callback(boost::bind(&TCPNode::verify_certificate, this, _1, _2));
#else
  m_socket = new boost::asio::ip::tcp::socket(m_io_service);
#endif
}

TCPNode::~TCPNode() {
  delete m_socket;
}

#ifdef SSL_ENABLED
bool TCPNode::verify_certificate(bool preverified, boost::asio::ssl::verify_context &ctx) {
    // The verify callback can be used to check whether the certificate that is
    // being presented is valid for the peer. For example, RFC 2818 describes
    // the steps involved in doing this for HTTPS. Consult the OpenSSL
    // documentation for more details. Note that the callback is called once
    // for each certificate in the certificate chain, starting from the root
    // certificate authority.

    // In this example we will simply print the certificate's subject name.
    char subject_name[256];
    X509 *cert = X509_STORE_CTX_get_current_cert(ctx.native_handle());
    X509_NAME_oneline(X509_get_subject_name(cert), subject_name, 256);

    return preverified;
}
#endif

void TCPNode::buffer2Results(Response &response,
                             vector_result32_list_t &results) {
  vector_result32_t result;

  uint64_t length = response.getHeader().body_length / sizeof(uint64_t);

  if (length > 0) {
    auto body = response.getBody().data.get();

    for (uint64_t i = 0; i < length; i += 2) {
      if (body[i] != 0xFFFFFFFFFFFFFFFF && body[i + 1] != 0xFFFFFFFFFFFFFFFF) {
        result.emplace_back(result32_t(body[i] + m_offset, (uint32_t) body[i + 1]));
      } else {
        results.emplace_back(result);
        result.clear();
      }
    }
  }
}

uint8_t *TCPNode::data2Buffer(vector8_list_t vectors) {
  auto buffer = new uint8_t[vectors.size() * vectors.front().size()]();

  size_t k = 0;
  for (auto vector : vectors) {
    std::memcpy(buffer + k, &vector[0], vector.size());
    k += vector.size();
  }

  return buffer;
}

void TCPNode::write(Request &request) {
  std::vector<boost::asio::const_buffer> buffers;

  request.setApiKey(m_config.getApiKey());
  request.setCheckSums();

  buffers.emplace_back(boost::asio::buffer(&request.getHeader(),
                                           sizeof(RequestHeader)));

  if (request.getHeader().body_length > 0) {
    buffers.emplace_back(boost::asio::buffer(request.getBody().data.get(),
                                             request.getHeader().body_length));
    buffers.emplace_back(boost::asio::buffer(&(request.getBody().checksum),
                                             sizeof(RequestBody::checksum)));
  }

  boost::asio::write(*m_socket, buffers);
}

Response TCPNode::read() {
  Response response;

  size_t rc = boost::asio::read(*m_socket,
                                boost::asio::buffer(&response.getHeader(),
                                                    sizeof(ResponseHeader)));

  if (rc == sizeof(ResponseHeader)
      && response.getHeader().checksum == response.header_checksum()) {
    if (response.getHeader().body_length > 0) {
      response.getBody().data.reset(new uint64_t[response.getHeader().body_length / sizeof(uint64_t)](), std::default_delete<uint64_t[]>());

      std::vector<boost::asio::mutable_buffer> buffers;
      buffers.emplace_back(boost::asio::buffer(response.getBody().data.get(),
                                               response.getHeader().body_length));
      buffers.emplace_back(boost::asio::buffer(&(response.getBody().checksum),
                                               sizeof(RequestBody::checksum)));

      boost::system::error_code ec;

      rc = boost::asio::read(*m_socket,
                             buffers,
                             boost::asio::transfer_all(),
                             ec);

      if (ec) {
        Logger::error(TAG, "Data not read or invalid checksum.");
        Logger::error(TAG, ec.message().c_str());
      }
    }
  } else {
    Logger::error(TAG, "Header not read or invalid checksum.");
  }

  return response;
}

void TCPNode::open() {
  boost::asio::ip::tcp::resolver resolver(m_io_service);
  boost::asio::ip::tcp::resolver::query
      address_query(m_config.getAddress(), std::to_string(m_config.getPort()));
  boost::asio::ip::tcp::resolver::iterator
      iterator = resolver.resolve(address_query);

#ifdef SSL_ENABLED
  boost::asio::connect(m_socket->lowest_layer(), iterator);
  m_socket->handshake(boost::asio::ssl::stream_base::client);
#else
  boost::asio::connect(*m_socket, iterator);
#endif
}

void TCPNode::close() {
#ifdef SSL_ENABLED
  m_socket->lowest_layer().close();
#else
  m_socket->close();
#endif
}

void TCPNode::reset() {
  RequestHeader requestHeader{};
  requestHeader.command = Command::RESET;
  requestHeader.body_length = 0;
  requestHeader.attribute_0 = 0;
  requestHeader.attribute_1 = 0;

  Request request;
  request.setHeader(requestHeader);

  write(request);

  Response r = read();

  if (r.getHeader().status != Status::SUCCESS) {
    throw TCPException(r.getHeader().status);
  }
}

void TCPNode::setDistanceMode(const IFlex::DistanceMode mode) {
  RequestHeader requestHeader{};
  requestHeader.command = Command::DISTANCE_MODE;
  requestHeader.body_length = 0;
  requestHeader.attribute_0 = mode;
  requestHeader.attribute_1 = 0;

  Request request;
  request.setHeader(requestHeader);

  write(request);

  Response r = read();

  if (r.getHeader().status != Status::SUCCESS) {
    throw TCPException(r.getHeader().status);
  }
}

void TCPNode::setQueryMode(const IFlex::QueryMode mode) {
  RequestHeader requestHeader{};
  requestHeader.command = Command::QUERY_MODE;
  requestHeader.body_length = 0;
  requestHeader.attribute_0 = mode;
  requestHeader.attribute_1 = 0;

  Request request;
  request.setHeader(requestHeader);

  write(request);

  Response r = read();

  if (r.getHeader().status != Status::SUCCESS) {
    throw TCPException(r.getHeader().status);
  }
}

void TCPNode::setReadCount(const uint16_t count) {
  RequestHeader requestHeader{};
  requestHeader.command = Command::READ_COUNT;
  requestHeader.body_length = 0;
  requestHeader.attribute_0 = count;
  requestHeader.attribute_1 = 0;

  Request request;
  request.setHeader(requestHeader);

  write(request);

  Response r = read();

  if (r.getHeader().status != Status::SUCCESS) {
    throw TCPException(r.getHeader().status);
  }
}

void TCPNode::setThreshold(const uint32_t threshold) {
  RequestHeader requestHeader{};
  requestHeader.command = Command::THRESHOLD;
  requestHeader.body_length = 0;
  requestHeader.attribute_0 = threshold;
  requestHeader.attribute_1 = 0;

  Request request;
  request.setHeader(requestHeader);

  write(request);

  Response r = read();

  if (r.getHeader().status != Status::SUCCESS) {
    throw TCPException(r.getHeader().status);
  }
}

void TCPNode::setThreshold(const uint32_t threshold_lower,
                           const uint32_t threshold_upper) {
  RequestHeader requestHeader{};
  requestHeader.command = Command::THRESHOLD;
  requestHeader.body_length = 0;
  requestHeader.attribute_0 = threshold_lower;
  requestHeader.attribute_1 = threshold_upper;

  Request request;
  request.setHeader(requestHeader);

  write(request);

  Response r = read();

  if (r.getHeader().status != Status::SUCCESS) {
    throw TCPException(r.getHeader().status);
  }
}

void TCPNode::dsLoad(const uint64_t offset, const vector8_list_t &vectors) {
  m_offset = offset;

  RequestHeader requestHeader{};
  requestHeader.command = Command::DS_LOAD;
  requestHeader.body_length = vectors.size() * vectors.front().size();
  requestHeader.attribute_0 = vectors.front().size();
  requestHeader.attribute_1 = 0;

  Request request;
  request.setHeader(requestHeader);
  request.getBody().data.reset(data2Buffer(vectors), std::default_delete<uint8_t[]>());

  write(request);

  Response r = read();

  if (r.getHeader().status != Status::SUCCESS) {
    throw TCPException(r.getHeader().status);
  }
}

void TCPNode::query(const vector8_list_t &vectors,
                    vector_result32_list_t &results) {
  RequestHeader requestHeader{};
  requestHeader.command = Command::QUERY;
  requestHeader.body_length = vectors.size() * vectors.front().size();
  requestHeader.attribute_0 = vectors.front().size();
  requestHeader.attribute_1 = 0;

  Request request;
  request.setHeader(requestHeader);
  request.getBody().data.reset(data2Buffer(vectors), std::default_delete<uint8_t[]>());

  write(request);

  Response r = read();

  if (r.getHeader().status == Status::SUCCESS) {
    buffer2Results(r, results);
  } else {
    throw TCPException(r.getHeader().status);
  }
}

void TCPNode::resetTimer() {
  RequestHeader requestHeader{};
  requestHeader.command = Command::RESET_TIMER;
  requestHeader.body_length = 0;
  requestHeader.attribute_0 = 0;
  requestHeader.attribute_1 = 0;

  Request request;
  request.setHeader(requestHeader);

  write(request);

  Response r = read();

  if (r.getHeader().status != Status::SUCCESS) {
    throw TCPException(r.getHeader().status);
  }
}

uint64_t TCPNode::getTimerValue() {
  RequestHeader requestHeader{};
  requestHeader.command = Command::GET_TIMER;
  requestHeader.body_length = 0;
  requestHeader.attribute_0 = 0;
  requestHeader.attribute_1 = 0;

  Request request;
  request.setHeader(requestHeader);

  write(request);

  Response r = read();

  if (r.getHeader().status != Status::SUCCESS) {
    throw TCPException(r.getHeader().status);
  }

  return r.getHeader().attribute_0;
}
}  // namespace IFlex
