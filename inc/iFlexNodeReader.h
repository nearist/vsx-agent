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
* File description: IFlexNode asynchronous reader.
*/
#ifndef TRUNK_SW_INC_IFLEX_NODE_ASYNC_READER_H_
#define TRUNK_SW_INC_IFLEX_NODE_ASYNC_READER_H_

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "iFlexTypes.h"

namespace IFlex {
class iFlexNodeReader {
  static constexpr uint32_t PACKET_PREAMBLE[2] = {0xFEDCBA98, 0x76543210};

 public:
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

  typedef struct {
    uint32_t preamble[2];
    uint16_t length;
    Command command;
    uint8_t fpga;
  } __attribute__((packed)) packet_header_t;

  typedef struct {
    uint16_t crc;
    uint16_t pid;
  } __attribute__((packed)) packet_footer_t;

  struct packet_t {
    packet_header_t header;
    packet_footer_t footer;
    std::shared_ptr<ulong_t> data;

    packet_t()
        : header(),
          footer(),
          data(new ulong_t[1](), std::default_delete<ulong_t[]>()) {}

  };

 private:
  void read_tail_handler(boost::system::error_code const &ec,
                         size_t bytes_transferred,
                         const std::shared_ptr<packet_t> &packet) {
    if (!ec) {
      m_callback(packet);

      read_head();
    } else {
      m_error_callback(ec);
    }
  }

  void read_head_handler(boost::system::error_code const &ec,
                         size_t bytes_transferred,
                         const std::shared_ptr<packet_t> &packet) {
    if (!ec) {
      if (packet->header.preamble[0] == PACKET_PREAMBLE[0]
          && packet->header.preamble[1] == PACKET_PREAMBLE[1]) {
        read_tail(packet);
      } else {
        read_head();
      }
    } else {
      m_error_callback(ec);
    }
  }

  void read_tail(const std::shared_ptr<packet_t> &packet) {
    if (packet->header.length > 0) {
      packet->data.reset(new ulong_t[packet->header.length - 1](),
                         std::default_delete<ulong_t[]>());

      std::vector<boost::asio::mutable_buffer> buffers;

      buffers.emplace_back(boost::asio::buffer(packet->data.get(),
                                               (packet->header.length - 1)
                                                   * sizeof(ulong_t)));
      buffers.emplace_back(boost::asio::buffer(&packet->footer,
                                               sizeof(packet_footer_t)));

      boost::asio::async_read(m_io_device,
                              buffers,
                              boost::bind(&iFlexNodeReader::read_tail_handler,
                                          this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred,
                                          packet));
    } else {
      m_callback(packet);
      read_head();
    }
  }

  void read_head() {
    std::shared_ptr<packet_t> packet(new packet_t());
    boost::asio::async_read(m_io_device,
                            boost::asio::buffer(&packet->header,
                                                sizeof(packet_header_t)),
                            boost::bind(&iFlexNodeReader::read_head_handler,
                                        this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred,
                                        packet));
  }

 public:
  iFlexNodeReader()
      : m_io_service(),
        m_io_device(m_io_service) {}

  void open(int32_t device_handle) {
    m_io_device =
        boost::asio::posix::stream_descriptor(m_io_service, device_handle);
  }

  void set_callbacks(std::function<void(std::shared_ptr<packet_t>)> callback,
                     std::function<void(boost::system::error_code const &ec)> error_callback) {
    m_callback = std::move(callback);
    m_error_callback = std::move(error_callback);
  }

  void start() {
    read_head();

    m_io_service.run();
  }

 private:
  boost::asio::io_service m_io_service;
  boost::asio::posix::stream_descriptor m_io_device;

  std::function<void(std::shared_ptr<packet_t>)> m_callback;
  std::function<void(boost::system::error_code const &ec)> m_error_callback;
};
}  // namespace IFlex
#endif  // TRUNK_SW_INC_IFLEX_NODE_ASYNC_READER_H_
