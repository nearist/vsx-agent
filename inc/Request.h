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
* File description: Request DTO.
*/
#ifndef TRUNK_SW_INC_REQUEST_H_
#define TRUNK_SW_INC_REQUEST_H_

#include <cstring>
#include <string>
#include <boost/crc.hpp>
#include "iFlexTypes.h"

namespace IFlex {

struct RequestHeader {
  Command command;
  uint32_t _RESERVED;
  uint64_t attribute_0;
  uint64_t attribute_1;
  uint64_t body_length;

  uint8_t api_key[8];

  uint32_t checksum;
} __attribute__((packed));

struct RequestBody {
  std::shared_ptr<uint8_t> data;
  uint32_t checksum;

  RequestBody() : data(new uint8_t[1](), std::default_delete<uint8_t[]>()),
                  checksum() {}

};

class Request {
  RequestHeader header;
  RequestBody body;

 public:
  Request() : header(), body() {}

  ~Request() = default;

  RequestHeader &getHeader() {
    return header;
  }

  void setHeader(const RequestHeader &header) {
    Request::header = header;
  }

  RequestBody &getBody() {
    return body;
  }

  void setBody(const RequestBody &body) {
    Request::body = body;
  }

  void setCheckSums() {
    header.checksum = header_checksum();
    body.checksum = body_checksum();
  }

  uint32_t header_checksum() {
    boost::crc_32_type result;
    result.process_bytes(
        &header,
        sizeof(RequestHeader) - sizeof(RequestHeader::checksum)
    );

    return (uint32_t) result.checksum();
  }

  uint32_t body_checksum() {
    boost::crc_32_type result;
    result.process_bytes(body.data.get(), header.body_length);
    return (uint32_t) result.checksum();
  }

  void setApiKey(const std::string &api_key) {
    std::string key = api_key.size() < 8 ?
                      api_key + std::string(8 - api_key.size(), ' ') :
                      api_key.substr(0, 8);
    std::memcpy(header.api_key, key.c_str(), 8);
  }

};
}  // namespace IFlex
#endif  // TRUNK_SW_INC_REQUEST_H_
