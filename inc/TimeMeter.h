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
* File description: Utility class for measuring time.
*/
#ifndef IFLEX_SW_TIME_METER_H
#define IFLEX_SW_TIME_METER_H

#include <chrono>

namespace IFlex {
class TimeMeter {
  typedef std::chrono::time_point
      <std::chrono::steady_clock, std::chrono::nanoseconds> nano_time_point;

  nano_time_point begin;

 public:
  TimeMeter() {
    begin = std::chrono::steady_clock::now();
  }

  int64_t nanoseconds() {
    auto end = std::chrono::steady_clock::now();

    return std::chrono::duration_cast<std::chrono::nanoseconds>
        (end - begin).count();
  }

  int64_t microseconds() {
    auto end = std::chrono::steady_clock::now();

    return std::chrono::duration_cast<std::chrono::microseconds>
        (end - begin).count();
  }

  int64_t milliseconds() {
    auto end = std::chrono::steady_clock::now();

    return std::chrono::duration_cast<std::chrono::milliseconds>
        (end - begin).count();
  }

  int64_t seconds() {
    auto end = std::chrono::steady_clock::now();

    return std::chrono::duration_cast<std::chrono::seconds>
        (end - begin).count();
  }
};
}
#endif //IFLEX_SW_TIME_METER_H
