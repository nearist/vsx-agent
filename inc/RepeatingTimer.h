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
* File description: Utility class for timer functions.
*/
#ifndef TRUNK_SW_INC_REPEATING_TIMER_H_
#define TRUNK_SW_INC_REPEATING_TIMER_H_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace IFlex {
class RepeatingTimer 
{
 public:
  RepeatingTimer(std::function<void(bool)> callback)
      : m_first_loop(true),
        m_seconds(0),
        m_callback(std::move(callback)),
        m_io_service(),
        m_strand_(m_io_service),
        m_timer(m_io_service) {

  }

  void init(uint64_t seconds) {
    m_seconds = seconds;

    m_timer.expires_from_now(boost::posix_time::seconds(m_seconds));
    m_timer.async_wait(
        m_strand_.wrap(boost::bind(&RepeatingTimer::handler, this))
    );
  }

  void run() {
    m_io_service.run();
  }

  void handler() {
    if (m_callback) {
      m_callback(m_first_loop);
      m_first_loop = false;
    }

    m_timer.expires_at(m_timer.expires_at() + boost::posix_time::seconds(m_seconds));
    m_timer.async_wait(
        m_strand_.wrap(boost::bind(&RepeatingTimer::handler, this))
    );
  }

  private:
    bool m_first_loop;

    uint64_t m_seconds;
    std::function<void(bool)> m_callback;

    boost::asio::io_service m_io_service;
    boost::asio::io_context::strand m_strand_;
    boost::asio::deadline_timer m_timer;

};
}  // namespace IFlex

#endif  // TRUNK_SW_INC_REPEATING_TIMER_H_
