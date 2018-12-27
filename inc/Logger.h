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
* File description: Utility class for wrapping spdlog.
*/
#ifndef TRUNK_SW_INC_LOGGER_H_
#define TRUNK_SW_INC_LOGGER_H_

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include "iFlexTypes.h"

namespace IFlex {

inline std::ostream &operator<<(std::ostream &os, const DistanceMode &d) {
  return os << DistanceMode2String(d);
}

inline std::ostream &operator<<(std::ostream &os, const QueryMode &d) {
  return os << QueryMode2String(d);
}

inline std::ostream &operator<<(std::ostream &os, const Status &d) {
  return os << Status2String(d);
}

inline std::ostream &operator<<(std::ostream &os, const Command &d) {
  return os << Command2String(d);
}

class Logger {
 private:
  explicit Logger(const std::string &filename,
                  const std::string &temperature_filename) {
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%F] [%l] %v");

    if (filename.empty()) {
      this->logger = spdlog::stdout_color_mt("iflex_logger");
    } else {
      this->logger = spdlog::rotating_logger_mt("iflex_logger",
                                                filename,
                                                10 * 1024 * 1024,
                                                3);
    }

    if (temperature_filename.empty()) {
      this->temperature_logger =
          spdlog::stdout_color_mt("iflex_temperature_logger");
    } else {
      this->temperature_logger = spdlog::rotating_logger_mt(
          "iflex_temperature_logger",
          temperature_filename,
          10 * 1024 * 1024,
          3);
    }
    this->temperature_logger->set_pattern("%E,%v", spdlog::pattern_time_type::local);

    spdlog::set_error_handler([](const std::string &msg) {
      std::cerr << "LOGGER_ERROR: " << msg << std::endl;
    });

    spdlog::flush_on(spdlog::level::debug);
    spdlog::set_level(spdlog::level::debug);
  }

  std::shared_ptr<spdlog::logger> logger;
  std::shared_ptr<spdlog::logger> temperature_logger;

  static std::shared_ptr<Logger> instance;
  static std::once_flag onceFlag;

 public:
  Logger(const Logger &) = delete;

  Logger &operator=(const Logger &) = delete;

  ~Logger() {
    spdlog::drop_all();
  }

  static Logger &getInstance(const std::string &filename,
                             const std::string &temperature_filename) {
    std::call_once(Logger::onceFlag, [&filename, &temperature_filename]() {
      instance.reset(new Logger(filename, temperature_filename));
    });

    return *(instance.get());
  }

  static void trace(const std::string &tag, const std::string &msg) {
    Logger::getInstance("", "").logger->trace("{0}: {1}", tag.c_str(), msg);
  }

  static void debug(const std::string &tag, const std::string &msg) {
    Logger::getInstance("", "").logger->debug("{0}: {1}", tag, msg);
  }

  static void info(const std::string &tag, const std::string &msg) {
    Logger::getInstance("", "").logger->info("{0}: {1}", tag.c_str(), msg);
  }

  static void warning(const std::string &tag, const std::string &msg) {
    Logger::getInstance("", "").logger->warn("{0}: {1}", tag.c_str(), msg);
  }

  static void error(const std::string &tag, const std::string &msg) {
    Logger::getInstance("", "").logger->error("{0}: {1}", tag.c_str(), msg);
  }

  static void critical(const std::string &tag, const std::string &msg) {
    Logger::getInstance("", "").logger->critical("{0}: {1}", tag.c_str(), msg);
  }

  template<typename... Args>
  static void trace(const std::string &tag,
                    const char *fmt,
                    const Args &... args) {
    std::string format = tag + ": " + fmt;
    Logger::getInstance("", "").logger->trace(format.c_str(), args...);
  }

  template<typename... Args>
  static void debug(const std::string &tag,
                    const char *fmt,
                    const Args &... args) {
    std::string format = tag + ": " + fmt;
    Logger::getInstance("", "").logger->debug(format.c_str(), args...);
  }

  template<typename... Args>
  static void info(const std::string &tag,
                   const char *fmt,
                   const Args &... args) {
    std::string format = tag + ": " + fmt;
    Logger::getInstance("", "").logger->info(format.c_str(), args...);
  }

  template<typename... Args>
  static void warning(const std::string &tag,
                      const char *fmt,
                      const Args &... args) {
    std::string format = tag + ": " + fmt;
    Logger::getInstance("", "").logger->warn(format.c_str(), args...);
  }

  template<typename... Args>
  static void error(const std::string &tag,
                    const char *fmt,
                    const Args &... args) {
    std::string format = tag + ": " + fmt;
    Logger::getInstance("", "").logger->error(format.c_str(), args...);
  }

  template<typename... Args>
  static void critical(const std::string &tag,
                       const char *fmt,
                       const Args &... args) {
    std::string format = tag + ": " + fmt;
    Logger::getInstance("", "").logger->critical(format.c_str(), args...);
  }

  static void hexdump(const std::string &tag, const void *ptr, size_t size) {
    if (getInstance("", "").logger->level() == spdlog::level::trace) {
      auto buf = (uint32_t *) (ptr);
      auto len = size / 4;

      std::string f;

      for (size_t i = 0; i < len; i += 8) {
        std::string line =
            fmt::format("{0}{1: >16}{2:0>8x}: ", i == 0 ? "\n" : "", " ", i);

        for (size_t j = 0; j < 8; j++) {
          if (i + j < len) {
            line += fmt::format("{0:0>8x} ", buf[i + j]);
          }
        }

        f += line + "\n";
      }

      f.erase(f.end() - 1);
      Logger::trace(tag, f);
    }
  }

  static void temperature(const std::string &tag,
                          const std::map<std::string, std::map<uint8_t, uint16_t>> data) {

    for (const auto &lane : data) {
      std::string f = lane.first + ",";
      for (const auto &dtr : lane.second) {
        f += fmt::format("0x{0:0>2X},", dtr.second);
      }
      f.erase(f.end() - 1);
      Logger::getInstance("", "").temperature_logger->info("{}", f);
    }
  }

  static void setLevel(spdlog::level::level_enum level) {
    Logger::getInstance("", "").logger->set_level(level);
  }

  static spdlog::level::level_enum getLevel() {
    return Logger::getInstance("", "").logger->level();
  }
};

}  // namespace IFlex

#endif  // TRUNK_SW_INC_LOGGER_H_
