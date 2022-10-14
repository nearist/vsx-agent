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
* File description: The entry point of iFlex Daemon
*/

#include <iostream>
#include <boost/program_options.hpp>
#include "Master.h"

std::string arg_parse(int argc, char *argv[]) {
  std::string config_file;

  boost::program_options::options_description desc{"iflexd\nUsage"};
  desc.add_options()
      ("help,h", "Usage information.")
      ("version,v", "Version information.")
      ("config,c",
       boost::program_options::value<std::string>(&config_file),
       "Configuration file path.");

  boost::program_options::positional_options_description pos_desc;
  pos_desc.add("config", 1);

  boost::program_options::command_line_parser parser{argc, argv};
  parser.options(desc).positional(pos_desc).allow_unregistered();
  boost::program_options::parsed_options parsed_options = parser.run();

  boost::program_options::variables_map vm;
  boost::program_options::store(parsed_options, vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    std::exit(0);
  } else if (vm.count("version")) {
    //std::cout << std::string(APP_VERSION) << std::endl;
    std::exit(0);
  } else if (vm.count("config") == 0) {
    std::cerr << desc << std::endl;
    std::exit(-1);
  }

  return config_file;
}

int main(int argc, char *argv[]) {
  try {
    //auto config_path = arg_parse(argc, argv);
    //temp change for debug purposes
    auto config_path = "/home/nearist/git/vsx-agent/conf/iflex.json";
    IFlex::Master s;
    s.load_config(config_path);
  } catch (std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  return 0;
}
