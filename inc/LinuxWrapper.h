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
* File description: Utility class for wrapping native functions.
*/
#ifndef TRUNK_SW_INC_LINUXWRAPPER_H_
#define TRUNK_SW_INC_LINUXWRAPPER_H_

#include <fcntl.h>
#include <unistd.h>
#include <cstring>

class LinuxWrapper {
 public:
  static const char _O_WRONLY = O_WRONLY;
  static const char _O_RDONLY = O_RDONLY;
  static const char _O_RDWR = O_RDWR;

  static int _open(const char *path, int oflag);
  static int _close(int fd);
  static ssize_t _write(int fd, const void *buf, size_t count);
  static ssize_t _read(int fd, void *buf, size_t count);
  static char *_strerror(int __errnum);
};

#endif  // TRUNK_SW_INC_LINUXWRAPPER_H_
