/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef ATHENA_CONFIG_UTILS_H_
#define ATHENA_CONFIG_UTILS_H_

#include <string>
#include <chrono>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <config.pb.h>

namespace athena {
namespace until{

bool   parse_config_text(const std::string& filename, athena::config::Config* config);
double get_machine_timestamp_s();
std::vector<std::string> split(const std::string &s, const std::string &seperator);
void string_replace( std::string &strBig, const std::string &strsrc, const std::string &strdst);
void pic_put_string(float num, int x, int y, cv::Mat &img);
void pic_put_string(std::string &text, int x, int y, cv::Mat &img);

template <class Type>
Type string2num(const std::string& str) {
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

}
}


#endif  // MODULES_DRIVERS_GNSS_UTILS_H_
