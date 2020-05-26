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

#include <fcntl.h>
#include <unistd.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <string>

#include "until.h"

namespace athena {
namespace until{


bool parse_config_text(const std::string& filename, athena::config::Config* config) {
    int fd = open(filename.c_str(), O_RDONLY);
    if (-1 == fd) {
        return false;
    }

    google::protobuf::io::FileInputStream fs(fd);
    if (!::google::protobuf::TextFormat::Parse(&fs, config)) {
        close(fd);
        return false;
    }

    close(fd);
    return true;
}

double get_machine_timestamp_s() {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> tp = std::chrono::time_point_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
    //std::cout <<"now timestamp: " << std::setiosflags(std::ios::fixed) << std::setprecision(8) << (static_cast<double>(timestamp))/1000000.0  <<std::endl;
    return (static_cast<double>(timestamp)) / 1000000.0;
}

std::vector<std::string> split(const std::string &s, const std::string &seperator) {
    std::vector<std::string> result;
    typedef std::string::size_type string_size;
    string_size i = 0;

    while(i != s.size()) {
        //找到字符串中首个不等于分隔符的字母；
        int flag = 0;
        while(i != s.size() && flag == 0){
            flag = 1;
            for(string_size x = 0; x < seperator.size(); ++x)
                if(s[i] == seperator[x]){
                    ++i;
                    flag = 0;
                    break;
                }
        }

        //找到又一个分隔符，将两个分隔符之间的字符串取出；
        flag = 0;
        string_size j = i;
        while(j != s.size() && flag == 0){
            for(string_size x = 0; x < seperator.size(); ++x)
                if(s[j] == seperator[x]){
                    flag = 1;
                    break;
                }
            if(flag == 0)
                ++j;
        }
        if(i != j){
            result.push_back(s.substr(i, j-i));
            i = j;
        }
    }
    return result;
}

void string_replace( std::string &strBig, const std::string &strsrc, const std::string &strdst)
{
    std::string::size_type pos = 0;
    std::string::size_type srclen = strsrc.size();
    std::string::size_type dstlen = strdst.size();

    while( (pos=strBig.find(strsrc, pos)) != std::string::npos )
    {
        strBig.replace( pos, srclen, strdst );
        pos += dstlen;
    }
}

void pic_put_string(float num, int x, int y, cv::Mat &img) {

    std::string text =  std::to_string(num);
    while (text.size() > 6) {
        text.pop_back();
    }
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1.5;
    int thickness = 1;
    int baseline;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

    cv::Point origin;
    origin.x = x ;
    origin.y = y + text_size.height;
    cv::putText(img, text, origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);

}

void pic_put_string(std::string &text, int x, int y, cv::Mat &img) {

    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1.5;
    int thickness = 1;
    int baseline;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

    cv::Point origin;
    origin.x = x ;
    origin.y = y + text_size.height;
    cv::putText(img, text, origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);

}

}
}
