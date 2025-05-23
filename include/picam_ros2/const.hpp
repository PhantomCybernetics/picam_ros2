#pragma once

#include <string>
#include <map>

const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string BLUE = "\033[34m";
const std::string YELLOW = "\033[33m";
const std::string MAGENTA = "\033[35m";
const std::string CYAN = "\033[36m";
const std::string WHITE = "\033[37m2";
const std::string CLR = "\033[0m";

const int NS_TO_SEC = 1000000000;
const int CLOCK_RATE = 90000;

enum IMAGE_OUTPUT_FORMAT : uint {
  BGR8,
  YUV420,
  MONO8
};

const std::map<uint, std::string> IMAGE_OUTPUT_FORMAT_NAMES = {
  { IMAGE_OUTPUT_FORMAT::BGR8, "bgr8" },
  { IMAGE_OUTPUT_FORMAT::YUV420, "yuv420" },
  { IMAGE_OUTPUT_FORMAT::MONO8, "mono8" },
};