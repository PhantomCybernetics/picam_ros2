#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <cstring>
#include <bitset>
#include <filesystem>
#include <thread>
#include <iostream>
#include <functional>

#include <unistd.h>
#include <sys/mman.h>

#include "builtin_interfaces/msg/time.hpp"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>

const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string BLUE = "\033[34m";
const std::string YELLOW = "\033[33m";
const std::string MAGENTA = "\033[35m";
const std::string CYAN = "\033[36m";
const std::string WHITE = "\033[37m2";
const std::string CLR = "\033[0m";

const int NS_TO_SEC = 1000000000;

void checkCameraStack();
void reloadUdevRules();
void getCurrentStamp(builtin_interfaces::msg::Time *stamp, uint64_t timestamp_ns);
void freeBuffer(void* opaque, uint8_t* data);
int xioctl(int fd, ulong ctl, void *arg);
uint32_t roundUp4096(uint32_t x);