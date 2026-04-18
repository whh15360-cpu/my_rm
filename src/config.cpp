#include "Config.hpp"

Config* Config::instance_ = nullptr;
std::mutex Config::mutex_;
