// L1IFUtil.hpp - Utility functions for L1 Interface
#pragma once
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstdint>

std::string get_iso8601_timestamp(uint32_t unix_time, uint32_t ms_offset);