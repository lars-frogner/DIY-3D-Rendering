#pragma once
#include "precision.hpp"
#include <vector>
#include <string>
#include <sstream>

namespace Impact {
namespace string_util {

template<typename Out>
void split(const std::string& s, char delim, Out result);

std::vector<std::string> split(const std::string& s, char delim = ' ');

std::string join(const std::vector<std::string>& sequence,
				 const std::string& separator,
				 imp_int start = 0, imp_int end = -1);

void ltrim(std::string& s);
void rtrim(std::string& s);
void trim(std::string& s);

} // string_util
} // Impact
