#pragma once
#include <string>
#include <vector>
#include <algorithm> 
#include <cctype>
#include <locale>
#include <sstream>
#include <algorithm>
#include <iterator>

namespace Geometry3D {
namespace util {

inline void ltrim(std::string& s);
inline void rtrim(std::string& s);
inline void trim(std::string& s);

template<typename Out>
inline void split(const std::string& s, char delim, Out result);

inline std::vector<std::string> split(const std::string& s, char delim = ' ');

inline void ltrim(std::string& s)
{
    s.erase(s.begin(),
            std::find_if(s.begin(),
                         s.end(),
                         [](int ch){ return !std::isspace(ch); }));
}

inline void rtrim(std::string& s)
{
    s.erase(std::find_if(s.rbegin(),
                         s.rend(),
                         [](int ch) { return !std::isspace(ch); }).base(),
            s.end());
}

inline void trim(std::string& s)
{
    ltrim(s);
    rtrim(s);
}

template<typename Out>
inline void split(const std::string& s, char delim, Out result)
{
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim))
    {
        if (!item.empty())
            *(result++) = item;
    }
}

inline std::vector<std::string> split(const std::string& s, char delim /* = ' '*/)
{
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

} // util
} // Geometry3D
