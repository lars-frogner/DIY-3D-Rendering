#include "string_util.hpp"
#include <vector>
#include <algorithm> 
#include <cctype>
#include <locale>
#include <iterator>

namespace Impact {
namespace string_util {

void trim(std::string& s)
{
    ltrim(s);
    rtrim(s);
}

void ltrim(std::string& s)
{
    s.erase(s.begin(),
            std::find_if(s.begin(),
                         s.end(),
                         [](int ch){ return !std::isspace(ch); }));
}

void rtrim(std::string& s)
{
    s.erase(std::find_if(s.rbegin(),
                         s.rend(),
                         [](int ch) { return !std::isspace(ch); }).base(),
            s.end());
}

template<typename Out>
void split(const std::string& s, char delim, Out result)
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

std::vector<std::string> split(const std::string& s, char delim /* = ' '*/)
{
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

} // string_util
} // Impact
