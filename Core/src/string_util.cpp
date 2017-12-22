#include "string_util.hpp"
#include <cassert>
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

std::string join(const std::vector<std::string>& sequence,
				 const std::string& separator,
				 imp_int start /* = 0 */, imp_int end /* = -1 */)
{
	imp_int sequence_length = static_cast<imp_int>(sequence.size());

	if (end < 0)
		end += sequence_length;

	assert(end >= start && start >= 0 && end < sequence_length);

	if (sequence_length == 0)
		return std::string();

	std::string concatenated = sequence[start];

	for (imp_int idx = start + 1; idx <= end; idx++)
	{
		concatenated += separator + sequence[idx];
	}

	return concatenated;
}

} // string_util
} // Impact
