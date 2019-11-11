#include "utils.h"

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

unsigned int stoui(const std::string &str,
                   size_t *idx,
                   int base)
{
    unsigned long u = std::stoul(str, idx, base);
    if (u > UINT_MAX) throw std::out_of_range(str);
    return u;
}

/*
 * Erase all Occurrences of given substring from main string.
 */
void EraseAllSubStr(std::string& ioMainStr, const std::string& iToErase)
{
    size_t pos = std::string::npos;

    // Search for the substring in string in a loop untill nothing is found
    while ((pos  = ioMainStr.find(iToErase) )!= std::string::npos)
    {
        // If found then erase it from string
        ioMainStr.erase(pos, iToErase.length());
    }
}

/*
 * Erase all Occurrences of all given substrings from main string using C++11 stuff
 */
std::string EraseSubStrings(const std::string& iMainStr, const std::vector<std::string>& iStrList)
{
    std::string new_str = iMainStr;
    // Iterate over the given list of substrings. For each substring call eraseAllSubStr() to
    // remove its all occurrences from main string.
    std::for_each(iStrList.begin(), iStrList.end(), std::bind(EraseAllSubStr, std::ref(new_str), std::placeholders::_1));
    return new_str;
}


std::string EraseSubString(const std::string& iMainStr, const std::string& iToErase)
{
    std::string tmp_str = iMainStr;
    size_t pos = std::string::npos;

    // Search for the substring in string in a loop untill nothing is found
    while ((pos  = tmp_str.find(iToErase) )!= std::string::npos)
    {
        // If found then erase it from string
        tmp_str.erase(pos, iToErase.length());
    }
    return tmp_str;
}

void FindAndReplaceAll(std::string& ioData, const std::string iToSearch, const std::string iReplaceStr)
{
    // Get the first occurrence
    size_t pos = ioData.find(iToSearch);

    // Repeat till end is reached
    while( pos != std::string::npos)
    {
        // Replace this occurrence of Sub String
        ioData.replace(pos, iToSearch.size(), iReplaceStr);
        // Get the next occurrence from the current position
        pos =ioData.find(iToSearch, pos + iReplaceStr.size());
    }
}
