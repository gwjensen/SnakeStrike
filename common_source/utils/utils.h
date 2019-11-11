#ifndef COMMON_UTILS_UTILS_H
#define COMMON_UTILS_UTILS_H

#include <opencv2/core.hpp>
#include "common_types.h"

std::string type2str( int type );
unsigned int stoui(const std::string &str,
                   size_t *idx = 0,
                   int base = 10);


void EraseAllSubStr(std::string& ioMainStr, const std::string& iToErase);

/*
 * Erase all Occurrences of all given substrings from main string using C++11 stuff
 */
std::string EraseSubStrings(const std::string& iMainStr, const std::vector<std::string>& iStrList);


std::string EraseSubString(const std::string& iMainStr, const std::string& iToErase);

void FindAndReplaceAll(std::string& ioData, const std::string iToSearch, const std::string iReplaceStr);

#endif //COMMON_UTILS_UTILS_H
