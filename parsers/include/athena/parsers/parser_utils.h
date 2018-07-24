#ifndef ATHENA_PARSER_UTILS_H
#define ATHENA_PARSER_UTILS_H

#include <string>
#include <vector>
#include <sstream>

namespace athena{
  namespace parsers{
    std::vector<double> to_std_vectord(std::string text);
    std::string findSubstringInVector(std::vector<std::string> substrings, std::string name);
  };
};

#endif
