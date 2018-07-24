#include "athena/parsers/parser_utils.h"

std::vector<double> athena::parsers::to_std_vectord(std::string text){
  std::vector<double> result;
  std::string col;
  std::stringstream buffer(text);
  while (std::getline(buffer, col, ',')) {
    std::stringstream temp1(col);
    double val;
    temp1 >> val;
    result.push_back(val);
  }
  return result;
}


std::string athena::parsers::findSubstringInVector(std::vector<std::string> substrings, std::string name){
  for (int i = 0; i < substrings.size(); i++){
    if (name.find(substrings.at(i)) != std::string::npos) {
      return substrings.at(i);
    }
  }
  return "";
}
