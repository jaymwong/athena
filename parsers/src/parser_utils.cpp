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
