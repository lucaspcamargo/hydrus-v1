#pragma once

#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>

typedef std::vector<std::string> stringvec_t;
typedef stringvec_t::iterator stringvec_it_t;

namespace Util {
  long parseLong( const std::string & str, int begin = 0, int end = -1)
  {
    if(end == -1)
      end = str.length();
    
    if(begin != 0 || end != str.length())
      return strtol(str.substr(begin, end - begin).c_str(), 0, 10);
    else
      return strtol(str.c_str(), 0, 10);
  }

  int parseInt( const std::string & str, int begin = 0, int end = -1)
  {
    return (int) parseLong(str, begin, end);
  }
  
  double parseDouble( const std::string & str, int begin = 0, int end = -1)
  {    
    if(end == -1)
      end = str.length();
    
    if(begin != 0 || end != str.length())
      return strtod(str.substr(begin, end - begin).c_str(), 0);
    else
      return strtod(str.c_str(), 0);
  }
  
  float parseFloat( const std::string & str, int begin = 0, int end = -1)
  {
    return (float) parseDouble(str, begin, end);
  }

  // from stackoverflow
  std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
      std::stringstream ss(s);
      std::string item;
      while (std::getline(ss, item, delim)) {
          if(item.length() && item[item.length()-1] == delim)
            item.erase(item.length() -1);
          elems.push_back(item);
      }
      return elems;
  }
  std::vector<std::string> split(const std::string &s, char delim) {
      std::vector<std::string> elems;
      split(s, delim, elems);
      return elems;
  }
  // end stackoverflow
}