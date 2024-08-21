//
// Created by pirotti on 23/04/23.
//

#ifndef PCAUTOREG_COMMON_H
#define PCAUTOREG_COMMON_H

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <string>
#include <stdexcept>
#include <iterator>
#include <sstream>

using namespace std;

extern float angular_resolution;
extern float support_size ;
extern std::string logfile;

extern float range_image_grid_res; // x10 less of support size this can be changed by -s

inline ostream& formatDateTime(
    ostream& out, const tm& t, const char* fmt) {
  const time_put<char>& dateWriter =
    use_facet<time_put<char> >(out.getloc());
  int n = (int)strlen(fmt);
  if (dateWriter.put
        (out, out, ' ', &t, fmt, fmt + n).failed()) {
    throw runtime_error("failure to format date time");
  }  
  return out;
}

inline string dateTimeToString(const tm& t, const char* format) {
  stringstream s;
  formatDateTime(s, t, format);
  return s.str();
}

inline tm now() {
  time_t now = time(0);
  return *localtime(&now);
}

inline std::vector<float>  calculateStat(std::vector<float> data) {
    float sum = 0.0, mean, standardDeviation = 0.0, rmse = 0.0;
    int i;

    for(i = 0; i < data.size(); ++i) {
        sum += data[i];
    }

    mean = sum /  data.size();

    for(i = 0; i <  data.size(); ++i) {
        standardDeviation += pow(data[i] - mean, 2);
        rmse += pow(data[i], 2);
    }

    std::vector<float>  dat(3);
    dat[0] = mean;
    dat[1] = sqrt(standardDeviation /  data.size());
    dat[2] = sqrt(rmse /  data.size());
    return dat;
}

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

static void appendLineToFile(std::string filepath, 
                             std::string line,
                             bool truncate=false)
{
    std::ofstream file;
    //can't enable exception now because of gcc bug that raises ios_base::failure with useless message
    //file.exceptions(file.exceptions() | std::ios::failbit);
    if(truncate)  {
        file.open(filepath, std::ios::out | std::ios::trunc );
    } else {
        file.open(filepath, std::ios::out | std::ios::app   );
    }
    if (file.fail())
        throw std::ios_base::failure(std::strerror(errno));


    //make sure write fails with exception if something is wrong
    file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);

    file << line << std::endl;
}



#endif //PCAUTOREG_COMMON_H
