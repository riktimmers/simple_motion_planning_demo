#ifndef _H_TIMING__
#define _H_TIMING__

#include <chrono>
#include <iostream>
#include <string>

class Timer {

  std::chrono::system_clock::time_point begin_time;
  std::string name_;

  public:
    Timer(std::string name): name_(name) {
      begin_time = std::chrono::high_resolution_clock::now();
    };
    ~Timer() {
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - begin_time);
      std::cout << name_ << ": " << duration.count() <<"\n";
    }

};
#endif