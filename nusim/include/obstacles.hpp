/// @file
/// @brief logic for handling collisions with obstacles

#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP
#include <vector>

class Obstacles {
public:
  std::vector<double> x;
  std::vector<double> y;
  double r;
};

#endif // OBSTACLES_HPP