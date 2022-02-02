#ifndef _H_NODE__
#define _H_NODE__

#include <memory>
#include <vector>
#include <math.h>

struct Node {
  std::vector<double> joint_positions;
  std::shared_ptr<Node> parent_node;
  
  std::shared_ptr<Node> left;
  std::shared_ptr<Node> right;

  Node(std::vector<double> _joint_positions) : joint_positions(_joint_positions), 
                                   left(nullptr),
                                   right(nullptr),
                                   parent_node(nullptr) {};
  Node() : left(nullptr), right(nullptr), parent_node(nullptr) {};
  
  double distance(const std::vector<double> &other_joint_positions) {
      double sum = 0;

      for (size_t index = 0; index < joint_positions.size(); ++index) {
        sum += std::pow(joint_positions.at(index) - other_joint_positions.at(index), 2);
      }

      return std::sqrt(sum);
    }
};

#endif