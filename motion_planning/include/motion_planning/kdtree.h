#ifndef _H_KDTREE__
#define _H_KDTREE__

#include "motion_planning/node.h"
#include <limits>

class KdTree {

  std::vector<std::shared_ptr<Node>> tree_;
  const size_t max_depth_{6};

  public:
    KdTree(const size_t size);
    void clear();
    void insertNode(const std::vector<double> &joint_positions);
    void insertNode(std::shared_ptr<Node> &node);
    std::shared_ptr<Node> findNode(std::shared_ptr<Node> &node);
  
  private:
    void addNode(std::shared_ptr<Node> &node, std::shared_ptr<Node> &new_node, const size_t i = 0);
    void findKNode(std::shared_ptr<Node> &node, std::shared_ptr<Node> &closest_node, float &closest_distance, std::shared_ptr<Node> &new_node, const size_t i = 0);

};
#endif