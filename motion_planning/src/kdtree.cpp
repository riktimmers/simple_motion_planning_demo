#include "motion_planning/kdtree.h"

//! Constructor
/** Constructor reserves capacity 
 * \param size, the size to reserve
 */
KdTree::KdTree(const size_t size) {
  tree_.reserve(size);
}

//! Function to clear the tree
void KdTree::clear() {
  tree_.clear();
}

//! Function to insert a node into the KdTree
/**
 * \param new_node, the new node to add to the KdTree
 */
void KdTree::insertNode(std::shared_ptr<Node> &new_node) {
  
  if (tree_.size() == 0) { // If tree is empty
    std::shared_ptr<Node> node = nullptr;
    addNode(node, new_node);
  } else { // Else select the first (root) node of the tree
    addNode(tree_.front(), new_node);
  }
}

//! Function to add a node to the KdTree
/**
 * \param node, the current node in the tree
 * \param new_node, the node to add into the tree
 * \param i, the depth of the tree
 */
void KdTree::addNode(std::shared_ptr<Node> &node, std::shared_ptr<Node> &new_node, const size_t i) {
  
  if (node == nullptr) { // If end if found, add the node
    tree_.push_back(new_node);
    node = new_node;
    return;
  }

  const size_t depth = i % max_depth_; // Determine the depth of the tree
  double value = new_node->joint_positions.at(depth);

  if (value < node->joint_positions.at(depth)) { // if smaller move the to the left
    addNode(node->left, new_node, i + 1);
  } else { // else move to the right
    addNode(node->right, new_node, i + 1);
  }
}

//! Function to find the closest node
/**
 * \param node, the node to find 
 * \return std::shared_ptr<Node>, the node that is the closest
 */
std::shared_ptr<Node> KdTree::findNode(std::shared_ptr<Node> &node) {
  float best_dist = std::numeric_limits<float>::max();
  std::shared_ptr<Node> closest_node(new Node());
  findKNode(tree_.front(), closest_node, best_dist, node);
  return closest_node;
}

//! Function to find the closest node, using recursive calls into the tree
/**
 * \param node, the current node in the tree
 * \param closest_node, the closest node so far 
 * \param closest_distance, the closest distance found so far
 * \param new_node, the node to find the closest node to
 * \param i, the depth of the tree
 */
void KdTree::findKNode(std::shared_ptr<Node> &node, std::shared_ptr<Node> &closest_node, float &closest_distance, std::shared_ptr<Node> &new_node, const size_t i) {

  if (node == nullptr) {
    return;
  } 

  double dist = node->distance(new_node->joint_positions);
  if (dist < closest_distance) { // Update the closest node and closest distance
    closest_distance = dist;
    closest_node = node;
  }
  
  size_t depth = i % max_depth_; // Determine the depth
  double value = new_node->joint_positions.at(depth);
  
  if (value < node->joint_positions.at(depth)) { // Look left of the tree
    findKNode(node->left, closest_node, closest_distance, new_node, i+1);

    if (value + closest_distance >= node->joint_positions.at(depth)) { // There might be a closer on the right side
      findKNode(node->right, closest_node, closest_distance, new_node, i+1);
    }
  } else { // Else look right
      findKNode(node->right, closest_node, closest_distance, new_node, i+1);

    if (value - closest_distance <= node->joint_positions.at(depth)) { // There might be a closer on the left
      findKNode(node->left, closest_node, closest_distance, new_node, i+1);
    }
  }
}