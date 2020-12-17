/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>
#include <iostream>
#include <queue>
#include <tuple>

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
  Node *root;

  KdTree() : root(NULL) {}

  void insert(std::vector<float> point, int id) {
    Node **c = &root;
    int level = 0;
    while (*c != NULL) {
      if (point[level % 2] < (*c)->point[level % 2]) {
        c = &(*c)->left;
      } else {
        c = &(*c)->right;
      }
      level += 1;
    }
    *c = new Node(point, id);
  }

  void searchHelper(std ::vector<float> target, Node *node, int level,
                    float distanceTol, std::vector<int> &ids) {
    if (node != NULL) {
      if (node->point[0] >= target[0] - distanceTol &&
          node->point[0] <= target[0] + distanceTol &&
          node->point[1] >= target[1] - distanceTol &&
          node->point[1] <= target[1] + distanceTol &&
          std::sqrt(std::pow(node->point[0] - target[0], 2) +
                    std::pow(node->point[1] - target[1], 2)) <= distanceTol) {
        ids.push_back(node->id);
      }
      if (node->point[level % 2] >= target[level % 2] - distanceTol) {
        searchHelper(target, node->left, level + 1, distanceTol, ids);
      }
      if (node->point[level % 2] <= target[level % 2] + distanceTol) {
        searchHelper(target, node->right, level + 1, distanceTol, ids);
      }
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);
    return ids;
  }
};
