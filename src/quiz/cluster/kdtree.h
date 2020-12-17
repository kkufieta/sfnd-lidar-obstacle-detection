/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <iostream>

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
    std::cout << c << std::endl;
    int level = 0;
    while (*c != NULL) {
      std::cout << "level: " << level << ", c: " << c << std::endl;
      if (point[level % 2] < (*c)->point[level % 2]) {
        c = &(*c)->left;
      } else {
        c = &(*c)->right;
      }
      level += 1;
    }
    *c = new Node(point, id);
    std::cout << "inserted node: " << c << std::endl;
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    return ids;
  }
};
