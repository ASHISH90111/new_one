#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

// Simple node used for A* search
struct Node
{
  int x;
  int y;
  double g;
  double h;
  double f;
  int px;
  int py;
};

// Comparator for priority queue (min-heap on f-cost)
struct Compare
{
  bool operator()(const Node & a, const Node & b) const
  {
    return a.f > b.f;
  }
};
