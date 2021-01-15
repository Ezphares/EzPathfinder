
#include "AStar.h"

#include <stdlib.h>

struct Grid {
  int w, h;
  char *block;
};

struct Point {
  int x, y;
};

ezpf_NodeID pointToId(struct Grid *grid, struct Point point) {
  return point.x + point.y * grid->w;
}

struct Point idToPoint(struct Grid *grid, ezpf_NodeID id) {
  struct Point p;
  p.x = id % grid->w;
  p.y = id / grid->w;
  return p;
}

float euclideanHeuristic(void *grid, ezpf_NodeID node, ezpf_NodeID target) {
  struct Point _node = idToPoint(grid, node), _target = idToPoint(grid, target);

  float dx = (float)(_target.x - _node.x);
  float dy = (float)(_target.y - _node.y);

  return dx * dx + dy * dy;
}

float manhattanHeuristic(void *grid, ezpf_NodeID node, ezpf_NodeID target) {
  struct Point _node = idToPoint(grid, node), _target = idToPoint(grid, target);

  return (float)abs(_node.x - _target.x) + abs(_node.y - _target.y);
}

float gridCost(void *grid, ezpf_NodeID node, ezpf_NodeID target) {
  struct Grid *_grid = (struct Grid *)grid;

  if (_grid->block[target] == '#') {
    float f;
    *(int *)&f = 0x7f800000;
    return f;
  }

  return manhattanHeuristic(grid, node, target);
}

int gridNeighborCount(void *grid, ezpf_NodeID node) {
  struct Grid *_grid = (struct Grid *)grid;
  struct Point _node = idToPoint(grid, node);

  // Cardinals, no edges
  int val = 0;
  if (_node.x > 0)
    val++;
  if (_node.y > 0)
    val++;
  if (_node.x < _grid->w - 1)
    val++;
  if (_node.y < _grid->h - 1)
    val++;

  return val;
}

void gridNeighbors(void *grid, ezpf_NodeID *buffer, ezpf_NodeID node) {
  struct Grid *_grid = (struct Grid *)grid;
  struct Point _node = idToPoint(grid, node);

  // Cardinals, no edges
  int i = 0;
  struct Point p;
  if (_node.x > 0) {
    p.x = _node.x - 1;
    p.y = _node.y;
    buffer[i++] = pointToId(grid, p);
  }
  if (_node.y > 0) {
    p.x = _node.x;
    p.y = _node.y - 1;
    buffer[i++] = pointToId(grid, p);
  }
  if (_node.x < _grid->w - 1) {
    p.x = _node.x + 1;
    p.y = _node.y;
    buffer[i++] = pointToId(grid, p);
  }
  if (_node.y < _grid->h - 1) {
    p.x = _node.x;
    p.y = _node.y + 1;
    buffer[i++] = pointToId(grid, p);
  }
}

ezpf_NodeID *getPathBuffer(int l) { return malloc(l * sizeof(ezpf_NodeID)); }

int main() {

  struct ezpf_AStarSettings settings;

  struct Grid g;
  g.h = 5;
  g.w = 8;
  g.block = "........"
            "#######."
            "........"
            ".##.####"
            "........";

  settings.heuristic = &euclideanHeuristic;
  settings.cost = &gridCost;
  settings.neighborCount = &gridNeighborCount;
  settings.neighbors = &gridNeighbors;

  settings.requestBuffer = &getPathBuffer;
  settings.nodeCount = g.h * g.w;
  settings.data = &g;

  ezpf_NodeID *buffer;

  struct Point from, to;
  from.x = 0;
  from.y = 0;
  to.x = 7;
  to.y = 4;
  pathfindAStar(&buffer, settings, pointToId(&g, from), pointToId(&g, to));
}
