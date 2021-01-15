#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef int ezpf_NodeID;

typedef float (*ezpf_Heuristic)(void *, ezpf_NodeID, ezpf_NodeID);
typedef float (*ezpf_Cost)(void *, ezpf_NodeID, ezpf_NodeID);

typedef int (*ezpf_NeighborCount)(void *, ezpf_NodeID);
typedef void (*ezpf_Neighbors)(void *, ezpf_NodeID *, ezpf_NodeID);

typedef ezpf_NodeID *(*ezpf_RequestBuffer)(int count);

struct ezpf_AStarSettings {
  ezpf_Heuristic heuristic;
  ezpf_Cost cost;
  ezpf_NeighborCount neighborCount;
  ezpf_Neighbors neighbors;
  ezpf_RequestBuffer requestBuffer;
  int nodeCount;
  void *data;
};

int pathfindAStar(ezpf_NodeID **out_result, struct ezpf_AStarSettings settings,
                  ezpf_NodeID from, ezpf_NodeID to);

#ifdef __cplusplus
}
#endif