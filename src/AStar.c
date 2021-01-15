#include "AStar.h"

#include <stdlib.h>
#include <string.h>

#define INF 0x7f800000
#define SETINF(f)                                                              \
  { *(int *)&(f) = INF; };

struct ezpf_AStarContext {
  int openCount;
  int neighborCap;
  ezpf_NodeID *open;
  ezpf_NodeID *parent;
  ezpf_NodeID *neighbors;
  float *gScore;
  float *fScore;
};

ezpf_NodeID popOpen(struct ezpf_AStarContext *ctx) {
  int idx = 0;
  ezpf_NodeID res = ctx->open[0];
  float val = ctx->fScore[res];

  for (int i = 1; i < ctx->openCount; i++) {
    const ezpf_NodeID test = ctx->open[i];
    if (ctx->fScore[test] < val) {
      idx = i;
      res = test;
      val = ctx->fScore[res];
    }
  }

  memmove(ctx->open + idx, ctx->open + idx + 1,
          (--ctx->openCount - idx) * sizeof(ezpf_NodeID));

  return res;
};

int ezpf_AStarLength(struct ezpf_AStarContext *ctx, ezpf_NodeID s,
                     ezpf_NodeID c) {
  int res = 0;

  while (c != s) {
    ++res;
    c = ctx->parent[c];
  }

  return res;
}

void ezpf_AStarReconstruct(ezpf_NodeID *out_result,
                           struct ezpf_AStarContext *ctx, ezpf_NodeID c,
                           int l) {
  for (int i = l - 1; i >= 0; i--) {
    out_result[i] = c;
    c = ctx->parent[c];
  }
}

int ezpf_GetNeightbors(struct ezpf_AStarContext *ctx,
                       struct ezpf_AStarSettings *settings, ezpf_NodeID c) {

  int n = settings->neighborCount(settings->data, c);

  if (n > ctx->neighborCap) {
    ctx->neighbors = realloc(ctx->neighbors, n * sizeof(ezpf_NodeID));
    ctx->neighborCap = n;
  }

  settings->neighbors(settings->data, ctx->neighbors, c);

  return n;
}

void ezpf_Open(struct ezpf_AStarContext *ctx, ezpf_NodeID n) {
  for (int i = 0; i < ctx->openCount; i++) {
    if (ctx->open[i] == n)
      return;
  }

  ctx->open[ctx->openCount++] = n;
}

int pathfindAStar(ezpf_NodeID **out_result, struct ezpf_AStarSettings settings,
                  ezpf_NodeID from, ezpf_NodeID to) {
  struct ezpf_AStarContext ctx;
  ctx.open = malloc(settings.nodeCount * sizeof(ezpf_NodeID));
  ctx.parent = malloc(settings.nodeCount * sizeof(ezpf_NodeID));
  ctx.fScore = malloc(settings.nodeCount * sizeof(float));
  ctx.gScore = malloc(settings.nodeCount * sizeof(float));

  for (int i = 0; i < settings.nodeCount; i++) {
    SETINF(ctx.gScore[i]);
    SETINF(ctx.fScore[i]);
  }

  ctx.openCount = 1;
  ctx.open[0] = from;
  ctx.fScore[from] = settings.heuristic(settings.data, from, to);
  ctx.gScore[from] = 0.0f;

  ctx.neighborCap = 8;
  ctx.neighbors = malloc(ctx.neighborCap * sizeof(ezpf_NodeID));

  int l = -1;

  while (ctx.openCount > 0) {
    ezpf_NodeID c = popOpen(&ctx);

    if (c == to) {
      l = ezpf_AStarLength(&ctx, from, to);
      *out_result = settings.requestBuffer(l);
      ezpf_AStarReconstruct(*out_result, &ctx, c, l);

      break;
    }

    int nn = ezpf_GetNeightbors(&ctx, &settings, c);
    for (int i = 0; i < nn; i++) {
      ezpf_NodeID n = ctx.neighbors[i];

      float g = ctx.gScore[c] + settings.cost(settings.data, c, n);
      if (g < ctx.gScore[n]) {
        ctx.parent[n] = c;
        ctx.gScore[n] = g;
        ctx.fScore[n] = g + settings.heuristic(settings.data, n, to);
        ezpf_Open(&ctx, n);
      }
    }
  }

  free(ctx.open);
  free(ctx.parent);
  free(ctx.fScore);
  free(ctx.gScore);
  free(ctx.neighbors);

  return l;
}
