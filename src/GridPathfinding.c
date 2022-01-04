#pragma once

#include "ezpf/GridPathfinding.h"
#include "ezpf/AStar.h"

#include <stdlib.h>
#include <string.h>

int ezpf_IsGridPassable(struct ezpf_Grid *grid, struct ezpf_Point point);

ezpf_NodeID ezpf_CellToID(struct ezpf_Grid *grid, struct ezpf_Point point)
{
    return point.x + point.y * grid->dimensions.x;
}

struct ezpf_Point ezpf_IDToCell(struct ezpf_Grid *grid, ezpf_NodeID id)
{
    struct ezpf_Point p;
    p.x = id % grid->dimensions.x;
    p.y = id / grid->dimensions.x;
    return p;
}

float ezpf_GridHeuristicInternal(
    void *grid, ezpf_NodeID node, ezpf_NodeID target)
{
    struct ezpf_Grid *_grid   = (struct ezpf_Grid *)grid;
    struct ezpf_Point _node   = ezpf_IDToCell(grid, node),
                      _target = ezpf_IDToCell(grid, target);

    if (_grid->heuristicsOverride)
    {
        return _grid->heuristicsOverride(_grid->userData, _node, _target);
    }

    // Fallback to euclidian
    float dx = (float)(_target.x - _node.x);
    float dy = (float)(_target.y - _node.y);

    return dx * dx + dy * dy;
}

float ezpf_GridCostInternal(void *grid, ezpf_NodeID node, ezpf_NodeID target)
{
    struct ezpf_Grid *_grid   = (struct ezpf_Grid *)grid;
    struct ezpf_Point _node   = ezpf_IDToCell(grid, node),
                      _target = ezpf_IDToCell(grid, target);

    // Fallback to 1/diagonalcost
    int manhattan = abs(_target.x - _node.x) + abs(_target.y - _node.y);

    return manhattan == 1 ? 1.0f : _grid->diagonalCost;
}

void ezpf_GridCardinalNeighbors(struct ezpf_Point *buffer, struct ezpf_Point p)
{
    buffer[0].x = p.x + 1;
    buffer[0].y = p.y;

    buffer[1].x = p.x - 1;
    buffer[1].y = p.y;

    buffer[2].x = p.x;
    buffer[2].y = p.y + 1;

    buffer[3].x = p.x;
    buffer[3].y = p.y - 1;
}

void ezpf_GridDiagonalNeighbors(struct ezpf_Point *buffer, struct ezpf_Point p)
{
    buffer[0].x = p.x + 1;
    buffer[0].y = p.y + 1;

    buffer[1].x = p.x - 1;
    buffer[1].y = p.y + 1;

    buffer[2].x = p.x + 1;
    buffer[2].y = p.y - 1;

    buffer[3].x = p.x - 1;
    buffer[3].y = p.y - 1;
}

int ezpf_ValidNode(struct ezpf_Grid *grid, struct ezpf_Point p)
{
    return p.x >= 0 && p.x < grid->dimensions.x && p.y >= 0
           && p.y < grid->dimensions.y && ezpf_IsGridPassable(grid, p);
}

int ezpf_GridNeighborCount(void *grid, ezpf_NodeID node)
{
    struct ezpf_Grid *_grid = (struct ezpf_Grid *)grid;
    struct ezpf_Point _node = ezpf_IDToCell(grid, node);

    struct ezpf_Point candidates[8];
    int candidateCount = 4;

    int val = 0;

    ezpf_GridCardinalNeighbors(candidates, _node);

    // Diagonals
    if (_grid->allowDiagonals)
    {
        ezpf_GridDiagonalNeighbors(candidates + 4, _node);
        candidateCount = 8;
    }

    for (int i = 0; i < candidateCount; i++)
    {
        if (ezpf_ValidNode(grid, candidates[i]))
        {
            val++;
        }
    }

    return val;
}

void ezpf_GridGetNeighbors(void *grid, ezpf_NodeID *buffer, ezpf_NodeID node)
{
    struct ezpf_Grid *_grid = (struct ezpf_Grid *)grid;
    struct ezpf_Point _node = ezpf_IDToCell(grid, node);

    struct ezpf_Point candidates[8];
    int candidateCount = 4;

    int val = 0;

    ezpf_GridCardinalNeighbors(candidates, _node);

    // Diagonals
    if (_grid->allowDiagonals)
    {
        ezpf_GridDiagonalNeighbors(candidates + 4, _node);
        candidateCount = 8;
    }

    for (int i = 0; i < candidateCount; i++)
    {
        if (ezpf_ValidNode(grid, candidates[i]))
        {
            buffer[val++] = ezpf_CellToID(_grid, candidates[i]);
        }
    }
}

ezpf_NodeID *ezpf_MallocNodes(int l)
{
    return malloc(l * sizeof(ezpf_NodeID));
}

int ezpf_GridPathfindInternal(
    struct ezpf_Point **out_Path,
    int pathMaxLength,
    struct ezpf_Grid *grid,
    struct ezpf_Point from,
    struct ezpf_Point to)
{
    struct ezpf_AStarSettings settings;

    settings.heuristic     = &ezpf_GridHeuristicInternal;
    settings.cost          = &ezpf_GridCostInternal;
    settings.neighborCount = &ezpf_GridNeighborCount;
    settings.neighbors     = &ezpf_GridGetNeighbors;
    settings.requestBuffer = &ezpf_MallocNodes;
    settings.nodeCount     = grid->dimensions.x * grid->dimensions.y;
    settings.data          = grid;

    ezpf_NodeID *buffer = 0;

    int l = ezpf_AStar(
        &buffer, settings, ezpf_CellToID(grid, from), ezpf_CellToID(grid, to));

    if (l > pathMaxLength)
    {
        if (pathMaxLength == -1)
        {
            pathMaxLength = l;
            *out_Path     = malloc(l * sizeof(struct ezpf_Point));
        }
    }

    if (l <= pathMaxLength)
    {
        for (int i = 0; i < l; i++)
        {
            (*out_Path)[i] = ezpf_IDToCell(grid, buffer[i]);
        }
    }

    free(buffer);
    return l;
}

int ezpf_ExplicitGridPassable(void *explicitGrid, struct ezpf_Point point)
{
    struct ezpf_ExplicitGridData *data
        = (struct ezpf_ExplicitGridData *)explicitGrid;

    return data->contents[point.x + data->width * point.y] != data->impassable;
}

void ezpf_GridInit(struct ezpf_Grid *grid, int width, int height)
{
    grid->dimensions.x       = width;
    grid->dimensions.y       = height;
    grid->passableFunc       = NULL;
    grid->heuristicsOverride = NULL;
    grid->costOverride       = NULL;
    grid->userData           = NULL;
    grid->allowDiagonals     = 0;
    grid->diagonalCost       = 1.0f;
}

int ezpf_GridPathfind(
    struct ezpf_Point **out_Path,
    struct ezpf_Grid *grid,
    struct ezpf_Point from,
    struct ezpf_Point to)
{
    return ezpf_GridPathfindInternal(out_Path, -1, grid, from, to);
}

int ezpf_GridPathfindBuffer(
    struct ezpf_Point *out_Path,
    int pathMaxLength,
    struct ezpf_Grid *grid,
    struct ezpf_Point from,
    struct ezpf_Point to)
{
    return ezpf_GridPathfindInternal(&out_Path, pathMaxLength, grid, from, to);
}

int ezpf_IsGridPassable(struct ezpf_Grid *grid, struct ezpf_Point point)
{
    return grid->passableFunc ? grid->passableFunc(grid->userData, point) : 1;
}
