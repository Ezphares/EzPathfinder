#pragma once

#include "GridPathfinding.h"
#include "AStar.h"

#include <stdlib.h>
#include <string.h>

int ezpf_IsGridPassable(struct ezpf_Grid *grid, struct ezpf_Point point);
void ezpf_GridAllocateContents(struct ezpf_Grid *grid, int size);
void ezpf_GridFreeContents(struct ezpf_Grid *grid);

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

float ezpf_EuclideanHeuristic(void *grid, ezpf_NodeID node, ezpf_NodeID target)
{
    struct ezpf_Point _node   = ezpf_IDToCell(grid, node),
                      _target = ezpf_IDToCell(grid, target);

    float dx = (float)(_target.x - _node.x);
    float dy = (float)(_target.y - _node.y);

    return dx * dx + dy * dy;
}

float ezpf_GridCost(void *grid, ezpf_NodeID node, ezpf_NodeID target)
{
    struct ezpf_Grid *_grid   = (struct ezpf_Grid *)grid;
    struct ezpf_Point _node   = ezpf_IDToCell(grid, node),
                      _target = ezpf_IDToCell(grid, target);

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

    settings.heuristic     = &ezpf_EuclideanHeuristic;
    settings.cost          = &ezpf_GridCost;
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

void ezpf_GridInit(struct ezpf_Grid *grid, int w, int h)
{
    grid->dimensions.x   = w;
    grid->dimensions.y   = h;
    grid->contents       = NULL;
    grid->allowDiagonals = 0;
    grid->diagonalCost   = 1.4142f;
}

void ezpf_GridSetContents(
    struct ezpf_Grid *grid,
    char impassable,
    const char *contents,
    int contentSize)
{
    grid->passableMode = GPM_BUFFERCHAR;
    grid->impassable   = impassable;
    ezpf_GridAllocateContents(grid, contentSize);
    memcpy(grid->contents, contents, contentSize);
}

void ezpf_GridSetCallback(
    struct ezpf_Grid *grid, ezpf_GridPassable callback, void *userdata)
{
    grid->passableMode = GPM_CALLBACK;
    grid->passableFunc = callback;
    grid->passableData = userdata;
}

void ezpf_GridDestroy(struct ezpf_Grid *grid)
{
    ezpf_GridFreeContents(grid);
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
    if (grid->passableMode == GPM_BUFFERCHAR)
    {
        return grid->contents[ezpf_CellToID(grid, point)] != grid->impassable;
    }
    else
    {
        return grid->passableFunc(grid->passableData, point);
    }
}

void ezpf_GridFreeContents(struct ezpf_Grid *grid)
{
    if (grid->passableMode == GPM_BUFFERCHAR && grid->contents)
    {
        free(grid->contents);
    }
}

void ezpf_GridAllocateContents(struct ezpf_Grid *grid, int size)
{
    ezpf_GridFreeContents(grid);
    grid->contents = malloc(size);
}
