#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef EZPF_POINT_TYPE
    struct ezpf_Point
    {
        int x, y;
    };
#else
typedef EZPF_POINT_TYPE ezpf_Point
#endif // !EZPF_POINT_TYPE

    typedef int (*ezpf_GridPassable)(void *, struct ezpf_Point);
    typedef float (*ezpf_GridCost)(
        void *, struct ezpf_Point, struct ezpf_Point);
    typedef float (*ezpf_GridHeuristic)(
        void *, struct ezpf_Point, struct ezpf_Point);

    struct ezpf_Grid
    {
        struct ezpf_Point dimensions;
        ezpf_GridPassable passableFunc;
        ezpf_GridHeuristic heuristicsOverride;
        ezpf_GridCost costOverride;
        void *userData;
        int allowDiagonals;
        float diagonalCost;
    };

    struct ezpf_ExplicitGridData
    {
        const char *contents;
        const char impassable;
        int width;
    };
    int ezpf_ExplicitGridPassable(void *explicitGrid, struct ezpf_Point point);

    void ezpf_GridInit(struct ezpf_Grid *grid, int width, int height);
    int ezpf_GridPathfind(
        struct ezpf_Point **out_Path,
        struct ezpf_Grid *grid,
        struct ezpf_Point from,
        struct ezpf_Point to);
    int ezpf_GridPathfindBuffer(
        struct ezpf_Point *out_Path,
        int pathMaxLength,
        struct ezpf_Grid *grid,
        struct ezpf_Point from,
        struct ezpf_Point to);

#ifdef __cplusplus
}
#endif