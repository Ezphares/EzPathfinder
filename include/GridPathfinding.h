#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    struct ezpf_Point
    {
        int x, y;
    };

    struct ezpf_Grid
    {
        struct ezpf_Point dimensions;
        char *contents;
        char impassable;
        int allowDiagonals;
        float diagonalCost;
    };

    void ezpf_GridInit(struct ezpf_Grid *grid, int w, int h);
    void ezpf_GridDestroy(struct ezpf_Grid *grid);

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