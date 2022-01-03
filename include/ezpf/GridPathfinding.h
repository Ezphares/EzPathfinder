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

    struct ezpf_Grid
    {
        struct ezpf_Point dimensions;
        enum ezpf_GridPassableMode
        {
            GPM_BUFFERCHAR,
            GPM_CALLBACK,
        } passableMode;
        union
        {
            struct
            {
                char *contents;
                char impassable;
            } _buffer;
            struct
            {
                ezpf_GridPassable passableFunc;
                void *passableData;
            } _callback;
        } _passable;

        int allowDiagonals;
        float diagonalCost;
    };

    void ezpf_GridInit(struct ezpf_Grid *grid, int w, int h);
    void ezpf_GridSetContents(
        struct ezpf_Grid *grid,
        char impassable,
        const char *contents,
        int contentSize);
    void ezpf_GridSetCallback(
        struct ezpf_Grid *grid, ezpf_GridPassable callback, void *userdata);
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