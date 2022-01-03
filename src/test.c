
#include "ezpf/GridPathfinding.h"

#include <stdlib.h>

void t1()
{
    struct ezpf_Grid g;

    ezpf_GridInit(&g, 8, 5);
    const char *map = "........"
                      "#######."
                      "........"
                      ".##.####"
                      "........";

    g.allowDiagonals = 1;

    ezpf_GridSetContents(&g, '#', map, strlen(map));

    struct ezpf_Point from = {.x = 0, .y = 0}, to = {.x = 7, .y = 4};

    struct ezpf_Point *buffer = 0;
    ezpf_GridPathfind(&buffer, &g, from, to);

    ezpf_GridDestroy(&g);
    free(buffer);
}

struct T2Data
{
    const char *map;
    int width;
};

int t2callback(void *data, struct ezpf_Point point)
{
    struct T2Data *_data = (struct T2Data *)data;

    return _data->map[point.x + _data->width * point.y] != '#';
}

void t2()
{
    struct ezpf_Grid g;

    ezpf_GridInit(&g, 8, 5);

    struct T2Data data;
    data.map = "........"
               "#######."
               "........"
               ".##.####"
               "........";
    data.width = 8;

    g.allowDiagonals = 1;

    ezpf_GridSetCallback(&g, t2callback, &data);

    struct ezpf_Point from = {.x = 0, .y = 0}, to = {.x = 7, .y = 4};

    struct ezpf_Point *buffer = 0;
    ezpf_GridPathfind(&buffer, &g, from, to);

    ezpf_GridDestroy(&g);
    free(buffer);
}

int main()
{
    t1();
    t2();
}
