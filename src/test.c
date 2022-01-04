
#include "ezpf/GridPathfinding.h"

#include <stdlib.h>

void t1()
{
    struct ezpf_ExplicitGridData data
        = {.contents = "........"
                       "#######."
                       "........"
                       ".##.####"
                       "........",
           .width      = 8,
           .impassable = '#'};

    struct ezpf_Grid g;
    ezpf_GridInit(&g, 8, 5);
    g.userData       = &data;
    g.allowDiagonals = 1;
    g.passableFunc   = &ezpf_ExplicitGridPassable;

    struct ezpf_Point from = {.x = 0, .y = 0}, to = {.x = 7, .y = 4};
    struct ezpf_Point *buffer = 0;
    ezpf_GridPathfind(&buffer, &g, from, to);

    free(buffer);
}

int main()
{
    t1();
}
