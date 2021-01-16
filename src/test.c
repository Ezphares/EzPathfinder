
#include "GridPathfinding.h"

#include <stdlib.h>

int main()
{
    struct ezpf_Grid g;

    ezpf_GridInit(&g, 8, 5);
    g.impassable = '#';
    const char* map = "........"
                      "#######."
                      "........"
                      ".##.####"
                      "........";

    g.allowDiagonals = 1;

    memcpy(g.contents, map, strlen(map));

    struct ezpf_Point from, to;
    from.x = 0;
    from.y = 0;
    to.x   = 7;
    to.y   = 4;

    struct ezpf_Point *buffer = 0;
    ezpf_GridPathfind(&buffer, &g, from, to);

    ezpf_GridDestroy(&g);
    free(buffer);
}
