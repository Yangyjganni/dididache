#include <stdio.h>
#include <string.h>
#include "index_map.h"
#include "FindRoad.h"


void testDij()
{
    printf("enter in two points\n");
    int a, b;
    scanf("%d%d", &a, &b);
    EdgeList pathList = dijkstra(a, b);
    printf("%d ", a);
    for(int i = 0; i < pathList.num; ++i) {
        printf("%d ", pathList.data[i].b);
    }
    printf("\n");
    deleteEdgeList(&pathList);
}

void testFindRoad()
{
    int x1, x2, y1, y2;
    do {
        printf("enter in two points x and y\n");

        scanf("%d%d%d%d", &x1, &y1, &x2, &y2);
        /*
        MoveList moveList = find_road(x1, y1, x2, y2);
        for (int i = 0; i < moveList.num; ++i) {
            printf("type: %d moveDis: %f turn_angle %d dest_x:%d dest_y:%d turn_r: %f\n", moveList.data[i].type,
                   moveList.data[i].dis, moveList.data[i].angle, moveList.data[i].dest_x, moveList.data[i].dest_y,
                   moveList.data[i].r);
        }
         */
        while(x1 != x2 && y1 != y2) {
            CarMove curMove = get_next_move(x1, y1, x2, y2);
            printf("type: %d moveDis: %f turn_angle %d start_x:%d start_y:%d dest_x:%d dest_y:%d turn_r: %f\n", curMove.type,
                    curMove.dis, curMove.angle, curMove.start_x, curMove.start_y, curMove.dest_x, curMove.dest_y, curMove.r);
            x1 = curMove.dest_x; y1 = curMove.dest_y;
        }
    }while(x1 >= 0);
}


int main() {
    //FindAllDis();
    testFindRoad();
    //testDij();
    printf("END\n");
    //printf("Hello, World!\n");
    return 0;
}